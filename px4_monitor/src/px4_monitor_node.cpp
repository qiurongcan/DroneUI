


#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

// === 新增：包含 VFR_HUD 和 BatteryState 的头文件 ===
#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/BatteryState.h>

#include <thread>
#include <mutex>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <atomic>
#include <vector>

// FTXUI 头文件
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/component_options.hpp>
#include <ftxui/component/screen_interactive.hpp>

using namespace ftxui;

// -----------------------------------------------------------------------------
// 数据结构
// -----------------------------------------------------------------------------
struct DroneState {
    bool connected = false;
    bool armed = false;
    std::string mode = "UNKNOWN";

    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    // === 新增：存储油门和电池数据 ===
    double throttle = 0.0;
    double battery_voltage = 0.0;
    double battery_percentage = 0.0;
};
struct TargetSetPoint {
    double x = 0.0; double y = 0.0; double z = 0.0; double yaw = 0.0;
};

// 全局变量
DroneState g_drone_state;
std::mutex g_state_mtx; 

TargetSetPoint g_target_sp; 
std::mutex g_target_mtx;    

std::string g_log_msg = "System Ready"; 
std::atomic<bool> g_enable_constant_publish(true); 

// ROS 全局对象
ros::Publisher g_local_pos_pub;
ros::Publisher g_move_base_goal_pub;
ros::ServiceClient g_arming_client;
ros::ServiceClient g_set_mode_client;
ros::ServiceClient g_param_get_client;
ros::ServiceClient g_param_set_client;

// -----------------------------------------------------------------------------
// ROS 回调与辅助函数
// -----------------------------------------------------------------------------
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_state_mtx);
    g_drone_state.connected = msg->connected;
    g_drone_state.armed = msg->armed;
    g_drone_state.mode = msg->mode;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_state_mtx);
    g_drone_state.x = msg->pose.position.x;
    g_drone_state.y = msg->pose.position.y;
    g_drone_state.z = msg->pose.position.z;

    tf::Quaternion q(
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    g_drone_state.roll = r * 180.0 / M_PI;
    g_drone_state.pitch = p * 180.0 / M_PI;
    g_drone_state.yaw = y * 180.0 / M_PI;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_state_mtx);
    g_drone_state.vx = msg->twist.linear.x;
    g_drone_state.vy = msg->twist.linear.y;
    g_drone_state.vz = msg->twist.linear.z;
}

// === 新增：处理 VFR_HUD (包含油门等) 的回调函数 ===
void vfr_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_state_mtx);
    g_drone_state.throttle = msg->throttle; // PX4中通常为 0.0 ~ 1.0 的浮点数
}

// === 新增：处理 BatteryState 的回调函数 ===
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_state_mtx);
    g_drone_state.battery_voltage = msg->voltage;
    g_drone_state.battery_percentage = msg->percentage; // 通常为 0.0 ~ 1.0 的浮点数
}

void set_log(std::string msg) { g_log_msg = msg; }

std::string fmt(double val, int precision = 2) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << val;
    return ss.str();
}

geometry_msgs::PoseStamped create_pose(double x, double y, double z, double yaw_deg) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    tf::Quaternion q;
    q.setRPY(0, 0, yaw_deg * M_PI / 180.0);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

bool set_mode(std::string mode_name) {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mode_name;
    if (g_set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        set_log("Mode switch to " + mode_name + " REQUESTED");
        return true;
    }
    set_log("Failed to switch to " + mode_name);
    return false;
}

std::string get_param(const std::string& param_id) {
    mavros_msgs::ParamGet srv;
    srv.request.param_id = param_id;
    if (g_param_get_client.call(srv) && srv.response.success) {
        if (srv.response.value.integer != 0 || srv.response.value.real == 0.0) 
            return std::to_string(srv.response.value.integer);
        else 
            return fmt(srv.response.value.real);
    }
    return "ERR";
}

void set_param(const std::string& param_id, const std::string& value_str) {
    try {
        mavros_msgs::ParamSet srv;
        srv.request.param_id = param_id;
        if (value_str.find('.') != std::string::npos) {
            srv.request.value.real = std::stof(value_str);
        } else {
            srv.request.value.integer = std::stol(value_str);
        }
        
        if (g_param_set_client.call(srv) && srv.response.success) {
            set_log("Set " + param_id + " success!");
        } else {
            set_log("Failed to set " + param_id);
        }
    } catch (...) {
        set_log("Invalid Param Format");
    }
}

// -----------------------------------------------------------------------------
// 后台线程逻辑
// -----------------------------------------------------------------------------
void constant_publisher_routine() {
    ros::Rate rate(20);
    while (ros::ok()) {
        if (g_enable_constant_publish) {
            TargetSetPoint target;
            {
                std::lock_guard<std::mutex> lock(g_target_mtx);
                target = g_target_sp;
            }
            auto pose = create_pose(target.x, target.y, target.z, target.yaw);
            g_local_pos_pub.publish(pose);
        }
        rate.sleep();
    }
}

void takeoff_routine(double h) {
    set_log("Auto Takeoff Sequence Started...");
    g_enable_constant_publish = false;
    ros::Rate rate(20);
    
    double current_x, current_y, current_yaw;
    {
        std::lock_guard<std::mutex> lock(g_state_mtx);
        current_x = g_drone_state.x;
        current_y = g_drone_state.y;
        current_yaw = g_drone_state.yaw;
    }

    auto pose = create_pose(current_x, current_y, h, current_yaw);

    for(int i=0; i<20 && ros::ok(); i++) {
        g_local_pos_pub.publish(pose);
        rate.sleep();
    }

    set_mode("OFFBOARD");
    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_req = ros::Time::now();

    while(ros::ok()) {
        DroneState state;
        { std::lock_guard<std::mutex> lock(g_state_mtx); state = g_drone_state; }

        if (!state.armed && (ros::Time::now() - last_req > ros::Duration(2.0))) {
            if(g_arming_client.call(arm_cmd) && arm_cmd.response.success) set_log("Vehicle ARMED");
            last_req = ros::Time::now();
        }

        g_local_pos_pub.publish(pose);

        if (state.mode == "OFFBOARD" && state.armed) {
            if (std::abs(state.z - h) < 0.2) {
                set_log("Takeoff Complete. Holding (AUTO.LOITER).");
                set_mode("AUTO.LOITER");
                {
                    std::lock_guard<std::mutex> lock(g_target_mtx);
                    g_target_sp.x = state.x; g_target_sp.y = state.y;
                    g_target_sp.z = state.z; g_target_sp.yaw = state.yaw;
                }
                break;
            }
        }
        rate.sleep();
    }
    g_enable_constant_publish = true;
}

// -----------------------------------------------------------------------------
// 主函数
// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "px4_ftxui_control");
    ros::NodeHandle nh;

    // ROS Init
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, vel_cb);
    
    // === 新增：订阅 VFR_HUD 和 Battery 话题 ===
    ros::Subscriber vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>("mavros/vfr_hud", 10, vfr_hud_cb);
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, battery_cb);

    g_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    g_move_base_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    
    g_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    g_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    g_param_get_client = nh.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
    g_param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");

    std::thread ros_thread([](){ ros::spin(); });
    std::thread pub_thread(constant_publisher_routine);

    // --- UI State Variables ---
    std::string tx_str = "0.0", ty_str = "0.0", tz_str = "1.5", tyaw_str = "0.0";
    std::string takeoff_h_str = "1.5";
    std::string p_ev_str = "0", p_gps_str = "0", p_hgt_str = "0";
    std::string mb_x = "1", mb_y = "0", mb_z = "1", mb_yaw = "0.0";

    int tab_index = 0;
    std::vector<std::string> tab_entries = { " 1. Flight Control ", " 2. PX4 Params ", " 3. Move Base " };
    auto tab_toggle = Toggle(&tab_entries, &tab_index);

    // =========================================================================
    // 1. Tab 1 (飞行控制) - 逻辑与渲染解耦封装
    // =========================================================================
    Component input_takeoff_h = Input(&takeoff_h_str, "H");
    auto btn_takeoff = Button("   AUTO TAKEOFF   ", [&] {
        try { double h = std::stof(takeoff_h_str); std::thread(takeoff_routine, h).detach(); } catch (...) { set_log("Invalid Takeoff Height"); }
    }, ButtonOption::Animated(Color::RedLight));

    auto btn_offboard = Button("  OFFBOARD (FLY)  ", [&] { std::thread([]{ set_mode("OFFBOARD"); }).detach(); }, ButtonOption::Animated(Color::CyanLight));
    auto btn_loiter = Button("AUTO.LOITER (HOLD)", [&] { 
        std::thread([]{ set_mode("AUTO.LOITER"); }).detach(); 
        DroneState c; { std::lock_guard<std::mutex> lock(g_state_mtx); c = g_drone_state; }
        { std::lock_guard<std::mutex> lock(g_target_mtx); g_target_sp = {c.x, c.y, c.z, c.yaw}; }
        tx_str = fmt(c.x); ty_str = fmt(c.y); tz_str = fmt(c.z); tyaw_str = fmt(c.yaw);
    }, ButtonOption::Animated(Color::GreenLight));
    auto btn_land = Button("    AUTO.LAND     ", [&] { std::thread([]{ set_mode("AUTO.LAND"); }).detach(); }, ButtonOption::Animated(Color::YellowLight));

    Component input_x = Input(&tx_str, "X"); Component input_y = Input(&ty_str, "Y");
    Component input_z = Input(&tz_str, "Z"); Component input_yaw = Input(&tyaw_str, "Yaw");
    auto btn_send_sp = Button(" UPDATE SETPOINT ", [&] {
        try {
            std::lock_guard<std::mutex> lock(g_target_mtx);
            g_target_sp = {std::stof(tx_str), std::stof(ty_str), std::stof(tz_str), std::stof(tyaw_str)};
            set_log("Setpoint Updated.");
        } catch (...) { set_log("Invalid Input"); }
    }, ButtonOption::Animated(Color::BlueLight));

    auto container_tab1 = Container::Horizontal({
        Container::Vertical({ Container::Horizontal({input_takeoff_h, btn_takeoff}), btn_offboard, btn_loiter, btn_land }),
        Container::Vertical({ Container::Horizontal({input_x, input_y}), Container::Horizontal({input_z, input_yaw}), btn_send_sp })
    });

    auto tab1_renderer = Renderer(container_tab1, [&] {
        TargetSetPoint target;
        { std::lock_guard<std::mutex> lock(g_target_mtx); target = g_target_sp; }
        
        auto action_ui = window(text(" QUICK ACTIONS ") | bold | hcenter, vbox({
            hbox({ text(" Alt(m): ") | center, input_takeoff_h->Render() | size(WIDTH, EQUAL, 6) | border, filler(), btn_takeoff->Render() }),
            separatorLight(), btn_offboard->Render() | xflex, btn_loiter->Render() | xflex, btn_land->Render() | xflex
        }));
        auto target_info = hbox({ text(" Target -> ") | dim, text(fmt(target.x) + ", " + fmt(target.y) + ", " + fmt(target.z) + " | Yaw: " + fmt(target.yaw)) | color(Color::Magenta) | bold });
        auto nav_ui = window(text(" NAVIGATION (FLY TO) ") | bold | hcenter, vbox({
            hbox({ hbox({ text(" X(m): ") | center, input_x->Render() | size(WIDTH, EQUAL, 8) | border }) | flex, hbox({ text(" Y(m): ") | center, input_y->Render() | size(WIDTH, EQUAL, 8) | border }) | flex }),
            hbox({ hbox({ text(" Z(m): ") | center, input_z->Render() | size(WIDTH, EQUAL, 8) | border }) | flex, hbox({ text(" Yaw(°): ") | center, input_yaw->Render() | size(WIDTH, EQUAL, 6) | border }) | flex }),
            filler(), btn_send_sp->Render() | hcenter, separatorLight(), target_info | hcenter
        }));
        return hbox({ action_ui | size(WIDTH, EQUAL, 35), nav_ui | flex });
    });

    // =========================================================================
    // 2. Tab 2 (参数配置) - 逻辑与渲染解耦封装
    // =========================================================================
    Component in_ev = Input(&p_ev_str, "Val"); Component in_gps = Input(&p_gps_str, "Val"); Component in_hgt = Input(&p_hgt_str, "Val");
    auto btn_get_ev = Button(" GET ", [&]{ p_ev_str = get_param("EKF2_EV_CTRL"); }, ButtonOption::Simple());
    auto btn_set_ev = Button(" SET ", [&]{ set_param("EKF2_EV_CTRL", p_ev_str); }, ButtonOption::Animated(Color::Red));
    auto btn_get_gps = Button(" GET ", [&]{ p_gps_str = get_param("EKF2_GPS_CTRL"); }, ButtonOption::Simple());
    auto btn_set_gps = Button(" SET ", [&]{ set_param("EKF2_GPS_CTRL", p_gps_str); }, ButtonOption::Animated(Color::Red));
    auto btn_get_hgt = Button(" GET ", [&]{ p_hgt_str = get_param("EKF2_HGT_REF"); }, ButtonOption::Simple());
    auto btn_set_hgt = Button(" SET ", [&]{ set_param("EKF2_HGT_REF", p_hgt_str); }, ButtonOption::Animated(Color::Red));

    auto params_comp = Container::Vertical({
        Container::Horizontal({ in_ev, btn_set_ev, btn_get_ev }),
        Container::Horizontal({ in_gps, btn_set_gps, btn_get_gps }),
        Container::Horizontal({ in_hgt, btn_set_hgt, btn_get_hgt })
    });

    auto tab2_renderer = Renderer(params_comp, [&] {
        return window(text(" PX4 PARAM CONFIGURATION ") | bold | hcenter, vbox({
            hbox({ text(" EKF2_EV_CTRL:  ") | size(WIDTH, EQUAL, 16), in_ev->Render() | border | flex, btn_set_ev->Render(), btn_get_ev->Render() }),
            text(" └─ Bitmask: 0=Disabled, 1=Horiz Pose, 2=Vert Pose, 4=3D Vel, 8=Yaw (e.g. 15=All, 0=None, 15=Default)") | dim | color(Color::GrayDark),
            separatorLight(),
            hbox({ text(" EKF2_GPS_CTRL: ") | size(WIDTH, EQUAL, 16), in_gps->Render() | border | flex, btn_set_gps->Render(), btn_get_gps->Render() }),
            text(" └─ Bitmask: 0=Disabled, 1=Lon/Lat, 2=Altitude, 4=3D Vel, 8=Dual Attenna head (e.g. 15=All, 0=None, 7=Default)") | dim | color(Color::GrayDark),
            separatorLight(),
            hbox({ text(" EKF2_HGT_REF:  ") | size(WIDTH, EQUAL, 16), in_hgt->Render() | border | flex, btn_set_hgt->Render(), btn_get_hgt->Render() }),
            text(" └─ Source: 0=Barometer, 1=GPS, 2=Range, 3=Vision(EV) (e.g. 1=Default)") | dim | color(Color::GrayDark)
        }));
    });

    // =========================================================================
    // 3. Tab 3 (Move Base) - 逻辑与渲染解耦封装
    // =========================================================================
    Component in_mb_x = Input(&mb_x, "X"); Component in_mb_y = Input(&mb_y, "Y");
    Component in_mb_z = Input(&mb_z, "Z"); Component in_mb_yaw = Input(&mb_yaw, "Yaw");
    
    auto btn_send_mb = Button(" PUBLISH ", [&]{
        try {
            auto pose = create_pose(std::stof(mb_x), std::stof(mb_y), std::stof(mb_z), std::stof(mb_yaw));
            g_move_base_goal_pub.publish(pose);
            set_log("Move Base Goal Published: " + mb_x + ", " + mb_y + ", " + mb_z);
        } catch(...) { set_log("Invalid Move Base Input"); }
    }, ButtonOption::Animated(Color::Green1));

    auto mb_comp = Container::Vertical({ Container::Horizontal({ in_mb_x, in_mb_y, in_mb_z, in_mb_yaw }), btn_send_mb });
    // auto mb_comp = Container::Horizontal({ in_mb_x, in_mb_y, in_mb_z, in_mb_yaw , btn_send_mb});

    auto tab3_renderer = Renderer(mb_comp, [&] {
        return window(text(" ROS NAVIGATION (MOVE BASE) ") | bold | hcenter, vbox({
            text(" Publish goal to /move_base_simple/goal ") | hcenter | dim,
            separatorLight(),
            hbox({ 
                hbox({ text(" X[m]: ") | center, in_mb_x->Render() | size(WIDTH, EQUAL, 8) | border }),
                hbox({ text(" Y[m]: ") | center, in_mb_y->Render() | size(WIDTH, EQUAL, 8) | border }),
                hbox({ text(" Z[m]: ") | center, in_mb_z->Render() | size(WIDTH, EQUAL, 8) | border }),
                hbox({ text(" Yaw[°]: ") | center, in_mb_yaw->Render() | size(WIDTH, EQUAL, 8) | border })
            }) | hcenter,
            separatorLight(),
            btn_send_mb->Render() | hcenter
        }));
    });

    // =========================================================================
    // 4. 全局布局引擎
    // =========================================================================
    auto tab_container = Container::Tab({ tab1_renderer, tab2_renderer, tab3_renderer }, &tab_index);
    
    auto main_container = Container::Vertical({
        tab_toggle,
        tab_container
    });

    auto screen = ScreenInteractive::Fullscreen();

    // =========================================================================
    // 5. 主渲染逻辑 (Main Renderer)
    // =========================================================================
    auto main_renderer = Renderer(main_container, [&] {
        DroneState current;
        { std::lock_guard<std::mutex> lock(g_state_mtx); current = g_drone_state; }

        // === 新增：处理油门和电池数据的显示格式与动态颜色 ===
        // 兼容处理：如果数据是 0~1 的小数则转为百分比显示
        double disp_thr = current.throttle <= 1.0 ? current.throttle * 100.0 : current.throttle;
        double disp_bat_pct = current.battery_percentage <= 1.0 ? current.battery_percentage * 100.0 : current.battery_percentage;
        
        // 电池电量过低警告颜色 (低于20%红，低于40%黄)
        Color bat_color = Color::Green;
        if (disp_bat_pct > 0.1 && disp_bat_pct < 20.0) bat_color = Color::Red;
        else if (disp_bat_pct > 0.1 && disp_bat_pct < 40.0) bat_color = Color::Yellow;

        // === 修改：在 SYSTEM STATUS 面板中追加这两行显示 ===
        auto status_panel = window(text(" SYSTEM STATUS ") | bold, vbox({
            hbox({ text(" Connection: ") | dim, text(current.connected ? "CONNECTED " : "DISCONNECTED") | color(current.connected ? Color::Green : Color::Red) | bold }),
            hbox({ text(" Flight Mode:") | dim, text(" " + current.mode) | color(Color::Yellow) | bold }),
            hbox({ text(" Arm Status: ") | dim, text(current.armed ? "ARMED  " : "DISARMED") | color(current.armed ? Color::Green : Color::Red) | bold }),
            separatorLight(),
            hbox({ text(" Throttle:   ") | dim, text(fmt(disp_thr, 1) + "%") | color(Color::Cyan) | bold }),
            hbox({ text(" Battery:    ") | dim, text(fmt(current.battery_voltage, 2) + "V (" + fmt(disp_bat_pct, 0) + "%)") | color(bat_color) | bold }),
        }));

        auto telemetry_content = hbox({
            vbox({
                text(" Position (m) ") | center | bold, separatorLight(),
                hbox({ text(" X: ") | dim, text(fmt(current.x)) | size(WIDTH, EQUAL, 8), text(" Vx: ") | dim, text(fmt(current.vx)) | size(WIDTH, EQUAL, 6) }),
                hbox({ text(" Y: ") | dim, text(fmt(current.y)) | size(WIDTH, EQUAL, 8), text(" Vy: ") | dim, text(fmt(current.vy)) | size(WIDTH, EQUAL, 6) }),
                hbox({ text(" Z: ") | dim, text(fmt(current.z)) | size(WIDTH, EQUAL, 8), text(" Vz: ") | dim, text(fmt(current.vz)) | size(WIDTH, EQUAL, 6) }),
            }) | flex,
            separatorDouble(),
            vbox({
                text(" Attitude (deg) ") | center | bold, separatorLight(),
                hbox({ text(" Roll:  ") | dim, text(fmt(current.roll)) | size(WIDTH, EQUAL, 7), gauge(0.5 + current.roll / 90.0) | flex }),
                hbox({ text(" Pitch: ") | dim, text(fmt(current.pitch)) | size(WIDTH, EQUAL, 7), gauge(0.5 + current.pitch / 90.0) | flex }),
                hbox({ text(" Yaw:   ") | dim, text(fmt(current.yaw)) | size(WIDTH, EQUAL, 7), gauge(0.5 + current.yaw / 360.0) | flex }),
            }) | flex
        });
        auto data_panel = window(text(" TELEMETRY DATA ") | bold, telemetry_content);

        return vbox({
            text("✈ PX4 ROS1 COMMAND CENTER") | bold | hcenter | color(Color::Cyan),
            separatorDouble(),
            hbox({ status_panel | size(WIDTH, EQUAL, 35), data_panel | flex }),
            separator(),
            tab_toggle->Render() | hcenter | border, 
            tab_container->Render() | flex, 
            filler(),
            hbox({ text(" LOG ") | bold | color(Color::Black) | bgcolor(Color::White), text(" " + g_log_msg) | color(Color::White) }) | border
        });
    });

    // 刷新线程
    std::thread refresh_thread([&screen] {
        while (ros::ok()) {
            screen.Post(Event::Custom);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    std::thread([&]{
        std::this_thread::sleep_for(std::chrono::seconds(1));
        p_ev_str = get_param("EKF2_EV_CTRL");
        p_gps_str = get_param("EKF2_GPS_CTRL");
        p_hgt_str = get_param("EKF2_HGT_REF");
    }).detach();

    screen.Loop(main_renderer); 

    g_enable_constant_publish = false;
    ros::shutdown();
    if (ros_thread.joinable()) ros_thread.join();
    if (pub_thread.joinable()) pub_thread.join();
    if (refresh_thread.joinable()) refresh_thread.join();

    return 0;
}
