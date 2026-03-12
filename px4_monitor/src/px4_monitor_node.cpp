#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

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
};

// 目标设定点结构
struct TargetSetPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
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
ros::ServiceClient g_arming_client;
ros::ServiceClient g_set_mode_client;

// -----------------------------------------------------------------------------
// ROS 回调
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

// -----------------------------------------------------------------------------
// 辅助函数
// -----------------------------------------------------------------------------
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
                    g_target_sp.x = state.x;
                    g_target_sp.y = state.y;
                    g_target_sp.z = state.z; 
                    g_target_sp.yaw = state.yaw;
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

    g_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    g_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    g_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    std::thread ros_thread([](){ ros::spin(); });
    std::thread pub_thread(constant_publisher_routine);

    // --- UI State ---
    std::string tx_str = "0.0";
    std::string ty_str = "0.0";
    std::string tz_str = "1.5";
    std::string tyaw_str = "0.0";
    std::string takeoff_h_str = "1.5";

    // =========================================================================
    // 1. 定义纯交互组件 (Components)
    // =========================================================================
    
    // -- 左侧：快捷动作区 (Takeoff & Modes) --
    Component input_takeoff_h = Input(&takeoff_h_str, "H");
    auto btn_takeoff = Button("   AUTO TAKEOFF   ", [&] {
        try {
            double h = std::stof(takeoff_h_str);
            std::thread(takeoff_routine, h).detach();
        } catch (...) { set_log("Invalid Takeoff Height"); }
    }, ButtonOption::Animated(Color::RedLight));

    auto btn_offboard = Button("  OFFBOARD (FLY)  ", [&] {
        std::thread([]{ set_mode("OFFBOARD"); }).detach();
    }, ButtonOption::Animated(Color::CyanLight));

    auto btn_loiter = Button("AUTO.LOITER (HOLD)", [&] {
        std::thread([]{ set_mode("AUTO.LOITER"); }).detach();
        DroneState current;
        { std::lock_guard<std::mutex> lock(g_state_mtx); current = g_drone_state; }
        {
            std::lock_guard<std::mutex> lock(g_target_mtx);
            g_target_sp.x = current.x; g_target_sp.y = current.y;
            g_target_sp.z = current.z; g_target_sp.yaw = current.yaw;
            tx_str = fmt(current.x); ty_str = fmt(current.y);
            tz_str = fmt(current.z); tyaw_str = fmt(current.yaw);
        }
    }, ButtonOption::Animated(Color::GreenLight));

    auto btn_land = Button("    AUTO.LAND     ", [&] {
        std::thread([]{ set_mode("AUTO.LAND"); }).detach();
    }, ButtonOption::Animated(Color::YellowLight));

    // 左侧逻辑容器
    auto action_comp = Container::Vertical({
        Container::Horizontal({input_takeoff_h, btn_takeoff}),
        btn_offboard,
        btn_loiter,
        btn_land
    });

    // -- 右侧：指点飞行导航区 (Navigation) --
    Component input_x = Input(&tx_str, "X");
    Component input_y = Input(&ty_str, "Y");
    Component input_z = Input(&tz_str, "Z");
    Component input_yaw = Input(&tyaw_str, "Yaw");
    auto btn_send_sp = Button(" UPDATE SETPOINT ", [&] {
        try {
            std::lock_guard<std::mutex> lock(g_target_mtx);
            g_target_sp.x = std::stof(tx_str);
            g_target_sp.y = std::stof(ty_str);
            g_target_sp.z = std::stof(tz_str);
            g_target_sp.yaw = std::stof(tyaw_str);
            set_log("Setpoint Updated. Switch to OFFBOARD to execute.");
        } catch (...) { set_log("Invalid Setpoint Input"); }
    }, ButtonOption::Animated(Color::BlueLight));

    // 右侧逻辑容器
    auto nav_comp = Container::Vertical({
        Container::Horizontal({input_x, input_y}),
        Container::Horizontal({input_z, input_yaw}),
        btn_send_sp
    });

    // 组合全局逻辑容器
    auto main_container = Container::Horizontal({
        action_comp, 
        nav_comp
    });

    auto screen = ScreenInteractive::Fullscreen();

    // =========================================================================
    // 2. 定义视觉渲染 (Renderer)
    // =========================================================================
    auto renderer = Renderer(main_container, [&] {
        DroneState current;
        TargetSetPoint target;
        { std::lock_guard<std::mutex> lock(g_state_mtx); current = g_drone_state; }
        { std::lock_guard<std::mutex> lock(g_target_mtx); target = g_target_sp; }

        // --- 顶部左：系统状态 ---
        auto status_panel = window(text(" SYSTEM STATUS ") | bold, vbox({
            hbox({ text(" Connection: ") | dim, text(current.connected ? "CONNECTED " : "DISCONNECTED") | color(current.connected ? Color::Green : Color::Red) | bold }),
            hbox({ text(" Flight Mode:") | dim, text(" " + current.mode) | color(Color::Yellow) | bold }),
            hbox({ text(" Arm Status: ") | dim, text(current.armed ? "ARMED  " : "DISARMED") | color(current.armed ? Color::Green : Color::Red) | bold }),
        }));

        // --- 顶部右：遥测数据 (位置 + 姿态) ---
        // 使用 hbox 将位置和姿态左右分开显示
        auto telemetry_content = hbox({
            // 左半边：位置与速度
            vbox({
                text(" Position (m) ") | center | bold,
                separatorLight(),
                hbox({ text(" X: ") | dim, text(fmt(current.x)) | size(WIDTH, EQUAL, 8), text(" Vx: ") | dim, text(fmt(current.vx)) | size(WIDTH, EQUAL, 6) }),
                hbox({ text(" Y: ") | dim, text(fmt(current.y)) | size(WIDTH, EQUAL, 8), text(" Vy: ") | dim, text(fmt(current.vy)) | size(WIDTH, EQUAL, 6) }),
                hbox({ text(" Z: ") | dim, text(fmt(current.z)) | size(WIDTH, EQUAL, 8), text(" Vz: ") | dim, text(fmt(current.vz)) | size(WIDTH, EQUAL, 6) }),
            }) | flex,
            
            separatorDouble(), // 中间竖线分隔

            // 右半边：姿态 (Roll/Pitch/Yaw) + 仪表盘模拟
            vbox({
                text(" Attitude (deg) ") | center | bold,
                separatorLight(),
                hbox({ 
                    text(" Roll:  ") | dim, 
                    text(fmt(current.roll)) | size(WIDTH, EQUAL, 7),
                    gauge(0.5 + current.roll / 90.0) | flex // 仪表条
                }),
                hbox({ 
                    text(" Pitch: ") | dim, 
                    text(fmt(current.pitch)) | size(WIDTH, EQUAL, 7),
                    gauge(0.5 + current.pitch / 90.0) | flex 
                }),
                hbox({ 
                    text(" Yaw:   ") | dim, 
                    text(fmt(current.yaw)) | size(WIDTH, EQUAL, 7),
                    gauge(0.5 + current.yaw / 360.0) | flex 
                }),
            }) | flex
        });

        auto data_panel = window(text(" TELEMETRY DATA ") | bold, telemetry_content);

        // --- 中左：快捷动作 UI ---
        auto action_ui = window(text(" QUICK ACTIONS ") | bold | hcenter, vbox({
            hbox({ 
                text(" Alt(m): ") | center, 
                input_takeoff_h->Render() | size(WIDTH, EQUAL, 6) | border, 
                filler(), 
                btn_takeoff->Render() 
            }),
            separatorLight(),
            btn_offboard->Render() | xflex,
            btn_loiter->Render() | xflex,
            btn_land->Render() | xflex
        }));

        // --- 中右：导航 UI ---
        auto target_info = hbox({
            text(" Target -> ") | dim,
            text(fmt(target.x) + ", " + fmt(target.y) + ", " + fmt(target.z) + " | Yaw: " + fmt(target.yaw)) | color(Color::Magenta) | bold
        });

        auto nav_ui = window(text(" NAVIGATION (FLY TO) ") | bold | hcenter, vbox({
            hbox({
                hbox({ text(" X(m): ") | center, input_x->Render() | size(WIDTH, EQUAL, 8) | border }) | flex,
                hbox({ text(" Y(m): ") | center, input_y->Render() | size(WIDTH, EQUAL, 8) | border }) | flex,
            }),
            hbox({
                hbox({ text(" Z(m): ") | center, input_z->Render() | size(WIDTH, EQUAL, 8) | border }) | flex,
                hbox({ text(" Yaw(°): ") | center, input_yaw->Render() | size(WIDTH, EQUAL, 6) | border }) | flex,
            }),
            filler(),
            btn_send_sp->Render() | hcenter,
            separatorLight(),
            target_info | hcenter
        }));

        // --- 整体拼接 ---
        return vbox({
            text("✈ PX4 ROS1 COMMAND CENTER") | bold | hcenter | color(Color::Cyan),
            separatorDouble(),
            
            // 顶部两块面板并排 (状态 + 遥测)
            hbox({ status_panel | size(WIDTH, EQUAL, 35), data_panel | flex }),
            
            // 中间控制面板并排
            hbox({ action_ui | size(WIDTH, EQUAL, 35), nav_ui | flex }),
            
            filler(), // 填充剩余空间，把日志推到最底下
            
            // 底部日志区
            hbox({
                text(" LOG ") | bold | color(Color::Black) | bgcolor(Color::White),
                text(" " + g_log_msg) | color(Color::White)
            }) | border
        });
    });

    // 刷新率控制
    std::thread refresh_thread([&screen] {
        while (ros::ok()) {
            screen.Post(Event::Custom);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    screen.Loop(renderer);

    // 退出清理
    g_enable_constant_publish = false;
    ros::shutdown();
    if (ros_thread.joinable()) ros_thread.join();
    if (pub_thread.joinable()) pub_thread.join();
    if (refresh_thread.joinable()) refresh_thread.join();

    return 0;
}


