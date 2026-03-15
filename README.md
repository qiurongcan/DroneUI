# DroneUI
基于Px4+FTXUI开发的无人机平台
>平台功能简述

可以实时监控PX4无人机的状态数据，设置无人机的定位源，设置无人机自动起飞，OFFBOARD/HOLD功能切换，自动降落,发布无人机位置，发布MOVE_BASE坐标功能

## LINUX环境下安装
```shell
git clone -b v6.1.5 https://github.com/ArthurSonzogni/FTXUI.git
cd FTXUI
mkdir build
cd build && cmake ..
make
sudo make install
```

## 运行
```shell
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/qiurongcan/DroneUI.git
cd ..
catkin_make

. devel/setup.bash
# 运行节点
# rosrun px4_monitor px4_monitor_node
# 建议采用下面这个方法
roslaunch px4_monitor px4_monitor.launch
```

## 无人机状态功能切换界面
![TAB1 界面](./px4_monitor/figs/Tab1.png)

## 无人机定位源切换界面
![TAB2 界面](/px4_monitor/figs/Tab2.png)

## MOVE BASE 发布界面
![TAB3 界面](./px4_monitor/figs/Tab3.png)



<!-- ## FTXUI安装及使用
参考链接：https://www.yuque.com/qqqrc/pvt0eg/dmzik2igke94t8qb?singleDoc# 《FTXUI的安装》 -->
