# 智能无人系统综合设计仿真环境

![sim.jpg](sim.jpg)

仿真环境建立在开源项目[hector_quadrotor](https://github.com/RAFALAMAO/hector-quadrotor-noetic)和[tianbot_mini](https://github.com/tianbot/tianbot_mini)基础上。

测试环境：Ubuntu 20.04/ROS Noetic

启动gazebo仿真环境，一个无人机，一个小车：

```
cd your_ws/src
source devel/setup.bash
roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
```

小车gmapping建图：

```
bash
source devel/setup.bash
roslaunch tianbot_slam gmapping.launch
```

小车导航，DWA：

```
bash
soure devel/setup.bash
roslaunch tianbot_nav navigation_demo.launch
```

小车跟随无人机（无避障）：

```
bash
soure devel/setup.bash
rosrun hector_quadrotor_demo turtle_tf2_listener.py
```

小车跟随无人机（避障）：

```
bash
soure devel/setup.bash
rosrun hector_quadrotor_demo mini_track_drone.py
```

无人机跟踪小车二维码：

```
bash
soure devel/setup.bash
##启动无人机键盘遥控节点，使相机对准二维码
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
##打开新终端
roslaunch rmtt_apriltag detection.launch
##打开新终端
roslaunch rmtt_tracker rmtt_tag_tracker.launch
```
