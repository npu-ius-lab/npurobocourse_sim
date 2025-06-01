2025-05-29：支持仿真测试和虚实结合测试

2024-06-16：无人机位姿话题重映射为 /pose。

# 智能无人系统综合设计仿真环境

![sim.jpg](sim.jpg)

仿真环境建立在开源项目[hector_quadrotor](https://github.com/RAFALAMAO/hector-quadrotor-noetic)和[tianbot_mini](https://github.com/tianbot/tianbot_mini)基础上。

测试环境：Ubuntu 20.04/ROS Noetic。

## Usage

编译：

```
catkin_make_isolated
```

启动gazebo仿真环境，一个无人机，一个小车：

### 仿真环境

```
roslaunch hector_quadrotor_demo sim.launch
```

### 虚实结合

```
roslaunch hector_quadrotor_demo sim_real.launch
```
 
小车gmapping建图：

```
roslaunch tianbot_mini slam.launch
```

小车amcl定位导航：

```
roslaunch tianbot_mini amcl.launch
```

小车跟随无人机（无避障）：

```
rosrun hector_quadrotor_demo turtle_tf2_listener.py
```

小车跟随无人机（避障）：

```
roslaunch tianbot_mini amcl.launch
rosrun rmtt_tracker pub_goal.py
```

无人机跟踪小车二维码：

```
##启动无人机键盘遥控节点，使相机对准二维码
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
##打开新终端
roslaunch rmtt_apriltag detection.launch
##打开新终端
roslaunch rmtt_tracker rmtt_tag_tracker.launch
```

## Competition

如果无人机摄像头需要改成垂直向下，修改npu_simulator/hector-quadrotor-noetic/hector_quadrotor/hector_quadrotor_description/urdf/quadrotor_hokuyo_utm30lx.urdf.xacro第22行，将摄像头角度改成：

```
<origin xyz="0.05 -0.0 -0.25" rpy="0 ${M_PI*90/180} 0"/>
```

## Copyright

Please check [here](LICENSE.txt).
