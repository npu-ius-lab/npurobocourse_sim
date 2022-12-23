####启动gazebo仿真环境，一个无人机，一个小车#####
1.cd your_ws/src
2.source devel/setup.bash
3.roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch

####小车建图####只提供gmapping
1.bash
2.source devel/setup.bash
3.roslaunch tibot_slam gmapping.launch

####小车导航####DWA
1.bash
2.soure devel/setup.bash
3.roslaunch tianbot_nav navigation_demo.launch


####小车跟随无人机（无避障）#######
1.bash
2.soure devel/setup.bash
3.rosrun hector_quadrotor_demo turtle_tf2_listener.py

####小车跟随无人机（避障）#########
1.bash
2.soure devel/setup.bash
3.rosrun hector_quadrotor_demo mini_track_drone.py

#####无人机跟踪小车二维码#######
1.bash
2.soure devel/setup.bash
3.rosrun teleop_twist_keyboard teleop_twist_keyboard.py ##启动无人机键盘遥控节点，使相机对准二维码
4.roslaunch rmtt_apriltag detection.launch   #开新终端
5.roslaunch rmtt_tracker rmtt_tag_tracker.launch   #开新终端

