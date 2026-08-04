# GUI 冒烟测试清单

在带桌面环境的 Ubuntu 24.04 上执行，用于人工目视验收。每项完成后勾选；任一项失败则记录现象并回到对应工作流排查。执行前：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

多终端工作流按 README「教学工作流」的终端顺序启动；每条 launch 命令默认打开 Gazebo GUI。

## 1. 渲染基础（sim.launch.py）

- [ ] `ros2 launch npu_sim_bringup sim.launch.py` 打开 Gazebo 客户端，无致命报错。
- [ ] 世界地面、圆环模型渲染正常，无缺失纹理或黑屏。
- [ ] 小车模型可见，顶部搭载 AprilTag 标记。
- [ ] 无人机模型可见，螺旋桨与机载相机外壳渲染正常。
- [ ] GUI 中仿真时间（右下角）持续走动；`ros2 topic hz /clock` 有输出。

## 2. 相机与标记可见性

- [ ] 默认 `forward` 预设：`ros2 run rqt_image_view rqt_image_view /npu/drone/camera/image_raw` 显示前视下倾画面。
- [ ] 重启为 `camera_preset:=down`：图像为垂直向下视角。
- [ ] 遥控无人机（`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/npu/drone/cmd_vel`）飞到小车上方，向下相机画面中清晰可见小车顶部标记。
- [ ] `ros2 launch npu_apriltag detection.launch.py camera_preset:=down` 后 `ros2 topic echo /npu/marker/pose` 输出位姿；标记离开视野后停止输出并在 `/diagnostics` 报错。

## 3. 工作流 2：无人机过环与航点

- [ ] `ros2 launch npu_tracker pass_and_waypoints.launch.py` 后无人机起飞。
- [ ] GUI 中目视无人机穿越圆环，再飞往航点。
- [ ] `ros2 topic echo /npu/mission/state --once` 依次经过 TAKEOFF → APPROACH_RING → TRAVERSE_RING → APPROACH_WAYPOINT → COMPLETE。

## 4. 工作流 3：小车 SLAM 建图

- [ ] `ros2 launch npu_mobile_robot slam.launch.py rviz:=true` 打开 RViz。
- [ ] 键盘遥控小车（README 工作流 3 终端 3 命令）移动，RViz 中地图逐步扩展，无 TF 报错。

## 5. 工作流 4：AMCL 定位导航

- [ ] `ros2 launch npu_mobile_robot amcl.launch.py rviz:=true` 后 RViz 显示课程地图与粒子云。
- [ ] 在 RViz 中下发「Nav2 Goal」，小车规划路径并到达目标。

## 6. 工作流 5：小车跟随无人机（无避障）

- [ ] `ros2 run npu_tracker follow_drone_tf` 启动。
- [ ] 键盘遥控无人机平移，GUI 中小车跟随无人机保持跟随距离。

## 7. 工作流 6：小车跟随无人机（避障）

- [ ] 在工作流 4 基础上运行 `ros2 run npu_tracker publish_follow_goal`。
- [ ] 遥控无人机移动，小车经 Nav2 规划绕行圆环等障碍物跟随。

## 8. 工作流 7：无人机跟踪小车标记

- [ ] 按 README 工作流 7 启动 `camera_preset:=down` 联合仿真、标记检测与 `ros2 launch npu_tracker tag_tracker.launch.py`。
- [ ] 遥控小车移动（或重启仿真后手动拖动小车模型），无人机自动悬停于标记上方并跟随。
- [ ] 遮挡标记后 `/npu/drone/cmd_vel` 输出归零，`/diagnostics` 报 `tag_tracker/marker_pose` 错误。

## 9. 工作流 8：虚实结合显示

- [ ] `ros2 launch npu_sim_bringup sim_real.launch.py` 打开 GUI，场景仅含静态显示无人机模型。
- [ ] `ros2 run npu_sim_tools publish_mock_pose` 后显示模型开始沿模拟轨迹运动。
- [ ] 依次以 `-p mode:=fixed`、`-p mode:=line`、`-p mode:=loop` 重启 mock 节点，显示模型分别静止、直线往返、圆周运动。
- [ ] 停止 mock 节点约 1 秒后显示模型停止移动，`/diagnostics` 报 `external_pose_display/display` 错误。

## 10. 工作流 9：相机朝向调整

- [ ] `ros2 launch npu_sim_bringup sim.launch.py camera_preset:=forward` 与 `camera_preset:=down` 两种预设的图像视角与 README 描述一致。

## 11. 收尾

- [ ] 所有终端 Ctrl+C 后进程正常退出，无残留 `gz sim` 进程（`pgrep -af "gz sim"` 为空）。
- [ ] `colcon test` 全量通过，`colcon test-result --verbose` 无失败项。
