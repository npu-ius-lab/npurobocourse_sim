# npurobocourse_sim_ros2

西北工业大学智能无人系统综合设计课程的 ROS 2 仿真环境，是 [npurobocourse_sim](https://github.com/npu-ius-lab/npurobocourse_sim)（ROS 1）的独立 ROS 2 版本。两个项目独立维护，本工作区不修改、不替代现有 ROS 1 仿真环境。

目标环境：Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic（`ros_gz`）。

## 工作区布局

```
src/
├── npu_sim_description/  # 世界、模型、传感器与相机预设（无节点）
├── npu_sim_bringup/      # Gazebo Harmonic 启动、ros_gz 桥接、适配器
├── npu_mobile_robot/     # 小车 SLAM、AMCL、Nav2 配置与启动
├── npu_tracker/          # 飞行任务、跟随与标记跟踪节点
├── npu_apriltag/         # 标记检测与检测结果归一化
└── npu_sim_tools/        # 模拟外部位姿、README 命令校验工具
```

## 依赖安装

需要已配置 ROS 2 Jazzy apt 源的 Ubuntu 24.04 系统，并安装 `python3-rosdep`、`python3-colcon-common-extensions`。

```bash
sudo rosdep init   # 首次使用时
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`external_pose_display_adapter` 还需要 Gazebo 的 Python 绑定（rosdep 暂无对应规则，需手动安装）：

```bash
sudo apt install python3-gz-transport13 python3-gz-msgs10
```

## 构建

```bash
colcon build --symlink-install
source install/setup.bash
```

之后每个新终端在执行 `ros2` 命令前都需要先加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 测试

```bash
colcon test
colcon test-result --verbose
```

`npu_sim_tools` 中的测试会校验本 README 引用的全部 `ros2 launch` / `ros2 run` 命令都能解析到存在的 package、launch 文件和 executable；也可手动运行：

```bash
ros2 run npu_sim_tools validate_readme_commands README.md src
```

## 清理

```bash
rm -rf build/ install/ log/
```

> 提示：若本机启用了 Conda/Mamba 环境，构建和运行前请先 `conda deactivate`，避免 CMake 或节点误用 Conda 的 Python。

## 教学工作流（验收矩阵）

每条工作流标注了必须在独立终端执行的命令（`终端 N`）及其前置 launch。所有 `ros2 launch` 仿真场景默认启动 Gazebo GUI；无桌面环境统一追加 `headless:=true`，见文末「GUI 与 headless 验收」。

| ROS 1 工作流 | ROS 2 对应命令 | 前置 |
|---|---|---|
| `roslaunch hector_quadrotor_demo sim.launch` | `ros2 launch npu_sim_bringup sim.launch.py` | 无 |
| `roslaunch rmtt_tracker rmtt_smach_to_pass.launch` | `ros2 launch npu_tracker pass_and_waypoints.launch.py` | 联合仿真 |
| `roslaunch tianbot_mini slam.launch` | `ros2 launch npu_mobile_robot slam.launch.py` | 联合仿真 |
| `roslaunch tianbot_mini amcl.launch` | `ros2 launch npu_mobile_robot amcl.launch.py` | 联合仿真 |
| `rosrun hector_quadrotor_demo turtle_tf2_listener.py` | `ros2 run npu_tracker follow_drone_tf` | 联合仿真 |
| `rosrun rmtt_tracker pub_goal.py` | `ros2 run npu_tracker publish_follow_goal` | 联合仿真 + AMCL |
| `roslaunch rmtt_apriltag detection.launch` | `ros2 launch npu_apriltag detection.launch.py` | 联合仿真 |
| `roslaunch rmtt_tracker rmtt_tag_tracker.launch` | `ros2 launch npu_tracker tag_tracker.launch.py` | 联合仿真 + 标记检测 |
| `roslaunch hector_quadrotor_demo sim_real.launch` | `ros2 launch npu_sim_bringup sim_real.launch.py` + `ros2 run npu_sim_tools publish_mock_pose` | 无 |
| 手工修改 Xacro 调整相机 | `ros2 launch npu_sim_bringup sim.launch.py camera_preset:=down` | 无 |

### 1. 联合仿真（无人机 + 小车）

终端 1：

```bash
ros2 launch npu_sim_bringup sim.launch.py
```

启动 Gazebo Harmonic、无人机模型和带标记的小车模型，并桥接 `/clock`、里程计、激光、相机等话题。启动后可用 `ros2 topic list` 观察到 `/npu/drone/pose`、`/npu/robot/odom`、`/npu/robot/scan` 等接口。

### 2. 无人机过环与航点

前置：联合仿真运行中。

终端 2：

```bash
ros2 launch npu_tracker pass_and_waypoints.launch.py
```

任务状态机依次完成起飞、穿越圆环、飞往航点；参数见 `src/npu_tracker/config/tasks.yaml`，可用 `tasks_file:=<路径>` 替换，无需修改源码。任务状态发布在 `/npu/mission/state`。

### 3. 小车 SLAM 建图

前置：联合仿真运行中。

终端 2：

```bash
ros2 launch npu_mobile_robot slam.launch.py
```

启动 `slam_toolbox` 在线建图；追加 `rviz:=true` 可打开 RViz 观察地图。终端 3 可用键盘遥控小车：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/npu/robot/cmd_vel
```

### 4. 小车 AMCL 定位导航

前置：联合仿真运行中。

终端 2：

```bash
ros2 launch npu_mobile_robot amcl.launch.py
```

加载项目默认地图（`src/npu_mobile_robot/maps/course_map.yaml`，可用 `map:=<路径>` 替换），启动 AMCL 与 Nav2 导航栈，追加 `rviz:=true` 可打开 RViz。之后可在 RViz 中下发导航目标。RViz 可查看 `/particlecloud` 粒子云、`/plan` 规划路径和 `/npu/robot/odom` 实际运动轨迹。

### 5. 小车跟随无人机（无避障）

前置：联合仿真运行中。

终端 2：

```bash
ros2 run npu_tracker follow_drone_tf
```

依据无人机位姿与小车 TF 直接发布 `/npu/robot/cmd_vel`；跟随距离、速度上限等由参数配置（如 `--ros-args -p follow_distance:=2.0`）。

### 6. 小车跟随无人机（避障）

前置：联合仿真运行中，且 AMCL 定位导航（工作流 4）已启动。

终端 3：

```bash
ros2 run npu_tracker publish_follow_goal
```

通过 Nav2 `NavigateToPose` action 周期下发跟随目标，由导航栈负责避障；不直接发布速度命令。RViz 中的 Path 显示规划路径，Odometry 历史轨迹用于核对实际运动过程。

### 7. 无人机跟踪小车标记

前置：联合仿真以垂直向下相机运行中。

终端 1（重启联合仿真并选用向下相机预设，若已在运行且预设正确可跳过）：

```bash
ros2 launch npu_sim_bringup sim.launch.py camera_preset:=down
```

终端 2（键盘遥控无人机，使相机对准小车上的标记）：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/npu/drone/cmd_vel
```

终端 3（标记检测）：

```bash
ros2 launch npu_apriltag detection.launch.py camera_preset:=down
```

终端 4（检测到标记后启动无人机跟踪）：

```bash
ros2 launch npu_tracker tag_tracker.launch.py
```

检测节点消费仿真相机图像，将选中的标记位姿发布到 `/npu/marker/pose`；跟踪节点依据它与无人机位姿计算速度命令，使无人机悬停于标记上方。检测缺失、目标不匹配或相机话题不可用时，`/diagnostics` 会给出明确状态并输出零速度命令。

### 8. 虚实结合显示

无需联合仿真。终端 1 启动显示场景（仅含一架静态显示无人机模型）：

```bash
ros2 launch npu_sim_bringup sim_real.launch.py
```

终端 2 发布模拟外部定位轨迹（替代真实定位毯/实体无人机）：

```bash
ros2 run npu_sim_tools publish_mock_pose
```

显示模型随即跟随 `/npu/external/drone_pose` 中的位姿移动。约定：话题 `/npu/external/drone_pose`，类型 `geometry_msgs/PoseStamped`，坐标系 `world`（ENU，米），姿态为右手四元数。`publish_mock_pose` 支持 `--ros-args -p mode:=fixed|line|loop`、`-p rate_hz:=`、`-p start_pose:="[x, y, z]"` 与 `-p seed:=`（确定性轨迹）。外部位姿超时或无效时停止更新显示模型，并在 `/diagnostics` 发布错误状态。接入真实定位源时，将同一话题替换为实际位姿即可，无需改动源码。

### 9. 相机朝向调整

无人机相机安装位姿由 `src/npu_sim_description/config/camera_presets/` 下的预设控制，无需修改模型源文件：

- `forward`：前视下倾 15°（课程默认，对应 ROS 1 的默认朝向）
- `down`：垂直向下（对应 ROS 1 中需要手工修改 Xacro 的效果）

启动联合仿真时通过 `camera_preset` 参数选择预设，例如：

```bash
ros2 launch npu_sim_bringup sim.launch.py camera_preset:=down
```

预设通过 `models/drone/model.sdf.template` 在模型生成时注入；`models/drone/model.sdf` 为 `forward` 预设的默认渲染结果，并由测试保证与模板同步。

## 接口参考

- 无人机位姿：`/npu/drone/pose`（`geometry_msgs/PoseStamped`，`world` 系）。ROS 1 教学材料中的 `/pose` 可用 `--ros-args -r /pose:=/npu/drone/pose` 重映射获得。
- 无人机速度控制：`/npu/drone/cmd_vel`（机体系线速度 + 偏航率，由 `drone_adapter` 限幅与超时保护）。
- 小车速度控制：`/npu/robot/cmd_vel`；小车 TF：`npu_robot/odom -> npu_robot/base_link`，AMCL 提供 `world -> npu_robot/odom`。
- 相机：`/npu/drone/camera/image_raw`、`/npu/drone/camera/camera_info`。
- 标记检测：`/npu/marker/pose`（`world` 系）。
- 时间约定：仿真中所有节点默认 `use_sim_time:=true`，时钟来自桥接的 `/clock`；位姿话题使用 `world` 系（ENU，米），姿态为右手四元数。
- 外部位姿：`/npu/external/drone_pose`（`world` 系，见工作流 8），为仅仿真用途的真实定位毯/实体无人机替代品。

## GUI 与 headless 验收

- GUI 验收（默认）：`ros2 launch npu_sim_bringup sim.launch.py` 或 `ros2 launch npu_sim_bringup sim_real.launch.py` 会打开 Gazebo 客户端，可目视确认世界渲染、无人机/小车模型与显示模型运动。
- GUI 冒烟测试：带桌面环境的完整人工验收按 [docs/gui_smoke_checklist.md](docs/gui_smoke_checklist.md) 执行，覆盖渲染、向下相机标记可见性与全部教学工作流。
- headless 验收（无桌面/CI）：对上述两条命令追加 `headless:=true`，仅启动 Gazebo 服务端；通过 `ros2 topic hz /npu/drone/camera/image_raw`、`ros2 topic echo /npu/drone/pose`、`ros2 topic echo /diagnostics` 等命令确认行为，SLAM/AMCL/跟随/跟踪工作流均以相同方式在无头模式下运行。

## License

见 [LICENSE.txt](LICENSE.txt)。
