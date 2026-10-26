# SPEC：ROS 2 教学项目精简修复清单

> 来源：`tasks/prd-ros2-review-fixes.md`
> 修订：2026-10-25 | 分支：`ros2` | 基线：`24e5075`

本清单替代此前冗长的实现与验收设计。保留审查发现的实际缺陷，以教学代码的精简、可读和可理解为优先，不追求覆盖所有理论异常情况。

## 1. 实施原则

- 只修复已确认的问题，不顺便重构或增加新功能。
- 保持现有 ROS 话题、坐标系、action 和启动命令兼容。
- 优先修改现有文件；只有职责无法合理安放时才新增节点。
- 不引入通用状态机、插件框架、额外 executor 或新第三方库。
- 中文注释只解释安全、时间和生命周期中的关键“为什么”，不复述代码。
- 每个问题保留一个直接回归验证，复用现有 fixture 和集成场景。

## 2. 修复清单

### 1. 小车命令安全

涉及：`npu_sim_bringup`、`follow_drone_tf.py`

- [ ] 新增一个小型 `robot_adapter`，订阅 `/npu/robot/cmd_vel`，原样转发到内部 `/npu/robot/cmd_vel_gz`。
- [ ] 复用 `CommandGate` 的接收时间、超时和一次性归零逻辑；不改变速度值，不增加诊断或限幅体系。
- [ ] 更新 bridge 和 launch；`follow_drone_tf` 退出前发布一次零速度。
- [ ] 用一个小型测试覆盖“转发、超时归零、新命令恢复”，并复用现有仿真测试检查公共命令链路。

### 2. Nav2 goal 生命周期与机器人 TF

涉及：`publish_follow_goal.py` 及现有 action 测试

- [ ] 在节点内使用单一阶段变量区分 `IDLE/SENDING/ACTIVE/CANCELING`，不新增状态机类。
- [ ] goal response 未返回时不得再次发送；拒绝或异常后释放在途状态并沿用现有退避。
- [ ] 取消被拒绝或失败时保留原 goal 句柄；输入仍陈旧时允许后续 tick 再次取消。
- [ ] 取消被接受后等待原 goal 结束，再发送替换目标，避免并发 goal。
- [ ] 增加 `tf_timeout_sec`，检查机器人 TF 的 `header.stamp`；查询失败或陈旧时清空 `_robot_xy` 并保护活动 goal。
- [ ] 复用 `MockNav2Server`，只增加“取消拒绝、慢 response、TF 单独停更”三个关键场景，不穷举回调组合。

### 3. 多航点超时

涉及：`mission_logic.py`

- [ ] 到达一个航点后，若存在下一航点，调用 `_enter(APPROACH_WAYPOINT, now)` 重置计时。
- [ ] 增加一个两航点回归测试；现有单航点和失败测试保持不变。

### 4. AprilTag 时间对齐

涉及：`detection_adapter.py`

- [ ] 无人机位姿回调直接发布 TF，并保留源 `header.stamp`，移除定时重盖时间戳的中继。
- [ ] 世界系标记变换使用与 `camera -> marker` 相同的检测时间。
- [ ] 增加源时间戳和查询时间的单元断言，运行现有渲染检测测试；不新增复杂运动真值场景。

### 5. Nav2 恢复行为与 RViz

涉及：`npu_mobile_robot`

- [ ] 给 `behavior_server` 增加与 controller 相同的 `cmd_vel` 重映射。
- [ ] 新增精简的 `amcl.rviz` 和 `slam.rviz`，只保留教学所需地图、激光、TF、初始位姿和 Nav2 Goal 等显示/工具。
- [ ] 两个配置使用 `world`；AMCL 激光使用 `/npu/robot/scan`，SLAM 显示 `/map`。
- [ ] 用静态配置断言检查上述字段和 launch 路径，运行现有 Nav2 集成测试；不新增 GUI 自动化或恢复行为运动矩阵。

### 6. Gazebo 执行确认

涉及：两个 bringup launch、`external_pose_display.py`

- [ ] 将 `set_pose` 改为 `/set_pose/blocking`；失败时不更新 `_last_sent`，保留现有重试和诊断。
- [ ] 保留现有 create 请求重试；create 返回成功后不再重复创建，只做有界 `gz model` 存在性检查，失败最终返回非零。
- [ ] 不创建新的 Gazebo 客户端或重试框架。
- [ ] 增加一个无效 SDF 检查和一个 `set_pose` 失败单元测试；正常行为由现有 launch 测试覆盖。

### 7. Mock 位姿仿真时间

涉及：`publish_mock_pose.py`

- [ ] 复用项目现有的 `use_sim_time` 默认覆盖模式，显式参数优先。
- [ ] `_start_time` 在第一次 tick 初始化，避免 `/clock` 到达前从零时刻积分。
- [ ] 用一个小型节点测试覆盖默认 sim time、显式 false 和首帧不跳变，不新增 launch 场景。

### 8. README 命令校验

涉及：`validate_readme_commands.py`

- [ ] 保持现有正则和命令范围不变。
- [ ] 外部 package 通过 ament 安装目录检查 launch 和 executable。
- [ ] 工作区 package 无可用 console scripts 时回退到已安装 executable；仍无法确认则报告问题，不静默通过。
- [ ] 增加外部缺 launch、外部缺 executable、空脚本回退三个小测试，并保留当前 README 有效命令测试。

### 9. 依赖与注释

- [ ] `npu_tracker` 补充 `action_msgs`、`launch`、`launch_ros`、`ament_index_python`。
- [ ] `npu_mobile_robot` 补充直接 launch 依赖、`rviz2` 和 `ament_cmake_pytest`。
- [ ] 通过 manifest 静态检查和正常构建验证，不要求独立容器或额外依赖矩阵。
- [ ] 仅在门控重新武装、goal 句柄所有权、航点计时、TF 时间对齐和 mock 首次计时处添加简短中文注释。

## 3. 最小验收

完成后运行：

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
source install/setup.bash
ros2 run npu_sim_tools validate_readme_commands README.md src
git diff --check
```

验收只要求：

- 上述关键回归场景通过，现有测试无回归。
- 无效 SDF 不再被当作创建成功，陈旧 TF 不再生成目标，取消失败不丢失活动 goal。
- 改动保持局部，无无关格式化、全项目注释或宽泛重构。
- GUI 仍按现有人工清单抽查，不作为新增自动化验收项。

## 4. 明确不做

- 不处理时间回拨、任意 NaN/非法参数和通用进程崩溃策略。
- 不扩展 README 命令语法，不增加 action 超时、重连或通用恢复框架。
- 不复制整份上游 RViz 配置，不新增 GUI 自动化。
- 不做穷举异步回调测试、独立容器测试或多轮长时间集成测试。
- 不提交、推送或创建 PR，除非用户另行要求。

## 5. 建议实施顺序

1. 小改动：多航点、Nav2/RViz、mock、README 校验、依赖。
2. 集中修改：小车门控、goal/TF、AprilTag、Gazebo。
3. 运行最小验收，并只针对实际 diff 做一次代码审查。
