# SPEC：通过 Gazebo 逐项检验 README 项目功能

> Technical specification derived from: `tasks/prd-gazebo-readme-acceptance.md`
> Generated: 2026-10-25 | Target branch: `ros2` | Commit: `24e5075` + 当前未提交修复

## 1. Summary

### 1.1 What This SPEC Covers

本 SPEC 定义如何对当前工作区执行 README 全部 9 条教学工作流的实际 Gazebo 验收，包括场景划分、临时探针、等效控制输入、GUI/图像证据、状态判定、日志结构和最终报告。

本任务只采集证据并报告结果，不修改 `src/` 功能实现，不新增永久验收脚本，也不要求所有功能必须通过才算完成检验。

### 1.2 PRD Reference

- Source: `tasks/prd-gazebo-readme-acceptance.md`
- User Stories covered: US-001 至 US-011
- Functional Requirements covered: FR-1 至 FR-15
- Baseline automated tests: 142 项通过，仅作为环境参考，不替代实际验收。

### 1.3 Design Decisions Summary

| Decision | Choice | Rationale |
|---|---|---|
| 场景划分 | 七组逻辑场景 | 保留工作流依赖，同时限制状态污染和重复启动 |
| 行为证据 | `/tmp/opencode` 中的一次性 ROS 探针 | 获得量化证据，不在项目中新增永久脚本 |
| 控制输入 | 有界 topic/action 等效输入 | 可复现、可记录、可确保停止，避免依赖键盘和鼠标自动化 |
| 场景执行顺序 | 每组先 headless，再按需重启 GUI | 同时满足客观行为验证与关键视觉检查 |
| GUI 证据 | Pillow XCB 截图、RViz/rqt、ROS 图像 | 当前已有 `DISPLAY=:0`、Pillow XCB 和 cv2，无需安装工具 |
| 日志 | 每组独立 headless/gui 目录及 JSONL 观测记录 | 便于追溯每条结论，避免日志串场 |
| 失败处理 | 记录、清理、继续独立项目 | 不修改实现，也不让单点失败掩盖其他结果 |
| 状态判定 | `PASS/FAIL/BLOCKED` 分开记录 headless 与 GUI | 防止把视觉缺证据或环境阻塞误报为通过 |

---

## 2. Architecture

### 2.1 System Context

```text
README / GUI checklist
          |
          v
  验收执行控制器（人工/agent）
          |
          +---- 基线与进程控制
          +---- 七组 Gazebo 场景
          |       +---- headless：ROS/Gazebo 客观证据
          |       +---- GUI：截图与人工视觉检查
          |
          +---- /tmp/opencode 一次性探针
          |       +---- topic / TF / action / diagnostics
          |       +---- 位姿、距离、状态序列、误差
          |
          +---- Pillow / rqt / ROS Image
          |
          v
  tasks/gazebo-readme-acceptance-logs/
          |
          v
  tasks/gazebo-readme-acceptance-report.md
```

`src/` 为只读对象。临时探针仅用于本次验收，不进入项目源码或安装空间。

### 2.2 Component Design

#### 验收执行控制器

由执行者按 SPEC 逐条运行命令，不负责修改产品功能。职责包括：

- 保存基线和环境信息。
- 为每组设置唯一 `ROS_DOMAIN_ID` 与 `GZ_PARTITION`。
- 启动、等待、终止 launch 和辅助节点。
- 调用探针及图像采集工具。
- 每组结束后保存状态并清理进程。
- 将实际命令和结果写入报告。

#### 临时行为探针

位于 `/tmp/opencode/npu_gazebo_acceptance/`：

| Probe | Scenario | Responsibility |
|---|---|---|
| `probe_common.py` | 共用 | 仅提供 JSONL 写入、ROS 时间转换和有界等待等少量辅助 |
| `joint_mission_probe.py` | S01 | 实体、接口、TF、任务状态序列和关键位姿 |
| `slam_probe.py` | S02 | 地图非空、地图变化、位移和生命周期状态 |
| `nav_follow_probe.py` | S03 | AMCL、NavigateToPose、跟随目标、取消与恢复 |
| `direct_follow_probe.py` | S04 | 相对距离、速度方向、收敛和退出归零 |
| `tag_probe.py` | S05 | 检测、时间戳、悬停误差、跟随和标记丢失 |
| `display_probe.py` | S06 | fixed/line/loop 轨迹和输入停止行为 |
| `camera_probe.py` | S07 | 两种预设的图像、CameraInfo 和时间戳 |

探针只依赖现有 `rclpy`、ROS 消息、NumPy、cv2 和 Pillow，不引入新库。

#### 图像证据采集

- Pillow `ImageGrab.grab(xdisplay=':0')`：截取 Gazebo、RViz 和 rqt 窗口或桌面。
- `rqt_image_view`：展示 `/npu/drone/camera/image_raw`。
- ROS Image + cv2：保存原始相机代表帧，避免仅依赖窗口截图。
- 图像文件使用 PNG；运动过程只保存关键帧，不录制长视频。

### 2.3 Module Interactions

每组按以下顺序执行：

1. 清理上一组进程并确认端口/partition 不冲突。
2. 记录本组 README 命令和环境变量。
3. 启动 headless 场景和临时探针。
4. 保存客观证据并判定 headless 子结果。
5. 若该组包含 GUI 检查，清理后以 GUI 模式重启同组。
6. 启动 RViz/rqt，保存截图和代表图像，完成视觉检查。
7. 汇总本组 `PASS/FAIL/BLOCKED`，保存日志索引。
8. 清理本组全部进程，再进入下一组。

### 2.4 File Structure

```text
/tmp/opencode/npu_gazebo_acceptance/
├── probe_common.py                 [TEMP]
├── joint_mission_probe.py          [TEMP]
├── slam_probe.py                   [TEMP]
├── nav_follow_probe.py             [TEMP]
├── direct_follow_probe.py          [TEMP]
├── tag_probe.py                    [TEMP]
├── display_probe.py                [TEMP]
└── camera_probe.py                 [TEMP]

tasks/
├── gazebo-readme-acceptance-report.md
└── gazebo-readme-acceptance-logs/
    ├── baseline/
    │   ├── environment.txt
    │   ├── git-status.txt
    │   └── colcon-build.log
    ├── s01_joint_mission/
    │   ├── headless/
    │   └── gui/
    ├── s02_slam/
    │   ├── headless/
    │   └── gui/
    ├── s03_amcl_nav_follow/
    │   ├── headless/
    │   └── gui/
    ├── s04_direct_follow/
    │   └── headless/
    ├── s05_tag_down/
    │   ├── headless/
    │   └── gui/
    ├── s06_display/
    │   ├── headless/
    │   └── gui/
    └── s07_camera_forward/
        ├── headless/
        └── gui/

src/                                [READ ONLY]
```

---

## 3. Data Model

### 3.1 Schema Changes

无数据库、ROS message 或项目配置变更。

### 3.2 Evidence Records

客观观测写入 JSONL，每行一条记录：

| Field | Type | Meaning |
|---|---|---|
| `scenario` | string | `s01` 至 `s07` |
| `mode` | string | `headless` 或 `gui` |
| `wall_time` | string | ISO 8601 采集时间 |
| `ros_time` | number/null | 当前 ROS 时间，秒 |
| `kind` | string | `pose/state/tf/action/diagnostic/metric/result` |
| `name` | string | 对应话题、实体或指标名称 |
| `value` | object | 实际观测值 |

每条工作流的结果记录包含：

| Field | Required content |
|---|---|
| `workflow` | README 编号与名称 |
| `commands` | 实际执行命令及等效输入 |
| `headless_status` | `PASS/FAIL/BLOCKED` |
| `gui_status` | `PASS/FAIL/BLOCKED/NOT_REQUIRED` |
| `overall_status` | `PASS/FAIL/BLOCKED` |
| `observations` | 状态、位置、距离、误差等关键数值 |
| `evidence` | 相对日志、JSONL、截图和图像路径 |
| `failure` | 失败现象、复现步骤和日志摘录；通过时为空 |

### 3.3 Relationships

- S03 的 Nav2 跟随依赖同组 AMCL 与导航前置通过。
- S04 的直接跟随不与 AMCL 共跑，避免 `world -> odom` 所有权冲突。
- S05 同时覆盖工作流 7 和 down 相机视觉证据。
- S07 覆盖 forward 预设；down 预设的接口证据也在此复核，避免仅凭 S05 的图像推断全部接口。
- 每个结果记录必须至少链接一条客观证据；GUI 结论必须链接截图或明确的人工观察记录。

### 3.4 Migration Plan

无需迁移。报告和日志为新增交付物，不修改已有 README、PRD/SPEC、测试或产品文件。

---

## 4. API Design

### 4.1 Interfaces

无 HTTP API。使用现有 ROS/Gazebo 接口：

| Interface | Input | Evidence output |
|---|---|---|
| `ros2 launch ... headless:=true` | 各组 README launch | launch 日志、实体与 ROS 接口 |
| `ros2 launch ... headless:=false` | GUI 复核 | Gazebo 窗口和截图 |
| `/npu/drone/cmd_vel` | 有界 `Twist` | 无人机位姿与任务/跟踪状态 |
| `/npu/robot/cmd_vel` | 有界 `Twist` | 小车里程计、地图和跟随行为 |
| `/navigate_to_pose` | 空闲区域 `PoseStamped` | goal 状态、最终位置和轨迹 |
| `/npu/robot/goal_pose` | 跟随节点生成目标 | 目标序列和对应 action 结果 |
| `/npu/marker/pose` | AprilTag 适配器输出 | 世界系检测位姿与时间戳 |
| `/diagnostics` | 各节点状态 | 错误、恢复和阻塞证据 |
| `/world/course_world/pose/info` | Gazebo 实体位姿 | 实体存在性和显示轨迹 |
| `/npu/drone/camera/image_raw` | Gazebo 相机图像 | PNG 代表帧和时间戳 |

### 4.2 Equivalent Input Contracts

- 键盘平移：使用短时、有限速度 `Twist`，结束后显式发送零速度。
- RViz Nav2 Goal：使用 `NavigateToPose` action 发送同一 `world` 系目标，并在报告中注明为等效操作。
- 标记丢失：通过移动机器人或无人机使标记离开视场；该方式记录为“可见性丢失等效输入”，不伪称物理盖板遮挡。
- 小车跟随恢复：停止并恢复相应 pose/TF 发布，记录停止和恢复时间。
- Mock 输入停止：终止 `publish_mock_pose`，不使用其他节点继续填充该话题。

### 4.3 Result Status Rules

| Status | Rule |
|---|---|
| `PASS` | 所有必要 headless 条件通过，且必要 GUI 条件通过或明确为 `NOT_REQUIRED` |
| `FAIL` | 任一产品行为不符合 PRD，包括超时、错误状态、错误坐标、无恢复或运动不符合预期 |
| `BLOCKED` | 因显示、依赖服务、采集工具或前置工作流不可用而无法获得必要证据 |
| `NOT_REQUIRED` | 仅用于没有额外 GUI 判定要求的子项，不作为整体工作流结果 |

客观行为失败不得改写为 `BLOCKED`。GUI 被阻塞时，整体不得因 headless 通过而标记 `PASS`。

### 4.4 Breaking Changes

无产品接口或行为变更。临时探针和报告不改变现有启动命令、话题、action 或配置。

---

## 5. Business Logic

### 5.1 Baseline and Preflight

1. 保存 `git branch --show-current`、HEAD、`git status --short` 和 diff 统计。
2. 保存 OS、ROS、Gazebo、Python、DISPLAY 和关键包版本。
3. 执行并保存 `colcon build --symlink-install` 日志。
4. 加载 `/opt/ros/jazzy/setup.bash` 和 `install/setup.bash`。
5. 检查 `rqt_image_view`、teleop、Pillow XCB、cv2 和必要 Gazebo Python bindings。
6. 对临时探针执行 Python 语法检查。
7. 确认初始没有属于本次验收的残留进程。

### 5.2 Seven Scenario Groups

| Scene | README workflows | Launch composition | Main evidence |
|---|---|---|---|
| S01 | 1、2 | 联合仿真 + `pass_and_waypoints` | 实体、接口、状态序列、起飞/过环位姿、GUI |
| S02 | 3 | 联合仿真 + SLAM | active、非空地图、位移前后地图变化、RViz |
| S03 | 4、6 | 联合仿真 + AMCL/Nav2 + `publish_follow_goal` | 定位、导航成功、跟随目标、取消恢复、RViz |
| S04 | 5 | 联合仿真 + `follow_drone_tf` | 跟随距离、速度方向、超时和退出归零 |
| S05 | 7、9 down | down 联合仿真 + detection + tag tracker | 标记检测、悬停、跟随、可见性丢失、图像 |
| S06 | 8 | `sim_real` + 三种 mock 模式 | 显示实体轨迹、时间源、停止与诊断、GUI |
| S07 | 9 forward | forward 联合仿真 | 图像、CameraInfo、安装参数、前视画面 |

### 5.3 Measurement Rules

#### S01：联合仿真与任务

- `/clock` 至少两个连续样本且时间递增。
- 实体查询必须同时出现 drone、robot、ring。
- 主要话题记录 frame、stamp 和至少一个有效数据样本。
- 状态序列按首次变化去重，要求顺序到达规定状态并最终 `COMPLETE`。
- 位姿证据包含起飞高度、圆环入口/出口附近位置和最终航点误差。
- GUI 截图至少包含世界整体和过环关键时刻。

#### S02：SLAM

- 生命周期状态必须为 active。
- 地图宽度、高度和非 unknown 单元数必须大于零，frame 为 `world`。
- 在有界速度输入前后分别记录地图哈希、已探索单元数和小车位移。
- 位移必须超过 0.2m，且地图哈希或已探索单元数发生变化。
- RViz 截图必须能看到地图、激光和 TF，固定系为 `world`。

#### S03：AMCL 与 Nav2 跟随

- 所有相关生命周期节点为 active。
- TF 链完整，AMCL 初始位置与出生点误差不超过 0.6m。
- x/y 协方差按现有集成测试阈值收敛到小于 0.25。
- 导航目标选择默认地图中的空闲坐标，记录 goal、status 和最终误差。
- 跟随阶段记录目标序列、机器人轨迹和 action 终态。
- 通过已知障碍边界和轨迹检查碰撞迹象；不得仅以 action 成功推断全程无碰撞。
- pose/TF 停止期间必须出现 goal 保护证据，恢复后必须出现新目标或恢复跟随。

#### S04：直接跟随

- 使用无障碍区域，计算无人机与机器人同一时刻的世界系水平距离。
- 移动阶段同时保存相对方向和机器人 `cmd_vel`，验证控制方向一致。
- 悬停后取连续 5 秒距离样本，稳定误差必须不超过 0.3m。
- 停止无人机位姿后检查命令超时归零。
- 正常终止跟随节点后检查最终零命令及 Gazebo 位姿不再持续变化。

#### S05：标记跟踪与 down 相机

- 同时验证检测数组包含 ID 8 和 `/npu/marker/pose` 有对应输出。
- 比较检测、TF 和输出 pose 的时间戳，确认对应同一检测时刻。
- 悬停误差取连续 5 秒样本，水平和相对高度误差均不超过 0.3m。
- 小车移动后记录无人机同方向位移。
- 标记离开视场后检查输出停止、零速度和诊断错误；恢复后检查重新检测和跟踪。
- 保存 rqt 窗口截图及原始 down 图像。

#### S06：虚实显示

- 确认不存在 `npu_drone`、`npu_robot`，目标显示实体为 `npu_drone_display`。
- fixed：连续样本位置不变。
- line：位移方向与轨迹 yaw 一致，累计位移大于 0.2m。
- loop：拟合或采样半径接近配置 `radius=1.0m`，允许 0.1m 误差。
- 核对 mock 与 display 使用的时钟域及消息时间戳。
- 停止 mock 后等待超过 1 秒，确认实体位置停止变化并出现 ERROR 诊断。
- GUI 截图记录显示实体及一种代表性运动状态。

#### S07：forward 相机

- 图像尺寸与 CameraInfo 一致，时间戳持续推进。
- 读取实际 forward 预设安装参数，确认配置为下倾 15°。
- 保存前视代表图像并与 down 图像并列说明，不仅凭配置文件推断视觉效果。

### 5.4 Edge Cases

- 前置联合仿真失败：先记录该组失败；后续组使用全新世界尝试，只有确实无法独立运行的项目才标为依赖阻塞。
- 单次基础设施启动失败：允许在新 partition/domain 下重试一次，保留两次日志。
- 产品行为不符或状态超时：直接记为 `FAIL`，不通过重复运行挑选成功样本。
- GUI 窗口存在但截图为空或全黑：标记视觉证据无效，相关 GUI 子项为 `BLOCKED`。
- action 返回成功但最终位置超差：整体为 `FAIL`，不能只看状态码。
- 探针自身异常：保存异常日志，若产品数据仍可由普通 ROS 命令获得则补充采集；否则标记采集阻塞。
- 场景异常退出：先保存日志和核心转储位置（若有），再按进程组清理。

---

## 6. Error Handling

### 6.1 Error Taxonomy

| Category | Detection | Status | Required evidence |
|---|---|---|---|
| 基础设施不可用 | 缺少依赖、显示或服务发现失败 | `BLOCKED` | 命令、错误输出、环境信息 |
| 产品启动失败 | launch 非零、实体缺失、核心节点退出 | `FAIL` | launch 日志、进程状态、实体查询 |
| 行为超时 | 预期状态、消息或运动在有界时间内未出现 | `FAIL` | 最后状态、等待时长、相关话题样本 |
| 行为错误 | 坐标、距离、状态或轨迹不满足要求 | `FAIL` | 数值、真值来源、日志 |
| GUI 证据不可用 | 无 DISPLAY、黑屏、截图失败 | `BLOCKED` | 环境信息和截图错误 |
| 采集失败 | 探针异常或证据文件缺失 | `BLOCKED` 或补采后判定 | 探针异常和补充证据 |
| 前置依赖失败 | 必需前置不能在新场景恢复 | `BLOCKED` | 前置失败和本组启动证据 |

### 6.2 Retry Strategy

- 仅基础设施发现或启动失败允许换用新 partition/domain 重试一次。
- 不对功能错误、导航失败、任务失败或跟踪误差进行选择性重试。
- 同一组若重试，所有结果均保留，报告说明最终采用哪次及原因。
- 不因 `colcon test` 曾通过而跳过实际失败项。

### 6.3 Failure Modes

- 单项失败不影响其他独立场景。
- S03 的 AMCL 失败会使 Nav2 跟随子项阻塞，但 AMCL 自身仍记录具体失败。
- S05 的检测失败会使跟踪子项阻塞，但相机图像仍可独立验收。
- 所有失败和阻塞场景均须执行清理，不能保留仿真供后续组隐式复用。

---

## 7. Security

### 7.1 Authentication & Authorization

无新增认证、权限或网络服务。所有 ROS/Gazebo 接口使用本地仿真环境，不使用 sudo 或真实硬件控制接口。

### 7.2 Input Validation

- 速度输入有界，并在每个输入段后显式归零。
- action 目标必须位于默认地图空闲区域。
- 每个场景使用独立 domain/partition，防止误控制其他仿真。
- 只终止本次记录的 PID/进程组，不使用宽泛的 `pkill` 清理其他用户进程。
- 临时脚本不修改项目文件或系统配置。

### 7.3 Data Protection

- 环境记录使用白名单，不 dump 全部环境变量。
- 日志不得包含 SSH、token、凭据或私有路径以外的敏感信息。
- 报告引用相对路径，原始日志仅保存在指定验收目录。
- 不上传图像或日志到外部服务。

---

## 8. Performance

### 8.1 Expected Load

- 七组 headless 场景，GUI 仅在有视觉要求时重启。
- 常规启动/服务等待预算为 90–120 秒。
- 任务完成等待预算为 180 秒，Nav2 action 为 180 秒，跟随/悬停收敛为 60 秒。
- 上述时间仅是验收等待上限，不作为产品性能指标。
- 不进行长时间 soak 或重复压力运行。

### 8.2 Optimization Strategy

- 复用同组必要前置，避免每个子项独立重启。
- 位姿、距离和诊断按 1–10Hz 采样，避免无意义的全频日志。
- 图像仅保存关键帧，JSONL 只保存验收所需字段。
- 失败时优先保存最小可复现证据，避免复制完整 build/install 目录。

### 8.3 Database Considerations

无数据库。证据采用文本、JSONL、PNG 和 Markdown 文件。

---

## 9. Testing Strategy

### 9.1 Preflight Checks

- 临时探针 Python 语法检查通过。
- 当前工作区构建成功，安装环境可加载。
- ROS/Gazebo 版本、GUI 和图像能力有记录。
- 联合仿真首组验证后，才继续依赖其接口的探针。

### 9.2 Headless Acceptance

- 每个工作流执行真实 README launch/run，而非仅运行已有 pytest。
- 探针从实际 ROS/Gazebo 接口采集证据。
- 所有阈值和超时在采集前固定，不根据结果临时放宽。
- 结果输出与原始观测分开保存，避免手工修改数值。

### 9.3 GUI and Image Acceptance

- 截图必须来自本次对应场景的 partition/domain。
- 图像检查记录窗口内容、固定坐标系、相机视角和可见模型。
- RViz 的 AMCL/SLAM 画面与对应 headless 场景分别记录，不混用历史截图。
- 使用 Read 图像能力逐图检查，无法读取或黑屏时不得判定视觉通过。

### 9.4 Acceptance Criteria Mapping

| US/FR | Verification | Type | Evidence |
|---|---|---|---|
| US-001 / FR-1,2,10 | 基线、环境、构建、目录和隔离检查 | preflight | `baseline/` |
| US-002 / FR-3,4,5–8 | 联合仿真实体、接口、TF 和 GUI | headless + GUI | S01 日志、JSONL、截图 |
| US-003 / FR-3,4,5–8 | 任务状态序列、位姿和 COMPLETE | headless + GUI | S01 状态与位姿、过环截图 |
| US-004 / FR-3,4,5–8 | SLAM active、地图变化和 RViz | headless + GUI | S02 地图对比、位移、截图 |
| US-005 / FR-3,4,5–8 | AMCL、TF、导航 action 和最终位置 | headless + GUI | S03 action、协方差、RViz |
| US-006 / FR-3,4,6–8 | 直接跟随距离、方向、超时和退出 | headless | S04 距离与命令序列 |
| US-007 / FR-3,4,6–9 | Nav2 跟随、路径、取消和恢复 | headless | S03 目标、轨迹、恢复记录 |
| US-008 / FR-3,4,5–9 | 检测、悬停、跟随、可见性丢失和恢复 | headless + GUI | S05 误差、诊断、图像 |
| US-009 / FR-3,4,5–9 | 三种 mock 模式、时间源和停止 | headless + GUI | S06 轨迹、诊断、截图 |
| US-010 / FR-3,4,5–8 | forward/down 图像和安装效果 | headless + GUI | S05/S07 图像和参数 |
| US-011 / FR-9,12–15 | 汇总、状态审计、日志索引和进程清理 | report | 最终报告、清理记录 |

---

## 10. Implementation Plan

### 10.1 Phases

1. **Preflight**：基线、构建、临时探针和证据目录。
2. **S01**：联合仿真与任务，验证探针和图像链路。
3. **S02–S03**：SLAM、AMCL 和 Nav2 跟随。
4. **S04–S05**：直接跟随、标记跟踪和 down 相机。
5. **S06–S07**：虚实显示和 forward 相机。
6. **汇总**：核对 9/9 覆盖、生成报告、检查日志链接和清理进程。

每组完成后立即写入该组结果，避免最后凭记忆补报告。

### 10.2 Issue Mapping

| Issue | SPEC Sections | Priority | Depends On |
|---|---|---|---|
| #1 Baseline/preflight | 2.4, 5.1 | high | — |
| #2 Joint + mission | 5.2 S01, 5.3 | high | #1 |
| #3 SLAM | 5.2 S02, 5.3 | high | #1 |
| #4 AMCL + Nav2 follow | 5.2 S03, 5.3 | high | #1 |
| #5 Direct follow | 5.2 S04, 5.3 | high | #1 |
| #6 Tag + down camera | 5.2 S05, 5.3 | high | #1 |
| #7 Virtual-real display | 5.2 S06, 5.3 | medium | #1 |
| #8 Forward camera | 5.2 S07, 5.3 | medium | #1 |
| #9 Report and cleanup | 3.2, 4.3, 9.4 | high | #2–#8 |

### 10.3 Incremental Delivery

- 每组结束即保存独立结果和证据，不等待全部完成后统一整理。
- 失败项只生成失败记录，不进入修复或复测循环。
- 最终只新增报告和日志；临时探针保留在 `/tmp/opencode`，不纳入项目交付。
- 不提交、推送或创建 PR。

---

## 11. Open Questions & Risks

### 11.1 Unresolved Questions

无。已确认七组场景、临时探针、可复现等效输入及 Pillow/ROS 图像方案。

### 11.2 Technical Risks

| Risk | Impact | Mitigation |
|---|---|---|
| GUI 窗口无法截取或黑屏 | 视觉项无法判定 | 先验证 Pillow XCB；失败则标记 `BLOCKED`，不伪造截图 |
| 动态仿真导致采样不同步 | 距离和误差出现偏差 | 使用同一时间戳/相邻样本并记录采样时间 |
| 多场景服务或话题串场 | 误读其他世界数据 | 独立 domain/partition，启动前检查并清理 |
| action 成功但位置不准确 | 状态码掩盖功能错误 | 同时验证最终位姿和轨迹 |
| 物理碰撞难以完全证明 | 碰撞结论过强 | 结合障碍边界、轨迹和可见现象，限定报告措辞 |
| 可见性丢失不等同物理遮挡 | 验收含义被夸大 | 报告明确标为等效输入，不声称执行了物理盖板操作 |
| 临时探针自身错误 | 证据缺失或误判 | 预检语法，保留原始 ROS 命令补采路径和异常日志 |
| 报告凭记忆补写 | 结论不可追溯 | 每组立即写入结果并核对证据链接 |

### 11.3 Assumptions

- 当前 `DISPLAY=:0` 和 Pillow XCB 在实际窗口创建后仍可用。
- `/tmp/opencode` 可写，且不需要安装额外软件。
- 当前未提交工作区在验收期间保持不变。
- 高位 ROS domain 和唯一 Gazebo partition 可由验收控制器分配。
- README 允许使用有记录的有界 topic/action 输入作为键盘和 RViz 操作的等效方式。
- GUI 视觉结论由实际截图检查得出，不能仅由节点日志推断。
