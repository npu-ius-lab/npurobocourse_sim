# SPEC: 前后对称双万向轮稳定性改进与 AMCL 粒子可视化

> Technical specification derived from: `tasks/prd-dual-caster-amcl-particles.md`
> Generated: 2026-11-07 | Aligned with implementation: 2026-11-09
> Target branch: `ros2` | Base commit: `a27e9b3`

## 1. Summary

### 1.1 What This SPEC Covers

本 SPEC 规定两项独立改动的实现方式：

1. 保留机器人尾部中心的原球形万向轮，并在车头中心新增相同万向轮，形成前后对称支撑，不调整其他底盘或驱动轮参数。
2. 修复 `amcl.rviz` 中粒子云显示器与 AMCL 实际消息类型不匹配的问题，使用 Nav2 原生 `ParticleCloud` 显示器，并验证粒子点可见且会更新。

不新增 ROS 节点、消息桥接、机器人模板系统或 AMCL 参数调整。

### 1.2 PRD Reference

- Source: `tasks/prd-dual-caster-amcl-particles.md`
- User Stories covered: US-001、US-002
- Functional Requirements covered: FR-1 至 FR-12
- 已确认的技术补充：
  - 机器人模型直接编辑 `model.sdf`，不新增机器人模板。
  - 原后轮保持原位置，新增前轮位于车头中心，形成前后对称支撑。

### 1.3 Design Decisions Summary

| Decision | Choice | Rationale |
|---|---|---|
| 机器人模型维护方式 | 直接修改 `models/robot/model.sdf` | 当前机器人没有模板；现有模板同步机制只服务于无人机相机 |
| 原万向轮处理 | 完整保留 `caster_link`、`caster_joint`、原位置和物理参数 | 最小化模型变更 |
| 双轮坐标 | 后轮 `x=-0.055, y=0`，前轮 `x=+0.055, y=0` | 关于驱动轮中心前后对称，提供启动和刹车支撑 |
| 新万向轮命名 | `caster_front_link` / `caster_front_joint` | 明确表示前部支撑并避免重命名现有组件 |
| 粒子显示插件 | `nav2_rviz_plugins/ParticleCloud` | 与 Jazzy AMCL 发布的 `nav2_msgs/msg/ParticleCloud` 匹配 |
| 原 PoseArray 显示器 | 替换，不保留重复显示器 | `geometry_msgs/PoseArray` 与当前粒子话题类型不兼容 |
| 粒子话题 | 使用 `/particle_cloud` | 已通过运行时节点与话题信息确认实际名称 |
| 粒子 QoS | Best Effort / Volatile / Keep Last(5) | 与 AMCL 实际 Best Effort 发布端兼容 |
| 新增依赖 | 在 `npu_mobile_robot/package.xml` 声明 `nav2_rviz_plugins` | 直接声明实际使用的 RViz 插件 |
| 其他 RViz 配置 | 不修改 `slam.rviz` | 符合 PRD 范围 |

---

## 2. Architecture

### 2.1 System Context

改动位于两个现有包中：

- `npu_sim_description`
  - 负责机器人 SDF 资产。
  - 双万向轮作为 `base_link` 下的两个固定关节子链接。
  - Gazebo 继续通过原有 `DiffDrive` 插件控制左右驱动轮。
- `npu_mobile_robot`
  - 负责 AMCL、Nav2 和 RViz 配置。
  - AMCL 继续原生发布 `/particle_cloud`。
  - RViz 直接使用 Nav2 插件订阅，不经过新增适配节点。

### 2.2 Component Design

#### 机器人双万向轮

| Component | Responsibility | Boundary |
|---|---|---|
| `caster_link` | 原车尾万向轮 | 保留名称、原位置和全部物理配置 |
| `caster_front_link` | 新增车头中心万向轮 | 复制原轮物理配置，仅链接名和纵向坐标不同 |
| `caster_joint` | 将原后轮固定到 `base_link` | 保留 parent、child 和 fixed 类型 |
| `caster_front_joint` | 将新增前轮固定到 `base_link` | 不参与 DiffDrive |
| `test_sdf_assets.py` | 验证双轮结构、对称性和参数一致性 | 不引入动力学阈值测试 |

#### AMCL 粒子可视化

| Component | Responsibility | Boundary |
|---|---|---|
| AMCL | 发布带权重的粒子云 | 算法和参数不变 |
| `amcl.rviz` | 默认加载原生粒子云显示器 | 只修改 AMCL 配置 |
| `nav2_rviz_plugins/ParticleCloud` | 将粒子显示为加权方向箭头 | 使用已安装的 Jazzy 插件 |
| `test_rviz_config.py` | 检查插件、话题、QoS 和其他显示器 | 不测试渲染像素 |
| `test_amcl_nav2_launch.py` | 验证实际 AMCL 粒子消息 | 不负责 GUI 验收 |

### 2.3 Module Interactions

#### 机器人控制链路

```text
/npu/robot/cmd_vel
        │
        ▼
robot_adapter
        │
        ▼
/npu/robot/cmd_vel_gz
        │
        ▼
Gazebo DiffDrive
        │
        ├── left_wheel_joint
        └── right_wheel_joint

base_link
        ├── caster_joint ────────► caster_link
        └── caster_front_joint ──► caster_front_link
```

新增万向轮不是驱动关节，不接收速度命令，也不加入 `DiffDrive` 配置。

#### AMCL 可视化链路

```text
AMCL
  │ publish nav2_msgs/msg/ParticleCloud
  ▼
/particle_cloud
  │ subscribe
  ▼
nav2_rviz_plugins/ParticleCloud
  │
  ▼
RViz，Fixed Frame = world
```

### 2.4 File Structure

```text
README.md                                      [MODIFY: 更正 AMCL 粒子话题说明]
src/
├── npu_sim_description/
│   ├── models/robot/
│   │   └── model.sdf                    [MODIFY: 对称布置双万向轮]
│   └── test/
│       └── test_sdf_assets.py           [MODIFY: 双轮结构和对称性测试]
└── npu_mobile_robot/
    ├── config/
    │   └── amcl.rviz                    [MODIFY: 使用原生 ParticleCloud 显示器]
    ├── test/
    │   ├── test_rviz_config.py          [MODIFY: 修正插件断言并检查唯一性]
    │   └── test_amcl_nav2_launch.py     [MODIFY: 检查实际粒子云消息]
    └── package.xml                      [MODIFY: 声明 nav2_rviz_plugins]
```

当前实现对应的验收材料位于：

- `tasks/evidence/dual-caster-amcl-particles/README.md`
- `tasks/evidence/dual-caster-amcl-particles/gazebo-front-rear-after-cycles.png`
- `tasks/evidence/dual-caster-amcl-particles/amcl-particles-updated.png`
- `tasks/evidence/dual-caster-amcl-particles/amcl-particles-after-motion.png`

明确不创建：

```text
src/npu_sim_description/models/robot/model.sdf.template
```

明确不修改：

- `src/npu_sim_description/models/drone/model.sdf`
- `src/npu_sim_description/models/drone/model.sdf.template`
- `src/npu_mobile_robot/config/slam.rviz`
- `src/npu_mobile_robot/config/nav2.yaml`
- AMCL、Nav2 或 robot adapter 的启动与控制逻辑

---

## 3. Data Model

### 3.1 Schema Changes

无数据库或持久化数据 Schema 变更。配置模型变更如下。

#### 万向轮位姿契约

| Link | Pose `(x y z roll pitch yaw)` | 来源 |
|---|---|---|
| `caster_link` | `-0.055 0 -0.0065 0 0 0` | 原后轮，位置和配置不变 |
| `caster_front_link` | `0.055 0 -0.0065 0 0 0` | 新增前轮 |

两个链接必须满足：

- `x_rear == -x_front == -0.055`
- `y_rear == y_front == 0`
- `z_rear == z_front == -0.0065`
- 两者姿态均为零旋转
- 万向轮中心前后间距为 `0.11 m`

#### 万向轮物理与外观契约

两个链接的以下配置必须一致：

| Property | Required value |
|---|---|
| Mass | `0.02` |
| `ixx`, `iyy`, `izz` | 均为 `0.000002` |
| `ixy`, `ixz`, `iyz` | 均为 `0` |
| Collision geometry | Sphere |
| Visual geometry | Sphere |
| Radius | `0.015` |
| ODE `mu` | `0.0` |
| ODE `mu2` | `0.0` |
| Ambient / Diffuse | `0.3 0.3 0.3 1` |

不得改变：

- `base_link` 的质量、惯性、碰撞和外观
- 左右驱动轮的尺寸、位姿、质量和摩擦
- `wheel_separation=0.1`
- `wheel_radius=0.0215`
- 激光雷达、标记和 DiffDrive 插件配置

### 3.2 Entity Definitions

#### 固定关节契约

| Joint | Type | Parent | Child |
|---|---|---|---|
| `caster_joint` | `fixed` | `base_link` | `caster_link` |
| `caster_front_joint` | `fixed` | `base_link` | `caster_front_link` |

模型中只能有一个名为 `caster_front_link` 的链接和一个名为 `caster_front_joint` 的关节。

#### RViz 粒子显示器契约

| Property | Required value |
|---|---|
| `Class` | `nav2_rviz_plugins/ParticleCloud` |
| `Name` | `ParticleCloud` |
| `Enabled` | `true` |
| `Value` | `true` |
| `Topic.Value` | `/particle_cloud` |
| `Topic.Depth` | `5` |
| `Topic.History Policy` | `Keep Last` |
| `Topic.Durability Policy` | `Volatile` |
| `Topic.Reliability Policy` | `Best Effort` |
| Fixed Frame | `world` |

插件的箭头颜色、长度和形状使用 Nav2 默认值，不在本次需求中定制。

### 3.3 Relationships

- 两个万向轮均为 `base_link` 的固定子链接。
- 两个万向轮与驱动轮之间不建立关节。
- 粒子显示器只订阅 AMCL 粒子云，不影响 Path、Odometry、Map、LaserScan 或 TF 显示器。
- `/particle_cloud` 消息中的 `header.frame_id` 应为 `world`，由 RViz 使用现有 TF 树转换。

### 3.4 Migration Plan

1. 修改机器人 SDF 并增加静态结构测试。
2. 将原 PoseArray 显示器原位替换为 ParticleCloud 显示器。
3. 增加直接插件依赖并更新测试。
4. 重新构建工作空间，使安装空间中的模型和 RViz 配置更新。
5. 分别执行 Gazebo 与 RViz 验收。

没有用户数据迁移。若需要回滚，仅回滚本节列出的文件；不涉及数据库或运行时状态清理。

---

## 4. API Design

### 4.1 Endpoints

无 HTTP Endpoint。ROS 接口契约如下：

| Kind | Name | Type | Change |
|---|---|---|---|
| Subscription | `/particle_cloud` | `nav2_msgs/msg/ParticleCloud` | RViz 改用匹配的订阅显示器 |
| Public command | `/npu/robot/cmd_vel` | `geometry_msgs/msg/Twist` | 不变 |
| Internal command | `/npu/robot/cmd_vel_gz` | `geometry_msgs/msg/Twist` | 不变 |
| Odometry | `/npu/robot/odom` | `nav_msgs/msg/Odometry` | 不变 |
| Path | `/plan` | `nav_msgs/msg/Path` | 不变 |
| Initial pose | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 不变 |
| Laser scan | `/npu/robot/scan` | `sensor_msgs/msg/LaserScan` | 不变 |

### 4.2 Request/Response Schemas

`/particle_cloud` 使用 Jazzy 已有消息，不自定义字段：

```text
std_msgs/Header header
  string frame_id
nav2_msgs/Particle[] particles
  geometry_msgs/Pose pose
  float64 weight
```

验收时要求：

- 消息类型为 `nav2_msgs/msg/ParticleCloud`
- `particles` 非空
- `header.frame_id == "world"`

### 4.3 Error Responses

无请求/响应式 API 错误码。配置加载、话题无数据和 TF 失败按第 6 节处理。

### 4.4 Breaking Changes

- 对外 ROS 话题、消息类型、坐标系和控制接口均无破坏性变更。
- RViz 内部显示配置由 PoseArray 改为 ParticleCloud，属于针对错误消息类型配置的修复。
- 原后轮位置不变；仅新增车头中心前轮及其固定关节。
- 不引入机器人模板或新的生成命令。

---

## 5. Business Logic

### 5.1 Core Algorithms

#### 双万向轮配置流程

1. 完整保留现有 `caster_link`、`caster_joint` 及其位姿。
2. 确认原后轮保持在 `(-0.055, 0, -0.0065)`。
3. 以该链接为基准新增 `caster_front_link`。
4. 将新前轮位姿设为 `(0.055, 0, -0.0065)`。
5. 确保两个链接除名称和位姿外，惯性、碰撞、表面摩擦及外观配置一致。
6. 新增 `caster_front_joint`，固定连接 `base_link` 与 `caster_front_link`。
7. 不将新关节加入 DiffDrive。
8. 直接保存 `model.sdf`，不增加模板渲染步骤。

#### AMCL 粒子显示修复流程

1. 找到原配置中订阅 `/particlecloud` 的 PoseArray 显示器；该旧配置已在本实现中移除。
2. 将旧显示器原位替换为 `nav2_rviz_plugins/ParticleCloud`，不增加重复显示器。
3. 使用实测话题 `/particle_cloud`，将可靠性改为 `Best Effort`，并保留 Volatile/Keep Last(5) QoS。
4. 保持 `Enabled: true` 和 `Value: true`。
5. 保持全局 Fixed Frame 为 `world`。
6. 保留 Path、Odometry、Map、LaserScan、TF、SetInitialPose 和 GoalTool 配置。
7. 声明 `nav2_rviz_plugins` 运行依赖。
8. 更新配置测试和 AMCL 集成测试。

### 5.2 Validation Rules

- 机器人模型必须能作为 XML/SDF 解析。
- 链接名和关节名必须唯一。
- 必须存在且只存在两个规定的万向轮链接。
- 两个万向轮必须关于驱动轮中心 `x=0` 前后对称。
- 两个万向轮必须都位于中心线 `y=0`，且分别位于 `x=-0.055` 和 `x=+0.055`。
- 两个万向轮的物理参数必须一致。
- DiffDrive 的左右关节仍必须分别为 `left_wheel_joint` 和 `right_wheel_joint`。
- `amcl.rviz` 必须且只能有一个启用的 `/particle_cloud` 粒子显示器。
- 不得再保留订阅 `/particle_cloud` 的 `rviz_default_plugins/PoseArray`。
- `slam.rviz` 不得因本需求新增粒子显示器。
- 无人机模板和生成文件的既有同步测试必须继续通过。

### 5.3 State Machine

无新增状态机。AMCL 生命周期、Nav2 生命周期和机器人命令看门狗均保持现状。

### 5.4 Edge Cases

| Edge case | Required handling |
|---|---|
| 将新增轮放在车尾左侧或右侧 | 拒绝，该布局不能提供前后对称支撑，也不能抑制启停俯仰 |
| 将原后轮移离 `(-0.055, 0, -0.0065)` | 拒绝，原后轮必须保持原位置 |
| 两个万向轮使用相同名称 | 拒绝，SDF 名称必须唯一 |
| 新轮被误配置为驱动关节 | 拒绝，只能使用固定关节 |
| 同时保留 PoseArray 和 ParticleCloud | 删除错误的 PoseArray，避免重复和类型不匹配 |
| `/particle_cloud` 没有消息 | 不判定 GUI 验收通过，先检查 AMCL 生命周期和地图输入 |
| 粒子消息存在但 RViz 不可见 | 检查插件类型、Enabled/Value、Fixed Frame 和 TF |
| 粒子 `frame_id` 不是 `world` | 视为验收失败，不通过修改 RViz 掩盖配置错误 |
| 修改模型后 Gazebo 仍使用旧资产 | 重新构建并确认环境已 source，再进行验收 |
| 双轮配置后仍有明显晃动 | 记录现象并停止，不得擅自调整其他物理参数 |
| 无图形显示环境 | 静态和 headless 测试可运行，但 GUI 验收不能在 headless 模式下判为通过 |

---

## 6. Error Handling

### 6.1 Error Taxonomy

| Error | Condition | Detection | Required response |
|---|---|---|---|
| SDF parse/load failure | XML 无效或链接/关节定义错误 | pytest 或 Gazebo 加载日志 | 修复 SDF，不跳过测试 |
| Caster symmetry failure | 坐标或参数不满足契约 | 静态结构测试 | 修正模型，不调整其他底盘参数 |
| RViz plugin load failure | ParticleCloud 插件不可加载 | RViz Displays 面板或日志 | 检查 `nav2_rviz_plugins` 安装与依赖 |
| Message type mismatch | 显示器与 `/particle_cloud` 类型不一致 | `ros2 topic type` / RViz 状态 | 使用 Nav2 ParticleCloud 插件 |
| Particle data unavailable | AMCL 未发布非空粒子云 | 集成测试或 topic 检查 | 检查 AMCL active、地图和激光输入 |
| TF unavailable | `world` 到粒子坐标系不可用 | RViz 状态或 TF 检查 | 修复定位/TF 流程，不修改固定坐标系 |
| Drone template test failure | 无人机模型与 forward 预设不一致 | `test_camera_presets.py` | 与本需求分开处理，不修改无人机资产规避 |

### 6.2 Retry Strategy

- 不新增运行时重试逻辑。
- AMCL 启动后允许等待其生命周期进入 active 并收到地图。
- 手工启动/停止测试使用连续速度发布，避免单条命令因订阅尚未建立而丢失。
- GUI 验收失败时应修复根因后重新运行，不以重复启动直至偶然通过作为验收依据。

### 6.3 Failure Modes

- Gazebo 或 AMCL 集成环境不可用时，可以完成静态测试，但必须将对应 GUI/集成验收标记为未完成。
- 粒子云消息缺失时，不得用静态伪造消息代替真实 AMCL 流程作为最终验收。
- 无显示环境下，不得仅凭 RViz YAML 解析成功宣称“粒子点可见”。

---

## 7. Security

### 7.1 Authentication & Authorization

不涉及身份认证、权限或外部服务。

### 7.2 Input Validation

- 仅修改仓库内受版本控制的 XML、YAML 和测试文件。
- 不解析用户提供的运行时 XML/YAML。
- 不执行由 RViz 或 SDF 配置内容生成的 shell 命令。

### 7.3 Data Protection

不涉及敏感数据、加密、审计日志或网络暴露面变化。

---

## 8. Performance

### 8.1 Expected Load

- AMCL 粒子规模保持现有配置：最少 500、最多 2000 个粒子。
- 粒子发布频率和更新条件由现有 AMCL 参数决定。
- 机器人仅增加一个固定链接、一个固定关节和简单球体碰撞/外观。
- 不新增 ROS 节点或额外消息转发。

### 8.2 Optimization Strategy

- 使用 Nav2 原生 RViz 插件直接渲染粒子云。
- 不增加 ParticleCloud 到 PoseArray 的转换节点。
- 不复制或缓存粒子消息。
- 不修改 AMCL 粒子数量来规避渲染问题。

### 8.3 Database Considerations

无数据库访问或索引需求。本次不设定额外性能阈值；现有仿真与 AMCL 集成测试不得出现新增超时。

---

## 9. Testing Strategy

### 9.1 Unit Tests

#### `test_sdf_assets.py`

新增或扩展测试以验证：

1. 两个万向轮链接和固定关节均存在且唯一。
2. 前后轮位姿严格符合第 3.1 节。
3. 两个轮的惯性、碰撞、摩擦和外观参数一致。
4. 原 DiffDrive 左右关节和参数保持不变。
5. 模型仍可被 XML 解析并由 Gazebo 加载。

#### `test_rviz_config.py`

更新并扩展测试以验证：

1. 使用 `nav2_rviz_plugins/ParticleCloud`。
2. `/particle_cloud` 的 Topic QoS 完整且正确。
3. `Enabled` 和 `Value` 均为 `true`。
4. 粒子显示器只有一个。
5. 不再存在订阅 `/particle_cloud` 的 PoseArray。
6. Fixed Frame 仍为 `world`。
7. Path `/plan` 和 Odometry `/npu/robot/odom` 配置保持不变。
8. `slam.rviz` 的既有检查继续通过。
9. `package.xml` 声明 `nav2_rviz_plugins`。

Focused commands:

```bash
/usr/bin/python3 -m pytest -q src/npu_sim_description/test/test_sdf_assets.py
/usr/bin/python3 -m pytest -q src/npu_sim_description/test/test_camera_presets.py
/usr/bin/python3 -m pytest -q src/npu_mobile_robot/test/test_rviz_config.py
```

### 9.2 Integration Tests

#### Gazebo 模型加载

运行现有 `test_gz_spawn.py`，确认修改后的机器人能够与无人机和场景模型共同加载，Gazebo 日志无新增错误。

#### AMCL 粒子发布

扩展 `test_amcl_nav2_launch.py`：

- 以 Best Effort QoS 订阅 `/particle_cloud`，消息类型为 `nav2_msgs/msg/ParticleCloud`。
- 在 AMCL active 后发布一次已知初始位姿以触发粒子更新。
- 等待至少一条粒子云消息。
- 断言 `particles` 非空。
- 断言 `header.frame_id == 'world'`。
- 保留并继续通过现有定位 TF 和导航测试。

测试使用独立 `ROS_DOMAIN_ID` 与 `GZ_PARTITION`，不得与手工 Gazebo 场景并行运行重复世界。

### 9.3 Edge Case Tests

#### GUI-01：三次启动和停止

1. 启动带 GUI 的联合仿真：
   ```bash
   ros2 launch npu_sim_bringup sim.launch.py
   ```
2. 每轮先等待车辆静止。
3. 以 10 Hz 发布 `linear.x=0.15` 的速度命令约 2 秒：
   ```bash
   timeout 2s ros2 topic pub -r 10 /npu/robot/cmd_vel \
     geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
   ```
   `timeout` 结束发布后的非零退出码属于预期行为。
4. 显式发布一次零速度：
   ```bash
   ros2 topic pub --once /npu/robot/cmd_vel \
     geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
   ```
5. 观察启动和停止过程中的底盘及激光雷达是否出现俯仰、侧倾或支撑弹跳，并记录模型 RPY。
6. 停止后立即采样，并在停止后 0.1 秒再次采样，避免漏掉短暂俯仰峰值。
7. 重复共三轮；三轮在启动、停止瞬间和停止后 0.1 秒的 roll/pitch 均约为零且无明显晃动，才可判为通过。

#### GUI-02：AMCL 粒子显示

1. 启动联合仿真。
2. 启动 AMCL 与 RViz：
   ```bash
   ros2 launch npu_mobile_robot amcl.launch.py rviz:=true
   ```
3. 确认 `/particle_cloud` 类型为 `nav2_msgs/msg/ParticleCloud`。
4. 确认 RViz Displays 中存在唯一且启用的 `ParticleCloud`。
5. 确认不手动添加显示器即可看到粒子箭头。
6. 使用 SetInitialPose 工具或向 `/initialpose` 发布已知位姿，观察粒子分布更新；本次证据采用后者。
7. 移动车辆或下发导航目标，观察粒子随定位更新。
8. 确认 Path、Odometry、Map、LaserScan 和 TF 仍可用。
9. 保存能够显示粒子点与 Displays 状态的截图作为验收证据。

在 WSL2 上使用用户已运行的 XLaunch；如常规截图变黑，按项目约定通过 Windows PowerShell `System.Drawing` 截取宿主机 GUI。

### 9.4 Acceptance Criteria Mapping

| US/FR | Test | Type | Description |
|---|---|---|---|
| US-001 AC1 | SDF-01 | Unit | 两个万向轮及其唯一链接、关节存在 |
| US-001 AC2 | SDF-02 | Unit | 物理、碰撞和外观参数一致 |
| US-001 AC3 | SDF-03 / Existing tests | Unit | 驱动轮、DiffDrive 和接口不变 |
| US-001 AC4 | SDF-02 / Diff review | Unit/Review | 无其他物理参数变化 |
| US-001 AC5 | Camera preset tests | Unit | 现有无人机模板同步机制无回归 |
| US-001 AC6 | GUI-01 | GUI | 三次启停及停止后 0.1 秒的 roll/pitch 均约为零，无明显晃动 |
| US-001 AC7 | Build / SDF / Spawn | Build/Integration | 构建、模型测试及 Gazebo 加载通过 |
| US-002 AC1 | RVIZ-01 | Unit | ParticleCloud 默认启用 |
| US-002 AC2 | RVIZ-02 / AMCL-01 | Unit/Integration | 话题、QoS 和消息类型匹配 |
| US-002 AC3 | GUI-02 | GUI | 默认配置直接显示粒子 |
| US-002 AC4 | GUI-02 | GUI | 初始位姿和移动后粒子更新 |
| US-002 AC5 | AMCL-01 / GUI-02 | Integration/GUI | 使用 world，粒子可见且无 TF 错误 |
| US-002 AC6 | RVIZ-03 | Unit | 唯一粒子显示，Path/Odometry 保留 |
| US-002 AC7 | RVIZ tests / GUI-02 | Unit/GUI | 自动测试和原生 RViz 检查通过 |
| US-002 AC8 | GUI-02 | GUI | 浏览器不适用于原生 RViz，以原生 GUI 证据完成视觉验收 |
| FR-1 | SDF-01/02 | Unit | 原后轮链接、位置与参数完整保留 |
| FR-2 | SDF-01 | Unit | 新增一个车头中心前轮 |
| FR-3 | SDF-01 | Unit | 前后轮纵向坐标镜像，横向和垂直坐标一致 |
| FR-4 | SDF-02 | Unit | 外观与尺寸一致 |
| FR-5 | SDF-02 | Unit | 碰撞与惯性一致 |
| FR-6 | Camera preset tests | Unit | 既有模板生成文件保持同步 |
| FR-7 | Existing adapter/launch tests | Integration | 公共控制接口不变 |
| FR-8 | RVIZ-01 | Unit | ParticleCloud 默认启用 |
| FR-9 | RVIZ-02 / AMCL-01 | Unit/Integration | 绑定实际 AMCL 话题 |
| FR-10 | AMCL-01 / GUI-02 | Integration/GUI | world 坐标和 TF 正常 |
| FR-11 | RVIZ-03 | Unit | Path 与 Odometry 保留 |
| FR-12 | GUI-02 | GUI | 无需手动配置即可显示 |

---

## 10. Implementation Plan

### 10.1 Phases

#### Phase 1：机器人模型

1. 保持原后轮及其位置不变。
2. 新增车头中心前轮和固定关节。
3. 扩展 SDF 静态测试。
4. 运行模型与 Gazebo 加载测试。

#### Phase 2：AMCL 可视化

1. 替换 RViz 显示器。
2. 添加直接插件依赖。
3. 更新 RViz 静态测试。
4. 扩展 AMCL 粒子消息集成测试。

Phase 1 与 Phase 2 无代码依赖，可分别实施和审查。

#### Phase 3：集成验收

1. 执行 GUI-01。
2. 执行 GUI-02。
3. 若仍有晃动或粒子不可见，记录失败，不扩大修改范围。
4. 完成全量项目验证。

全量验证命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
ros2 run npu_sim_tools validate_readme_commands README.md src
git diff --check
```

### 10.2 Issue Mapping

| Issue | SPEC Sections | Priority | Depends On |
|---|---|---|---|
| Issue 1：对称双万向轮 | 2.2、3.1–3.3、5.1、9.1、GUI-01 | High | — |
| Issue 2：AMCL 粒子可视化 | 2.2、3.2、4、5.1、9.1–9.2、GUI-02 | High | — |
| Issue 3：联合回归与验收证据 | 9.3–9.4、10.1 | High | Issue 1、Issue 2 |

### 10.3 Incremental Delivery

- 两项修复可以分别提交，不需要 feature flag。
- 每项合入前必须通过对应静态测试。
- 最终交付必须同时包含 GUI-01 和 GUI-02 的验收证据。
- 不采用灰度发布；仿真资产和 RViz 配置在重新构建后直接生效。
- 回滚按文件级变更执行，无持久化状态迁移。

---

## 11. Open Questions & Risks

### 11.1 Unresolved Questions

无。

已解决的 PRD 冲突：

- PRD 中泛化的“机器人模板同步”按当前代码结构解释为：机器人直接维护 `model.sdf`；无人机现有模板同步要求保持不变。
- 用户后续明确纠正了早期双后轮方案：左右已有差速轮支撑，最终采用前后中心对称万向轮，以消除启动和刹车俯仰。
- 原后轮位置、全部物理参数和控制逻辑均保持不变；新增前轮是唯一模型结构变化。

### 11.2 Technical Risks

| Risk | Impact | Mitigation |
|---|---|---|
| 双轮增加接触点后仍出现晃动 | 无法达到稳定性验收 | 固定测试场景复现并报告，不擅自修改其他物理参数 |
| “无明显晃动”具有主观性 | GUI 验收结论可能不一致 | 固定三次启停流程，记录停止后 0.1 秒的 RPY，并保留截图证据 |
| 旧 PoseArray 测试掩盖消息类型错误 | 粒子仍不可见 | 同时检查插件类型和真实 AMCL 消息类型 |
| 仅静态检查而缺少真实粒子数据 | 无法证明默认可视化有效 | 增加 AMCL 集成断言并执行 GUI-02 |
| 安装空间仍使用旧配置 | 测试结果与源码不一致 | 重新构建、source `install/setup.bash` 后验收 |
| 浏览器工具无法操作原生 RViz | PRD 通用浏览器验收项不适用 | 使用原生 RViz GUI 检查与截图代替 |
| 并行 Gazebo 场景相互干扰 | 粒子或稳定性测试偶发失败 | 每个场景使用独立 ROS domain 和 Gazebo partition |

### 11.3 Assumptions

- ROS 2 Jazzy 的 AMCL 持续使用 `nav2_msgs/msg/ParticleCloud`；当前环境的接口和插件安装已核实。
- 遵循 ROS 坐标约定，正 `X` 为车辆前方；`caster_link` 是原后轮，`caster_front_link` 是新增前轮。
- 前后轮坐标固定为 `x=-0.055/+0.055, y=0, z=-0.0065`，不需要实现阶段重新选择。
- 不增加自动化动力学阈值测试；GUI 验收使用 `gz model --pose` 记录三轮启停姿态。
- 不在本次工作中更新历史 SPEC 文档中旧的 PoseArray 设计记录。
- README 中的粒子话题说明同步更正为 `/particle_cloud`，工作流命令不变。
- 实测 AMCL 发布端使用 Best Effort QoS；RViz 订阅必须兼容该策略。
