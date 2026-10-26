# SPEC：Gazebo 失败工作流修复技术方案

> Technical specification derived from: `tasks/prd-fix-gazebo-failed-workflows.md`
> Generated: 2026-10-26 | Target branch: `ros2` | Commit: `24e5075` + 当前未提交修复

## 1. Summary

### 1.1 What This SPEC Covers

本 SPEC 定义三项失败工作流的具体修复方式：

- 使用有界 ACK 确认，保证直接跟随节点的最终零速度被订阅者接收。
- 通过 RViz 配置补齐 AMCL 粒子云、Nav2 规划路径和机器人实际轨迹。
- 在临时验收探针中保存完整轨迹和 action 取消上下文，区分正常替换与真实异常。

实现仅限本项目节点、RViz 配置、测试和验收探针，不修改 Nav2/RViz 上游源码，不新增常驻发布节点或通用框架。

### 1.2 PRD Reference

- Source: `tasks/prd-fix-gazebo-failed-workflows.md`
- User Stories covered: US-001 至 US-005
- Functional Requirements covered: FR-1 至 FR-17
- Related evidence: `tasks/gazebo-readme-acceptance-report.md`

### 1.3 Design Decisions Summary

| Decision | Choice | Rationale |
|---|---|---|
| 最终零命令确认 | `wait_for_all_acked(Duration(seconds=0.2))` | 直接确认可靠订阅 ACK，且等待时间短于 0.5 秒门控超时 |
| ACK 失败处理 | 记录错误并在有界时间内继续关闭 | 避免静默失败或无限等待 |
| AMCL 粒子云 | PoseArray `/particlecloud` | 使用 AMCL 原生消息，无新增发布节点 |
| Nav2 规划路径 | Path `/plan` | 复用 Nav2 现有规划输出 |
| 实际轨迹 | Odometry `/npu/robot/odom`，`Keep: 500` | 无需新增轨迹发布器，约可显示最近 10 秒轨迹 |
| 完整轨迹证据 | 临时探针以 10Hz 保存 JSONL | 满足不低于 5Hz 的要求，不增加产品常驻 recorder |
| 替换 ERROR | 保留上游文本并关联 goal 上下文 | 不隐藏异常，也不承诺消除正常替换的上游输出 |
| Goal 生命周期 | 保持现有四阶段实现 | 已实现旧 goal 终态后再替换，避免无谓重写 |

---

## 2. Architecture

### 2.1 System Context

```text
follow_drone_tf 退出
        |
        v
发布零 Twist -> wait_for_all_acked(0.2s)
        |                 |
        |                 +-- ACK 成功：关闭节点
        |                 +-- ACK 失败：记录错误并有界关闭
        v
/npu/robot/cmd_vel 订阅者收到零命令
```

```text
AMCL /particlecloud ----> PoseArray 显示
Nav2 /plan -------------> Path 显示
/npu/robot/odom --------> Odometry 历史显示（Keep=500）
        |
        +----------------> 临时探针 10Hz JSONL 轨迹与取消上下文
```

### 2.2 Component Design

#### `follow_drone_tf`

仅修改正常退出路径：

1. 发布一次零 `Twist`。
2. 调用 publisher 的 `wait_for_all_acked`，超时 0.2 秒。
3. 返回值或异常表明未确认时记录错误。
4. 在确认或有界等待结束后销毁节点并关闭 rclpy。

不增加线程、manager 或固定 sleep，不改变运行中的控制逻辑。

#### `amcl.rviz`

新增三个显示项：

| Display | Topic | Purpose |
|---|---|---|
| `rviz_default_plugins/PoseArray` | `/particlecloud` | 显示 AMCL 粒子云 |
| `rviz_default_plugins/Path` | `/plan` | 显示 Nav2 规划路径 |
| `rviz_default_plugins/Odometry` | `/npu/robot/odom` | 显示实际机器人轨迹 |

保留现有 Grid、Map、LaserScan、TF、SetInitialPose 和 Nav2 GoalTool，固定系继续为 `world`。

#### `publish_follow_goal`

保持当前 `IDLE/SENDING/ACTIVE/CANCELING` 结构和取消语义，不重写状态机。仅通过测试证明：

- 固定目标无取消时成功结束。
- 正常替换时旧 goal 终态先于新 goal。
- 非预期失败仍进入退避和错误诊断。

#### 临时验收探针

仅修改 `/tmp/opencode/npu_gazebo_acceptance/nav_follow_probe.py`：

- 每 0.1 秒记录机器人世界系位置和时间戳。
- 将完整轨迹写入 JSONL，而非只保存首末位置。
- 记录 `/plan` 是否收到及 Path 所需消息类型。
- 关联 `follow_path` ERROR 与 goal ID、取消时间、终态和新 goal 提交时间。
- 固定目标阶段与替换阶段分开检查，避免把正常替换文本误判为固定目标错误。

### 2.3 Module Interactions

1. `amcl.launch.py` 继续启动相同 Nav2 节点和 RViz。
2. AMCL 发布 `/particlecloud`，Nav2 planner 发布 `/plan`，Gazebo 发布 `/npu/robot/odom`。
3. RViz 从这三个话题直接显示，不改变其 QoS、frame 或所有权。
4. 跟随节点继续生成 `/npu/robot/goal_pose` 和 Nav2 action goal。
5. 探针仅订阅和记录，不向 Nav2 注入额外 goal，不修改产品状态。
6. 直接跟随退出 ACK 发生在公共 `/npu/robot/cmd_vel` 上，因此可证明不是 `robot_adapter` 的 0.5 秒兜底归零。

### 2.4 File Structure

```text
src/
├── npu_tracker/
│   ├── npu_tracker/
│   │   └── follow_drone_tf.py              [MODIFY: bounded ACK on exit]
│   └── test/
│       ├── test_follow_exit.py             [NEW: five real exit/subscriber checks]
│       └── test_follow_goal_action.py      [MODIFY: fixed success and abort cases]
└── npu_mobile_robot/
    ├── config/
    │   └── amcl.rviz                       [MODIFY: PoseArray, Path, Odometry]
    └── test/
        └── test_rviz_config.py             [MODIFY: display/topic/history assertions]

/tmp/opencode/npu_gazebo_acceptance/
└── nav_follow_probe.py                     [TEMP: dense trajectory and error context]

tasks/
├── gazebo-readme-acceptance-fix-report.md   [NEW: before/after acceptance]
└── gazebo-readme-acceptance-logs/
    ├── fix_s03_amcl_nav_follow/
    │   ├── headless/
    │   └── gui/
    └── fix_s04_direct_follow/
        └── headless/
```

原验收报告和原始失败日志保持不变。

---

## 3. Data Model

### 3.1 Schema Changes

无数据库或 ROS message 定义变化。

### 3.2 Evidence Records

完整轨迹使用 JSONL，每行至少包含：

| Field | Type | Meaning |
|---|---|---|
| `scenario` | string | 复测场景 ID |
| `stamp` | number | 位姿时间戳，秒 |
| `x`, `y` | number | 世界系机器人位置，米 |
| `yaw` | number | 世界系航向，弧度 |
| `source` | string | `/npu/robot/odom` 或对应 TF 来源 |

取消上下文记录至少包含：

| Field | Meaning |
|---|---|
| `goal_id` | 被替换 goal 的 ID |
| `cancel_requested_at` | 取消请求时间 |
| `terminal_status` | 旧 goal 的终态 |
| `terminal_at` | 旧 goal 终态时间 |
| `new_goal_submitted_at` | 新 goal 提交时间 |
| `classification` | `expected_replacement`、`input_stale` 或 `unexpected_failure` |

### 3.3 Relationships

- `new_goal_submitted_at >= terminal_at` 是替换时序的强制条件。
- 固定目标没有 `cancel_requested_at`，成功终态应为 `STATUS_SUCCEEDED`。
- 正常替换取消使用 `expected_replacement`，不得写成 `succeeded`。
- 轨迹采样使用相同世界系，碰撞和卡死分析必须引用完整 JSONL，而非只引用统计摘要。

### 3.4 Migration Plan

无需迁移。新增复测报告，不覆盖原报告或原始失败证据。

---

## 4. API Design

### 4.1 Interfaces

| Interface | Change | Contract |
|---|---|---|
| `Publisher.wait_for_all_acked` | 新增调用 | 退出零命令等待最多 0.2 秒 |
| `/npu/robot/cmd_vel` | 不变 | 最终零 Twist 必须被真实订阅者接收 |
| `/particlecloud` | 新增 RViz 订阅 | `geometry_msgs/PoseArray`，显示 AMCL 粒子 |
| `/plan` | 新增 RViz 订阅 | `nav_msgs/Path`，显示规划路径 |
| `/npu/robot/odom` | 新增 RViz 订阅 | `nav_msgs/Odometry`，保留 500 个历史位置 |
| 临时轨迹 JSONL | 新增验收输出 | 10Hz、带时间戳的连续轨迹 |

### 4.2 RViz Configuration Contracts

PoseArray、Path 和 Odometry 均使用项目现有可靠 QoS 写法：

- `History Policy: Keep Last`
- `Durability Policy: Volatile`
- `Reliability Policy: Reliable`

Path 保留 `Buffer Length: 1` 以显示最新规划；Odometry 使用 `Keep: 500` 形成历史轨迹。

### 4.3 Error Responses

| Condition | Required behavior |
|---|---|
| 最终 publish 抛异常 | 记录异常并继续有界关闭，不得静默忽略 |
| ACK 返回 false | 记录未确认错误，focused 测试和复测均不得判 PASS |
| `/particlecloud` 无消息 | AMCL GUI 视觉项不得判 PASS |
| `/plan` 无消息 | Nav2 Path 视觉项不得判 PASS |
| 固定 goal 被取消或失败 | 不得标记为预期替换或成功 |
| 非预期 ABORTED/结果异常 | 保留失败退避和错误诊断 |

### 4.4 Breaking Changes

无公共 ROS 接口、启动命令或消息类型变更。仅新增退出等待、RViz 显示和验收证据。

---

## 5. Business Logic

### 5.1 Exit ACK Sequence

1. 正常跟随期间保持现有 timer 和控制行为。
2. 捕获正常退出信号后，首先发布零 `Twist`。
3. 对同一 publisher 调用 `wait_for_all_acked(Duration(seconds=0.2))`。
4. ACK 成功后销毁节点并关闭 rclpy。
5. 超时或异常时记录失败原因，再按现有顺序有界关闭。
6. 不在 ACK 后启动任何新速度发布。

测试必须观察公共话题，不能只检查函数被调用或 Gazebo 最后速度为零。

### 5.2 AMCL and Nav2 Visualization

- PoseArray 使用默认箭头显示，便于观察粒子分布和收敛。
- Path 使用 `/plan`，不创建 `/npu/robot/plan` 等替代话题。
- Odometry 使用 `Keep: 500`；在 50Hz 输入下约保留最近 10 秒轨迹。
- 所有显示使用 `world` 固定系，依赖现有 TF 链，不新增 `map` 别名。
- 若实际视觉被遮挡，调整视图角度或缩放，而不是修改消息或判定阈值。

### 5.3 Fixed, Replacement, and Unexpected Goals

#### 固定目标

- 不发送取消请求。
- 最终状态为 `STATUS_SUCCEEDED`。
- 验收日志中不得出现 `Failed to get result for follow_path in node halt!`。

#### 正常替换

- 旧 goal 进入取消流程。
- 等待旧 goal result 后才允许新 goal 提交。
- 诊断使用 `replaced`，而不是 `succeeded`。
- 上游 ERROR 文本保留，并通过 goal ID 和时序证明其属于预期替换。

#### 非预期失败

- `ABORTED`、结果异常或超时不得归为 `replaced`。
- 使用现有失败退避和错误诊断。
- 不得通过过滤日志将其隐藏。

### 5.4 Dense Trajectory and Evidence Analysis

1. 探针每 0.1 秒记录一次机器人位置。
2. 将每个样本立即追加到 JSONL，避免异常时只剩少量内存数据。
3. 跟随结束后保存独立轨迹摘要：采样数、平均频率、首末位置和异常区间。
4. 使用完整序列检查障碍边界、持续无位移和异常往返。
5. 固定目标与替换阶段的 `follow_path` 日志分别统计，避免混淆固定目标错误与预期替换错误。

### 5.5 Edge Cases

- ACK 在 0.2 秒内未完成：记录失败，不能再用“publisher 已调用”代替“订阅者已收到”。
- 节点在退出前已无订阅者：ACK 失败按验收失败处理，不掩盖。
- AMCL 粒子云存在但视图过远：允许调整 RViz 视图后截图，原始消息和判定不变。
- 实际 `/plan` 与 planner 配置不同：以真实话题检查为准，但不得新增发布节点来伪造该接口。
- 轨迹记录过程中探针异常：原始 JSONL 保留，不能把采样不足判为通过。

---

## 6. Error Handling

### 6.1 Error Taxonomy

| Error | Detection | Required handling |
|---|---|---|
| Exit publish/ACK failure | 异常或 false | 记录并判定修复验证失败 |
| AMCL 显示缺失 | 配置或 GUI 检查失败 | 修正配置并复测，不新增消息发布器 |
| 固定 goal 异常 | 非预期终态或 ERROR | 保留失败退避和诊断 |
| 替换 goal 结果异常 | result exception/status | 保留句柄、退避和错误诊断，不与预期替换混淆 |
| 证据不足 | 采样频率不足或文件缺失 | 该项不得判 PASS |

### 6.2 Retry Strategy

- ACK 只进行一次有界等待，不使用循环 sleep 或无限重试。
- 固定目标功能失败不通过重复运行挑选成功。
- 临时探针或基础设施失败可以单独修正后重跑，并保留原始日志。
- 复测只覆盖工作流 4、5、6及必要受影响项。

### 6.3 Failure Modes

- `wait_for_all_acked` 在当前中间件返回 false：测试明确失败，不改为无依据的固定等待。
- `/particlecloud` 或 `/plan` 没有发布：报告依赖缺失，不伪造可视化通过。
- 正常替换仍有上游 ERROR：报告保留文本和 goal 上下文，不将其描述为已消除。
- 退出后仍漂移：即使 ACK 成功，也继续判失败并检查是否还有其他命令来源。

---

## 7. Security

### 7.1 Authentication & Authorization

无新增网络服务、权限或真实硬件控制接口，全部操作限于本地仿真。

### 7.2 Input Validation

- 测试速度输入必须有界，并在每段输入后显式归零。
- 复测使用独立 ROS domain 和 Gazebo partition。
- 只终止本次创建的进程组，不清理其他用户进程。
- 验收探针不修改产品参数或 Nav2 goal。

### 7.3 Data Protection

- 环境日志使用白名单，不记录凭据或完整敏感环境变量。
- 报告与图像不上传外部服务。
- 原失败日志和复测日志均保存在项目指定 `tasks/` 目录。

---

## 8. Performance

### 8.1 Expected Load

- 退出 ACK 最多增加 0.2 秒一次性关闭等待，不影响正常控制频率。
- Odometry 保留 500 个显示对象，属于有界 RViz 配置。
- 轨迹按 10Hz 记录，数据量随场景时长线性增长但不写入数据库。

### 8.2 Optimization Strategy

- 复用现有 publisher、Nav2 话题和 RViz 配置。
- 不新增常驻节点、executor 线程或重复路径发布。
- 复测仅覆盖失败工作流和必要回归，不重复全部 9 项。

### 8.3 Database Considerations

无数据库访问或迁移。

---

## 9. Testing Strategy

### 9.1 Unit and Focused Tests

#### `test_follow_exit.py`

- 使用真实 ROS publisher/subscriber 和实际 `follow_drone_tf` 进程。
- 提供无人机位姿和机器人 TF，先观察非零命令。
- 向节点发送正常退出信号，记录收到最终零 Twist 的时间。
- 要求零命令在 0.5 秒前到达，之后无新的非零命令。
- 连续运行 5 轮，每轮均检查退出结果。

#### RViz 静态测试

扩展 `test_rviz_config.py`：

- 断言 PoseArray `/particlecloud`。
- 断言 Path `/plan`。
- 断言 Odometry `/npu/robot/odom` 且 `Keep >= 100`，本方案实际为 500。
- 断言原有 Map、LaserScan、TF、Nav2 GoalTool 和 `world` 不变。

#### Action 测试

扩展 `test_follow_goal_action.py`：

- 固定目标成功结束，`cancel_requests == 0`，结果为 `succeeded`。
- 正常替换时记录旧终态和新 goal 提交顺序。
- 非预期 ABORTED 使用失败退避和错误诊断。

### 9.2 Integration and GUI Tests

- 重跑 AMCL/Nav2 场景，实际观察粒子云、Path 和 Odometry 轨迹。
- 重跑直接跟随场景，使用真实公共话题确认退出零命令和 5 秒漂移。
- 固定 Nav2 目标阶段检查没有同类 `follow_path` result ERROR。
- 正常替换阶段保留 ERROR 文本，并核对 goal ID 和提交时序。
- 使用 XLaunch/Windows 主机截图保存修复后视觉证据。

### 9.3 Edge Case Tests

- 连续 5 次退出避免单次时序偶然性。
- 固定目标无取消、正常替换和异常 ABORTED 分开验证。
- 轨迹采样率低于 5Hz 时验收失败。
- `/plan` 或粒子云缺失时不得仅凭 RViz 启动判定通过。

### 9.4 Acceptance Criteria Mapping

| US/FR | Test | Type | Description |
|---|---|---|---|
| US-001 / FR-1–3 | `test_follow_exit.py` 五轮 + 工作流 5 复测 | ROS integration | 实际零命令、ACK 时限和退出停车 |
| US-002 / FR-4–5 | RViz 静态检查 + AMCL GUI | static + GUI | 粒子云、固定系和现有工具 |
| US-003 / FR-6–9 | RViz 静态检查 + 10Hz 轨迹 + Nav2 GUI | static + integration + GUI | Path、实际轨迹和完整证据 |
| US-004 / FR-10–12 | fixed/replacement/aborted 测试与日志关联 | action + integration | 区分预期替换和真实失败 |
| US-005 / FR-13–17 | focused、失败工作流、`colcon test`、前后报告 | regression + acceptance | 不放宽阈值、不删除失败记录 |

最终验证：

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
source install/setup.bash
ros2 run npu_sim_tools validate_readme_commands README.md src
git diff --check
```

---

## 10. Implementation Plan

### 10.1 Phases

1. **RViz 修复**：增加 PoseArray、Path、Odometry，并完成静态测试。
2. **退出修复**：加入有界 ACK 和五轮真实退出测试。
3. **Action 与证据**：补固定/异常 goal 测试，完善 10Hz 轨迹和错误上下文。
4. **复测**：重跑工作流 4、5、6，保存修复后日志和 GUI 图像。
5. **收尾**：完整 `colcon test`、README 命令校验、diff 检查，生成 `gazebo-readme-acceptance-fix-report.md`。

### 10.2 Issue Mapping

| Issue | SPEC Sections | Priority | Depends On |
|---|---|---|---|
| #1 AMCL/Nav2 RViz | 2.2, 4.2, 5.2 | high | — |
| #2 直接跟随退出 ACK | 2.1, 5.1, 9.1 | high | — |
| #3 Goal 异常与完整轨迹 | 3.2, 5.3, 5.4 | high | #1 |
| #4 失败项复测与报告 | 9.2–9.4, 10.1 | high | #1–#3 |

### 10.3 Incremental Delivery

- 每项独立完成后先运行 focused tests，再进入对应 Gazebo 复测。
- 原报告与失败证据不覆盖，修复结果写入独立 fix report。
- 不提交、推送或创建 PR。

---

## 11. Open Questions & Risks

### 11.1 Unresolved Questions

无。已确认使用有界 ACK，并保留、关联正常替换的上游 ERROR 文本。

### 11.2 Technical Risks

| Risk | Impact | Mitigation |
|---|---|---|
| 中间件未在 0.2 秒内返回 ACK | 退出修复仍失败 | 明确记录并判 FAIL，不用固定 sleep 伪装成功 |
| SIGINT 已导致 context 关闭 | ACK 可能异常 | 保留异常日志并以真实订阅测试验证 |
| `/plan` QoS 或名称不匹配 | Path 不显示 | 先验证真实接口，不新增发布器；失败则记录依赖问题 |
| Odometry 历史仅覆盖最近约 10 秒 | 长轨迹视觉不完整 | GUI 用于观察最近轨迹，完整分析使用 10Hz JSONL |
| 正常替换仍有上游 ERROR | 用户可能误解为未修复 | 报告区分“固定目标无异常”和“替换日志已关联”，不承诺全部消除 |
| 探针采样或文件写入失败 | 证据不足 | 保留失败状态，不凭少量首末样本判通过 |

### 11.3 Assumptions

- `/npu/robot/cmd_vel` 继续使用默认可靠 QoS，`wait_for_all_acked` 对该 publisher 有效。
- AMCL 持续发布 `/particlecloud`，planner 持续发布 `/plan`。
- `/npu/robot/odom` 约 50Hz，`Keep: 500` 约覆盖最近 10 秒。
- 临时探针继续位于 `/tmp/opencode`，不纳入产品源码。
- 原始失败报告和日志保持不变，复测证据使用新的 `fix_` 目录。
