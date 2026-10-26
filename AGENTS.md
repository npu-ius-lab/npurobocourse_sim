# AGENTS.md

## Environment

- Target: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.
- Source environments in every ROS shell:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ```
- Use `/usr/bin/python3` for direct pytest/scripts. The Conda Python can lack `numpy`/`yaml`; `conda deactivate` is the alternative.
- `external_pose_display_adapter` also needs `sudo apt install python3-gz-transport13 python3-gz-msgs10`; rosdep does not install these bindings.

## Build and verification

Run the complete verification sequence from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
ros2 run npu_sim_tools validate_readme_commands README.md src
git diff --check
```

Always run `colcon test-result --verbose`; a finished `colcon test` is not sufficient evidence of zero failures. CI additionally requires the two Gazebo Python packages above.

Useful focused commands:

```bash
colcon test --packages-select npu_tracker && colcon test-result --verbose
source install/setup.bash
/usr/bin/python3 -m pytest -q src/npu_mobile_robot/test/test_rviz_config.py
/usr/bin/python3 -m pytest -q src/npu_mobile_robot/test/test_rviz_config.py::test_amcl_has_initial_pose_and_nav2_goal_tools
```

No separate formatter, linter, or pre-commit configuration exists; do not invent one.

## Package boundaries and entrypoints

- `src/npu_sim_description`: world, SDF models and camera presets; no runtime nodes.
- `src/npu_sim_bringup`: `sim.launch.py` / `sim_real.launch.py`, Gazebo spawning and bridges. Adapters: `drone_adapter`, `robot_adapter`, `wait_for_clock`, `external_pose_display_adapter`.
- `src/npu_mobile_robot`: launch/config only for SLAM, AMCL, Nav2 and RViz; no custom runtime nodes.
- `src/npu_tracker`: `mission`, `follow_drone_tf`, `publish_follow_goal`, `tag_tracker`.
- `src/npu_apriltag`: `detection_adapter`, which normalizes detections to `/npu/marker/pose` in `world`.
- `src/npu_sim_tools`: `publish_mock_pose` and `validate_readme_commands`.

`build/`, `install/`, and `log/` are generated; edit only `src/` for implementation. `tasks/` contains plans and acceptance evidence, not runtime code.

## Invariants that must not be broken

- `model.sdf` is the checked-in forward rendering of `model.sdf.template`. Change the template/presets and regenerate or synchronize it; `test_camera_presets.py` enforces exact sync.
- `sim.launch.py` waits for `/clock` before starting adapters. Gazebo `create` succeeds only after entity verification, and display `set_pose` uses the `/blocking` endpoint; queued responses are not proof of execution.
- Preserve public command topics. Robot input `/npu/robot/cmd_vel` passes through `robot_adapter` before the internal `/npu/robot/cmd_vel_gz` bridge. Nav2 controller and `behavior_server` both remap to the public topic.
- `publish_follow_goal` uses `IDLE/SENDING/ACTIVE/CANCELING`. Never clear a goal handle before a confirmed terminal result; replacement waits for the old result. Result failures must not cause concurrent goals.
- AprilTag TF uses the source pose timestamp, and world-marker lookup uses the same detection time; do not restamp with relay time.
- `follow_drone_tf` uses `SignalHandlerOptions.NO`, then publishes zero and waits up to 0.2s for ACK on exit. Preserve this ordering; SIGINT otherwise invalidates the publisher first.
- Frames are `world`, `npu_robot/odom`, and `npu_robot/base_link`, not `map`. Use the local RViz configs; `amcl.rviz` also includes particles, Path and Odometry history. Do not revert to upstream defaults.

## Integration and GUI checks

- Many tests launch real Gazebo/Nav2 rather than mocks. Use a distinct `ROS_DOMAIN_ID` and `GZ_PARTITION` per scenario, and never run duplicate worlds concurrently. Clean up only processes you own.
- Use `headless:=true` for CI/no display. The README maps all nine teaching workflows and their prerequisites.
- On WSL2, use the user's already-running XLaunch; do not restart it or require sudo. Rootless Pillow/Qt captures can be black, so capture Gazebo/RViz evidence through Windows PowerShell `System.Drawing` on the host.
- `docs/gui_smoke_checklist.md` is the visual checklist. `docs/spec-gazebo-readme-acceptance.md` and `docs/spec-fix-gazebo-failed-workflows.md` contain the acceptance and repair plans; use them before repeating acceptance work.
