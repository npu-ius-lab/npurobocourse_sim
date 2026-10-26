import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose as GzPose
except ImportError as exc:
    raise ImportError(
        'Gazebo Python bindings not found; install them with: '
        'sudo apt install python3-gz-transport13 python3-gz-msgs10') from exc

from npu_sim_bringup.display_logic import DisplayConfig, validate_external_pose


class ExternalPoseDisplayAdapter(Node):
    """Drives the static npu_drone_display entity from an external
    localization pose topic through the simulator entity-pose service.

    Poses that are non-finite, out of bounds, degenerate, expressed in the
    wrong frame, or stale are rejected; stale input stops entity updates
    and raises an error diagnostic."""

    def __init__(self):
        super().__init__('external_pose_display_adapter')

        self.declare_parameter('external_pose_topic', '/npu/external/drone_pose')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('world_name', 'course_world')
        self.declare_parameter('entity_name', 'npu_drone_display')
        self.declare_parameter('external_pose_timeout_sec', 1.0)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('service_timeout_ms', 500)
        self.declare_parameter('bounds_x', [-12.0, 12.0])
        self.declare_parameter('bounds_y', [-12.0, 12.0])
        self.declare_parameter('bounds_z', [0.0, 6.0])

        def _bounds(name):
            values = list(self.get_parameter(name).value)
            if len(values) != 2 or values[0] >= values[1]:
                raise ValueError(f'{name} must be [min, max] with min < max')
            return (float(values[0]), float(values[1]))

        self._config = DisplayConfig(
            world_frame=self.get_parameter('world_frame').value,
            entity_name=self.get_parameter('entity_name').value,
            bounds_x=_bounds('bounds_x'),
            bounds_y=_bounds('bounds_y'),
            bounds_z=_bounds('bounds_z'),
        )
        self._pose_timeout = self.get_parameter('external_pose_timeout_sec').value
        self._service_timeout_ms = int(self.get_parameter('service_timeout_ms').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        if update_rate_hz <= 0.0:
            raise ValueError('update_rate_hz must be positive')
        world_name = self.get_parameter('world_name').value
        self._set_pose_service = f'/world/{world_name}/set_pose/blocking'

        self._last_pose = None
        self._last_pose_wall = None
        self._last_sent = None
        self._stale = False
        self._last_error = 'waiting for external pose'
        self._gz_node = GzNode()

        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(
            PoseStamped,
            self.get_parameter('external_pose_topic').value,
            self._on_pose, 10)

        self.create_timer(1.0 / update_rate_hz, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f'external_pose_display_adapter started, entity '
            f'{self._config.entity_name} via {self._set_pose_service}')

    def _wall_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg: PoseStamped) -> None:
        position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w)
        error = validate_external_pose(
            position, quaternion, msg.header.frame_id, self._config)
        if error is not None:
            self._last_error = f'rejected external pose: {error}'
            self.get_logger().warn(self._last_error)
            return
        self._last_pose = (position, quaternion)
        self._last_pose_wall = self._wall_now()

    def _fresh_pose(self):
        if self._last_pose is None or self._last_pose_wall is None:
            return None
        if self._wall_now() - self._last_pose_wall > self._pose_timeout:
            return None
        return self._last_pose

    def _tick(self) -> None:
        pose = self._fresh_pose()
        if pose is None:
            if self._last_pose is not None:
                self._stale = True
                self._last_error = 'external pose stale, display update stopped'
            return
        if self._stale:
            self._stale = False
            self._last_error = ''
        (x, y, z), (qx, qy, qz, qw) = pose
        if pose == self._last_sent:
            return
        request = GzPose()
        request.name = self._config.entity_name
        request.position.x = x
        request.position.y = y
        request.position.z = z
        request.orientation.x = qx
        request.orientation.y = qy
        request.orientation.z = qz
        request.orientation.w = qw
        try:
            ok, response = self._gz_node.request(
                self._set_pose_service, request, GzPose, Boolean,
                self._service_timeout_ms)
        except Exception as exc:
            self._last_error = f'set_pose service call failed: {exc}'
            self.get_logger().warn(self._last_error)
            return
        if not ok or not response.data:
            self._last_error = 'set_pose service rejected the display pose'
            self.get_logger().warn(self._last_error)
            return
        self._last_sent = pose
        self._last_error = ''

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'external_pose_display/display'
        status.hardware_id = self._config.entity_name
        if self._fresh_pose() is None:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_error or 'external pose missing'
        elif self._last_error:
            status.level = DiagnosticStatus.WARN
            status.message = self._last_error
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'display entity follows external pose'
        status.values.append(KeyValue(
            key='entity', value=self._config.entity_name))
        array.status.append(status)
        self._diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ExternalPoseDisplayAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
