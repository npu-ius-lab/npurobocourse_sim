import rclpy
from rclpy.node import Node
from rclpy.time import Time

from apriltag_msgs.msg import AprilTagDetectionArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo
import tf2_ros

from npu_apriltag.detection_logic import mount_quaternion, select_detection


class DetectionAdapter(Node):
    """Normalizes apriltag_ros detections to a world-frame marker pose.

    Publishes only geometry_msgs/PoseStamped on /npu/marker/pose. Whenever
    the image, camera info, drone pose, TF chain, or the selected marker is
    missing or stale, no pose is published and a diagnostic explains why.
    """

    def __init__(self, **kwargs):
        super().__init__('detection_adapter', **kwargs)

        self.declare_parameter('image_topic', '/npu/drone/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/npu/drone/camera/camera_info')
        self.declare_parameter('detections_topic', '/npu/drone/detections')
        self.declare_parameter('drone_pose_topic', '/npu/drone/pose')
        self.declare_parameter('marker_id', 8)
        self.declare_parameter('marker_frame', 'marker')
        self.declare_parameter('camera_frame', 'npu_drone/front_cam')
        self.declare_parameter('base_frame', 'npu_drone/base_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('tf_timeout_sec', 1.0)
        self.declare_parameter('camera_translation', [0.05, 0.0, -0.25])
        self.declare_parameter('camera_rpy', [0.0, 0.2617993877991494, 0.0])

        self._image_topic = self.get_parameter('image_topic').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value
        self._marker_id = self.get_parameter('marker_id').value
        self._marker_frame = self.get_parameter('marker_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._world_frame = self.get_parameter('world_frame').value
        self._pose_timeout = self.get_parameter('pose_timeout_sec').value
        self._tf_timeout = self.get_parameter('tf_timeout_sec').value

        translation = list(self.get_parameter('camera_translation').value)
        camera_rpy = list(self.get_parameter('camera_rpy').value)
        if len(translation) != 3 or len(camera_rpy) != 3:
            raise ValueError('camera_translation and camera_rpy must be 3-element lists')

        self._last_detections = None
        self._last_detections_wall = None
        self._last_camera_info = None
        self._last_camera_info_wall = None
        self._last_drone_pose = None
        self._last_drone_pose_wall = None
        self._failure = 'waiting for input'

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_camera_mount(translation, camera_rpy)

        self._pose_pub = self.create_publisher(PoseStamped, '/npu/marker/pose', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(
            AprilTagDetectionArray,
            self.get_parameter('detections_topic').value,
            self._on_detections, 10)
        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(
            PoseStamped,
            self.get_parameter('drone_pose_topic').value,
            self._on_drone_pose, 10)

        self.create_timer(0.05, self._relay_drone_tf)
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f'detection_adapter started, selecting marker id {self._marker_id}')

    def _wall_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_camera_mount(self, translation, camera_rpy) -> None:
        quat = mount_quaternion(camera_rpy)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._base_frame
        transform.child_frame_id = self._camera_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self._tf_static.sendTransform(transform)

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._last_detections = msg
        self._last_detections_wall = self._wall_now()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._last_camera_info = msg
        self._last_camera_info_wall = self._wall_now()

    def _on_drone_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self._world_frame:
            self.get_logger().warn(
                f'ignoring drone pose in frame {msg.header.frame_id!r}, '
                f'expected {self._world_frame!r}')
            return
        self._last_drone_pose = msg
        self._last_drone_pose_wall = self._wall_now()

    def _relay_drone_tf(self) -> None:
        if not self._fresh(self._last_drone_pose, self._last_drone_pose_wall,
                           self._pose_timeout):
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._world_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._last_drone_pose.pose.position.x
        transform.transform.translation.y = self._last_drone_pose.pose.position.y
        transform.transform.translation.z = self._last_drone_pose.pose.position.z
        transform.transform.rotation = self._last_drone_pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _fresh(self, msg, wall_stamp, timeout) -> bool:
        if msg is None or wall_stamp is None:
            return False
        return (self._wall_now() - wall_stamp) <= timeout

    def _check_inputs(self):
        if not self.get_publishers_info_by_topic(self._image_topic):
            return f'no publisher on image topic {self._image_topic}'
        if not self._fresh(self._last_camera_info, self._last_camera_info_wall,
                           self._pose_timeout):
            return 'camera info missing or stale'
        if self._last_camera_info.header.frame_id != self._camera_frame:
            return (f'camera info frame {self._last_camera_info.header.frame_id!r} '
                    f'!= {self._camera_frame!r}')
        if not self._fresh(self._last_drone_pose, self._last_drone_pose_wall,
                           self._pose_timeout):
            return 'drone pose missing or stale'
        if not self._fresh(self._last_detections, self._last_detections_wall,
                           self._pose_timeout):
            return 'no fresh detections'
        if select_detection(self._last_detections.detections, self._marker_id) is None:
            return f'marker id {self._marker_id} not in current detections'
        return None

    def _tick(self) -> None:
        failure = self._check_inputs()
        if failure is not None:
            self._failure = failure
            return
        try:
            marker_tf = self._tf_buffer.lookup_transform(
                self._camera_frame, self._marker_frame, Time())
            transform = self._tf_buffer.lookup_transform(
                self._world_frame, self._marker_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self._failure = f'marker TF unavailable: {exc}'
            return
        stamp = Time.from_msg(marker_tf.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > self._tf_timeout:
            self._failure = f'marker TF stale ({age:.1f}s old)'
            return

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self._world_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self._pose_pub.publish(pose)
        self._failure = ''

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'detection_adapter/marker_pose'
        status.hardware_id = 'npu_drone'
        if self._failure:
            status.level = DiagnosticStatus.ERROR
            status.message = f'{self._failure}, no marker pose output'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'publishing marker id {self._marker_id} pose in world'
        array.status.append(status)
        self._diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionAdapter()
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
