import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescImuStamped

from transforms3d import euler


class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # Parameters
        self.declare_parameter('namespace', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('drive_topic', '')
        self.declare_parameter('vesc_imu_topic', '/sensors/imu')
        self.declare_parameter('imu_topic', '/imu')

        self.namespace = self.get_parameter('namespace').value
        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        vesc_imu_topic = self.get_parameter('vesc_imu_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        # State
        self.ego_steer = 0.0

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)

        # Subscribers
        self.create_subscription(
            AckermannDriveStamped,
            drive_topic,
            self.drive_callback,
            10
        )

        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.create_subscription(
            VescImuStamped,
            vesc_imu_topic,
            self.vesc_imu_callback,
            10
        )

        self.get_logger().info(
            f"Bridging VESC IMU '{vesc_imu_topic}' → '{imu_topic}'"
        )

    def drive_callback(self, msg):
        self.ego_steer = msg.drive.steering_angle

    def odom_callback(self, msg):
        self.publish_wheel_tf(msg.header.stamp)

    def vesc_imu_callback(self, msg: VescImuStamped):
        imu = Imu()

        imu.header = msg.header

        # Angular velocity
        imu.angular_velocity = msg.imu.angular_velocity

        # Linear acceleration
        imu.linear_acceleration = msg.imu.linear_acceleration

        # Do NOT provide orientation (tell EKF explicitly)
        imu.orientation_covariance[0] = -1.0

        self.imu_pub.publish(imu)

    def publish_wheel_tf(self, stamp):
        ts = TransformStamped()
        quat = euler.euler2quat(0.0, 0.0, self.ego_steer, axes='sxyz')

        ts.header.stamp = stamp
        ts.transform.rotation.w = quat[0]
        ts.transform.rotation.x = quat[1]
        ts.transform.rotation.y = quat[2]
        ts.transform.rotation.z = quat[3]

        ts.header.frame_id = f'{self.namespace}/front_left_hinge'
        ts.child_frame_id = f'{self.namespace}/front_left_wheel'
        self.br.sendTransform(ts)

        ts.header.frame_id = f'{self.namespace}/front_right_hinge'
        ts.child_frame_id = f'{self.namespace}/front_right_wheel'
        self.br.sendTransform(ts)


def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
