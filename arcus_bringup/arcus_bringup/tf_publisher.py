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
        self.declare_parameter('namespace', 'ego_racecar')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('ekf_odom_topic', '/odometry/filtered')
        self.declare_parameter('slam_map_topic', '/slam_map')
        self.declare_parameter(
            'map_path',
            '/home/arcus/arcus/slam_map_saver/slam_maps/slam_map_20260325_015852'
        )
        self.declare_parameter(
            'slam_maps_dir',
            '/home/arcus/arcus/slam_map_saver/slam_maps'
        )
        self.declare_parameter('map_img_ext', '.pgm')
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)
        self.declare_parameter('kb_teleop', True)
        self.declare_parameter('localize', False)
        self.declare_parameter('run_slam', True)
        self.declare_parameter('run_ekf', True)
        self.declare_parameter('pure_pursuit', False)
        self.declare_parameter('disparity', True)
        


        self.namespace = self.get_parameter('namespace').value
        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        # State
        self.ego_steer = 0.0

        # TF broadcaster
        self.br = TransformBroadcaster(self)

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

    def drive_callback(self, msg):
        self.ego_steer = msg.drive.steering_angle

    def odom_callback(self, msg):
        self.publish_wheel_tf(msg.header.stamp)

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
