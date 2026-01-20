 # MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import numpy as np
from transforms3d import euler
import glob
import os

class tf_publisher(Node):
    def __init__(self):
        
        #Added default values for parameters to avoid warnings
        self.declare_parameter('namespace', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('scan_topic','')
        self.declare_parameter('drive_topic','')
        self.declare_parameter('map_path', '')
        self.declare_parameter('map_img_ext', '')
        self.declare_parameter('kb_teleop', True)
        self.declare_parameter('localize', False)
        self.declare_parameter('run_slam', False)
        self.declare_parameter('slam_maps_dir', '')
        self.declare_parameter('run_ekf', False)


        ego_drive_topic = self.get_parameter('drive_topic')
        self.namespace = self.get_parameter('namespace').value
        ego_odom_topic = self.get_parameter('odom_topic').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
        
        self.vesc_odom_sub = self.create_subscription(
            Odometry,
            ego_odom_topic,
            self.odom_callback,
            10)

    def odom_callback(self, odom_msg):
        self._publish_transforms(odom_msg)
    
    

    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True


    def _publish_transforms(self, odom_msg):

        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)


def main(args=None):
    rclpy.init(args=args)
    hardware_bridge = tf_publisher()
    rclpy.spin(hardware_bridge)

if __name__ == '__main__':
    main()