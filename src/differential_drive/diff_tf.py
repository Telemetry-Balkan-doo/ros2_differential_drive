#!/usr/bin/env python
#
# Copyright (C) 2012 Jon Stephan.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# ----------------------------------
# Portions of this code borrowed from the arbotix_python diff_controller.
#
# diff_controller.py - controller for a differential drive
# Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Vanadium Labs LLC nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rclpy
from rclpy.node import Node
from math import sin, cos, atan2
import numpy as np

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped

NS_TO_SEC= 1000000000

int_high = 0
int_low = 32767


class DiffTf(Node):
    """
       diff_tf.py - follows the output of a wheel encoder and
       creates tf and odometry messages.
       some code borrowed from the arbotix diff_controller script
       A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
    """

    def __init__(self):
        super().__init__("diff_tf")
        self.nodename = "diff_tf"
        self.get_logger().info(f"-I- {self.nodename} started")


        self.declare_parameter('wheels_distance', 0.38)
        self.base_width = self.get_parameter('wheels_distance').get_parameter_value().double_value
        self.get_logger().info('Started with base_width: %s' % self.base_width)

        #### parameters #######
        self.rate_hz = self.declare_parameter("rate_hz", 10.0).value # the rate at which to publish the transform   
        self.create_timer(1.0/self.rate_hz, self.update)  #rate_hz = 100 (old)

        self.ticks_meter = float(
            self.declare_parameter('ticks_meter', 15293).value)  # The number of wheel encoder ticks per meter of travel #15293
        # self.base_width = float(self.declare_parameter('base_width', 0.38).value)  # The wheel base width in meters   0.38
        self.get_logger().info(f"ticks_meter: {self.ticks_meter }")

        self.declare_parameter('use_base_link', True)
        self.use_base_link = self.get_parameter('use_base_link').value

        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value

        self.odom_frame_id = self.declare_parameter('odom_frame_id',
                                                    'odom').value  # the name of the odometry reference frame

        self.encoder_min = self.declare_parameter('encoder_min', -2000000000).value
        self.encoder_max = self.declare_parameter('encoder_max', 2000000000).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        # internal data
        self.rticks_per_sec = 0
        self.lticks_per_sec = 0

        self.enc_left = None  # wheel encoder readings
        self.enc_right = None
        self.left = 0.0  # actual values coming back from robot
        self.right = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now()
        
        # !!! addading deserialize parametrs x y theta (th)

        # subscriptions
        self.create_subscription(Int32, "lticks_per_sec", self.lticks_listener_calback, 10)
        self.create_subscription(Int32, "rticks_per_sec", self.rticks_listener_calback, 10)
        self.create_subscription(Int32, "lwheel", self.lwheel_callback, 10)
        self.create_subscription(Int32, "rwheel", self.rwheel_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.initialpose_calback, 10)

        self.create_subscription(
            Float64,
            'o2d_set_wheels_distance',
            self.__o2d_wheels_distance_subscriber_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)

    def __o2d_wheels_distance_subscriber_callback(self, msg):
        wheels_distance = np.array(msg.data)
        self.get_logger().info('%s' % wheels_distance)
        self.base_width = wheels_distance

    def update(self):
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / NS_TO_SEC

        # calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0

            d_vel_left = 0
            d_vel_right = 0

        else:
        # #############################################################
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
            d_vel_left = (self.lticks_per_sec) / self.ticks_meter
            d_vel_right = (self.rticks_per_sec) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right
        # #############################################################

        # self.rticks_per_sec = 0
        # self.lticks_per_sec = 0

        # distance traveled is the average of the two wheels 
        d = (d_left + d_right) / 2
        d_vel = (d_vel_left + d_vel_right) / 2

        # this approximation works (in radians) for small angles
        th = (d_right - d_left) / self.base_width
        th_vel = (d_vel_right - d_vel_left) / self.base_width
        # #############################################################

        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed

        # #############################################################

        if d != 0:
            # calculate distance traveled in x and y
            x = cos(th) * d
            y = -sin(th) * d
            # calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        # transform_stamped_msg = TransformStamped()
        # transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        # # transform_stamped_msg.header.frame_id = self.base_frame_id
        # # transform_stamped_msg.child_frame_id = self.odom_frame_id
        # transform_stamped_msg.header.frame_id = self.odom_frame_id
        # transform_stamped_msg.child_frame_id = self.base_frame_id
        # transform_stamped_msg.transform.translation.x = self.x
        # transform_stamped_msg.transform.translation.y = self.y
        # transform_stamped_msg.transform.translation.z = 0.0
        # transform_stamped_msg.transform.rotation.x = quaternion.x
        # transform_stamped_msg.transform.rotation.y = quaternion.y
        # transform_stamped_msg.transform.rotation.z = quaternion.z
        # transform_stamped_msg.transform.rotation.w = quaternion.w

        # self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = d_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = th_vel
        self.odom_pub.publish(odom)


    def lwheel_callback(self, msg):
        enc = msg.data
       
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult = self.lmult + 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult = self.lmult - 1
    
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc
           
    def rwheel_callback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult = self.rmult + 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult = self.rmult - 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

    
    def lticks_listener_calback(self, msg):
        self.lticks_per_sec = msg.data

    def rticks_listener_calback(self, msg):
        self.rticks_per_sec = msg.data

    def initialpose_calback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.th = self.quaternion_to_theta(self.x, self.y)

    def quaternion_to_theta(self, q_z, q_w):
        return 2 * atan2(q_z, q_w)
    
def main(args=None):
    rclpy.init(args=args)
    diff_tf = DiffTf()

    try:
        rclpy.spin(diff_tf)
    except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
        pass

    diff_tf.destroy_node()
    if rclpy.get_default_context().ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
