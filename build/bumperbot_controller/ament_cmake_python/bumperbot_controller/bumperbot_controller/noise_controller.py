#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped 
import numpy as np 
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from math import cos,sin
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class NoisyController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        self.declare_parameter("wheel_radius",0.033)
        self.declare_parameter("wheel_separation",0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel_radius : {self.wheel_radius}")
        self.get_logger().info(f"Using wheel_separation : {self.wheel_separation}")

        self.left_prev_pose = 0.0
        self.right_prev_pose = 0.0
        self.prev_t = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0

        self.joint_state_sub = self.create_subscription(JointState,"joint_states",self.jointCB,10)
        self.odom_pub = self.create_publisher(Odometry,"bumperbot_odom_noisy",10)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint_ekf"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 0.0
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0
        
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        self.prev_time_ = self.get_clock().now()


    def jointCB(self,msg):

        left_curr_pose = msg.position[1] + np.random.normal(0,0.005)
        right_curr_pose = msg.position[0] + np.random.normal(0,0.005)
        curr_t = Time.from_msg(msg.header.stamp)

        dl = left_curr_pose - self.left_prev_pose
        dr = right_curr_pose - self.right_prev_pose
        dt = curr_t - self.prev_t

        dt = (dt.nanoseconds/S_TO_NS)

        fi_left = dl/dt
        fi_right = dr/dt

        linear = self.wheel_radius*(fi_right+fi_left)/2
        angular = self.wheel_radius*(fi_right-fi_left)/self.wheel_separation

        d_s = linear*dt
        d_theta = angular*dt

        self.theta+=d_theta
        self.x += d_s*cos(self.theta)
        self.y += d_s*sin(self.theta)

        quaternion = quaternion_from_euler(0,0,self.theta)

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular

        self.transform_stamped_.transform.translation.x = self.x
        self.transform_stamped_.transform.translation.y = self.y
        self.transform_stamped_.transform.rotation.x = quaternion[0]
        self.transform_stamped_.transform.rotation.y = quaternion[1]
        self.transform_stamped_.transform.rotation.z = quaternion[2]
        self.transform_stamped_.transform.rotation.w = quaternion[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)

        # self.get_logger().info(f"Linear velocity: {linear} and angular velocity: {angular}")
        # self.get_logger().info(f"x_pos: {self.x} y_pos: {self.y} orientation = {self.theta}")

        self.get_logger().info(f"The message is being published on the bumperbot_odom topic.")

        self.odom_pub.publish(self.odom_msg)
        

def main():
    rclpy.init()
    noisyController = NoisyController()
    rclpy.spin(noisyController)
    noisyController.destroy_node()
    rclpy.shutdown()

if __name__  == "__main__":
    main()