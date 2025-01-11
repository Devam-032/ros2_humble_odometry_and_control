#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped 
import numpy as np 
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        self.declare_parameter("wheel_radius",0.033)
        self.declare_parameter("wheel_separation",0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel_radius : {self.wheel_radius}")
        self.get_logger().info(f"Using wheel_separation : {self.wheel_separation}")

        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray,"simple_velocity_controller/commands",10)
        self.vel_sub = self.create_subscription(TwistStamped,"bumperbot_controller/cmd_vel",self.velCb,10)

        self.speed_conversion = np.array([
            [ self.wheel_radius/2 , self.wheel_radius/2 ],
            [ self.wheel_radius/self.wheel_separation , -self.wheel_radius/self.wheel_separation ]
        ])

        self.get_logger().info(f"The conversion matrix is : {self.speed_conversion}")

        self.left_prev_pose = 0.0
        self.right_prev_pose = 0.0
        self.prev_t = self.get_clock().now()

        self.joint_state_sub = self.create_subscription(JointState,"joint_states",self.jointCB,10)

    def velCb(self,msg):
        robot_speed = np.array([
            [msg.twist.linear.x],
            [msg.twist.angular.z]
        ])

        wheel_speed = np.matmul((np.linalg.inv(self.speed_conversion)),robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0],wheel_speed[0,0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)

    def jointCB(self,msg):
        left_curr_pose = msg.position[1]
        right_curr_pose = msg.position[0]
        curr_t = Time.from_msg(msg.header.stamp)

        dl = left_curr_pose - self.left_prev_pose
        dr = right_curr_pose - self.right_prev_pose
        dt = curr_t - self.prev_t

        fi_left = dl/(dt.nanoseconds/S_TO_NS)
        fi_right = dr/(dt.nanoseconds/S_TO_NS)

        linear = self.wheel_radius*(fi_right+fi_left)/2
        angular = self.wheel_radius*(fi_right-fi_left)/self.wheel_separation

        self.get_logger().info(f"Linear velocity: {linear} and angular velocity: {angular}")

def main():
    rclpy.init()
    simplecontroller = SimpleController()
    rclpy.spin(simplecontroller)
    simplecontroller.destroy_node()
    rclpy.shutdown()

if __name__  == "__main__":
    main()