#/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped 
import numpy as np 

class SimpleController(Node):
    def __inti__(self):
        super(). __init__('simple_controller')

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

    def velCb(self,msg):
        robot_speed = np.array([
            [msg.twist.linear.x],
            [msg.twist.angular.z]
        ])

        wheel_speed = np.matmul((np.linalg.inv(self.speed_conversion)),robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0],wheel_speed[0,0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)


def main():
    rclpy.init()
    simplecontroller = SimpleController()
    rclpy.spin(simplecontroller)
    simplecontroller.destroy_node()
    rclpy.shutdown()

if __name__  == "__main__":
    main()