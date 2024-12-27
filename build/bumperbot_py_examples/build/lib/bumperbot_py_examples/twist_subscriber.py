import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist

class TwistSub(Node):
    def __init__(self):
        super().__init__('twist_sub')
        self.sub = self.create_subscription(Twist,'cmd_vel',self.twistCb,10)

    def twistCb(self,msg):
        self.get_logger().info(f"The linear velocity is {msg.linear.x} and the angular velocity is {msg.angular.z}")

def main():
    rclpy.init()
    twistsub = TwistSub()
    rclpy.spin(twistsub)
    twistsub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()