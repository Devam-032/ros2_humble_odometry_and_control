import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Simplepublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")

        self.pub = self.create_publisher(String,'chatter',10)

        self.counter = 0
        self.frequency = 1.0

        self.get_logger().info("Publishing at %d Hz"%self.frequency)

        self.timer=self.create_timer(self.frequency,self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data =  "Hello Ros2 - counter: %d"%self.counter

        self.pub.publish(msg)
        self.counter+=1

def main():
    rclpy.init()
    simplepublisher = Simplepublisher()
    rclpy.spin(simplepublisher)
    simplepublisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()