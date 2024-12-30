import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from math import cos,sin

class SimpleTurtlesimKinematics(Node):

    def __init__(self):
        super().__init__("turtlesim_ref_pose")
        self.sub1 = self.create_subscription(Pose,"/turtle1/pose",self.posecb0,10)
        self.sub2 = self.create_subscription(Pose,"/turtle2/pose",self.posecb1,10)

        self.pub_msg = self.create_publisher(Pose,"turtle_ref/pose",10)

        self.last_turtle1_pose = Pose()
        self.last_turtle2_pose = Pose()

    def posecb0(self,msg):
        self.last_turtle1_pose = msg
        
    def posecb1(self,msg):
        self.last_turtle2_pose = msg
        Tx = self.last_turtle1_pose.x - self.last_turtle2_pose.x
        Ty = self.last_turtle1_pose.y - self.last_turtle2_pose.y
        theta = self.last_turtle1_pose.theta - self.last_turtle2_pose.theta
        self.get_logger().info(f"""\n
            Rotation_matrix = |{cos(theta)}| |{-sin(theta)}|
                              |{sin(theta)}| |{cos(theta)}|  """)
        msgx = Pose()
        msgx.x = Tx
        msgx.y = Ty
        self.pub_msg.publish(msgx)

def main():
    rclpy.init()
    simpleTurtlesimKinematics = SimpleTurtlesimKinematics()
    rclpy.spin(simpleTurtlesimKinematics)
    simpleTurtlesimKinematics.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()