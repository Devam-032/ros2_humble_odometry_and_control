import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleKinematics(Node):
    def __init__(self):
        super().__init__('simple_kinematics_tf')

        self.last_x = 0.0
        self.last_increment = 0.05

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        self.static_tf_stamped = TransformStamped()
        self.dynamic_tf_stamped = TransformStamped()

        self.static_tf_stamped.header.stamp = self.get_clock().now().to_msg()
        self.static_tf_stamped.header.frame_id = "bumperbot_base"
        self.static_tf_stamped.child_frame_id = "bumperbot_top"
        self.static_tf_stamped.transform.translation.x = 0.0 
        self.static_tf_stamped.transform.translation.y = 0.0
        self.static_tf_stamped.transform.translation.z = 0.3
        self.static_tf_stamped.transform.rotation.x = 0.0
        self.static_tf_stamped.transform.rotation.z = 0.0
        self.static_tf_stamped.transform.rotation.y = 0.0
        self.static_tf_stamped.transform.rotation.w = 1.0

        self.dynamic_tf_stamped.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tf_stamped.header.frame_id = "odom"
        self.dynamic_tf_stamped.child_frame_id = "bumperbot_base"
        

        self.static_tf_broadcaster.sendTransform(self.static_tf_stamped)

        self.timer_ = self.create_timer(0.1,self.timerCb)
    
    def timerCb(self):

        self.dynamic_tf_stamped.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tf_stamped.header.frame_id = "odom"
        self.dynamic_tf_stamped.child_frame_id = "bumperbot_base"
        self.dynamic_tf_stamped.transform.translation.x = self.last_x + self.last_increment
        self.dynamic_tf_stamped.transform.translation.y = 0.0
        self.dynamic_tf_stamped.transform.translation.z = 0.0
        self.dynamic_tf_stamped.transform.rotation.x = 0.0
        self.dynamic_tf_stamped.transform.rotation.y = 0.0
        self.dynamic_tf_stamped.transform.rotation.z = 0.0
        self.dynamic_tf_stamped.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_tf_stamped)

        self.last_x = self.dynamic_tf_stamped.transform.translation.x

def main():
    rclpy.init()
    node_ = SimpleKinematics()
    rclpy.spin(node_)
    node_.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main() 