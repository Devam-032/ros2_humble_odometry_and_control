import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster,TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform

class SimpleKinematics(Node):
    def __init__(self):
        super().__init__('simple_kinematics_tf')

        self.last_x = 0.0
        self.last_increment = 0.05

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        self.static_tf_stamped = TransformStamped()
        self.dynamic_tf_stamped = TransformStamped()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

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

        self.get_transform_srv_ = self.create_service(GetTransform,"get_transform",self.getTfCb)
    
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

    def getTfCb(self,req,res):
        self.get_logger().info("Requested Transform between %s and %s" % (req.frame_id,req.child_frame_id))
        
        requested_transform = TransformStamped()

        try:
            requested_transform = self.tf_buffer.lookup_transform(req.frame_id,req.child_frame_id,rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("An error occurred while transforming %s and %s" % (req.frame_id,req.child_frame_id))
            res.success = False 
            return res
        res.transform = requested_transform
        res.success = True
        return res     

def main():
    rclpy.init()
    node_ = SimpleKinematics()
    rclpy.spin(node_)
    node_.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main() 