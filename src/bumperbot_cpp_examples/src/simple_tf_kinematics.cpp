#include "bumperbot_cpp_examples/simple_tf_kinematics.hpp"

SimpleTFKinematics::SimpleTFKinematics(const std::string &name)
    :Node(name)
{
    static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    static_tf_broadcaster_stamped.header.stamp = get_clock()->now();
    static_tf_broadcaster_stamped.header.frame_id = "bumperbot_base";
    static_tf_broadcaster_stamped.child_frame_id = "bumperbot_top";
    static_tf_broadcaster_stamped.transform.translation.x = 0.0;
    static_tf_broadcaster_stamped.transform.translation.y = 0.0;
    static_tf_broadcaster_stamped.transform.translation.z = 0.3;
    static_tf_broadcaster_stamped.transform.rotation.x = 0.0;
    static_tf_broadcaster_stamped.transform.rotation.y = 0.0;
    static_tf_broadcaster_stamped.transform.rotation.z = 0.0;
    static_tf_broadcaster_stamped.transform.rotation.w = 1.0;

    static_tf_broadcaster -> sendTransform(static_tf_broadcaster_stamped);

    RCLCPP_INFO_STREAM(get_logger(),"Publishing static transform between " <<static_tf_broadcaster_stamped.header.frame_id <<" and "<<static_tf_broadcaster_stamped.child_frame_id);

}

int main(int argc, char* argv[])    
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleTFKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}