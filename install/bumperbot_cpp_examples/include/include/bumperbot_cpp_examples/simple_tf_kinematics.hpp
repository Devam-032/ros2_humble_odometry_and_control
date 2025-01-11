#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

class SimpleTFKinematics : public rclcpp::Node
{
public:
    SimpleTFKinematics(const std::string &name);

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
    geometry_msgs::msg::TransformStamped static_tf_broadcaster_stamped;
};

#endif 