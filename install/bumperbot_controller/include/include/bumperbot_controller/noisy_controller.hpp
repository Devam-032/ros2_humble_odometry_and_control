#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>

class NoisyController : public rclcpp::Node
{
    public:
        NoisyController(const std::string &name);

    private:
        void jointCb(const sensor_msgs::msg::JointState &msg);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        double wheel_radius;
        double wheel_separation;

        double left_prev_pose;
        double right_prev_pose;
        rclcpp::Time prev_time;

        double x;
        double y;
        double theta;

        tf2::Quaternion q;
        nav_msgs::msg::Odometry odom;
        tf2_ros::TransformBroadcaster odom_br;
        geometry_msgs::msg::TransformStamped odom_tf;
};

#endif