#ifndef TURTLE_REF_POSE_HPP
#define TURTLE_REF_POSE_HPP

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

using std::placeholders::_1;

class TurtleRefPose : public rclcpp::Node
{
    public:
        TurtleRefPose(const std::string &name);

    private:
        rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pub;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub1;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub2;

        void posecb0(const turtlesim::msg::Pose & pose);
        void posecb1(const turtlesim::msg::Pose & pose);

        turtlesim::msg::Pose turtle1_pose;
        turtlesim::msg::Pose turtle2_pose;

};

#endif