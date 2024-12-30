#include "bumperbot_cpp_examples/turtle_ref_pose.hpp"

using std::placeholders::_1;

TurtleRefPose::TurtleRefPose(const std::string &name) : Node(name)
{
    sub1 = create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,
    std::bind(&TurtleRefPose::posecb0,this,_1));
    sub2 = create_subscription<turtlesim::msg::Pose>("/turtle2/pose",10,
    std::bind(&TurtleRefPose::posecb1,this,_1));
    pub = create_publisher<turtlesim::msg::Pose>("/turtle_distance", 10);
};

void TurtleRefPose::posecb0(const turtlesim::msg::Pose & pose){
    turtle1_pose = pose;
}

void TurtleRefPose::posecb1(const turtlesim::msg::Pose & pose){
    turtle2_pose = pose;
    float Tx = turtle1_pose.x - turtle2_pose.x;
    float Ty = turtle1_pose.y - turtle2_pose.y;
    turtlesim::msg::Pose msg;
    msg.x = Tx;
    msg.y = Ty;
    pub->publish(msg);
}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleRefPose>("turtle_ref");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}