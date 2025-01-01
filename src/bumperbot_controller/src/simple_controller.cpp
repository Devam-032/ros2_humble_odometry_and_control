#include "bumperbot_controller/simple_controller.hpp"
#include<Eigen/Geometry>  

using std::placeholders::_1;

SimpleController::SimpleController(const std::string &name) : Node(name)
{
    declare_parameter("wheel_radius",0.033);
    declare_parameter("wheel_separation",0.17);
    
    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_separation = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(),"Using wheel radius"<<wheel_radius);    
    RCLCPP_INFO_STREAM(get_logger(),"Using wheel separation"<<wheel_separation);

    wheel_cmd_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands",10);
    vel_sub = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel",10,
    std::bind(&SimpleController::velCb,this,_1));

    speed_conversion << wheel_radius/2,wheel_radius/2,wheel_radius/wheel_separation,-wheel_radius/wheel_separation;

    RCLCPP_INFO_STREAM(get_logger(),"The conversion matrix is \n " << speed_conversion);

}

void SimpleController::velCb(const geometry_msgs::msg::TwistStamped &msg){
    Eigen::Vector2d robot_speed(msg.twist.linear.x,msg.twist.angular.z);

    Eigen::Vector2d wheel_speed = speed_conversion.inverse()*robot_speed;
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    wheel_cmd_pub->publish(wheel_speed_msg);

}

int main(int argc, char* argv[]){
    
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}