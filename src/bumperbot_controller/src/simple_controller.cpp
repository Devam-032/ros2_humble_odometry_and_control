#include "bumperbot_controller/simple_controller.hpp"
#include<Eigen/Geometry>  

using std::placeholders::_1;

SimpleController::SimpleController(const std::string &name) : 
Node(name),left_prev_pose(0.0),right_prev_pose(0.0),x(0.0),y(0.0),odom_br(this)
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

    joint_sub = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
    std::bind(&SimpleController::jointCb,this,_1));

    prev_time = get_clock()->now();

    odom_pub = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom",10);

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    

}

void SimpleController::velCb(const geometry_msgs::msg::TwistStamped &msg){
    Eigen::Vector2d robot_speed(msg.twist.linear.x,msg.twist.angular.z);

    Eigen::Vector2d wheel_speed = speed_conversion.inverse()*robot_speed;
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    wheel_cmd_pub->publish(wheel_speed_msg);

}

void SimpleController::jointCb(const sensor_msgs::msg::JointState &msg){
    double dp_left = msg.position[1] - left_prev_pose;
    double dp_right = msg.position[0] - right_prev_pose;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time;

    left_prev_pose = msg.position[1];
    right_prev_pose = msg.position[0];
    prev_time = msg_time;

    double fl = dp_left/dt.seconds();
    double fr = dp_right/dt.seconds();

    double linear = wheel_radius*(fl+fr)/2;
    double angular = wheel_radius*(fr-fl)/wheel_separation;

    RCLCPP_INFO_STREAM(get_logger(), "linear:"<<linear<<" angular:"<<angular);

    theta += angular*dt.seconds();
    x += linear*dt.seconds()*cos(theta);
    y += linear*dt.seconds()*sin(theta);

    RCLCPP_INFO_STREAM(get_logger(),"x:"<<x<<"y:"<<y<<"theta:"<<theta);

    odom.header.stamp = this->get_clock()->now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    q.setRPY(0.0,0.0,theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = linear;
    odom.twist.twist.angular.z = angular;

    odom_pub->publish(odom);

    odom_tf.transform.translation.x = x;
    odom_tf.transform.translation.y = y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();
    odom_tf.header.stamp = this->get_clock()->now();

    odom_br.sendTransform(odom_tf);

}

int main(int argc, char* argv[]){
    
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}