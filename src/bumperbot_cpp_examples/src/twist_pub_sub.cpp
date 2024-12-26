#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include<chrono>

using std::placeholders::_1;

class Twistpubsub : public rclcpp::Node
{
    public:
    Twistpubsub() : Node("twist_pub_sub")
    {
        pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel1",10);
        sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,std::bind(&Twistpubsub::twistCB,this,_1));
    }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;

        void twistCB(const geometry_msgs::msg::Twist &msg) const
        {
            double a = 10*(msg.linear.x);
            double b = 10*(msg.angular.z);

            auto message = geometry_msgs::msg::Twist();
            message.linear.x = a;
            message.angular.z = b;
            pub->publish(message);
        }
};

int main(int argc, char** argv)
{   
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Twistpubsub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}