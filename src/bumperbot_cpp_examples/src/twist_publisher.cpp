#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>

using namespace std::chrono_literals;

class TwistPublisher : public rclcpp::Node
{
    public:
        TwistPublisher() : Node("twist_publisher")
        {
            pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            timer_ = create_wall_timer(500ms,std::bind(&TwistPublisher::timerCallback,this));
            RCLCPP_INFO(get_logger(),"Publishing Twist message at every .5s");
        }

    private:

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
        rclcpp::TimerBase::SharedPtr timer_;

        unsigned int a = 1;
        int b = 2;
        int c = 3;


        void timerCallback(){
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = a;
            message.linear.y = b;
            message.angular.z = c;

            pub->publish(message);
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TwistPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}