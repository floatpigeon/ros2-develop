#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher"), count_(0)
    {
        // 创建发布者，发布std_msgs::msg::String类型消息到"chatter"话题
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        
        // 创建定时器，每500ms发布一次消息
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SimplePublisher::publish_message, this));
            
        RCLCPP_INFO(this->get_logger(), "Publisher node initialized");
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS2! Count: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
