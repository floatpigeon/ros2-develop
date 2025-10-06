#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimpleSubscriber : public rclcpp::Node
{
public:
    SimpleSubscriber() : Node("simple_subscriber")
    {
        // 创建订阅者，订阅"chatter"话题
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10, std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Subscriber node initialized");
    }

private:
    // 回调函数，处理接收到的消息
    void topic_callback(const std_msgs::msg::String & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}
