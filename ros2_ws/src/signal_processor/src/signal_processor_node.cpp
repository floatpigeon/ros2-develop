#include "rclcpp/rclcpp.hpp"
#include "signal_processor/msg/signal.hpp"
#include "std_msgs/msg/float64.hpp"

class SignalProcessor : public rclcpp::Node
{
public:
    SignalProcessor() : Node("signal_processor")
    {
        // 创建订阅者
        subscriber_ = this->create_subscription<signal_processor::msg::Signal>(
            "raw_signals", 10, std::bind(&SignalProcessor::process_signals, this, std::placeholders::_1));
            
        // 创建发布者（处理后的信号）
        processed_pub_ = this->create_publisher<std_msgs::msg::Float64>("processed_signal", 10);
        // 用于可视化原始正弦信号
        sine_pub_ = this->create_publisher<std_msgs::msg::Float64>("sine_signal", 10);
        // 用于可视化方波信号
        square_pub_ = this->create_publisher<std_msgs::msg::Float64>("square_signal", 10);
            
        RCLCPP_INFO(this->get_logger(), "Signal processor initialized");
    }

private:
    void process_signals(const signal_processor::msg::Signal & msg) const
    {
        // 当正弦信号与方波信号同号时，输出原正弦信号，否则输出0
        double processed = (msg.sine * msg.square > 0) ? msg.sine : 0.0;
        
        // 发布处理后的信号
        auto processed_msg = std_msgs::msg::Float64();
        processed_msg.data = processed;
        processed_pub_->publish(processed_msg);
        
        // 发布原始信号用于可视化
        auto sine_msg = std_msgs::msg::Float64();
        sine_msg.data = msg.sine;
        sine_pub_->publish(sine_msg);
        
        auto square_msg = std_msgs::msg::Float64();
        square_msg.data = msg.square;
        square_pub_->publish(square_msg);
    }
    
    rclcpp::Subscription<signal_processor::msg::Signal>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr processed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sine_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}
