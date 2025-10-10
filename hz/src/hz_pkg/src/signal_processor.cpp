#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <mutex>

class SignalProcessor : public rclcpp::Node
{
public:
    SignalProcessor() : Node("signal_processor")
    {
        // 订阅正弦波信号
        sine_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "sine_wave", 10, 
            std::bind(&SignalProcessor::sine_callback, this, std::placeholders::_1));
            
        // 订阅方波信号
        square_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "square_wave", 10, 
            std::bind(&SignalProcessor::square_callback, this, std::placeholders::_1));
            
        // 创建处理后信号的发布者
        processed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("processed_signal", 10);
        
        latest_sine_ = 0.0;
        latest_square_ = 0.0;
        has_sine_ = false;
        has_square_ = false;
    }

private:
    // 处理正弦波回调
    void sine_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_sine_ = msg->data;
        has_sine_ = true;
        process_signals();
    }
    
    // 处理方波回调
    void square_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_square_ = msg->data;
        has_square_ = true;
        process_signals();
    }
    
    void process_signals()
    {
        // 确保信号收到
        if (!has_sine_ || !has_square_) {
            return;
        }
        
        double processed_value;
        if ((latest_sine_ >= 0 && latest_square_ >= 0) || 
            (latest_sine_ < 0 && latest_square_ < 0)) {
            processed_value = latest_sine_;
        } else {
            processed_value = 0.0;
        }
        
        // 发布处理后的信号
        auto msg = std_msgs::msg::Float64();
        msg.data = processed_value;
        processed_publisher_->publish(msg);
        
        // 每100次处理打印一次状态
        if (count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Sine: %.2f, Square: %.2f, Processed: %.2f",
                      latest_sine_, latest_square_, processed_value);
        }
        count_++;
    }
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sine_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr square_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr processed_publisher_;
    
    // 存储最新的信号值
    double latest_sine_;
    double latest_square_;
    bool has_sine_;
    bool has_square_;
    std::mutex data_mutex_; 
    size_t count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}
    