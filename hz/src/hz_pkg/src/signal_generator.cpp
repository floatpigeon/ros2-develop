#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

using namespace std::chrono_literals;

class SignalGenerator : public rclcpp::Node
{
public:
    SignalGenerator() : Node("signal_generator")
    {
        // 创建发布者，分别发布正弦波和方波
        sine_publisher_ = this->create_publisher<std_msgs::msg::Float64>("sine_wave", 10);
        square_publisher_ = this->create_publisher<std_msgs::msg::Float64>("square_wave", 10);
        
        // 以1000Hz的频率发布信号
        timer_ = this->create_wall_timer(
            5ms, std::bind(&SignalGenerator::timer_callback, this));
            
        // 初始化时间
        start_time_ = this->now();
    }

private:
    void timer_callback()
    {
        double t = (this->now() - start_time_).seconds();
        
        double sine_value = std::sin(2 * M_PI * 10 * t);
        
        // 生成1Hz方波:
        double square_value = (std::sin(2 * M_PI * 1 * t) >= 0) ? 1.0 : -1.0;
        
        // 发布正弦波
        auto sine_msg = std_msgs::msg::Float64();
        sine_msg.data = sine_value;
        sine_publisher_->publish(sine_msg);
        
        // 发布方波
        auto square_msg = std_msgs::msg::Float64();
        square_msg.data = square_value;
        square_publisher_->publish(square_msg);
    
        if (count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Sine: %.2f, Square: %.2f", sine_value, square_value);
        }
        count_++;
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sine_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_publisher_;
    rclcpp::Time start_time_;
    size_t count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}
    