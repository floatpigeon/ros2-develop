#include "rclcpp/rclcpp.hpp"
#include "signal_processor/msg/signal.hpp"
#include <cmath>

using namespace std::chrono_literals;

class SignalGenerator : public rclcpp::Node
{
public:
    SignalGenerator() : Node("signal_generator"), time_(0.0)
    {
        // 创建发布者，发布频率500Hz
        publisher_ = this->create_publisher<signal_processor::msg::Signal>("raw_signals", 10);
        
        // 500Hz定时器
        timer_ = this->create_wall_timer(
            2ms, std::bind(&SignalGenerator::generate_signals, this));
            
        RCLCPP_INFO(this->get_logger(), "Signal generator initialized (500Hz)");
    }

private:
    void generate_signals()
    {
        // 计算时间（秒）
        time_ += 0.002;  // 2ms = 1/500秒
        
        // 生成10Hz正弦信号 (sin(2πft))
        double sine = std::sin(2 * M_PI * 10 * time_);
        
        // 生成1Hz方波信号 (±1)
        double square = (std::fmod(time_, 1.0) < 0.5) ? 1.0 : -1.0;
        
        // 创建消息并发布
        auto msg = signal_processor::msg::Signal();
        msg.sine = sine;
        msg.square = square;
        msg.time = time_;
        publisher_->publish(msg);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<signal_processor::msg::Signal>::SharedPtr publisher_;
    double time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}
