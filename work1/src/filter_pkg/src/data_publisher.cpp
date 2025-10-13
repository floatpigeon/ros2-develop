#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <random>

class DataPublisherNode : public rclcpp::Node
{
public:
    DataPublisherNode() : Node("data_publisher")
    {
        // 创建发布者
        raw_data_pub_ = this->create_publisher<std_msgs::msg::Float64>("raw_data", 10);
        
        // 设置发布频率
        double publish_frequency = 100.0;
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(1e6 / publish_frequency)),
            std::bind(&DataPublisherNode::timer_callback, this)
        );
        
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::normal_distribution<double>(0.0, 0.15); 
        
        RCLCPP_INFO(this->get_logger(), "Data publisher node initialized. Publishing to 'raw_data'");
    }

private:
    void timer_callback()
    {
        // 获取当前时间
        auto time = this->now().seconds();
        
        // 生成基础信号：2Hz的正弦波 1Hz的余弦波
        double base_frequency1 = 2.0;
        double base_frequency2 = 1.0;
        double clean_data = 0.7 * sin(2 * M_PI * base_frequency1 * time) + 
                           0.3 * cos(2 * M_PI * base_frequency2 * time);
        
        // 添加随机噪声
        double noise = dist_(gen_);
        double noisy_data = clean_data + noise;
        
        // 发布原始数据
        auto msg = std_msgs::msg::Float64();
        msg.data = noisy_data;
        raw_data_pub_->publish(msg);
    }
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_data_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 gen_;
    std::normal_distribution<double> dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
