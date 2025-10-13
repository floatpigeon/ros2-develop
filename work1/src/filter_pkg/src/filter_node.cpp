#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>
#include <algorithm>

class FilterNode : public rclcpp::Node
{
public:
    FilterNode() : Node("filter_node")
    {
        // 中值滤波器参数
        median_window_size_ = 7u;
        
        // 低通滤波器参数
        lowpass_alpha_ = 0.15;
        lowpass_initialized_ = false;
        lowpass_value_ = 0.0;
        
        // 创建订阅者
        raw_data_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "raw_data",
            10,
            std::bind(&FilterNode::data_callback, this, std::placeholders::_1)
        );
        
        // 创建发布者
        median_pub_ = this->create_publisher<std_msgs::msg::Float64>("median_filtered", 10);
        lowpass_pub_ = this->create_publisher<std_msgs::msg::Float64>("lowpass_filtered", 10);
        
        RCLCPP_INFO(this->get_logger(), "Filter node initialized");
        RCLCPP_INFO(this->get_logger(), "Median filter window size: %zu", median_window_size_);
        RCLCPP_INFO(this->get_logger(), "Lowpass filter alpha: %f", lowpass_alpha_);
    }

private:
    void data_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double raw_data = msg->data;
        
        // 中值滤波处理
        process_median_filter(raw_data);
        
        // 低通滤波处理
        process_lowpass_filter(raw_data);
    }
    
    void process_median_filter(double data)
    {
        // 将新数据添加到窗口
        median_window_.push_back(data);
        
        if (median_window_.size() > median_window_size_)
        {
            median_window_.pop_front();
        }
        
        // 窗口填满后才进行滤波
        if (median_window_.size() == median_window_size_)
        {
            std::deque<double> sorted_data = median_window_;
            std::sort(sorted_data.begin(), sorted_data.end());
            
            // 计算中值
            double median;
            if (median_window_size_ % 2 == 1)
            {
                median = sorted_data[median_window_size_ / 2];
            }
            else
            {
                median = (sorted_data[median_window_size_ / 2 - 1] + 
                         sorted_data[median_window_size_ / 2]) / 2.0;
            }
            
            // 发布中值滤波结果
            auto msg = std_msgs::msg::Float64();
            msg.data = median;
            median_pub_->publish(msg);
        }
    }
    
    void process_lowpass_filter(double data)
    {
        // 初始化
        if (!lowpass_initialized_)
        {
            lowpass_value_ = data;
            lowpass_initialized_ = true;
            return;
        }
        
        lowpass_value_ = lowpass_alpha_ * data + (1 - lowpass_alpha_) * lowpass_value_;
        
        // 发布低通滤波结果
        auto msg = std_msgs::msg::Float64();
        msg.data = lowpass_value_;
        lowpass_pub_->publish(msg);
    }
    
    // 订阅者
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr raw_data_sub_;
    
    // 发布者
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr median_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lowpass_pub_;
    
    // 中值滤波器变量
    std::deque<double> median_window_;
    size_t median_window_size_;
    
    // 低通滤波器变量
    double lowpass_alpha_;
    double lowpass_value_;
    bool lowpass_initialized_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}
