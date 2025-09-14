#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// 创建一个继承自rclcpp::Node的类
class VelocityPublisher : public rclcpp::Node
{
public:
    // 构造函数，初始化节点名称
    VelocityPublisher() : Node("velocity_publisher")
    {
        // 创建发布者，话题名/turtle1/cmd_vel，消息类型Twist，队列长度10
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        // 创建定时器，设置回调函数和周期(10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 100ms = 10Hz
            std::bind(&VelocityPublisher::timer_callback, this));
    }

private:
    // 定时器回调函数，定期发布消息
    void timer_callback()
    {
        // 创建消息对象
        auto msg = geometry_msgs::msg::Twist();
        // 设置线速度和角速度
        msg.linear.x = 0.5;    // x方向线速度
        msg.angular.z = 0.2;   // z方向角速度
        
        // 发布消息
        publisher_->publish(msg);
        // 打印日志信息
        RCLCPP_INFO(this->get_logger(), "陀螺在旋转");
    }
    
    // 声明发布者和定时器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建节点并运行
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    
    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}
