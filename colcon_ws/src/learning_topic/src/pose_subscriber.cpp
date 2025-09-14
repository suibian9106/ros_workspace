#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

// 创建一个继承自rclcpp::Node的类
class PoseSubscriber : public rclcpp::Node
{
public:
    // 构造函数，初始化节点名称
    PoseSubscriber() : Node("pose_subscriber")
    {
        // 创建订阅者，话题名/turtle1/pose，队列长度10，设置回调函数
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&PoseSubscriber::pose_callback, this, std::placeholders::_1));
    }

private:
    // 订阅回调函数，处理收到的消息
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) const
    {
        // 打印收到的位置信息
        RCLCPP_INFO(this->get_logger(), "Turtle Pose: x:%0.6f y:%0.6f", msg->x, msg->y);
    }
    
    // 声明订阅者
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建节点并运行
    rclcpp::spin(std::make_shared<PoseSubscriber>());
    
    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}
