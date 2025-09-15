#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

// 全局变量，用于存储发布状态
bool pubCommand = false;

// 服务回调函数
bool commandCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  pubCommand = !pubCommand; // 切换发布状态

  // 显示请求数据
  RCLCPP_INFO(rclcpp::get_logger("turtle_command_server"),
              "Publish turtle velocity command [%s]",
              pubCommand ? "Yes" : "No");

  // 设置反馈数据
  response->success = true;
  response->message = "Change turtle command state!";

  return true;
}

int main(int argc, char **argv) {
  // 初始化ROS2节点
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = rclcpp::Node::make_shared("turtle_command_server");

  // 创建服务
  auto service = node->create_service<std_srvs::srv::Trigger>("/turtle_command",
                                                              commandCallback);

  // 创建发布器
  auto publisher =
      node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

  // 循环等待回调函数
  RCLCPP_INFO(node->get_logger(), "Ready to receive turtle command.");

  // 设置循环频率
  rclcpp::Rate loop_rate(10);

  while (rclcpp::ok()) {
    // 查看一次回调函数队列
    rclcpp::spin_some(node);

    // 如果标志为true，则发布速度命令
    if (pubCommand) {
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = 0.5;
      msg.angular.z = 0.2;
      publisher->publish(msg);
    }

    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}