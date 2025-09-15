#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  // 初始化ROS2节点
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = std::make_shared<rclcpp::Node>("turtle_spawn");

  // 创建服务客户端
  auto client = node->create_client<turtlesim::srv::Spawn>("/spawn");

  // 等待服务可用
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  // 设置请求数据
  auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
  request->x = 2.0;
  request->y = 2.0;
  request->name = "turtle2";

  // 发送请求并等待响应
  RCLCPP_INFO(node->get_logger(),
              "Call service to spawn turtle[x: %.2f, y: %.2f, name: %s]",
              request->x, request->y, request->name.c_str());

  auto result = client->async_send_request(request);

  // 等待响应结果
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Spawn turtle successfully [name: %s]",
                result.get()->name.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service /spawn");
  }

  rclcpp::shutdown();
  return 0;
}