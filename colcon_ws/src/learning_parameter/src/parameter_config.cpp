#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <string>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  int red, green, blue;

  // 初始化ROS2节点
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = std::make_shared<rclcpp::Node>("parameter_config");

  // 读取背景颜色参数
  // ROS2中使用node->get_parameter()方法替代ros::param::get()
  node->get_parameter("/turtlesim/background_r", red);
  node->get_parameter("/turtlesim/background_g", green);
  node->get_parameter("/turtlesim/background_b", blue);

  // ROS2中使用RCLCPP_INFO替代ROS_INFO
  RCLCPP_INFO(node->get_logger(), "Get Background Color: R=%d, G=%d, B=%d", red,
              green, blue);

  // 设置背景颜色参数
  // ROS2中使用node->set_parameter()方法替代ros::param::set()
  node->set_parameter(rclcpp::Parameter("/turtlesim/background_r", 255));
  node->set_parameter(rclcpp::Parameter("/turtlesim/background_g", 255));
  node->set_parameter(rclcpp::Parameter("/turtlesim/background_b", 255));

  RCLCPP_INFO(node->get_logger(),
              "Set Background Color to White: R=255, G=255, B=255");

  // 重新读取背景颜色参数以验证设置
  node->get_parameter("/turtlesim/background_r", red);
  node->get_parameter("/turtlesim/background_g", green);
  node->get_parameter("/turtlesim/background_b", blue);

  RCLCPP_INFO(node->get_logger(), "Re-get Background Color: R=%d, G=%d, B=%d",
              red, green, blue);

  // 创建服务客户端
  auto client = node->create_client<std_srvs::srv::Empty>("/clear");

  // 等待服务可用
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  // 创建请求并调用服务
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = client->async_send_request(request);

  // 等待服务调用完成
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // 等待1秒
  rclcpp::sleep_for(1s);

  // 关闭ROS2
  rclcpp::shutdown();

  return 0;
}