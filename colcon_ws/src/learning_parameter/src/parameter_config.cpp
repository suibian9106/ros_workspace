#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    int red, green, blue;

    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("parameter_config");
    
    // 创建参数客户端，连接到turtlesim节点的参数服务
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "turtlesim");
    
    // 等待turtlesim节点的参数服务可用
    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for turtlesim parameter service...");
    }

    // 读取背景颜色参数
    red = parameters_client->get_parameter<int>("background_r");
    green = parameters_client->get_parameter<int>("background_g");
    blue = parameters_client->get_parameter<int>("background_b");

    RCLCPP_INFO(node->get_logger(), "Get Background Color: R=%d, G=%d, B=%d", red, green, blue);

    // 设置背景颜色参数
    parameters_client->set_parameters({
        rclcpp::Parameter("background_r", 255),
        rclcpp::Parameter("background_g", 255),
        rclcpp::Parameter("background_b", 255)
    });

    RCLCPP_INFO(node->get_logger(), "Set Background Color to White: R=255, G=255, B=255");

    // 重新读取背景颜色参数
    red = parameters_client->get_parameter<int>("background_r");
    green = parameters_client->get_parameter<int>("background_g");
    blue = parameters_client->get_parameter<int>("background_b");

    RCLCPP_INFO(node->get_logger(), "Re-get Background Color: R=%d, G=%d, B=%d", red, green, blue);

    // 创建服务客户端，调用/clear服务
    auto clear_client = node->create_client<std_srvs::srv::Empty>("/clear");
    
    // 等待clear服务可用
    while (!clear_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for clear service...");
    }

    // 调用clear服务
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = clear_client->async_send_request(request);

    // 等待服务调用完成
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Clear service called successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call clear service");
    }

    rclcpp::sleep_for(1s);
    rclcpp::shutdown();

    return 0;
}