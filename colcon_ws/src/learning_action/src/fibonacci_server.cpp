#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

// 使用别名简化类型名称
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciActionServer : public rclcpp::Node {
public:
  explicit FibonacciActionServer() : Node("fibonacci_action_server") {
    // 创建Action服务器
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this,
        "fibonacci", // Action名称
        std::bind(&FibonacciActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&FibonacciActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&FibonacciActionServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Fibonacci Action Server has been started.");
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  // 处理目标请求
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Fibonacci::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d",
                goal->order);
    (void)uuid; // 防止未使用变量警告

    // 验证目标有效性（order必须为正整数）
    if (goal->order <= 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid goal order: %d. Must be a positive integer.",
                  goal->order);
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 处理取消请求
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle; // 防止未使用变量警告
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 处理已接受的目标
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    // 在新线程中执行目标，避免阻塞执行器
    std::thread{
        std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1),
        goal_handle}
        .detach();
  }

  // 执行目标
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // 初始化循环变量
    auto result = std::make_shared<Fibonacci::Result>();
    auto feedback = std::make_shared<Fibonacci::Feedback>();

    // 获取目标请求
    auto goal = goal_handle->get_goal();

    // 初始化斐波那契数列
    feedback->sequence.clear();
    feedback->sequence.push_back(0);
    feedback->sequence.push_back(1);

    // 设置循环频率（1秒一次）
    rclcpp::Rate loop_rate(1s);

    // 计算斐波那契数列
    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // 检查是否被取消
      if (goal_handle->is_canceling()) {
        result->sequence = feedback->sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // 更新斐波那契数列
      feedback->sequence.push_back(feedback->sequence[i] +
                                   feedback->sequence[i - 1]);

      // 发布反馈
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publishing feedback");

      // 等待下一次迭代
      loop_rate.sleep();
    }

    // 检查目标是否完成
    if (rclcpp::ok()) {
      result->sequence = feedback->sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FibonacciActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}