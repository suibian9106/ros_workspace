#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
#include <future>

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciActionClient : public rclcpp::Node {
public:
  explicit FibonacciActionClient()
      : Node("fibonacci_action_client"), goal_done_(false) {
    // 创建Action客户端
    this->client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

    // 设置定时器，在服务器可用后发送目标
    timer_ = this->create_wall_timer(
        1s, std::bind(&FibonacciActionClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  // 发送目标
  void send_goal() {
    using namespace std::placeholders;

    // 取消定时器，只发送一次目标
    timer_->cancel();

    // 等待Action服务器
    if (!this->client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    // 创建目标
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10; // 计算斐波那契数列的前10个数字

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // 设置发送目标的选项
    auto send_goal_options =
        rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);

    // 异步发送目标
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  // 目标响应回调
  void goal_response_callback(
      std::shared_future<GoalHandleFibonacci::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  // 反馈回调
  void
  feedback_callback(GoalHandleFibonacci::SharedPtr,
                    const std::shared_ptr<const Fibonacci::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Next number in sequence: %ld",
                feedback->partial_sequence.back());
  }

  // 结果回调
  void result_callback(const GoalHandleFibonacci::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    // 打印结果
    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->sequence) {
      RCLCPP_INFO(this->get_logger(), "%ld", number);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<FibonacciActionClient>();

  // 等待结果
  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}