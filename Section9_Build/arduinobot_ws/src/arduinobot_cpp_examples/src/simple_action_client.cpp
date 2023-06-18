#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/fibonacci.hpp"

#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace arduinobot_cpp_examples
{
class SimpleActionClient : public rclcpp::Node
{
public:
  explicit SimpleActionClient(const rclcpp::NodeOptions& options)
    : Node("simple_action_client", options)
  {
    client_ = rclcpp_action::create_client<arduinobot_msgs::action::Fibonacci>(this, "fibonacci");
    timer_ = create_wall_timer(1s, std::bind(&SimpleActionClient::timerCallback, this));
  }

private:
  rclcpp_action::Client<arduinobot_msgs::action::Fibonacci>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timerCallback()
  {
    timer_->cancel();

    if (!client_->wait_for_action_server())
    {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = arduinobot_msgs::action::Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<arduinobot_msgs::action::Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&SimpleActionClient::goalCallback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&SimpleActionClient::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&SimpleActionClient::resultCallback, this, _1);
    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goalCallback(const rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedbackCallback(rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::SharedPtr,
                        const std::shared_ptr<const arduinobot_msgs::action::Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
  }

  void resultCallback(
      const rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};
}  // namespace arduinobot_cpp_examples

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::SimpleActionClient)