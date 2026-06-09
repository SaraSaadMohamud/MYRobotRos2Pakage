#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_msgs/action/fibonacci.hpp"

namespace my_cpp_pkg
{
class SimpleActionClient : public rclcpp::Node
{
public:
    explicit SimpleActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("simple_action_client_node", options)
    {
        client_ = rclcpp_action::create_client<ros2_msgs::action::Fibonacci>(this, "fibonacci");
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SimpleActionClient::TimerCallback, this));
    }

private:
    rclcpp_action::Client<ros2_msgs::action::Fibonacci>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void TimerCallback()
    {
        timer_->cancel();

        if (!client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(get_logger(), "Action Server not available after waiting");
            return;  
        }

        auto goal_msg = ros2_msgs::action::Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO(get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<ros2_msgs::action::Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](rclcpp_action::ClientGoalHandle<ros2_msgs::action::Fibonacci>::SharedPtr goal_handle)
            {
                GoalCallback(goal_handle);
            };

        send_goal_options.feedback_callback =
            [this](
                rclcpp_action::ClientGoalHandle<ros2_msgs::action::Fibonacci>::SharedPtr goal_handle,
                std::shared_ptr<const ros2_msgs::action::Fibonacci::Feedback> feedback)
            {
                FeedbackCallback(goal_handle, feedback);
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<ros2_msgs::action::Fibonacci>::WrappedResult & result)
            {
                ResultCallback(result);
            };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void GoalCallback(rclcpp_action::ClientGoalHandle<ros2_msgs::action::Fibonacci>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by the server");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal is accepted by the Server");
        }
    }

    void FeedbackCallback(
        rclcpp_action::ClientGoalHandle<ros2_msgs::action::Fibonacci>::SharedPtr,
        std::shared_ptr<const ros2_msgs::action::Fibonacci::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in the sequence received: ";
        for (auto number : feedback->partial_sequence)
        {
            ss << number << " ";
        }
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    }

    void ResultCallback(const rclcpp_action::ClientGoalHandle<ros2_msgs::action::Fibonacci>::WrappedResult result)
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
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

        rclcpp::shutdown();
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(my_cpp_pkg::SimpleActionClient)