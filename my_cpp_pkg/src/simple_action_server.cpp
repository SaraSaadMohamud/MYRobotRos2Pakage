#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "ros2_msgs/action/fibonacci.hpp"
#include <memory>

using namespace std::placeholders;
using namespace std; 

namespace my_cpp_pkg
{
class SimpleActionServer : public rclcpp::Node
{
public:
    explicit SimpleActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("simple_action_server", options)
    {
        action_server_ = rclcpp_action::create_server<ros2_msgs::action::Fibonacci>(
            this, "fibonacci",
            std::bind(&SimpleActionServer::GoalCallback, this, _1, _2),
            std::bind(&SimpleActionServer::CancleCallback, this, _1),
            std::bind(&SimpleActionServer::AcceptedCallback, this, _1)
        );

        RCLCPP_INFO(get_logger(), "Starting the action Server");
    }

private:
    rclcpp_action::Server<ros2_msgs::action::Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse GoalCallback(const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ros2_msgs::action::Fibonacci::Goal> goal)
    {
        (void)uuid; 

        RCLCPP_INFO(get_logger(), "Recieved goal request with ordwe %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void AcceptedCallback(std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_msgs::action::Fibonacci>> goal_handle)
    {
        std::thread{std::bind(&SimpleActionServer::Execute, this, _1), goal_handle}.detach();
    }

   void Execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_msgs::action::Fibonacci>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(1);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ros2_msgs::action::Fibonacci::Feedback>();
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    auto result = std::make_shared<ros2_msgs::action::Fibonacci::Result>();
    for (int i = 1; i < goal->order && rclcpp::ok(); ++i)
    {
        if (goal_handle->is_canceling())
        {
            result->sequence = sequence;
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal Canceled");
            return;
        }
        sequence.push_back(sequence[i] + sequence[i-1]);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "Publishing Feedback");

        loop_rate.sleep();
    }

    if (rclcpp::ok())
    {
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal Succeeded");
    }
    }
    rclcpp_action::CancelResponse CancleCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_msgs::action::Fibonacci>> goal_handle)
    {
        (void)goal_handle;   

        RCLCPP_INFO(get_logger(), "Recieve request to cancle the goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
};

} 

RCLCPP_COMPONENTS_REGISTER_NODE(my_cpp_pkg::SimpleActionServer)