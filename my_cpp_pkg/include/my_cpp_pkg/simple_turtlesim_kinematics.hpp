#ifndef SIMPLE_TURTLESIM_KINEMATICS_HPP_
#define SIMPLE_TURTLESIM_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class SimpleTurtelsimKinematics : public rclcpp::Node
{
    public:
    SimpleTurtelsimKinematics(const std::string &name) ;
    private:
    void CallbackTurtle1Pose(const turtlesim::msg::Pose& pose);
    void CallbackTurtle2Pose(const turtlesim::msg::Pose& pose);

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;

    turtlesim::msg::Pose turtle1_last_pose_;
    turtlesim::msg::Pose turtle2_last_pose_;
};

#endif // SIMPLE_TURTLESIM_KINEMATICS_HPP_