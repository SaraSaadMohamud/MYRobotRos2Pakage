#ifndef SIMPLE_CONTROLLER_HPP_
#define SIMPLE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Core>

class SimpleController : public rclcpp :: Node 
{
    public:
    SimpleController(const std::string &name);

    private:
    void CallbackVelocity(const geometry_msgs::msg::TwistStamped &msg);
    void CallbackJoint(const sensor_msgs::msg::JointState &msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscriper_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double wheel_reduis_;
    double wheel_separation_;

    double left_wheel_prev_pos_;
    double rigth_whee_prev_pos_;
    rclcpp::Time prev_time_;
    Eigen::Matrix2d speed_conversion_;

    double x_;
    double y_;
    double theta_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif //SIMPLE_CONTROLLER_HPP_