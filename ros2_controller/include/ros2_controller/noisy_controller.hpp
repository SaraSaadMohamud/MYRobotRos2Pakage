#ifndef Noisy_CONTROLLER_HPP_
#define Noisy_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class NoisyController : public rclcpp :: Node 
{
    public:
    NoisyController(const std::string &name);

    private:
    void CallbackJoint(const sensor_msgs::msg::JointState &msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double wheel_reduis_;
    double wheel_separation_;

    double left_wheel_prev_pos_;
    double rigth_whee_prev_pos_;
    rclcpp::Time prev_time_;

    double x_;
    double y_;
    double theta_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif //Noisy_CONTROLLER_HPP_