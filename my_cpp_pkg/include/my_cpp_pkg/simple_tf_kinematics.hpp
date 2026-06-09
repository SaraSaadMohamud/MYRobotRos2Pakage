#ifndef  SIMPLE_TF_KINEMATICS_HPP_
#define  SIMPLE_TF_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <my_robot_interfaces/srv/get_transform.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <memory>

class SimpleTfKinematics : public rclcpp :: Node
{
    public:
    SimpleTfKinematics(std::string name);
    private:

    void CallbackTimer();

    std::__shared_ptr<tf2_ros::StaticTransformBroadcaster> tf2_static_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_daynamic_broadcaster_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped daynamic_transform_stamped_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<my_robot_interfaces::srv::GetTransform>::SharedPtr get_transform_srv_;
    double x_increment_;
    double last_x_;
    int rotation_counter_;
    tf2::Quaternion last_orintation_;
    tf2::Quaternion oriantation_increment_;

    bool CallbackGetTransform(const my_robot_interfaces::srv::GetTransform::Request::SharedPtr request,
                              const my_robot_interfaces::srv::GetTransform::Response::SharedPtr response);
    
};

#endif //SIMPLE_TF_KINEMATICS_HPP_