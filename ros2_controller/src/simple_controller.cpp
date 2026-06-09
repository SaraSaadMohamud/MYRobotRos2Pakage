#include "ros2_controller/simple_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.hpp>

using namespace std::placeholders;

SimpleController::SimpleController(const std::string &name): Node(name),
                left_wheel_prev_pos_(0.0),
                rigth_whee_prev_pos_(0.0),
                x_(0.0),
                y_(0.0),
                theta_(0.0)
                              
{
    declare_parameter("wheel_reduis",0.033);
    declare_parameter("wheel_separation",0.17);

    wheel_reduis_ = get_parameter("wheel_reduis").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO(get_logger(),"Using wheel_reduis: %0.2f\n",wheel_reduis_);
    RCLCPP_INFO(get_logger(),"Using wheel_separation: %0.2f\n",wheel_separation_);

    prev_time_ = get_clock()->now();
    wheel_cmd_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands",10);
    velocity_subscriper_ = create_subscription<geometry_msgs::msg::TwistStamped> ("/ros2_controller/cmd_vel",10,
                           std::bind(&SimpleController::CallbackVelocity,this,_1));
    joint_subscriber_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
                        std::bind(&SimpleController::CallbackJoint,this,_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ros2_controller/odom",10);
    
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";
    speed_conversion_<<wheel_reduis_/2,wheel_reduis_/2,wheel_reduis_/wheel_separation_,-wheel_reduis_/wheel_separation_;
    
    RCLCPP_INFO_STREAM(get_logger(),"the conversion matrix is :\n"<<speed_conversion_);

}   

void SimpleController:: CallbackVelocity(const geometry_msgs::msg::TwistStamped &msg)
{
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    wheel_cmd_publisher_ ->publish(wheel_speed_msg);
}

void SimpleController:: CallbackJoint(const sensor_msgs::msg::JointState &msg)
{
    double db_left = msg.position.at(0) - left_wheel_prev_pos_;
    double db_right = msg.position.at(1) - rigth_whee_prev_pos_;

    rclcpp::Time msg_time =  msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    left_wheel_prev_pos_ =  msg.position.at(0);
    rigth_whee_prev_pos_ = msg.position.at(1);
    prev_time_ = msg_time;

    double fi_left  = db_left  / dt.seconds();
    double fi_rigth = db_right / dt.seconds();

    double linear = (wheel_reduis_ * fi_rigth + wheel_reduis_ * fi_left) / 2.0;
    double angualr = (wheel_reduis_ * fi_rigth - wheel_reduis_*fi_left) / wheel_separation_;
    
    double d_s = (wheel_reduis_*db_right + wheel_reduis_*db_left)/2.0;
    double d_theta = (wheel_reduis_*db_right - wheel_reduis_*db_left)/wheel_separation_;

    theta_ += d_theta;
    theta_ = atan2(sin(theta_), cos(theta_));

    x_ += d_s * cos(theta_);
    y_ +=d_s * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angualr;

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    transform_stamped_.header.stamp = get_clock()->now();

    odom_pub_->publish(odom_msg_);
    transform_broadcaster_->sendTransform(transform_stamped_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}