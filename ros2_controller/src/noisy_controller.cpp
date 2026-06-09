#include "ros2_controller/noisy_controller.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <random>

using namespace std::placeholders;

NoisyController::NoisyController(const std::string &name): Node(name),
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
    
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ros2_controller/odom_noisy",10);
    
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_ekf";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_fooyprint_noisy";

}  
void NoisyController:: CallbackJoint(const sensor_msgs::msg::JointState &msg)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.005);
    double wheel_encoder_left = msg.position.at(1) + left_encoder_noise(noise_generator);
    double wheel_encoder_right = msg.position.at(0) + right_encoder_noise(noise_generator);
    double db_left = wheel_encoder_left  - left_wheel_prev_pos_;
    double db_right = wheel_encoder_right - rigth_whee_prev_pos_;

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
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}