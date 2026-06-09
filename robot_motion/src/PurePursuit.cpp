#include "robot_motion/pure_pursite.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <tf2/time.hpp>
#include <algorithm>
#include <cmath>

#include <nav2_util/node_utils.hpp>

using namespace std::placeholders;

namespace robot_motion
{
   void PurePursuit::configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name, 
            std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent;
        auto node = node_.lock();
        costmap_ros_ = costmap_ros;
        plugin_name_ = name;
        logger_ = node->get_logger();
        clock_ = node->get_clock();
        tf_ = tf;

        nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + "look_ahead_distance",rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + "max_linear_velocity",rclcpp::ParameterValue(0.3));
        nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + "max_angular_velocity",rclcpp::ParameterValue(1.0));

        node->get_parameter(plugin_name_ + "look_ahead_distance",look_ahead_distance_);
        node->get_parameter(plugin_name_ + "max_linear_velocity",max_linear_velocity_);
        node->get_parameter(plugin_name_ + "max_angular_velocity",max_angular_velocity_);

        carrot_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/PurePursuit/carrot_pose",10);
    }

    void PurePursuit::cleanup()
    {
        RCLCPP_INFO(logger_,"Cleaning up plugin PurePursite");
        carrot_pose_pub_.reset();
    }
    void PurePursuit::activate()
    {
        RCLCPP_INFO(logger_,"Activating plugin PurePursite");
    }
    void PurePursuit::deactivate()
    {
        RCLCPP_INFO(logger_,"Deactivating plugin PurePursite");
    }

    void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
    {
        global_plan_ = path;
    }

    geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &robot_pose,const geometry_msgs::msg::Twist &,
            nav2_core::GoalChecker * )
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = robot_pose.header.frame_id;
        if (global_plan_.poses.empty()) return cmd_vel;

       

        if (!TransformPlan("map")) return cmd_vel;

        auto carrot_pose = GetCarrotPose(robot_pose);
        carrot_pose_pub_->publish(carrot_pose);

        // Transform carrot into robot body frame
        tf2::Transform robot_tf, carrot_tf, carrot_robot_tf;
        tf2::fromMsg(robot_pose.pose, robot_tf);
        tf2::fromMsg(carrot_pose.pose, carrot_tf);
        carrot_robot_tf = robot_tf.inverse() * carrot_tf;
        tf2::toMsg(carrot_robot_tf, carrot_pose.pose);

        double carrot_x = carrot_pose.pose.position.x;
        double carrot_y = carrot_pose.pose.position.y;
        double angle_to_carrot = std::atan2(carrot_y, carrot_x);

        // UNIFIED CONTROL LAW — always move, scale linear by alignment
        // When perfectly aligned: linear = max, angular = small correction
        // When 90° off:          linear = 30% max, angular = max turn
        // When 180° off:         linear = 0 (truly behind), full rotation
        double alignment = std::cos(angle_to_carrot); // 1.0=ahead, 0.0=side, -1.0=behind

        if (alignment < -0.1) {
            // Carrot is genuinely behind (>~96 degrees) — rotate only
            cmd_vel.twist.linear.x  = 0.0;
            cmd_vel.twist.angular.z = std::clamp(
                1.5 * angle_to_carrot,
                -max_angular_velocity_,
                max_angular_velocity_);
        } else {
            // Carrot is ahead or to the side — ALWAYS drive forward
            // Scale linear velocity: full speed when aligned, minimum when sideways
            double min_linear = 0.08;  // never go slower than 8cm/s forward
            double linear_scale = std::max(alignment, min_linear / max_linear_velocity_);
            cmd_vel.twist.linear.x = max_linear_velocity_ * linear_scale;

            // Pure pursuit curvature for steering
            double curvature = GetCurvature(carrot_pose.pose);
            cmd_vel.twist.angular.z = std::clamp(
                curvature * cmd_vel.twist.linear.x,
                -max_angular_velocity_,
                max_angular_velocity_);
        }

        RCLCPP_INFO(logger_,
            "Carrot RF(%.2f,%.2f) angle=%.1f° | lin=%.2f ang=%.2f",
            carrot_x, carrot_y,
            angle_to_carrot * 180.0 / M_PI,
            cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        
        return(cmd_vel);
    }

    void PurePursuit::setSpeedLimit(const double & , const bool & ){}
   geometry_msgs::msg::PoseStamped PurePursuit::GetCarrotPose(
    const geometry_msgs::msg::PoseStamped &robot_pose)
    {
        // Step 1: find closest path point to robot current position
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
            double dx = global_plan_.poses[i].pose.position.x
                        - robot_pose.pose.position.x;
            double dy = global_plan_.poses[i].pose.position.y
                        - robot_pose.pose.position.y;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }

        // Step 2: walk FORWARD from closest point until lookahead distance
        for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
            double dx = global_plan_.poses[i].pose.position.x
                        - robot_pose.pose.position.x;
            double dy = global_plan_.poses[i].pose.position.y
                        - robot_pose.pose.position.y;
            if (std::sqrt(dx*dx + dy*dy) >= look_ahead_distance_)
                return global_plan_.poses[i];
        }

        // Step 3: all remaining poses within lookahead — return goal
        return global_plan_.poses.back();
    }

    double PurePursuit::GetCurvature(const geometry_msgs::msg::Pose &carrot_pose)
    {
        const double L = (carrot_pose.position.x * carrot_pose.position.x) +
                         (carrot_pose.position.y * carrot_pose.position.y);
        if (L > 0.001)
            return 2.0 * carrot_pose.position.y / L;
        return 0.0;
    }

    bool PurePursuit::TransformPlan(const std::string &frame)
    {
        if (global_plan_.header.frame_id == frame)
            return true;

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_->lookupTransform(
                frame, global_plan_.header.frame_id,
                tf2::TimePointZero,
                tf2::durationFromSec(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(logger_,
                "Transform plan failed [%s] -> [%s]: %s",
                global_plan_.header.frame_id.c_str(), frame.c_str(), ex.what());
            return false;
        }

        for (auto &pose : global_plan_.poses)
            tf2::doTransform(pose, pose, transform);

        global_plan_.header.frame_id = frame;
        return true;
    }

} // namespace robot_motion
# include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_motion::PurePursuit,nav2_core::Controller)