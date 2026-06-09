#include "robot_motion/pd_motion_planner.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <tf2/time.hpp>
#include <algorithm>

#include <nav2_util/node_utils.hpp>

using namespace std::placeholders ;

namespace robot_motion
{
    void PDMotionPlanner::configure(
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

        next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose",10);

        nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".kp",rclcpp::ParameterValue(2.0));
        nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".kd",rclcpp::ParameterValue(0.1));
        nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".step_size",rclcpp::ParameterValue(0.2));
        nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_linear_velocity",rclcpp::ParameterValue(0.3));
        nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_angular_velocity",rclcpp::ParameterValue(1.0));

        node->get_parameter(plugin_name_ + ".kp",kp_);
        node->get_parameter(plugin_name_ + ".kd",kd_);
        node->get_parameter(plugin_name_ + ".step_size",step_size_);
        node->get_parameter(plugin_name_ + ".max_linear_velocity",max_linear_velocity_);
        node->get_parameter(plugin_name_ + ".max_angular_velocity",max_angular_velocity_);
    }

    void PDMotionPlanner::cleanup()
    {
        RCLCPP_INFO(logger_,"Cleaning up plugin PDMotionPlanner");
        next_pose_pub_.reset();
    }
    void PDMotionPlanner::activate()
    {
        RCLCPP_INFO(logger_,"Activating plugin PDMotionPlanner");
        last_cycle_time_ = clock_->now();
    }
    void PDMotionPlanner::deactivate()
    {
        RCLCPP_INFO(logger_,"Deactivating plugin PDMotionPlanner");
    }

    void PDMotionPlanner::setPlan(const nav_msgs::msg::Path & path)
    {
        global_plan_ = path;
    }
    geometry_msgs::msg::TwistStamped PDMotionPlanner::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & robot_pose,const geometry_msgs::msg::Twist &,
        nav2_core::GoalChecker * )
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = robot_pose.header.frame_id;

        if(global_plan_.poses.empty())
        {
            RCLCPP_ERROR(logger_,"Empty plan!");
            return cmd_vel;
        }
        
        if(!TransformPlan(robot_pose.header.frame_id))
        {
            RCLCPP_INFO(logger_,"Unenable to transform plan in robot fram!");
            return cmd_vel;
        }

        auto next_pose = GetNextPose(robot_pose);

        next_pose_pub_ ->publish(next_pose);
        tf2::Transform robot_tf,next_pose_tf, next_pose_robot_tf;
        tf2::fromMsg(robot_pose.pose, robot_tf);
        tf2::fromMsg(next_pose.pose,next_pose_tf);

       next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;
        double linear_error = next_pose_robot_tf.getOrigin().getX();
        double angular_error = std::atan2(next_pose_robot_tf.getOrigin().getY(),next_pose_robot_tf.getOrigin().getX());

        double dt = (clock_->now() - last_cycle_time_).seconds();
        if(dt <= 0) dt = 0.1;

        double linear_error_derivative = (linear_error - prev_linear_error_) /dt ;
        double anuglar_error_derivative = (angular_error - prev_angular_error_)/ dt;

        cmd_vel.twist.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative,-max_linear_velocity_,max_linear_velocity_);
        cmd_vel.twist.angular.z = std::clamp(kp_ * angular_error + kd_ * anuglar_error_derivative,-max_angular_velocity_,max_angular_velocity_);

        last_cycle_time_ = clock_->now();
        prev_linear_error_ = linear_error;
        prev_angular_error_ = angular_error;

        return(cmd_vel);
    }

    bool PDMotionPlanner::TransformPlan(const std::string &frame)
    {
        if(global_plan_.header.frame_id == frame)
        {
            return(true);
        }
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tf_->lookupTransform(frame,global_plan_.header.frame_id,tf2::TimePointZero);
        }catch(tf2::LookupException &ex)
        {
            RCLCPP_ERROR_STREAM(logger_,"Couldn't transform plan from frame"<<global_plan_.header.frame_id<<"to"<<frame);
            return(false);
        }

        for(auto &pose : global_plan_.poses)
        {
            tf2::doTransform(pose,pose,transform);
        }
        global_plan_.header.frame_id = frame;
        return(true);
    }

    geometry_msgs::msg::PoseStamped PDMotionPlanner::GetNextPose(
    const geometry_msgs::msg::PoseStamped &robot_pose)
    {
        for (const auto &pose : global_plan_.poses)
        {
            double dx = pose.pose.position.x - robot_pose.pose.position.x;
            double dy = pose.pose.position.y - robot_pose.pose.position.y;
            if (std::sqrt(dx * dx + dy * dy) > step_size_)
                return pose;
        }
        // All remaining waypoints are within step_size_ — return the final goal
        return global_plan_.poses.back();
    }

    void PDMotionPlanner::setSpeedLimit(const double &, const bool &){}
}

# include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_motion::PDMotionPlanner,nav2_core::Controller)