#include "robot_planning/AStar_planning.hpp"
#include <tf2/exceptions.hpp>
#include <queue>
#include <vector>
#include <algorithm>
#include <memory>

namespace robot_planning
{
    void AStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        tf_ = tf;
        global_frame_ = costmap_ros->getGlobalFrameID();

        smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_,"smooth_path");
        if(!smooth_client_-> wait_for_action_server(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(node_->get_logger(),"Action Server not availble after waiting!");
        }
    }

    void AStarPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(),"Cleaning up %s of type AStarPlanner",name_.c_str());
    }
    void AStarPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(),"Activating  %s of type AStarPlanner",name_.c_str());
    }
    void AStarPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(),"Deactivating up  %s of type AStarPlanner",name_.c_str());
    }

nav_msgs::msg::Path AStarPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                    const geometry_msgs::msg::PoseStamped &goal,std::function<bool()>)
{
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
    std::vector<GraphNode> visited_nodes;

    GraphNode start_node = WorldToGrid(start.pose);
    GraphNode goal_node = WorldToGrid(goal.pose);
    start_node.heuristics =  ManhattanDistance(start_node, goal_node);
    pending_nodes.push(start_node);

    GraphNode active_node;

    while (!pending_nodes.empty() && rclcpp::ok())
    {
        active_node = pending_nodes.top();
        pending_nodes.pop();

        if (active_node == goal_node)
            break;

        for (const auto& dir : directions)
        {
            GraphNode new_node = active_node + dir;

            if (std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() &&
                PoseOnMap(new_node) &&
                costmap_->getCost(new_node.x,new_node.y) < 99)
            {
                new_node.cost = active_node.cost + 1 +  costmap_->getCost(new_node.x,new_node.y);
                new_node.heuristics  = ManhattanDistance(new_node,goal_node);
                new_node.prev = std::make_shared<GraphNode>(active_node);

                pending_nodes.push(new_node);
                visited_nodes.push_back(new_node);
            }
        }
    }

    // Reconstruct path
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;

    GraphNode current = active_node;
    while (current.prev && rclcpp::ok())
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = node_->now();
        pose_stamped.header.frame_id = global_frame_;
        pose_stamped.pose = GridToWorld(current);
        path.poses.push_back(pose_stamped);

        current = *current.prev;
    }

    // Add start pose if needed
    if (!path.poses.empty())
    {
        geometry_msgs::msg::PoseStamped start_stamped;
        start_stamped.header.frame_id = global_frame_;
        start_stamped.pose = GridToWorld(start_node);
        path.poses.push_back(start_stamped);
    }

    std::reverse(path.poses.begin(), path.poses.end());

    nav2_msgs::action::SmoothPath::Goal path_smooth;
    path_smooth.path = path;
    path_smooth.check_for_collisions = false;
    path_smooth.smoother_id = "simple_smoother";
    path_smooth.max_smoothing_duration.sec = 10;
   auto future = smooth_client_ ->async_send_goal(path_smooth);
   if(future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
   {
    auto goal_handle = future.get();
    if(goal_handle)
    {
        auto result_future = smooth_client_ ->async_get_result(goal_handle);
        if(result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
        {
           auto result_path = result_future.get();
           if(result_path.code == rclcpp_action::ResultCode::SUCCEEDED)
           {
                path = result_path.result->path;
           }
        }
    }
   }
    return path;
}

geometry_msgs::msg::Pose AStarPlanner::GridToWorld(const GraphNode& node)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * costmap_->getResolution()+ costmap_->getOriginX();
    pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
    pose.orientation.w = 1.0;
    return pose;
}

 double AStarPlanner::ManhattanDistance(const GraphNode& node, const GraphNode& goal_node)
 {
    return (abs(node.x - goal_node.x) + abs(node.y - goal_node.y));
 }

GraphNode AStarPlanner::WorldToGrid(const geometry_msgs::msg::Pose& pose)
{
    int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) /costmap_->getResolution());
    return GraphNode(grid_x, grid_y);
}

bool AStarPlanner::PoseOnMap(const GraphNode& node)
{
    return (node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) &&
            node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY()));
}

unsigned int AStarPlanner::PoseToCell(const GraphNode& node)
{
    return node.y * costmap_->getSizeInCellsX() + node.x;
}

}  

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_planning::AStarPlanner, nav2_core::GlobalPlanner)