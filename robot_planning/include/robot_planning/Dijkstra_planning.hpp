#ifndef DIJKSTRA_PLANNING_HPP
#define DIJKSTRA_PLANNING_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_core/global_planner.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_msgs/action/smooth_path.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/buffer.hpp>
#include <memory>

namespace robot_planning
{

struct GraphNode
{
    int x;
    int y;
    int cost = 0;
    std::shared_ptr<GraphNode> prev = nullptr;

    GraphNode(int in_x, int in_y) : x(in_x), y(in_y) {}
    GraphNode() : GraphNode(0, 0) {}

    bool operator>(const GraphNode& other) const { return cost > other.cost; }  
    bool operator==(const GraphNode& other) const { return x == other.x && y == other.y; }

    GraphNode operator+(const std::pair<int, int>& dir) const
    {
        return GraphNode(x + dir.first, y + dir.second);
    }
};

class DijkstraPlanner : public nav2_core::GlobalPlanner
{
public:
    DijkstraPlanner() = default;
    ~DijkstraPlanner() = default;

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, 
        std::string name,std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)override;
    
    void cleanup()override;
    void activate()override;
    void deactivate()override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancle_checker)override;

private:

    rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D *costmap_;
    std::string global_frame_,name_;

    void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

    geometry_msgs::msg::Pose GridToWorld(const GraphNode& node);
    GraphNode WorldToGrid(const geometry_msgs::msg::Pose& pose);
    bool PoseOnMap(const GraphNode& node);
    unsigned int PoseToCell(const GraphNode& node);

};

}  

#endif