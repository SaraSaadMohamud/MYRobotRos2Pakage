// safety_stop.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Bug 7 fix: The DANGER state was declared in the enum but NEVER set inside
//   LaserCallback. The loop only checked `range <= warning_distance_` (the
//   outer if and inner if were identical), so the FSM could only ever reach
//   FREE or WARNING — DANGER was a dead code path. Two conditions are now
//   checked in order: danger_distance_ first (innermost ring), then
//   warning_distance_ (outer ring).
//
// Bug 8 fix: The constructor called wait_for_action_server() in a blocking
//   while-loop on the main thread. If joy_teleop was not yet running this
//   would block the entire node forever — nothing else could initialise,
//   including the laser subscriber. Replaced with non-blocking one-shot checks
//   that log a warning and continue. The action client will still connect once
//   the server becomes available.
// ─────────────────────────────────────────────────────────────────────────────

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <math.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/qos.hpp"
#include <thread>

using namespace std::placeholders;

enum State
{
    FREE    = 0,
    WARNING = 1,
    DANGER  = 2
};

class SafetyStop : public rclcpp::Node
{
public:
    SafetyStop() : Node("safety_stop_node"),
                   is_first_msg_{true},
                   state_{State::FREE},
                   prev_state_{State::FREE}
    {
        declare_parameter<double>("danger_distance",  0.2);
        declare_parameter<double>("warning_distance", 0.6);
        declare_parameter<std::string>("scan_topic",         "scan");
        declare_parameter<std::string>("safety_stop_topic",  "safety_stop");

        danger_distance_  = get_parameter("danger_distance").as_double();
        warning_distance_ = get_parameter("warning_distance").as_double();
        std::string scan_topic        = get_parameter("scan_topic").as_string();
        std::string safety_stop_topic = get_parameter("safety_stop_topic").as_string();

        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&SafetyStop::LaserCallback, this, _1));

        safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
            safety_stop_topic, 10);

        zones_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "zones", 10);

        decrease_speed_client_ = rclcpp_action::create_client<
            twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_decrease");

        increase_speed_client_ = rclcpp_action::create_client<
            twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_increase");

        // non-blocking check — log a warning and carry on.
        // The action client will connect once the server appears.
        if (!decrease_speed_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(),
                "Action /joy_turbo_decrease not yet available — will retry on first use.");
        }
        if (!increase_speed_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(),
                "Action /joy_turbo_increase not yet available — will retry on first use.");
        }

        // Build visualisation markers
        visualization_msgs::msg::Marker warning_zone;
        warning_zone.id     = 0;
        warning_zone.action = visualization_msgs::msg::Marker::ADD;
        warning_zone.type   = visualization_msgs::msg::Marker::CYLINDER;
        warning_zone.scale.z = 0.001;
        warning_zone.scale.x = warning_distance_ * 2;
        warning_zone.scale.y = warning_distance_ * 2;
        warning_zone.color.r = 1.0f;
        warning_zone.color.g = 0.984f;
        warning_zone.color.b = 0.0f;
        warning_zone.color.a = 0.5f;
        zones_.markers.push_back(warning_zone);

        visualization_msgs::msg::Marker danger_zone;
        danger_zone.id     = 1;
        danger_zone.action = visualization_msgs::msg::Marker::ADD;
        danger_zone.type   = visualization_msgs::msg::Marker::CYLINDER;
        danger_zone.scale.z = 0.001;
        danger_zone.scale.x = danger_distance_ * 2;
        danger_zone.scale.y = danger_distance_ * 2;
        danger_zone.color.r = 1.0f;
        danger_zone.color.g = 0.0f;
        danger_zone.color.b = 0.0f;
        danger_zone.color.a = 0.5f;
        zones_.markers.push_back(danger_zone);
    }

private:
    bool    is_first_msg_;
    double  danger_distance_, warning_distance_;
    State   state_, prev_state_;
    visualization_msgs::msg::MarkerArray zones_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             safety_stop_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_speed_client_;
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_speed_client_;

    void LaserCallback(const sensor_msgs::msg::LaserScan &msg)
    {
        //  Evaluate DANGER first (innermost ring), then WARNING.
        // Previously both the outer and inner if-statements checked the same
        // condition (range <= warning_distance_), so DANGER was unreachable.
        state_ = State::FREE;

        for (const auto &range : msg.ranges)
        {
            if (std::isinf(range) || std::isnan(range))
                continue;

            if (range <= danger_distance_)
            {
                state_ = State::DANGER;
                break;              // innermost ring — no need to check further
            }
            else if (range <= warning_distance_)
            {
                state_ = State::WARNING;
                // do NOT break — a closer reading may still promote to DANGER
            }
        }

        if (state_ != prev_state_)
        {
            std_msgs::msg::Bool is_safety_stop;

            if (state_ == State::WARNING)
            {
                is_safety_stop.data = false;
                zones_.markers.at(0).color.a = 1.0f;
                zones_.markers.at(1).color.a = 0.5f;
                decrease_speed_client_->async_send_goal(
                    twist_mux_msgs::action::JoyTurbo::Goal());
            }
            else if (state_ == State::DANGER)
            {
                is_safety_stop.data = true;    // actually stops the robot
                zones_.markers.at(0).color.a = 1.0f;
                zones_.markers.at(1).color.a = 1.0f;
            }
            else   // FREE
            {
                is_safety_stop.data = false;
                zones_.markers.at(0).color.a = 0.5f;
                zones_.markers.at(1).color.a = 0.5f;
                increase_speed_client_->async_send_goal(
                    twist_mux_msgs::action::JoyTurbo::Goal());
            }

            prev_state_ = state_;
            safety_stop_pub_->publish(is_safety_stop);
        }

        // Set frame_id once on the first scan
        if (is_first_msg_)
        {
            for (auto &zone : zones_.markers)
                zone.header.frame_id = "laser_link";
            is_first_msg_ = false;
        }

        zones_pub_->publish(zones_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}