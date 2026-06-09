#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::placeholders;

class TwistRelay : public rclcpp::Node
{
    public:
        TwistRelay() : Node("twist_relay")
        {
            // ── Chain A: simple_controller mode ───────────────────────────
            // Converts unstamped Twist  →  TwistStamped for simple_controller.
            // Input : /ros2_controller/cmd_vel_unstamped  (from twist_mux in simple mode)
            // Output: /ros2_controller/cmd_vel            (read by simple_controller.cpp)
            controller_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                "/ros2_controller/cmd_vel_unstamped",
                10,
                std::bind(&TwistRelay::ControllerTwistCallback, this, _1)
            );

            controller_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
                "/ros2_controller/cmd_vel",
                10
            );

            // ── Chain B: joystick → twist_mux ─────────────────────────────
            // joy_teleop publishes TwistStamped on /input_joy/cmd_vel_stamped.
            // twist_mux expects an unstamped Twist on the topic named in
            // twist_mux_topics.yaml → topics → joystick → topic: joy_vel
            //
            // BUG FIX: was publishing to "/input_joy/cmd_vel" which nobody
            // reads. Changed to "joy_vel" so twist_mux receives the command.
            joy_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                "/input_joy/cmd_vel_stamped",
                10,
                std::bind(&TwistRelay::JoyTwistCallback, this, _1)
            );
            joy_pub_ = create_publisher<geometry_msgs::msg::Twist>(
                "joy_vel",   // ← FIXED (was "/input_joy/cmd_vel")
                10
            );
        }

    private:

        void ControllerTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            geometry_msgs::msg::TwistStamped twist_stamped;
            twist_stamped.header.stamp = get_clock()->now();
            twist_stamped.twist = *msg;
            controller_pub_->publish(twist_stamped);
        }

        void JoyTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
            geometry_msgs::msg::Twist twist;
            twist = msg->twist;
            joy_pub_->publish(twist);
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    controller_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        joy_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}