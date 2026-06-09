#include <rclcpp/rclcpp.hpp>
#include "my_robot_interfaces/msg/led_pannel_state.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class LedPannelNode : public rclcpp :: Node
{
    public:
    LedPannelNode(): Node("led_pannel"),led_states_(3,0)
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedPannelState>("led_panel_state",10);
        timer_ = this->create_wall_timer(5s, std::bind(&LedPannelNode::PublishLedState,this));
        service_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                    std::bind(&LedPannelNode::CallbackLedStatus,this,_1,_2));
        RCLCPP_INFO(this->get_logger(),"Led pannel node has been started!.");
    }
    private:
    void CallbackLedStatus(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                            const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        int64_t led_number = request->led_number;
        int64_t led_state = request ->state;

        if( (led_number<0 )|| (led_number >= (int64_t)led_states_.size()))
        {
            response->success =  false;
            return;
        }

        if((led_state != 0) && (led_state != 1))
        {
            response->success = false;
            return;
        }
        led_states_.at(led_number) = led_state;
        response->success = true;
        PublishLedState();
    }

    void PublishLedState()
    {
        auto msg = my_robot_interfaces::msg::LedPannelState();
        msg.led_status = led_states_;
        publisher_->publish(msg);
    }
    std::vector<int64_t> led_states_;
        rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr service_;
        rclcpp::Publisher<my_robot_interfaces::msg::LedPannelState>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<LedPannelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}