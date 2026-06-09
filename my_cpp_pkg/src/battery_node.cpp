#include <rclcpp/rclcpp.hpp>
#include "my_robot_interfaces/srv/set_led.hpp"
#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class BatteryNode: public rclcpp ::Node
{
    public:
    BatteryNode():Node("battery_node")
    {
        last_time_battery_state_change_ = get_clock()->now().seconds();
        timer_ = this->create_wall_timer(0.1s,std::bind(&BatteryNode::CheckBatteryState,this));
        client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while(!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(),"Wait for the Server...");
        }
        RCLCPP_INFO(this->get_logger(),"Battery node has beening starting!.");
    }
    private:
    void CheckBatteryState()
    {
        auto time_now = get_clock()->now().seconds();
        if(battery_state_ == "full")
        {
            if(time_now - last_time_battery_state_change_ >4.0)
            {
                battery_state_ = "empty";
                RCLCPP_INFO(this->get_logger(),"Battery is Empty! Charging it!.");
                last_time_battery_state_change_=time_now;
                SetLed(2,1);
            }
        }
        else
        {
            if(time_now - last_time_battery_state_change_ >6.0)
            {
                battery_state_ = "full";
                RCLCPP_INFO(this->get_logger(),"Battery is Full again.");
                last_time_battery_state_change_ = time_now;
                SetLed(2,0);
            }
        }
    }

    void SetLed(int led_namber, int state)
    {
        auto request = my_robot_interfaces::srv::SetLed::Request::SharedPtr();
        request->led_number = led_namber;
        request->state = state;

        auto feture = client_->async_send_request(request,std::bind(&BatteryNode::CallbackLedState,this,_1));

    }
    void CallbackLedState(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future)
    {
        auto response = future.get();
        if(response ->success)
        {
            RCLCPP_INFO(this->get_logger(),"Led State was changed!.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"Led State was not changed!.");
        }
    }
    std::string battery_state_;
    double last_time_battery_state_change_;

    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}