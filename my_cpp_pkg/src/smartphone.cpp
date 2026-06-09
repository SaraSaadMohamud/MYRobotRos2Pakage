#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>

#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class SmartPhone : public rclcpp :: Node
{
    public:
    SmartPhone():Node("smart_phone")
    {
        subscriper_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_news",10,
            std::bind(&SmartPhone::CallbackRobotNews,this,_1));
        RCLCPP_INFO(this->get_logger(),"SmartPhone has been started!.");

    }
    private:
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriper_;
    void CallbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"%s",msg->data.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SmartPhone>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}