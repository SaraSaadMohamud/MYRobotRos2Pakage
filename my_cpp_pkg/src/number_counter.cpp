#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>
#include <example_interfaces/srv/set_bool.hpp>
#include <memory>

using namespace std::placeholders;

class NumberCounter : public rclcpp ::Node
{
    public :
    NumberCounter() : Node("number_count"),counter_(0)
    {
        subscriper_ = this ->create_subscription<example_interfaces::msg::Int64>(
                    "number",10,
                    std::bind(&NumberCounter::CallbackNumberCount,this,_1));
                    RCLCPP_INFO(this->get_logger(),"Number Counter has been started!.");
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count",10);

        service_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter",std::bind(&NumberCounter::CallbackResetCounter,this,_1,_2));
        RCLCPP_INFO(this->get_logger(),"Rest counter Service has been startd!.");
    }
    private:
    void CallbackNumberCount(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto msg_b = example_interfaces::msg::Int64();
        msg_b.data = counter_;
        publisher_->publish(msg_b);
    }
    void CallbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                              const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if(request->data)
        {
            counter_ = 0;
            response->success = true;
            response->message = "counter has been reset!.";
        }
        else
        {
            response->success = false;
            response->message = "counter has not been reset!.";
        }
    }
    int counter_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriper_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}