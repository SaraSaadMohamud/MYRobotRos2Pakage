#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>
#include <memory>

using namespace std::chrono_literals;
class NumberPublisherNode : public rclcpp :: Node
{
    public:
    NumberPublisherNode() : Node("number_publisher"),counter_(2)
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number",10);
        timer_ = this->create_wall_timer(1s,std::bind(&NumberPublisherNode::NumberPublisher,this));
        RCLCPP_INFO(this->get_logger(),"Number publisher has been started!.");
    }
    private:
    void NumberPublisher()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = counter_;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    long long counter_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}