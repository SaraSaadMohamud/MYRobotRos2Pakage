#include "rclcpp/rclcpp.hpp"
#include <memory>

class MyNode : public rclcpp::Node
{
    public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(),"Hello sara!.");
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::TimerCallback,this));
    }
    private:
    void TimerCallback()
    {
        RCLCPP_INFO(this->get_logger(),"Hello aaaaaaaa %d:",counter_);
        counter_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
   
   /* 
   // basic way to create a node
   rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("cpp_test");
    RCLCPP_INFO(node ->get_logger(),"Hello Sara!.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    */
   
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}