#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <memory>

using namespace std::placeholders;

class AddTwoIntServes : public rclcpp ::Node
{
    public:
    AddTwoIntServes():Node("add_two_int")
    {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                   std::bind(&AddTwoIntServes::CallbackAddTwoInt,this,_1,_2));
        RCLCPP_INFO(this->get_logger(),"Add two ints serves has been strarted!.");
    }
    private:
    void CallbackAddTwoInt(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                           const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(),"%d + %d = %d",
                    (int)request->a, (int)request->b , (int)response->sum);
    }
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AddTwoIntServes>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}
