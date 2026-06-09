#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class AddTwoIntClient : public rclcpp :: Node
{
    public:
    AddTwoIntClient():Node("add_two_int_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void CallAddTwoInt(int a, int b)
    {
        while(! client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(),"Wait for the Serves....");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request ->a = a;
        request ->b = b;
        
        client_->async_send_request(request,std::bind(&AddTwoIntClient::CallbackAddTwoInts,this,_1));

    }

    private:
    void CallbackAddTwoInts(const rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture feture)
    {
        auto response = feture .get();
        RCLCPP_INFO(this->get_logger(),"Sum: %d",(int)response->sum);
    }
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AddTwoIntClient>();
    node-> CallAddTwoInt(10,5);
    node-> CallAddTwoInt(10,15);
    node-> CallAddTwoInt(10,25);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}