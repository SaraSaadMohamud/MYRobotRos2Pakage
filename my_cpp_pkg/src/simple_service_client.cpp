#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/srv/add_two_ints.hpp>
#include<memory>
#include<chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

class SimpleServiceClient : public rclcpp :: Node
{
    public:
    SimpleServiceClient(int a, int b): Node("simple_service_client")
    {
        client_ = this->create_client<my_robot_interfaces::srv::AddTwoInts>("add_two_ints");

        auto request_ = std::make_shared<my_robot_interfaces::srv::AddTwoInts::Request>();
        request_->a = a;
        request_->b = b;

        while(!client_->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(),"Interrupt during waiting the Service!.\n");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Waiting for the Service...");
        }

        auto result_ = client_->async_send_request(request_,
            std::bind(&SimpleServiceClient::CallbackResponse,this,_1));

    }
    private:
    void CallbackResponse(rclcpp::Client<my_robot_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        if(future.valid())
        {
            RCLCPP_INFO_STREAM(this->get_logger(),"Response is : "<<future.get()->sum);
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(),"Service Failure!.");
        }
    }

    rclcpp::Client<my_robot_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main (int argc, char **argv)
{
    rclcpp::init(argc,argv);
    if(argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Wrong number of arguments! Usage: simple_service-client A B");
        return(1);
    }

    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}