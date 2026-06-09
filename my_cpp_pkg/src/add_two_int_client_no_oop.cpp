#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <memory>

using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node= std::make_shared<rclcpp::Node>("add_two_int_no_oop");
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while(!client->wait_for_service(1s))
    {
        RCLCPP_INFO(node->get_logger(),"wait for the Server...");
    }

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request ->a = 6;
    request ->b = 2;

    auto feture = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, feture);

    auto response = feture.get();
    RCLCPP_INFO(node->get_logger(),"%d + %d = %d",
                    (int)request->a, (int)request->b , (int)response->sum);
                    
    rclcpp::shutdown();
    return(0);
}