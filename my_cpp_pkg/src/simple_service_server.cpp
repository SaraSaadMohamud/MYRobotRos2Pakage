#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/srv/add_two_ints.hpp>
#include <memory>

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp :: Node
{
    public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        service_ = this->create_service<my_robot_interfaces::srv::AddTwoInts>("add_two_ints",
                    std::bind(&SimpleServiceServer::CallbackService,this,_1,_2));

        RCLCPP_INFO(this->get_logger(),"Service node has been started!.\n");

    }
    private:
    void CallbackService(const my_robot_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                        const my_robot_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"New request is rescive: a = "
                            <<request->a<<", and b = "<<request->b<<std::endl);

        response->sum = request->a + request->b ;
        RCLCPP_INFO_STREAM(this->get_logger(),"Response of Suming = "<<response->sum<<" !.\n");
    }
    rclcpp::Service<my_robot_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}