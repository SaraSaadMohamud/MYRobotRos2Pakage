#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>
#include <chrono>

using namespace std::chrono_literals;

class SimpleSerialReciver : public rclcpp :: Node
{
    public:
    SimpleSerialReciver() : Node ("simple_serial_receiver")
    {
        declare_parameter<std::string>("port","/dev/ttyACM0");
        port_ = get_parameter("port").as_string();
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        publisher_ = create_publisher<std_msgs::msg::String>("serial_reciver",10);
        timer_ = create_wall_timer(0.01s,std::bind(&SimpleSerialReciver::CallbackTimer,this));
    }

     void CallbackTimer()
    {
        auto message = std_msgs::msg::String();
        if(rclcpp::ok() && arduino_.IsDataAvailable())
        {
            arduino_.ReadLine(message.data);
        }
        
       publisher_->publish(message);
    }
    ~SimpleSerialReciver()
    {
        arduino_.Close();
    }
    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort arduino_;
    std::string port_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleSerialReciver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}