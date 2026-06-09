#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class SimpleQosSubscriper : public rclcpp :: Node
{
    public:
    SimpleQosSubscriper():Node("simple_qos_subscriper"),qos_profile_sub_(10)
    {
        declare_parameter<std::string>("reliability","system_default");
        declare_parameter<std::string>("durability","system_default");

        const auto reliability = get_parameter("reliability").as_string();
        const auto durability = get_parameter("durability").as_string();

        if(reliability == "best_effort")
        {
            qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            RCLCPP_INFO(get_logger(),"[Reliability]: Best Effort");
        }
        else if (reliability == "reliable")
        {
            qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            RCLCPP_INFO(get_logger(),"[Reliability]: Reliable");
        }
        else if (reliability == "system_default")
        {
            qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
            RCLCPP_INFO(get_logger(),"[Reliability]: System Default");
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(),"Selected Reliability Qos: "<<reliability<<"doesn't exist!");
            return;
        }

        if(durability == "volatile")
        {
            qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            RCLCPP_INFO(get_logger(),"[Durability]: Volatile");
        }
        else if (durability == "transient_local")
        {
            qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            RCLCPP_INFO(get_logger(),"[Durability]: Transient Local");
        }
        else if (durability == "system_default")
        {
            qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
            RCLCPP_INFO(get_logger(),"[Durability]: System Default");
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(),"Selected Durability Qos: "<<durability<<"doesn't exist!");
            return;
        }
        subscriper_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",qos_profile_sub_,
            std::bind(&SimpleQosSubscriper::MsgCallback,this,_1));
    }

    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriper_;
    rclcpp::QoS qos_profile_sub_;
    void MsgCallback(const std_msgs::msg::String &msg)const
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleQosSubscriper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}