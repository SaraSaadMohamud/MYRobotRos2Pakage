#include "my_cpp_pkg/simple_turtlesim_kinematics.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

SimpleTurtelsimKinematics::SimpleTurtelsimKinematics(const std::string &name) :Node(name)
{
    turtle1_pose_sub_ = create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,
                        std::bind(&SimpleTurtelsimKinematics::CallbackTurtle1Pose,this,_1));
    
    turtle2_pose_sub_ = create_subscription<turtlesim::msg::Pose>("/turtle2/pose",10,
                        std::bind(&SimpleTurtelsimKinematics::CallbackTurtle2Pose,this,_1));
}


void SimpleTurtelsimKinematics:: CallbackTurtle1Pose(const turtlesim::msg::Pose& pose)
{
    turtle1_last_pose_ = pose;
}

void SimpleTurtelsimKinematics:: CallbackTurtle2Pose(const turtlesim::msg::Pose& pose)
{
    turtle2_last_pose_ = pose;
    
    float Tx = turtle2_last_pose_.x - turtle1_last_pose_.x;
    float Ty = turtle2_last_pose_.y - turtle1_last_pose_.y;

    float theta_rad = turtle2_last_pose_.theta - turtle1_last_pose_.theta;
    float theta_deg = (180 * theta_rad) / 3.14 ;
    RCLCPP_INFO(get_logger(),"\nThe translation vector \
    between turtle1 and turtle2 is:\nTx: %.2f\nTy: %.2f\nthe rotation matrix between turtle1 and turtle2:\n\
    Theta:(rad): %0.2f\nTheta(deg): %0.2f\n|R11     R12| : |%0.2f       %0.2f|\n\
    |R21        R22|: |%0.2f        %0.2f|\n",
    Tx, Ty, theta_rad,theta_deg,std::cos(theta_deg),-std::sin(theta_deg),std::sin(theta_deg),std::cos(theta_deg));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleTurtelsimKinematics>("simple_turtlesim");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}