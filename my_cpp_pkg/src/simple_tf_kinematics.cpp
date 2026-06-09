#include "my_cpp_pkg/simple_tf_kinematics.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

SimpleTfKinematics::SimpleTfKinematics(std::string name) : Node(name), x_increment_(0.05),last_x_(0.0),rotation_counter_(0)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf2_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf2_daynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster> (*this);

    static_transform_stamped_.header.stamp = get_clock()->now();

    static_transform_stamped_.header.frame_id = "ros2_base";
    static_transform_stamped_.child_frame_id = "ros2_top";

    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.3;

    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;

    tf2_static_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO_STREAM(get_logger(),"Publish Static transform between "
    <<static_transform_stamped_.header.frame_id<<", and "
    <<static_transform_stamped_.child_frame_id<<std::endl);

    timer_ = create_wall_timer(0.1s, std::bind(&SimpleTfKinematics::CallbackTimer, this));

    get_transform_srv_ = create_service<my_robot_interfaces::srv::GetTransform>("get_transform",
                        std::bind(&SimpleTfKinematics::CallbackGetTransform,this,_1,_2));

    last_orintation_.setRPY(0,0,0);
    oriantation_increment_.setRPY(0,0,0.05);
}

void SimpleTfKinematics::CallbackTimer()
{

    daynamic_transform_stamped_.header.stamp = get_clock()->now();
    daynamic_transform_stamped_.header.frame_id = "odom";
    daynamic_transform_stamped_.child_frame_id = "ros2_base";

    daynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;
    daynamic_transform_stamped_.transform.translation.y = 0.0;
    daynamic_transform_stamped_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q = last_orintation_ * oriantation_increment_;
    q.normalize();
    daynamic_transform_stamped_.transform.rotation.x = q.x();
    daynamic_transform_stamped_.transform.rotation.y = q.y();
    daynamic_transform_stamped_.transform.rotation.z = q.z();
    daynamic_transform_stamped_.transform.rotation.w = q.w();

    tf2_daynamic_broadcaster_->sendTransform(daynamic_transform_stamped_);

    last_x_ = daynamic_transform_stamped_.transform.translation.x;

    //RCLCPP_INFO_STREAM(get_logger(),"Publish daynamic transform between "
    //<<daynamic_transform_stamped_.header.frame_id<<", and "
    //<<daynamic_transform_stamped_.child_frame_id<<"\n");

    rotation_counter_++;
    last_orintation_ = q;

    if(rotation_counter_ >= 100)
    {
        oriantation_increment_ = oriantation_increment_.inverse();
        rotation_counter_ = 0;
    }
}


bool SimpleTfKinematics::CallbackGetTransform(const my_robot_interfaces::srv::GetTransform::Request::SharedPtr request,
                              const my_robot_interfaces::srv::GetTransform::Response::SharedPtr response)
{
    RCLCPP_INFO_STREAM(get_logger(),"Requsted transform betwwen "
    <<request->frame_id<<", and "<<request->child_frame_id<<" \n");

    geometry_msgs::msg::TransformStamped requested_transform;

    try
    {
        requested_transform = tf_buffer_->lookupTransform(request->frame_id,
                            request->child_frame_id,tf2::TimePointZero);
    }
    catch(tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(get_logger(),"Error occured when transform between "
        <<request->frame_id<<", and "<<request->child_frame_id<<" : "<<ex.what());
        response->success = false;
        return(true);

    }
    response->transform = requested_transform;
    response->success = true;
    return(true);
    
}


int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleTfKinematics>("Simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return(0);
}