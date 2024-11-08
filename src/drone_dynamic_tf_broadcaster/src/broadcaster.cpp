#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class FramePublisher : public rclcpp::Node
{
    public:
        FramePublisher()
        : Node("drone_dynamic_tf_broadcaster")
        {

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&FramePublisher::handle_drone_pose, this, std::placeholders::_1));
        }

    private:
        void handle_drone_pose(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
        {
            geometry_msgs::msg::TransformStamped t;

            // Read message content and assign it to
            // corresponding tf variables
            t.header.stamp = this->get_clock()->now();
            // t.header.stamp = msg->header.stamp;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";

            // Turtle only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = msg->pose.position.x;
            t.transform.translation.y = msg->pose.position.y;
            t.transform.translation.z = msg->pose.position.z;

            // For the same reason, turtle can only rotate around one axis
            // and this why we set rotation in x and y to 0 and obtain
            // rotation in z axis from the message
            t.transform.rotation.x = msg->pose.orientation.x;
            t.transform.rotation.y = msg->pose.orientation.y;
            t.transform.rotation.z = msg->pose.orientation.z;
            t.transform.rotation.w = msg->pose.orientation.w;

            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
