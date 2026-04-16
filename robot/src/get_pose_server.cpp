#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_interfaces/srv/get_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <memory>

class ServerNode : public rclcpp::Node {
    public:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;

        ServerNode() : Node("pose_server") {

            srv_ = create_service<ros_interfaces::srv::GetPose>(
                    "get_pose", std::bind(&ServerNode::callback, this,
                        std::placeholders::_1, std::placeholders::_2));
            std::string target_frame_ ="map";

            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        rclcpp::Service<ros_interfaces::srv::GetPose>::SharedPtr srv_;

        void callback(
                const std::shared_ptr<ros_interfaces::srv::GetPose::Request> request,
                const std::shared_ptr<ros_interfaces::srv::GetPose::Response> response) {
            std::string fromFrameRel = "base_link";
            std::string toFrameRel = "map";
            (void)request; // unused
            geometry_msgs::msg::TransformStamped t;

            // Look up for the transformation between target_frame and turtle2 frames
            // and send velocity commands for turtle2 to reach target_frame
            try {
                t = tf_buffer_->lookupTransform(
                        toFrameRel, fromFrameRel,
                        tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                        this->get_logger(), "Could not transform %s to %s: %s",
                        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                return;
            }
            response->pose.position.x = t.transform.translation.x;
            response->pose.position.y = t.transform.translation.y;
            response->pose.position.z = t.transform.translation.z;
            response->pose.orientation.x = t.transform.rotation.x;
            response->pose.orientation.y = t.transform.rotation.y;
            response->pose.orientation.z = t.transform.rotation.z;
            response->pose.orientation.w = t.transform.rotation.w;
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServerNode>());
    rclcpp::shutdown();
    return 0;
}

// End of Code
