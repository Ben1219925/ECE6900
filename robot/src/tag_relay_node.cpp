#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using AprilTagArray = apriltag_msgs::msg::AprilTagDetectionArray;
using PoseMsg = geometry_msgs::msg::Pose;

class TagRelayNode : public rclcpp::Node {
public:
    TagRelayNode() : Node("tag_relay_node") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscription for Camera 1
        sub1_ = this->create_subscription<AprilTagArray>(
            "camera1/detections", 10, 
            std::bind(&TagRelayNode::tag_callback, this, std::placeholders::_1));

        // Subscription for Camera 2
        sub2_ = this->create_subscription<AprilTagArray>(
            "camera2/detections", 10, 
            std::bind(&TagRelayNode::tag_callback, this, std::placeholders::_1));

        // Create TWO publishers
        pub_map_ = this->create_publisher<PoseMsg>("nav_tag_pose", 10);
        pub_odom_ = this->create_publisher<PoseMsg>("track_tag_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&TagRelayNode::publish_last_known, this));
            
        RCLCPP_INFO(this->get_logger(), "Tag Relay Node Started (Dual Camera & Dual Frame Mode).");
    }

private:
   void tag_callback(const AprilTagArray::SharedPtr msg) {
        if (msg->detections.empty()) return;

        // Note: This logic takes the first detection from the message.
        // If both cameras see the tag, the latest message received will update the pose.
        auto detection = msg->detections[0];
        std::string tag_frame = detection.family + ":" + std::to_string(detection.id);

        // Check if BOTH transforms are available
        if (!tf_buffer_->canTransform("map", tag_frame, tf2::TimePointZero, tf2::durationFromSec(0.05)) ||
            !tf_buffer_->canTransform("odom", tag_frame, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
            return;
        }

        try {
            // 1. Lookup MAP transform (for Nav2)
            auto t_map = tf_buffer_->lookupTransform("map", tag_frame, tf2::TimePointZero);
            last_pose_map_.position.x = t_map.transform.translation.x;
            last_pose_map_.position.y = t_map.transform.translation.y;
            last_pose_map_.position.z = t_map.transform.translation.z;
            last_pose_map_.orientation = t_map.transform.rotation;

            // 2. Lookup ODOM transform (for Arm Tracking)
            auto t_odom = tf_buffer_->lookupTransform("odom", tag_frame, tf2::TimePointZero);
            last_pose_odom_.position.x = t_odom.transform.translation.x;
            last_pose_odom_.position.y = t_odom.transform.translation.y;
            last_pose_odom_.position.z = t_odom.transform.translation.z;
            last_pose_odom_.orientation = t_odom.transform.rotation;
            
            has_seen_tag_ = true;
            
        } catch (const tf2::TransformException & ex) {
            RCLCPP_DEBUG(this->get_logger(), "TF transient error: %s", ex.what());
        }
    }

    void publish_last_known() {
        if (has_seen_tag_) {
            pub_map_->publish(last_pose_map_); 
            pub_odom_->publish(last_pose_odom_); 
        }
    }

    // Two subscriber handles
    rclcpp::Subscription<AprilTagArray>::SharedPtr sub1_;
    rclcpp::Subscription<AprilTagArray>::SharedPtr sub2_;
    
    rclcpp::Publisher<PoseMsg>::SharedPtr pub_map_;
    rclcpp::Publisher<PoseMsg>::SharedPtr pub_odom_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    PoseMsg last_pose_map_;
    PoseMsg last_pose_odom_;
    bool has_seen_tag_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagRelayNode>());
    rclcpp::shutdown();
    return 0;
}
