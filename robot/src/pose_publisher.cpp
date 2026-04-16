#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <memory>
using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node 
{
  public:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;

    PosePublisher() : Node("pose_publisher") 
  {

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pose2d", 10);
    auto timer_callback =
      [this]() -> void {
        std::string fromFrameRel = "base_link";
        std::string toFrameRel = "map";
        geometry_msgs::msg::TransformStamped t;
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
        geometry_msgs::msg::Pose2D pose;
        pose.x = t.transform.translation.x;
        pose.y = t.transform.translation.y;
        pose.theta = tf2::getYaw(t.transform.rotation);
        this->publisher_->publish(pose);
      };
    timer_ = this->create_wall_timer(100ms, timer_callback);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  };
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}

// End of Code
