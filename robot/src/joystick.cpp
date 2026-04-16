// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "read_write_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node
{
public:
      MinimalPublisher()
  : Node("minimal_publisher")
  {

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&MinimalPublisher::joy_callback, this, std::placeholders::_1));
    pos_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
            std::bind(&MinimalPublisher::pos_callback, this, std::placeholders::_1));


    publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
   
    set_initial_position();
    };

private:
    const double MIN_RAD = -3.14;
    const double MAX_RAD = 3.14;
    void pos_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        pos_ = msg;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        { 
            //A button buttons[0] id 1
            //B button buttons[1] id 2
            //X button buttons[2] id 3
            //Y button buttons[3] id 4
            //LB button buttons[4] up/lef
            //RB button buttons[5] down/right
            auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
            double new_pos;
            if(msg->buttons[0]){
                new_pos= pos_->position[0];
                message.id = 1;
                if(msg->buttons[4]){
                    new_pos += 0.3;
                }
                else if(msg->buttons[5]){
                    new_pos -= 0.3;
                }
                new_pos = std::clamp(new_pos, MIN_RAD, MAX_RAD);
                message.position = static_cast<uint32_t>((new_pos * 4096.0 / (2.0 * M_PI)) + 2048.0);
                this->publisher_->publish(message);

            }
            if(msg->buttons[1]){
                new_pos= pos_->position[1];
                message.id = 2;
                if(msg->buttons[4]){
                    new_pos += 0.1;
                }
                else if(msg->buttons[5]){
                    new_pos -= 0.3;
                }
                new_pos= std::clamp(new_pos, MIN_RAD, MAX_RAD);
                message.position = static_cast<uint32_t>((new_pos * 4096.0 / (2.0 * M_PI)) + 2048.0);
                this->publisher_->publish(message);

            }
            if(msg->buttons[2]){
                new_pos= pos_->position[2];
                message.id = 3;
                if(msg->buttons[4]){
                    new_pos += 0.1;
                }
                else if(msg->buttons[5]){
                    new_pos -= 0.3;
                }
                new_pos = std::clamp(new_pos, MIN_RAD, MAX_RAD);
                message.position = static_cast<uint32_t>((new_pos * 4096.0 / (2.0 * M_PI)) + 2048.0);
                this->publisher_->publish(message);

            }
            if(msg->buttons[3]){
                new_pos= pos_->position[3];
                message.id = 4;
                if(msg->buttons[4]){
                    new_pos += 0.1;
                }
                else if(msg->buttons[5]){
                    new_pos -= 0.3;
                }
                new_pos = std::clamp(new_pos, MIN_RAD, MAX_RAD);
                message.position = static_cast<uint32_t>((new_pos * 4096.0 / (2.0 * M_PI)) + 2048.0);
                this->publisher_->publish(message);
            }
           if(msg->buttons[10]){
                new_pos= pos_->position[4];
                message.id = 5;
                if(msg->buttons[4]){
                    new_pos += 0.005; // Increment by 5 mm
                }
                else if(msg->buttons[5]){
                    new_pos -= 0.005; // Decrement by 5 mm
                }
                // Clamp the new position to the valid linear range (0 to 0.04 meters)
                new_pos = std::clamp(new_pos, 0.0, 0.04);
                
                // Convert the linear position (meters) to the Dynamixel's raw uint32_t value
                // Assuming a 0-0.04 m linear travel corresponds to a 0-4095 raw value.
                message.position = static_cast<uint32_t>((new_pos / 0.04) * 4095.0);
                this->publisher_->publish(message);
            }


        }

      void set_initial_position()
      {
        auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
        // Set a position of 0 radians for joints 1-5
        // 0 radians converts to a raw value of 2048 in the Dynamixel SDK's uint32_t
        message.position = 2048;

        for (int id = 5; id >= 1; --id) {
            message.id = id;
            this->publisher_->publish(message);
        }

      }
      sensor_msgs::msg::JointState::SharedPtr pos_;
      rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pos_subscription_;
    };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
