// Copyright 2021 ROBOTIS CO., LTD.
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

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "read_write_node.hpp"

using namespace std::chrono_literals;
using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_LOAD 126

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
//#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB1"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

class ArmNode : public rclcpp::Node
{
    public:
        ArmNode() : Node("arm")
        {       
            this->declare_parameter("qos_depth", 10);
            int8_t qos_depth = 0;
            this->get_parameter("qos_depth", qos_depth);

            const auto QOS_RKL10V =
            rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

            set_position_subscriber_ =
                this->create_subscription<SetPosition>(
                "set_position",
                QOS_RKL10V,
                [this](const SetPosition::SharedPtr msg) -> void
                {
                  uint8_t dxl_error = 0;

                  // Position Value of X series is 4 byte data.
                  // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
                  uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

                  // Write Goal Position (length : 4 bytes)
                  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
                  dxl_comm_result =
                  packetHandler->write4ByteTxRx(
                    portHandler,
                    (uint8_t) msg->id,
                    ADDR_GOAL_POSITION,
                    goal_position,
                    &dxl_error
                  );

                  if (dxl_comm_result != COMM_SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
                  } else if (dxl_error != 0) {
                    RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
                  } else {
                    RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
                  }
                }
            );

            auto timer_callback =
              [this]() -> void {
                auto message = sensor_msgs::msg::JointState();
                message.header.stamp = this->get_clock()->now();
                message.name = {"shoulder_joint", "elbow_joint", "wrist_joint", "gripper_joint", "left_finger_joint", "right_finger_joint"};
                message.position = {0,0,0,0,0,0};
                message.effort = {0,0,0,0,0,0};
                for(int servo=1;servo<=5;servo++){
                    uint32_t present_position;
                    uint16_t present_load;
                    dxl_comm_result = packetHandler->read4ByteTxRx(
                        portHandler,
                        (uint8_t) servo,
                        ADDR_PRESENT_POSITION,
                        reinterpret_cast<uint32_t *>(&present_position),
                        &dxl_error
                    );

                    dxl_comm_result = packetHandler->read2ByteTxRx(
                        portHandler,
                        (uint8_t) servo,
                        ADDR_PRESENT_LOAD,
                        reinterpret_cast<uint16_t *>(&present_load),
                        &dxl_error
                    );

                    if (servo == 5) { 
                        // Convert to linear position in meters (0 to 0.04 m)
                        message.position[servo-1] = ((double)present_position) / 4096.0 * 0.04;
                    } else {
                        // Convert to angular position in radians
                        message.position[servo-1]=((double)(present_position)-2048.0)/4096.0*6.283185;
                    }
                    message.effort[servo-1] = (double)((int16_t)present_load)/1000.0 * 1.5;
                }
                message.position[5] = message.position[4];
                message.effort[5] = message.effort[4];

                this->publisher_->publish(message);
            };           
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
            timer_ = this->create_wall_timer(500ms, timer_callback);
          }

        private:
          rclcpp::TimerBase::SharedPtr timer_;
          rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
          rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
        };
            

 
void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("arm"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("arm"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("arm"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("arm"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("arm"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("arm"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("arm"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("arm"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto armnode = std::make_shared<ArmNode>();
  rclcpp::spin(armnode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
