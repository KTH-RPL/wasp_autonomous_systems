// Copyright 1996-2023 Cyberbotics Ltd.
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

#include <webots_ros2_driver/plugins/static/Ros2Motor.hpp>

#include <cmath>
#include <webots/robot.h>
#include <webots_ros2_driver/utils/Math.hpp>

namespace webots_ros2_driver {
  void Ros2Motor::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    mNode = node;
    mMotor = wb_robot_get_device(parameters["name"].c_str());

    assert(mMotor != 0);

    // Set position to infinity so the motor spins continuously under
    // velocity control instead of servoing to a fixed target position.
    wb_motor_set_position(mMotor, INFINITY);
    wb_motor_set_velocity(mMotor, 0.0);

    const std::string topicName =
      parameters.count("topicName") ? parameters["topicName"] : "~/" + getFixedNameString(parameters["name"]);
    mSubscriber = mNode->create_subscription<std_msgs::msg::Float64>(
      topicName, rclcpp::SensorDataQoS().reliable(), std::bind(&Ros2Motor::onMessageReceived, this, std::placeholders::_1));
  }

  void Ros2Motor::step() {
  }

  void Ros2Motor::onMessageReceived(const std_msgs::msg::Float64::SharedPtr message) {
    wb_motor_set_velocity(mMotor, message->data);
  }
}  // namespace webots_ros2_driver
