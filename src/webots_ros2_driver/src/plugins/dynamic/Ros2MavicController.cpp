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

#include <webots_ros2_driver/plugins/dynamic/Ros2MavicController.hpp>

#include <algorithm>
#include "pluginlib/class_list_macros.hpp"

#include <webots/robot.h>

namespace webots_ros2_driver {

  // Same constants as the stock (upstream) mavic_driver.py, minus the
  // horizontal-velocity (cmd_vel) terms this course's assignment doesn't use.
  static constexpr double K_ROLL_P = 50.0;
  static constexpr double K_PITCH_P = 30.0;
  static constexpr double K_YAW_P = 2.0;

  // Hover-thrust disturbance ("battery weakening"), exact port of the real
  // mavic_driver.py: uniform[-0.5,0.5] once per connection, then every 100
  // physics steps nudged by uniform[-0.01, 0.0] (one-sided random walk).
  static constexpr int DRIFT_STEP_INTERVAL = 100;

  static double clamp(double value, double lo, double hi) {
    return std::min(std::max(value, lo), hi);
  }

  void Ros2MavicController::init(webots_ros2_driver::WebotsNode *node,
                                  std::unordered_map<std::string, std::string> &parameters) {
    mNode = node;
    mThrust = 0.0;
    mOffsets[0] = mOffsets[1] = mOffsets[2] = mOffsets[3] = 0.0;

    mGyro = wb_robot_get_device("gyro");
    mInertialUnit = wb_robot_get_device("inertial unit");
    mFrontRightMotor = wb_robot_get_device("front right propeller");
    mFrontLeftMotor = wb_robot_get_device("front left propeller");
    mRearRightMotor = wb_robot_get_device("rear right propeller");
    mRearLeftMotor = wb_robot_get_device("rear left propeller");

    assert(mGyro != 0 && mInertialUnit != 0);
    assert(mFrontRightMotor != 0 && mFrontLeftMotor != 0 && mRearRightMotor != 0 && mRearLeftMotor != 0);

    // Enable directly rather than relying on another plugin (e.g. Ros2IMU) to
    // have already done so - wb_*_enable is idempotent, so this is safe even
    // if one is also declared in the URDF.
    wb_gyro_enable(mGyro, static_cast<int>(wb_robot_get_basic_time_step()));
    wb_inertial_unit_enable(mInertialUnit, static_cast<int>(wb_robot_get_basic_time_step()));

    for (WbDeviceTag motor : {mFrontRightMotor, mFrontLeftMotor, mRearRightMotor, mRearLeftMotor}) {
      wb_motor_set_position(motor, INFINITY);
      wb_motor_set_velocity(motor, 0.0);
    }

    const std::string topicName = parameters.count("topicName") ? parameters["topicName"] : "thrust";
    mSubscriber = mNode->create_subscription<std_msgs::msg::Float64MultiArray>(
      topicName, rclcpp::SensorDataQoS().reliable(),
      std::bind(&Ros2MavicController::onThrustReceived, this, std::placeholders::_1));

    std::random_device rd;
    mRng.seed(rd());
    std::uniform_real_distribution<double> dist(-0.5, 0.5);
    mThrustOffset = dist(mRng);
    mStepCount = 0;
  }

  void Ros2MavicController::onThrustReceived(const std_msgs::msg::Float64MultiArray::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mMutex);
    if (message->data.size() == 1) {
      mThrust = message->data[0];
      mOffsets[0] = mOffsets[1] = mOffsets[2] = mOffsets[3] = 0.0;
    } else if (message->data.size() == 5) {
      mThrust = message->data[0];
      mOffsets[0] = message->data[1];
      mOffsets[1] = message->data[2];
      mOffsets[2] = message->data[3];
      mOffsets[3] = message->data[4];
    }
    // any other size: ignore, keep the previous command
  }

  void Ros2MavicController::step() {
    double thrust;
    double offsets[4];
    {
      std::lock_guard<std::mutex> lock(mMutex);
      thrust = mThrust;
      std::copy(std::begin(mOffsets), std::end(mOffsets), std::begin(offsets));
    }

    const double roll = wb_inertial_unit_get_roll_pitch_yaw(mInertialUnit)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(mInertialUnit)[1];
    const double roll_velocity = wb_gyro_get_values(mGyro)[0];
    const double pitch_velocity = wb_gyro_get_values(mGyro)[1];
    const double yaw_velocity = wb_gyro_get_values(mGyro)[2];

    const double roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_velocity;
    const double pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_velocity;
    const double yaw_input = K_YAW_P * (0.0 - yaw_velocity);

    ++mStepCount;
    if (mStepCount % DRIFT_STEP_INTERVAL == 0) {
      std::uniform_real_distribution<double> driftDist(-0.01, 0.0);
      mThrustOffset += driftDist(mRng);
    }
    const double effectiveThrust = thrust + mThrustOffset;

    const double m1 = effectiveThrust + offsets[0] + yaw_input + pitch_input + roll_input;
    const double m2 = effectiveThrust + offsets[1] - yaw_input + pitch_input - roll_input;
    const double m3 = effectiveThrust + offsets[2] - yaw_input - pitch_input + roll_input;
    const double m4 = effectiveThrust + offsets[3] + yaw_input - pitch_input - roll_input;

    // Same sign convention as stock mavic_driver.py: propellers[0]=front right,
    // [1]=front left, [2]=rear right, [3]=rear left.
    wb_motor_set_velocity(mFrontRightMotor, -m1);
    wb_motor_set_velocity(mFrontLeftMotor, m2);
    wb_motor_set_velocity(mRearRightMotor, m3);
    wb_motor_set_velocity(mRearLeftMotor, -m4);
  }
}  // namespace webots_ros2_driver

PLUGINLIB_EXPORT_CLASS(webots_ros2_driver::Ros2MavicController, webots_ros2_driver::PluginInterface)
