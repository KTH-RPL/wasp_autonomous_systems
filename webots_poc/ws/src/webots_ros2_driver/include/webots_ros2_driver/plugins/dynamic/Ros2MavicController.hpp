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

// EXPERIMENTAL: this plugin does not exist upstream. It is a C++ port of
// webots_ros2_mavic's mavic_driver.py (the embedded Python plugin, which
// crashes on macOS - see PythonPlugin.cpp), modified to accept an externally
// computed thrust value in place of its own vertical PID, matching the
// WASP AS course's assignment_4 Thrust(m1..m4)/altitude_manual interface:
// a single shared thrust plus four per-motor offsets, summed per motor
// (thrust + m1_offset -> motor1, etc). It still runs the same roll/pitch/yaw
// stabilization as the stock driver, and - critically - applies all four
// motor velocities atomically within a single step(), avoiding the
// independent-topic timing race that four separate Ros2Motor subscriptions
// have at liftoff.
//
// Also reproduces last year's "hover thrust isn't constant" disturbance -
// an exact port of the real mavic_driver.py from
// github.com/danielduberg/webots_ros2 (pinned as a submodule in the ht24/
// ht25 course repos, never itself committed to the course repo, recovered
// by fetching that fork directly): a uniform[-0.5, 0.5] offset drawn once
// per connection, then every 100 physics steps nudged by
// uniform[-0.01, 0.0] (a one-sided random walk - only ever grows the
// required thrust, modeling "battery weakening"). Added directly to the
// commanded thrust, same as the original.

#ifndef ROS2_MAVIC_CONTROLLER_HPP
#define ROS2_MAVIC_CONTROLLER_HPP

#include <mutex>
#include <random>
#include <unordered_map>

#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>

namespace webots_ros2_driver {

  class Ros2MavicController : public PluginInterface {
  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

  private:
    void onThrustReceived(const std_msgs::msg::Float64MultiArray::SharedPtr message);

    webots_ros2_driver::WebotsNode *mNode;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mSubscriber;

    WbDeviceTag mGyro;
    WbDeviceTag mInertialUnit;
    WbDeviceTag mFrontRightMotor;
    WbDeviceTag mFrontLeftMotor;
    WbDeviceTag mRearRightMotor;
    WbDeviceTag mRearLeftMotor;

    std::mutex mMutex;
    double mThrust;
    double mOffsets[4];

    std::mt19937 mRng;
    double mThrustOffset;
    unsigned long mStepCount;
  };

}  // namespace webots_ros2_driver

#endif
