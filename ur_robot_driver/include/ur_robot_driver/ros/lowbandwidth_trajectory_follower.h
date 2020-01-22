/*
 * Copyright 2017, 2018 Jarek Potiuk (low bandwidth trajectory follower)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <future>
#include <string>
#include <thread>
#include <vector>
#include "ur_robot_driver/log.h"
#include "ur_robot_driver/ros/action_trajectory_follower_interface.h"
#include "ur_robot_driver/comm/server.h"

class LowBandwidthTrajectoryFollower : public ActionTrajectoryFollowerInterface
{
public:
  LowBandwidthTrajectoryFollower(int reverse_port, bool version_3);

  bool start();
  bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt);
  void stop();

  virtual ~LowBandwidthTrajectoryFollower(){};

private:
  int reverse_port_;
  std::thread comm_thread_;
  std::atomic<bool> connected_;
  std::unique_ptr<ur_driver::comm::URServer> server_;

  std::mutex trajectory_mutex_;
  std::vector<TrajectoryPoint> trajectory_;
  std::atomic<bool> cancel_request_;
  std::atomic<bool> trajectory_execution_finished_;
  std::atomic<bool> trajectory_execution_success_;

  void runSocketComm();
  bool executePoint(const std::array<double, 6> &positions, const std::array<double, 6> &velocities, double sample_number,
                    double time_in_seconds, bool is_sentinel);

};
