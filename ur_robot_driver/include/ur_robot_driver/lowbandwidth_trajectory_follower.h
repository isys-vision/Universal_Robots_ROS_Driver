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
#include <functional>
#include <future>
#include <string>
#include <thread>
#include <vector>
#include "ur_robot_driver/action_trajectory_follower_interface.h"
#include "ur_client_library/comm/tcp_server.h"

class LowBandwidthTrajectoryFollower : public ActionTrajectoryFollowerInterface
{
public:
  LowBandwidthTrajectoryFollower(uint32_t reverse_port, std::function<void(bool)> handle_program_state);

  ~LowBandwidthTrajectoryFollower(){};

  bool start();
  bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt);
  void stop();

private:
  int reverse_port_;
  std::function<void(bool)> handle_program_state_;

  urcl::comm::TCPServer server_;

  std::mutex trajectory_mutex_;
  std::vector<TrajectoryPoint> trajectory_;
  std::vector<TrajectoryPoint> current_trajectory_;
  std::atomic<bool> cancel_request_;
  std::atomic<bool> trajectory_execution_finished_;
  std::atomic<bool> trajectory_execution_success_;

  int client_fd_;
  int sent_message_num_;

  void connectionCallback(const int filedescriptor);
  void disconnectionCallback(const int filedescriptor);
  void messageCallback(const int filedescriptor, char* buffer);

  bool executePoint(const std::array<double, 6> &positions, const std::array<double, 6> &velocities, double sample_number,
                    double time_in_seconds, bool is_sentinel);

};
