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

#include "ur_robot_driver/lowbandwidth_trajectory_follower.h"
#include <endian.h>
#include <ros/ros.h>
#include <cmath>
#include <chrono>

static const std::array<double, 6> EMPTY_VALUES = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

LowBandwidthTrajectoryFollower::LowBandwidthTrajectoryFollower(uint32_t reverse_port, std::function<void(bool)> handle_program_state)
  : reverse_port_(reverse_port)
  , handle_program_state_(handle_program_state)
  , server_(reverse_port)
  , watchdog_reset_(false)
  , cancel_request_(false)
  , trajectory_execution_finished_(false)
  , trajectory_execution_success_(false)
  , client_fd_(-1)
  , sent_message_num_(-1)
{
    handle_program_state_(false);
    server_.setMessageCallback(std::bind(&LowBandwidthTrajectoryFollower::messageCallback, this, std::placeholders::_1, std::placeholders::_2));
    server_.setConnectCallback(std::bind(&LowBandwidthTrajectoryFollower::connectionCallback, this, std::placeholders::_1));
    server_.setDisconnectCallback(std::bind(&LowBandwidthTrajectoryFollower::disconnectionCallback, this, std::placeholders::_1));
    server_.setMaxClientsAllowed(1);
    server_.start(); // Start event handling
    ROS_INFO("[LowBandwidthTrajectoryFollower] Initialized");
}

bool LowBandwidthTrajectoryFollower::executePoint(const std::array<double, 6> &positions,
                                                     const std::array<double, 6> &velocities, double sample_number,
                                                     double time_in_seconds, bool is_sentinel)
{
  if (client_fd_ < 0) {
    ROS_ERROR("[LowBandwidthTrajectoryFollower] Execute Point not possible, no connection to robot");
    return false;
  }

  std::ostringstream out;

  out << "(";
  out << (int) is_sentinel << ",";
  out << sample_number << ",";
  for (auto const &pos : positions)
  {
    out << pos << ",";
  }
  for (auto const &vel : velocities)
  {
    out << vel << ",";
  }
  out << time_in_seconds << ")\r\n";

  // ugly but it's the most efficient and fastest way
  // We have only ASCII characters and we can cast char -> uint_8
  const std::string tmp = out.str();
  const char *formatted_message = tmp.c_str();
  const uint8_t *buf = (uint8_t *)formatted_message;

  size_t written;
  ROS_DEBUG("Sending message %s", formatted_message);

  return server_.write(client_fd_, buf, strlen(formatted_message) + 1, written);
}

bool LowBandwidthTrajectoryFollower::start()
{
    //ROS_DEBUG("Start LowBandwidthTrajectoryFollower");
    return true;  // not sure
}

bool LowBandwidthTrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt)
{
    // TODO: in some cases the driver gets stuck in this execute loop
    // - if many trajectories are sent in quick succession, i.e. if the jogging buttons are pressed in very quick succession
    // - unplugging the cable immediately stops messageCallback() from reveiving any messages and without a disconnectionCallback() for some reason
    //   'trajectory_execution_finished_' is then never set to true if this happens while executing a trajectory.
    ROS_INFO("[LowBandwidthTrajectoryFollower] Starting execution of trajectory request");
    {
        std::unique_lock<std::mutex> trajectory_lock (trajectory_mutex_);
        trajectory_ = trajectory;   // starts execution of trajectory
    }

    // Initialize watchdog
    const std::chrono::milliseconds messageCallbackTimeout(1000);
    auto lastMessageCallbackTime = std::chrono::steady_clock::now();
    watchdog_reset_ = true;

    // Wait for trajectory execution to finish and handle watchdog
    trajectory_execution_finished_ = false;
    while (!trajectory_execution_finished_) {
        cancel_request_ = (bool) interrupt;

        if (watchdog_reset_) {
            lastMessageCallbackTime = std::chrono::steady_clock::now();
            watchdog_reset_ = false;
        } else {
            auto now = std::chrono::steady_clock::now();
            if (now - lastMessageCallbackTime > messageCallbackTimeout) {
                ROS_ERROR("[LowBandwidthTrajectoryFollower] No message received from robot within timeout (%d ms) during trajectory execution! Connection to robot lost.", static_cast<int>(messageCallbackTimeout.count()));
                sent_message_num_ = -1;
                trajectory_execution_finished_ = true;
                trajectory_execution_success_ = false;
                current_trajectory_.clear();
            }
        }
    }
    ROS_INFO("[LowBandwidthTrajectoryFollower] Execute done, returning result (cancel: %i)", (int) cancel_request_);

    return trajectory_execution_success_;
}

void LowBandwidthTrajectoryFollower::stop()
{
    //ROS_DEBUG("Stop LowBandwidthTrajectoryFollower");
}

void LowBandwidthTrajectoryFollower::connectionCallback(const int filedescriptor)
{
    if (client_fd_ < 0)
    {
        ROS_INFO("[LowBandwidthTrajectoryFollower] Robot connected at FD %d. Ready to receive trajectory commands.", filedescriptor);
        client_fd_ = filedescriptor;
        handle_program_state_(true);
    }
    else
    {
        ROS_ERROR("[LowBandwidthTrajectoryFollower] Connection request received while connection is already established. Only one "
                   "connection is allowed at a time. Ignoring this request.");
    }
}

void LowBandwidthTrajectoryFollower::disconnectionCallback(const int filedescriptor)
{
    ROS_INFO("[LowBandwidthTrajectoryFollower] Client at FD %d disconnected.", filedescriptor);
    client_fd_ = -1;
    handle_program_state_(false);
    sent_message_num_ = -1;
    trajectory_execution_finished_ = true;
    trajectory_execution_success_ = false;
    current_trajectory_.clear();
}

void LowBandwidthTrajectoryFollower::messageCallback(const int filedescriptor, char* buffer)
{
    watchdog_reset_ = true; // Signal that a message was received

    // Fetch new trajectories, trajectory_execution_finished_ signals status
    if (current_trajectory_.empty()){
        std::unique_lock<std::mutex> trajectory_lock (trajectory_mutex_);
        current_trajectory_ = trajectory_;
        trajectory_.clear();
    }

    int current_trajectory_size = static_cast<int>(current_trajectory_.size());

    int req_message_num = atoi((const char *)buffer);
    if (req_message_num == -1) {
        ROS_INFO("Received success message");
        trajectory_execution_success_ = true;
        sent_message_num_ = -1;
        trajectory_execution_finished_ = true;
        current_trajectory_.clear();
        return;
    } else if (req_message_num > sent_message_num_ && !current_trajectory_.empty()){
        ROS_INFO("Received request from robot: %i", req_message_num);

        if (sent_message_num_ < req_message_num) {
            if (cancel_request_) {
                ROS_INFO("Cancel requested, sending sentinel: %i", req_message_num);
                trajectory_execution_success_ = executePoint(EMPTY_VALUES, EMPTY_VALUES, req_message_num, 0.0, true);
            } else if (req_message_num < current_trajectory_size) {
                ROS_INFO("Sending waypoint: %i", req_message_num);
                trajectory_execution_success_ = executePoint(current_trajectory_[req_message_num].positions, current_trajectory_[req_message_num].velocities, req_message_num,
                            current_trajectory_[req_message_num].time_from_start.count() / 1e6, false);
            } else if (req_message_num >= current_trajectory_size) {
                ROS_INFO("Sending sentinel: %i", req_message_num);
                trajectory_execution_success_ = executePoint(EMPTY_VALUES, EMPTY_VALUES, req_message_num, 0.0, true);
            }
        }
        sent_message_num_ = req_message_num;


        if (!trajectory_execution_success_) {
            sent_message_num_ = -1;
            trajectory_execution_finished_ = true;
            current_trajectory_.clear();
            return;
        }
    } else {
        // Everything else is interpreted as Keepalive signal
        // ROS_INFO("Keepalive received: (value %i)", req_message_num);
        return;
    }
}
