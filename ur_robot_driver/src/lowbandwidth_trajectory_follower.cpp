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

static const std::array<double, 6> EMPTY_VALUES = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

LowBandwidthTrajectoryFollower::LowBandwidthTrajectoryFollower(uint32_t reverse_port, std::function<void(bool)> handle_program_state)
  : reverse_port_(reverse_port)
  , client_fd_(-1)
  , handle_program_state_(handle_program_state)
  , connected_(false)
  , cancel_request_(false)
  , trajectory_execution_finished_(false)
  , trajectory_execution_success_(false)
  , sent_message_num_(-1)
{
  runSocketComm(); // start tcp server
}


bool LowBandwidthTrajectoryFollower::executePoint(const std::array<double, 6> &positions,
                                                     const std::array<double, 6> &velocities, double sample_number,
                                                     double time_in_seconds, bool is_sentinel)
{
  if (!server_) {
      ROS_ERROR("Execute Point not possible, communication not running");
      return false;
  }
  if (!connected_) {
    ROS_ERROR("Execute Point not possible, no connection to robot");
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
  ROS_INFO("[LowBandwidthTrajectoryFollower] Sending message %s", formatted_message);

  return server_->write(client_fd_, buf, strlen(formatted_message) + 1, written);
}

void LowBandwidthTrajectoryFollower::runSocketComm()
{
    ROS_INFO("[LowBandwidthTrajectoryFollower] Awaiting incoming robot connection");
    server_.reset(new urcl::comm::TCPServer(reverse_port_));
    handle_program_state_(false);
    server_->setMessageCallback(std::bind(&LowBandwidthTrajectoryFollower::messageCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    server_->setConnectCallback(std::bind(&LowBandwidthTrajectoryFollower::connectionCallback, this, std::placeholders::_1));
    server_->setDisconnectCallback(std::bind(&LowBandwidthTrajectoryFollower::disconnectionCallback, this, std::placeholders::_1));
    server_->setMaxClientsAllowed(10);
    server_->start();
    ROS_INFO("[LowBandwidthTrajectoryFollower] TCP Server on Port %d started.", reverse_port_);
}

bool LowBandwidthTrajectoryFollower::start()
{
//  if (connected_)
    return true;  // not sure
}

bool LowBandwidthTrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt)
{
    // TODO: in some cases the driver gets stuck in this execute loop
    // - if many trajectories are sent in quick succession, i.e. if the jogging buttons are pressed in very quick succession
    // - unplugging the cable immediately stops messageCallback() from reveiving any messages
    //   'trajectory_execution_finished_' is then never set to true if this happens while executing a trajectory.
    {
        std::unique_lock<std::mutex> trajectory_lock (trajectory_mutex_);
        trajectory_ = trajectory;   // starts execution of trajectory
    }
    trajectory_execution_finished_ = false;
    //while (!trajectory_execution_finished_ && running_comm_thread_) {
    while (!trajectory_execution_finished_) {
        cancel_request_ = (bool) interrupt;
    }
    ROS_INFO("[LowBandwidthTrajectoryFollower] Execute done, returning result (cancel: %i)", (int) cancel_request_);

    return trajectory_execution_success_;
}

void LowBandwidthTrajectoryFollower::stop()
{
//  ROS_DEBUG("LowBandwidthTrajectoryFollower::stop()");
//  if (!connected_ || !server_)
//    return;

//  server_->disconnectClient();
    //  connected_ = false;
}

bool LowBandwidthTrajectoryFollower::isConnected()
{
    return connected_;
}

void LowBandwidthTrajectoryFollower::disconnectionCallback(const int filedescriptor)
{
    ROS_ERROR("[LowBandwidthTrajectoryFollower] Connection to Robot lost.");
    client_fd_ = -1;
    connected_ = false;
    handle_program_state_(connected_);
    trajectory_execution_finished_ = true;
    sent_message_num_ = -1;
    current_trajectory_.clear();
}

void LowBandwidthTrajectoryFollower::messageCallback(const int filedescriptor, char* buffer, int nbytesrecv)
{
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
            ROS_INFO("Trajectory executin failed!");
            sent_message_num_ = -1;
            trajectory_execution_finished_ = true;
            current_trajectory_.clear();
            return;
        }
    } else {
        // Everything else is interpreted as Keepalive signal
        //ROS_INFO("Keepalive received: (value %i)", req_message_num);
        return;
    }
}

void LowBandwidthTrajectoryFollower::connectionCallback(const int filedescriptor)
{
    if (client_fd_ < 0){
        ROS_INFO("[LowBandwidthTrajectoryFollower] Robot connected. Ready to receive control commands.");
        connected_ = true;
        client_fd_ = filedescriptor;
        handle_program_state_(true);
    } else {
        ROS_ERROR("[LowBandwidthTrajectoryFollower] Connection request received while connection already established. Only one "
                        "connection is allowed at a time. Ignoring this request.");
    }
}