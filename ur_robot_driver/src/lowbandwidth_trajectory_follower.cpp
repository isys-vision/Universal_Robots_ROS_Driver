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

LowBandwidthTrajectoryFollower::LowBandwidthTrajectoryFollower(int reverse_port, bool version_3)
  : reverse_port_(reverse_port)
    , comm_thread_()
    , connected_(false)
    , cancel_request_(false)
    , trajectory_execution_finished_(false)
    , trajectory_execution_success_(false)
{

  if (!version_3)
  {
    ROS_ERROR("Low Bandwidth Trajectory Follower only works for interface version > 3");
    std::exit(-1);
  }

  comm_thread_ = std::thread(&LowBandwidthTrajectoryFollower::runSocketComm, this);
  ROS_INFO("Low Bandwidth Trajectory Follower is initialized!");
}

bool LowBandwidthTrajectoryFollower::executePoint(const std::array<double, 6> &positions,
                                                     const std::array<double, 6> &velocities, double sample_number,
                                                     double time_in_seconds, bool is_sentinel)
{
  if (!connected_ && server_) {
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
  ROS_DEBUG("Sending message %s", formatted_message);

  return server_->write(buf, strlen(formatted_message) + 1, written);
}

void LowBandwidthTrajectoryFollower::runSocketComm()
{
    while (true)    // reconnect loop
    {
        ROS_INFO("Awaiting incoming robot connection");
        server_.reset(new urcl::comm::URServer(reverse_port_));
        if (!server_->bind())
        {
          ROS_ERROR("Failed to bind server, the port %d is likely already in use", reverse_port_);
          std::exit(-1);
        }
        if (!server_->accept())
        {
            ROS_ERROR("Failed to accept incoming robot connection");
            continue;
        }

        ROS_INFO("Robot successfully connected");
        connected_ = true;

        int sent_message_num = -1;
        char *line[MAX_SERVER_BUF_LEN];
        std::vector<TrajectoryPoint> current_trajectory;


        while (true) {   // keepalive and process trajectory loop

            // Fetch new trajectories, trajectory_execution_finished_ signals status
            if (current_trajectory.empty()){
                std::unique_lock<std::mutex> trajectory_lock (trajectory_mutex_);
                current_trajectory = trajectory_;
                trajectory_.clear();
            }
            int current_trajectory_size = static_cast<int>(current_trajectory.size());

            // Read data from robot: we expect a package in each timestep.
            // Either we receive next requested point or same point index which has already be sent.
            if (!server_->readLine((char *)line, MAX_SERVER_BUF_LEN)) {
              ROS_INFO("Connection to robot lost!");
              connected_ = false;
              trajectory_execution_finished_ = true;
              current_trajectory.clear();
              server_->disconnectClient();
              break;
            }

            int req_message_num = atoi((const char *)line);
            if (req_message_num == -1) {
               ROS_DEBUG("Received success message");
               trajectory_execution_success_ = true;
               sent_message_num = -1;
               trajectory_execution_finished_ = true;
               current_trajectory.clear();
               continue;
            } else if (req_message_num > sent_message_num && !current_trajectory.empty()){
                ROS_DEBUG("Received request from robot: %i", req_message_num);

                if (sent_message_num < req_message_num) {
                    if (cancel_request_) {
                        ROS_DEBUG("Cancel requested, sending sentinel: %i", req_message_num);
                        trajectory_execution_success_ = executePoint(EMPTY_VALUES, EMPTY_VALUES, req_message_num, 0.0, true);
                    } else if (req_message_num < current_trajectory_size) {
                        ROS_DEBUG("Sending waypoint: %i", req_message_num);
                        trajectory_execution_success_ = executePoint(current_trajectory[req_message_num].positions, current_trajectory[req_message_num].velocities, req_message_num,
                                    current_trajectory[req_message_num].time_from_start.count() / 1e6, false);
                    } else if (req_message_num >= current_trajectory_size) {
                        ROS_DEBUG("Sending sentinel: %i", req_message_num);
                        trajectory_execution_success_ = executePoint(EMPTY_VALUES, EMPTY_VALUES, req_message_num, 0.0, true);
                    }
                }
                sent_message_num = req_message_num;


                if (!trajectory_execution_success_) {
                    sent_message_num = -1;
                    trajectory_execution_finished_ = true;
                    current_trajectory.clear();
                    continue;
                }
            } else {
                // Everything else is interpreted as Keepalive signal
                //ROS_DEBUG("Keepalive received: (value %i)", req_message_num);
                continue;
            }

        }

        server_.reset(nullptr);


    }
}

bool LowBandwidthTrajectoryFollower::start()
{
//  ROS_INFO("Starting LowBandwidthTrajectoryFollower");

//  if (connected_)
    return true;  // not sure


}



bool LowBandwidthTrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt)
{
    {
        std::unique_lock<std::mutex> trajectory_lock (trajectory_mutex_);
        trajectory_ = trajectory;   // starts execution of trajectory
    }
    trajectory_execution_finished_ = false;
    while (!trajectory_execution_finished_) {
        cancel_request_ = (bool) interrupt;
    }
    ROS_INFO("Execute done, returning result (cancel: %i)", (int) cancel_request_);

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
