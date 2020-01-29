// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
//
// Many parts from this (Most of the URScript program) comes from the ur_modern_driver
// Copyright 2017, 2018 Simon Rasmussen (refactor)
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/ur/ur_driver_low_bandwidth.h"
#include "ur_robot_driver/exceptions.h"
#include "ur_robot_driver/primary/primary_parser.h"
#include "ur_robot_driver/ros/lowbandwidth_trajectory_follower.h"
#include <memory>
#include <sstream>

#include <ur_robot_driver/ur/calibration_checker.h>

namespace ur_driver
{

static const std::array<double, 6> EMPTY_VALUES = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

static const std::string BEGIN_REPLACE("{{BEGIN_REPLACE}}");
static const std::string SERVOJ_TIME("{{SERVOJ_TIME}}");
static const std::string SERVOJ_TIME_WAITING("{{SERVOJ_TIME_WAITING}}");
static const std::string SERVOJ_GAIN("{{SERVOJ_GAIN}}");
static const std::string SERVOJ_LOOKAHEAD_TIME("{{SERVOJ_LOOKAHEAD_TIME}}");
static const std::string REVERSE_IP("{{REVERSE_IP}}");
static const std::string REVERSE_PORT("{{REVERSE_PORT}}");
static const std::string MAX_JOINT_DIFFERENCE("{{MAX_JOINT_DIFFERENCE}}");

ur_driver::UrDriverLowBandwidth::UrDriverLowBandwidth(const std::string& robot_ip, std::vector<std::string> joint_names,
                              const std::string& script_file,
                              const std::string& output_recipe_file, const std::string& input_recipe_file,
                              std::function<void(bool)> handle_program_state, bool headless_mode,
                              std::unique_ptr<ToolCommSetup> tool_comm_setup, const std::string& calibration_checksum,
                              const uint32_t reverse_port, const uint32_t script_sender_port)
    : servoj_time_(0.002)
    , servoj_time_waiting_(0.001)
    , servoj_gain_(300.0)
    , servoj_lookahead_time_(0.1)
    , max_joint_difference_(0.0001)
    , max_velocity_ (10.0)
    , joint_names_(joint_names)
    , reverse_interface_active_(false)
    , reverse_port_(reverse_port)
    , handle_program_state_(handle_program_state)
    , robot_ip_(robot_ip)
{
  LOG_DEBUG("Initializing urdriver");
  LOG_DEBUG("Initializing RTDE client");
  rtde_client_.reset(new rtde_interface::RTDEClient(robot_ip_, notifier_, output_recipe_file, input_recipe_file));

  primary_stream_.reset(new comm::URStream<ur_driver::primary_interface::PackageHeader>(
      robot_ip_, ur_driver::primary_interface::UR_PRIMARY_PORT));
  secondary_stream_.reset(new comm::URStream<ur_driver::primary_interface::PackageHeader>(
      robot_ip_, ur_driver::primary_interface::UR_SECONDARY_PORT));
  secondary_stream_->connect();
  LOG_INFO("Checking if calibration data matches connected robot.");
  checkCalibration(calibration_checksum);

  if (!rtde_client_->init())
  {
    throw UrException("Initialization of RTDE client went wrong.");
  }

  std::string local_ip = rtde_client_->getIP();
  robot_version_ = rtde_client_->getVersion();
  reverse_port_ = reverse_port;

  ros::param::get("~servoj_time", servoj_time_);
  ros::param::get("~servoj_time_waiting", servoj_time_waiting_);
  ros::param::get("~servoj_gain", servoj_gain_);
  ros::param::get("~servoj_lookahead_time", servoj_lookahead_time_);
  ros::param::get("~max_joint_difference", max_joint_difference_);
  ros::param::get("~max_velocity", max_velocity_);

  LOG_INFO("Used parameters (UR script):");
  LOG_INFO("  servoj_time %f, servoj_time_waiting %f, "
           "servoj_gain: %f, servoj_lookahead_time: %f, max_joint_difference: %f, max_velocity: %f",
           servoj_time_, servoj_time_waiting_, servoj_gain_, servoj_lookahead_time_, max_joint_difference_, max_velocity_);

  std::string prog = readScriptFile(script_file);
  prog.replace(prog.find(SERVOJ_TIME_WAITING), SERVOJ_TIME_WAITING.length(), std::to_string(servoj_time_waiting_));
  prog.replace(prog.find(SERVOJ_TIME), SERVOJ_TIME.length(), std::to_string(servoj_time_));
  prog.replace(prog.find(SERVOJ_GAIN), SERVOJ_GAIN.length(), std::to_string(servoj_gain_));
  prog.replace(prog.find(SERVOJ_LOOKAHEAD_TIME), SERVOJ_LOOKAHEAD_TIME.length(), std::to_string(servoj_lookahead_time_));
  prog.replace(prog.find(REVERSE_IP), REVERSE_IP.length(), local_ip);
  prog.replace(prog.find(REVERSE_PORT), REVERSE_PORT.length(), std::to_string(reverse_port));
  prog.replace(prog.find(MAX_JOINT_DIFFERENCE), MAX_JOINT_DIFFERENCE.length(), std::to_string(max_joint_difference_));

  traj_follower_.reset(new LowBandwidthTrajectoryFollower(reverse_port, robot_version_.major >= 3));
  action_server_.reset(new ActionServer(traj_follower_, joint_names_, max_velocity_));
  action_server_->start();

  std::stringstream begin_replace;
  if (tool_comm_setup != nullptr)
  {
    if (robot_version_.major < 5)
    {
      throw ToolCommNotAvailable("Tool communication setup requested, but this robot version does not support using "
                                 "the tool communication interface. Please check your configuration.",
                                 5, robot_version_.major);
    }
    begin_replace << "set_tool_voltage("
                  << static_cast<std::underlying_type<ToolVoltage>::type>(tool_comm_setup->getToolVoltage()) << ")\n";
    begin_replace << "set_tool_communication("
                  << "True"
                  << ", " << tool_comm_setup->getBaudRate() << ", "
                  << static_cast<std::underlying_type<Parity>::type>(tool_comm_setup->getParity()) << ", "
                  << tool_comm_setup->getStopBits() << ", " << tool_comm_setup->getRxIdleChars() << ", "
                  << tool_comm_setup->getTxIdleChars() << ")";
  }
  prog.replace(prog.find(BEGIN_REPLACE), BEGIN_REPLACE.length(), begin_replace.str());

  in_headless_mode_ = headless_mode;
  if (in_headless_mode_)
  {
    full_robot_program_ = "def externalControl():\n";
    std::istringstream prog_stream(prog);
    std::string line;
    while (std::getline(prog_stream, line))
    {
      full_robot_program_ += "\t" + line + "\n";
    }
    full_robot_program_ += "end\n";
    sendRobotProgram();
  }
  else
  {
    script_sender_.reset(new comm::ScriptSender(script_sender_port, prog));
    script_sender_->start();
    LOG_DEBUG("Created script sender");
  }

  //  watchdog_thread_ = std::thread(&UrDriverLowBandwidth::startWatchdog, this);

  LOG_DEBUG("Initialization done");
}

std::unique_ptr<rtde_interface::DataPackage> ur_driver::UrDriverLowBandwidth::getDataPackage()
{
  std::chrono::milliseconds timeout(100);  // We deliberately have a quite large timeout here, as the robot itself
                                           // should command the control loop's timing.
  return rtde_client_->getDataPackage(timeout);
}

bool UrDriverLowBandwidth::writeJointCommand(const vector6d_t& values)
{
  if (reverse_interface_active_)
  {
    return reverse_interface_->write(&values);
  }
  return false;
}

bool UrDriverLowBandwidth::writeKeepalive()
{
  if (reverse_interface_active_)
  {
    vector6d_t* fake = nullptr;
    return reverse_interface_->write(fake, 1);
  }
  return false;
}

void UrDriverLowBandwidth::startRTDECommunication()
{
  rtde_client_->start();
}

bool UrDriverLowBandwidth::stopControl()
{
  if (reverse_interface_active_)
  {
    vector6d_t* fake = nullptr;
    return reverse_interface_->write(fake, 0);
  }
  return false;
}

void UrDriverLowBandwidth::startWatchdog()
{
  handle_program_state_(false);
  reverse_interface_.reset(new comm::ReverseInterface(reverse_port_));
  reverse_interface_active_ = true;
  LOG_DEBUG("Created reverse interface");

  while (true)
  {
    LOG_INFO("Robot ready to receive control commands.");
    handle_program_state_(true);
    while (reverse_interface_active_ == true)
    {
      std::string keepalive = readKeepalive();

      if (keepalive == std::string(""))
      {
        reverse_interface_active_ = false;
      }
    }

    LOG_INFO("Connection to robot dropped, waiting for new connection.");
    handle_program_state_(false);
    reverse_interface_->~ReverseInterface();
    reverse_interface_.reset(new comm::ReverseInterface(reverse_port_));
    reverse_interface_active_ = true;
  }
}

std::string UrDriverLowBandwidth::readScriptFile(const std::string& filename)
{
  std::ifstream ifs(filename);
  std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return content;
}
std::string UrDriverLowBandwidth::readKeepalive()
{
  if (reverse_interface_active_)
  {
    return reverse_interface_->readKeepalive();
  }
  else
  {
    return std::string("");
  }
}

void UrDriverLowBandwidth::checkCalibration(const std::string& checksum)
{
  if (primary_stream_ == nullptr)
  {
    throw std::runtime_error("checkCalibration() called without a primary interface connection being established.");
  }
  primary_interface::PrimaryParser parser;
  comm::URProducer<ur_driver::primary_interface::PackageHeader> prod(*primary_stream_, parser);
  prod.setupProducer();

  CalibrationChecker consumer(checksum);

  comm::INotifier notifier;

  comm::Pipeline<ur_driver::primary_interface::PackageHeader> pipeline(prod, consumer, "Pipeline", notifier);
  pipeline.run();

  while (!consumer.isChecked())
  {
    ros::Duration(1).sleep();
  }
  ROS_DEBUG_STREAM("Got calibration information from robot.");
}

rtde_interface::RTDEWriter& UrDriverLowBandwidth::getRTDEWriter()
{
  return rtde_client_->getWriter();
}

bool UrDriverLowBandwidth::sendScript(const std::string& program)
{
  if (secondary_stream_ == nullptr)
  {
    throw std::runtime_error("Sending script to robot requested while there is no primary interface established. This "
                             "should not happen.");
  }

  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  auto program_with_newline = program + '\n';

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  if (secondary_stream_->write(data, len, written))
  {
    LOG_DEBUG("Sent program to robot:\n%s", program_with_newline.c_str());
    return true;
  }
  LOG_ERROR("Could not send program to robot");
  return false;
}

bool UrDriverLowBandwidth::sendRobotProgram()
{
  if (in_headless_mode_)
  {
    return sendScript(full_robot_program_);
  }
  else
  {
    LOG_ERROR("Tried to send robot program directly while not in headless mode");
    return false;
  }
}
}  // namespace ur_driver
