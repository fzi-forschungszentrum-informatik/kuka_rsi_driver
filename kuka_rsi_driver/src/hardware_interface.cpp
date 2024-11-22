// Copyright 2024 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!\file hardware_interface.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-21
 */
#include "kuka_rsi_driver/hardware_interface.h"

#include "kuka_rsi_driver/tracing.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace kuka_rsi_driver {

std::vector<hardware_interface::StateInterface> KukaRsiHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_joint_positions[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &m_joint_efforts[i]);
  }

  constexpr std::array cartesian_states = {"position.x",
                                           "position.y",
                                           "position.z",
                                           "orientation.x",
                                           "orientation.y",
                                           "orientation.z",
                                           "orientation.w"};
  for (std::size_t i = 0; i < cartesian_states.size(); ++i)
  {
    state_interfaces.emplace_back("tcp", cartesian_states[i], &m_cartesian_position[i]);
  }

  state_interfaces.emplace_back("robot_state", "program_state", &m_program_state);
  state_interfaces.emplace_back("speed_scaling", "speed_scaling_factor", &m_speed_scaling);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaRsiHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_joint_commands[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
KukaRsiHardwareInterface::on_init(const hardware_interface::HardwareInfo& /*system_info*/)
{
  m_joint_positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_joint_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_joint_efforts.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  m_cartesian_position.resize(7, std::numeric_limits<double>::quiet_NaN());

  for (const auto& joint : info_.joints)
  {
    // Verify position command interface
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu command interfaces, 1 expected",
                   joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has command interface '%s', expected %s",
                   joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Verify state interface
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu state interfaces, 2 expected",
                   joint.name.c_str(),
                   joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has state interface '%s', expected %s",
                   joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has state interface '%s', expected %s",
                   joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  m_rsi_factory.emplace();
  m_control_buf.emplace(*m_rsi_factory);

  const auto listen_address = info_.hardware_parameters["listen_address"];
  const auto listen_port    = info_.hardware_parameters["listen_port"];

  if (listen_address.empty() || listen_port.empty())
  {
    RCLCPP_FATAL(get_logger(),
                 "Hardware interface requires parameters listen_address and listen_port for RSI "
                 "communication");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string sentype   = "KukaRsiDriver";
  const auto sentype_it = info_.hardware_parameters.find("sentype");
  if (sentype_it != info_.hardware_parameters.end())
  {
    sentype = sentype_it->second;
  }

  try
  {
    m_control_thread.emplace(sentype,
                             listen_address,
                             std::stoi(listen_port),
                             &(*m_control_buf),
                             &(*m_rsi_factory),
                             get_logger());
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not initialize hardware: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
KukaRsiHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  m_control_thread->stop();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
KukaRsiHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  m_control_thread->start();

  while (rclcpp::ok())
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for RSI connection...");

    if (const auto state = m_control_buf->popStates(); state)
    {
      for (std::size_t i = 0; i < m_joint_positions.size(); ++i)
      {
        m_joint_positions[i] = state->axis_actual_pos[i] * M_PI / 180.;
        m_joint_efforts[i]   = state->axis_eff[i];
        m_joint_commands[i]  = m_joint_positions[i];
      }

      m_cartesian_position[0] = state->cartesian_actual_pos.x / 1000;
      m_cartesian_position[1] = state->cartesian_actual_pos.y / 1000;
      m_cartesian_position[2] = state->cartesian_actual_pos.z / 1000;

      state->cartesian_actual_pos.getQuaternion(m_cartesian_position[3],
                                                m_cartesian_position[4],
                                                m_cartesian_position[5],
                                                m_cartesian_position[6]);

      m_program_state = static_cast<double>(state->program_status);
      m_speed_scaling = state->speed_scaling / 100;

      m_control_buf->zeroOffsets();

      RCLCPP_INFO(get_logger(), "Initialized RSI communication");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }

  RCLCPP_ERROR(get_logger(), "Could not configure hardware");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn
KukaRsiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
KukaRsiHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KukaRsiHardwareInterface::read(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period)
{
  KUKA_RSI_DRIVER_TRACEPOINT(read, time.nanoseconds(), period.nanoseconds());

  if (!m_control_thread->running())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (const auto state = m_control_buf->popStates(); state)
  {
    for (std::size_t i = 0; i < m_joint_positions.size(); ++i)
    {
      m_joint_positions[i] = state->axis_actual_pos[i] * M_PI / 180.;
      m_joint_efforts[i]   = state->axis_eff[i];
    }

    m_cartesian_position[0] = state->cartesian_actual_pos.x / 1000;
    m_cartesian_position[1] = state->cartesian_actual_pos.y / 1000;
    m_cartesian_position[2] = state->cartesian_actual_pos.z / 1000;

    state->cartesian_actual_pos.getQuaternion(m_cartesian_position[3],
                                              m_cartesian_position[4],
                                              m_cartesian_position[5],
                                              m_cartesian_position[6]);

    m_program_state = static_cast<double>(state->program_status);
    if (state->program_status == ProgramStatus::RUNNING)
    {
      m_speed_scaling = state->speed_scaling / 100;
    }
    else
    {
      m_speed_scaling = 0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KukaRsiHardwareInterface::write(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period)
{
  KUKA_RSI_DRIVER_TRACEPOINT(write, time.nanoseconds(), period.nanoseconds());

  if (!m_control_thread->running())
  {
    return hardware_interface::return_type::ERROR;
  }

  const auto cmd = m_rsi_factory->createCyclicCommand();

  const auto current_state = lifecycle_state_.id();

  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    for (std::size_t i = 0; i < m_joint_commands.size(); ++i)
    {
      cmd->axis_command_pos[i] = m_joint_commands[i] * 180. / M_PI;
    }
  }

  cmd->write_time = std::chrono::steady_clock::now();

  if (!m_control_buf->pushCommand(cmd))
  {
    RCLCPP_INFO(get_logger(), "Control thread not keeping up");
    /*
     * This seems to happen at the start of activating - not sure what to do about it
     */
    // return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

} // namespace kuka_rsi_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kuka_rsi_driver::KukaRsiHardwareInterface,
                       hardware_interface::SystemInterface)
