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

hardware_interface::CallbackReturn
KukaRsiHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    m_interface_config.emplace(info_);

    m_rsi_factory.emplace();
    m_control_buf.emplace(*m_rsi_factory);

    const auto listen_address = info_.hardware_parameters["listen_address"];
    const auto listen_port    = info_.hardware_parameters["listen_port"];

    if (listen_address.empty() || listen_port.empty())
    {
      throw std::runtime_error{"Hardware interface requires parameters listen_address and "
                               "listen_port for RSI communication"};
    }

    std::string sentype   = "KukaRsiDriver";
    const auto sentype_it = info_.hardware_parameters.find("sentype");
    if (sentype_it != info_.hardware_parameters.end())
    {
      sentype = sentype_it->second;
    }

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
      setState(*state);

      const auto& joint_command_position_interfaces =
        m_interface_config->jointCommandPositionInterfaces();
      for (std::size_t i = 0; i < state->axis_actual_pos.size(); ++i)
      {
        set_command(joint_command_position_interfaces[i], state->axis_actual_pos[i] * M_PI / 180.);
      }

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
    setState(*state);
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
    const auto& joint_command_position_interfaces =
      m_interface_config->jointCommandPositionInterfaces();
    for (std::size_t i = 0; i < joint_command_position_interfaces.size(); ++i)
    {
      cmd->axis_command_pos[i] = get_command(joint_command_position_interfaces[i]) * 180. / M_PI;
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

void KukaRsiHardwareInterface::setState(const RsiState& state)
{
  // Joint states
  const auto& joint_position_interfaces = m_interface_config->jointStatePositionInterfaces();
  const auto& joint_effort_interfaces   = m_interface_config->jointStateEffortInterfaces();
  for (std::size_t i = 0; i < state.axis_actual_pos.size(); ++i)
  {
    set_state(joint_position_interfaces[i], state.axis_actual_pos[i] * M_PI / 180.);
    set_state(joint_effort_interfaces[i], state.axis_eff[i]);
  }

  // TCP
  const auto& tcp_interfaces = m_interface_config->tcpSensorInterfaces();
  set_state(tcp_interfaces[0], state.cartesian_actual_pos.x() / 1000);
  set_state(tcp_interfaces[1], state.cartesian_actual_pos.y() / 1000);
  set_state(tcp_interfaces[2], state.cartesian_actual_pos.z() / 1000);

  std::array<double, 4> tcp_rpy;
  state.cartesian_actual_pos.getQuaternion(tcp_rpy[0], tcp_rpy[1], tcp_rpy[2], tcp_rpy[3]);
  for (std::size_t i = 0; i < tcp_rpy.size(); ++i)
  {
    set_state(tcp_interfaces[i + 3], tcp_rpy[i]);
  }

  // Robot state
  set_state(m_interface_config->robotStateInterface(), static_cast<double>(state.program_status));
  if (state.program_status == ProgramStatus::RUNNING)
  {
    set_state(m_interface_config->speedScalingInterface(), state.speed_scaling / 100);
  }
  else
  {
    set_state(m_interface_config->speedScalingInterface(), 0.0);
  }
}

} // namespace kuka_rsi_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kuka_rsi_driver::KukaRsiHardwareInterface,
                       hardware_interface::SystemInterface)
