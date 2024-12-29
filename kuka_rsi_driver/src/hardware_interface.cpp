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

constexpr std::array TCP_SENSOR_STATE_INTERFACES = {"position.x",
                                                    "position.y",
                                                    "position.z",
                                                    "orientation.x",
                                                    "orientation.y",
                                                    "orientation.z",
                                                    "orientation.w"};

namespace kuka_rsi_driver {

hardware_interface::CallbackReturn
KukaRsiHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Robot joints:");
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto& joint = info_.joints[i];

    // Verify joint position command interface
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
    m_joint_command_pos_ifaces.push_back(joint.name + "/" + joint.command_interfaces[0].name);

    // Verify joint state interface
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
    m_joint_state_pos_ifaces.push_back(joint.name + "/" + joint.state_interfaces[0].name);
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has state interface '%s', expected %s",
                   joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
    m_joint_state_eff_ifaces.push_back(joint.name + "/" + joint.state_interfaces[1].name);

    RCLCPP_INFO(get_logger(), "  - %zu: %s", i, joint.name.c_str());
  }

  // Verify sensors
  if (info_.sensors.size() != 1)
  {
    RCLCPP_FATAL(get_logger(), "Expected 1 sensor element, but found %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Verify TCP sensor
  if (!info_.sensors[0].command_interfaces.empty())
  {
    RCLCPP_FATAL(get_logger(),
                 "Sensor '%s' (tcp) should have no command interfaces, but found %zu",
                 info_.sensors[0].name.c_str(),
                 info_.sensors[0].command_interfaces.size());
  }
  if (info_.sensors[0].state_interfaces.size() != TCP_SENSOR_STATE_INTERFACES.size())
  {
    RCLCPP_FATAL(get_logger(),
                 "Sensor '%s' (tcp) should have %zu state interfaces, but found %zu",
                 info_.sensors[0].name.c_str(),
                 TCP_SENSOR_STATE_INTERFACES.size(),
                 info_.sensors[0].state_interfaces.size());
  }
  RCLCPP_INFO(get_logger(), "TCP sensor '%s' state interfaces:", info_.sensors[0].name.c_str());
  for (std::size_t i = 0; i < info_.sensors[0].state_interfaces.size(); ++i)
  {
    RCLCPP_INFO(get_logger(),
                "  - %zu: %s (%s)",
                i,
                info_.sensors[0].state_interfaces[i].name.c_str(),
                TCP_SENSOR_STATE_INTERFACES[i]);
    m_sensor_tcp_state_ifaces.push_back(info_.sensors[0].name + "/" +
                                        info_.sensors[0].state_interfaces[i].name);
  }

  // Verify gpios
  if (info_.gpios.size() != 2)
  {
    RCLCPP_FATAL(get_logger(), "Expected 2 gpio elements, but found %zu", info_.gpios.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Verify robot state
  if (!info_.gpios[0].command_interfaces.empty() || (info_.gpios[0].state_interfaces.size() != 1))
  {
    RCLCPP_FATAL(get_logger(),
                 "GPIO '%s' (robot state) should provide one state interface, but found %zu state "
                 "interfaces and %zu command interfaces",
                 info_.gpios[0].name.c_str(),
                 info_.gpios[0].state_interfaces.size(),
                 info_.gpios[0].command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  m_gpio_robot_state_iface = info_.gpios[0].name + "/" + info_.gpios[0].state_interfaces[0].name;
  RCLCPP_INFO(get_logger(), "Robot state interface: %s", m_gpio_robot_state_iface.c_str());

  // Verify speed scaling
  if (!info_.gpios[1].command_interfaces.empty() || (info_.gpios[1].state_interfaces.size() != 1))
  {
    RCLCPP_FATAL(get_logger(),
                 "GPIO '%s' (speed scaling) should provide one state interface, but found %zu "
                 "state interfaces and %zu command interfaces",
                 info_.gpios[1].name.c_str(),
                 info_.gpios[1].state_interfaces.size(),
                 info_.gpios[1].command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  m_gpio_speed_scaling_state_iface =
    info_.gpios[1].name + "/" + info_.gpios[1].state_interfaces[0].name;
  RCLCPP_INFO(
    get_logger(), "Speed scaling state interface: %s", m_gpio_speed_scaling_state_iface.c_str());

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
      setState(*state);
      for (std::size_t i = 0; i < state->axis_actual_pos.size(); ++i)
      {
        set_command(m_joint_command_pos_ifaces[i], state->axis_actual_pos[i] * M_PI / 180.);
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
    for (std::size_t i = 0; i < m_joint_command_pos_ifaces.size(); ++i)
    {
      cmd->axis_command_pos[i] = get_command(m_joint_command_pos_ifaces[i]) * 180. / M_PI;
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
  for (std::size_t i = 0; i < state.axis_actual_pos.size(); ++i)
  {
    set_state(m_joint_state_pos_ifaces[i], state.axis_actual_pos[i] * M_PI / 180.);
    set_state(m_joint_state_eff_ifaces[i], state.axis_eff[i]);
  }

  // TCP
  set_state(m_sensor_tcp_state_ifaces[0], state.cartesian_actual_pos.x / 1000);
  set_state(m_sensor_tcp_state_ifaces[1], state.cartesian_actual_pos.y / 1000);
  set_state(m_sensor_tcp_state_ifaces[2], state.cartesian_actual_pos.z / 1000);

  std::array<double, 4> tcp_rpy;
  state.cartesian_actual_pos.getQuaternion(tcp_rpy[0], tcp_rpy[1], tcp_rpy[2], tcp_rpy[3]);
  for (std::size_t i = 0; i < tcp_rpy.size(); ++i)
  {
    set_state(m_sensor_tcp_state_ifaces[i + 3], tcp_rpy[i]);
  }

  // Robot state
  set_state(m_gpio_robot_state_iface, static_cast<double>(state.program_status));
  if (state.program_status == ProgramStatus::RUNNING)
  {
    set_state(m_gpio_speed_scaling_state_iface, state.speed_scaling / 100);
  }
  else
  {
    set_state(m_gpio_speed_scaling_state_iface, 0.0);
  }
}

} // namespace kuka_rsi_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kuka_rsi_driver::KukaRsiHardwareInterface,
                       hardware_interface::SystemInterface)
