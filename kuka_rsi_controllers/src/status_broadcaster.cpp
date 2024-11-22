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

/*!\file status_broadcaster.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-10-23
 */
#include "kuka_rsi_controllers/status_broadcaster.h"

namespace kuka_rsi_controllers {

controller_interface::InterfaceConfiguration
StatusBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result{};
  result.type = controller_interface::interface_configuration_type::NONE;

  return result;
}
controller_interface::InterfaceConfiguration
StatusBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result{};
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  result.names.push_back("robot_state/program_state");
  result.names.push_back("speed_scaling/speed_scaling_factor");

  return result;
}

controller_interface::return_type StatusBroadcaster::update(const rclcpp::Time& time,
                                                            const rclcpp::Duration& period)
{
  if (m_robot_state_pub && m_robot_state_pub->trylock())
  {
    m_robot_state_pub->msg_.program_state.state =
      static_cast<std::uint8_t>(state_interfaces_[0].get_value());
    m_robot_state_pub->unlockAndPublish();
  }

  if (m_speed_scaling_pub && m_speed_scaling_pub->trylock())
  {
    m_speed_scaling_pub->msg_.data = state_interfaces_[1].get_value();
    m_speed_scaling_pub->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn StatusBroadcaster::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
StatusBroadcaster::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    const auto robot_state_pub = get_node()->create_publisher<kuka_rsi_interfaces::msg::RobotState>(
      "~/robot_state", rclcpp::SystemDefaultsQoS());
    m_robot_state_pub =
      std::make_unique<realtime_tools::RealtimePublisher<kuka_rsi_interfaces::msg::RobotState>>(
        robot_state_pub);

    const auto speed_scaling_pub = get_node()->create_publisher<std_msgs::msg::Float64>(
      "~/speed_scaling", rclcpp::SystemDefaultsQoS());
    m_speed_scaling_pub =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
        speed_scaling_pub);
  }
  catch (const std::exception& ex)
  {
    fprintf(stderr, "Exception thrown during configuration: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
StatusBroadcaster::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
StatusBroadcaster::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace kuka_rsi_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kuka_rsi_controllers::StatusBroadcaster,
                       controller_interface::ControllerInterface)
