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

/*!\file rsi_config.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2025-03-06
 */
#include "kuka_rsi_driver/rsi_config.h"

#include <algorithm>
#include <array>
#include <fmt/format.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace kuka_rsi_driver {

RsiConfig::RsiConfig(const hardware_interface::HardwareInfo& info)
{
  for (std::size_t i = 0; i < info.joints.size(); ++i)
  {
    const auto& joint = info.joints[i];

    verifyComponent(joint,
                    fmt::format("joint {}", i),
                    {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT},
                    {hardware_interface::HW_IF_POSITION});

    m_interface_config.joint_position_command_interfaces.push_back(
      joint.name + "/" + joint.command_interfaces[0].name);

    m_interface_config.joint_position_state_interfaces.push_back(joint.name + "/" +
                                                                 joint.state_interfaces[0].name);
    m_interface_config.joint_effort_state_interfaces.push_back(joint.name + "/" +
                                                               joint.state_interfaces[1].name);
  }

  // Verify sensors
  if (info.sensors.size() != 1)
  {
    throw std::runtime_error{
      fmt::format("Expected 1 sensor element, but found %zu", info.sensors.size())};
  }
  verifyComponent(info.sensors[0],
                  "tcp",
                  {"position.x",
                   "position.y",
                   "position.z",
                   "orientation.x",
                   "orientation.y",
                   "orientation.z",
                   "orientation.w"},
                  {});
  std::transform(
    info.sensors[0].state_interfaces.cbegin(),
    info.sensors[0].state_interfaces.cend(),
    m_interface_config.tcp_state_interfaces.begin(),
    [&](const auto& interface) { return info.sensors[0].name + '/' + interface.name; });

  // Verify gpios
  if (info.gpios.size() != 2)
  {
    throw std::runtime_error{
      fmt::format("Expected 2 gpio elements, but found %zu", info.gpios.size())};
  }

  verifyComponent(info.gpios[0], "robot state", {"program_state"}, {});
  m_interface_config.robot_state_state_interface =
    info.gpios[0].name + "/" + info.gpios[0].state_interfaces[0].name;

  verifyComponent(info.gpios[1], "speed scaling", {"speed_scaling_factor"}, {});
  m_interface_config.speed_scaling_state_interface =
    info.gpios[1].name + "/" + info.gpios[1].state_interfaces[0].name;
}

const InterfaceConfig& RsiConfig::interfaceConfig() const
{
  return m_interface_config;
}

void RsiConfig::verifyComponent(const hardware_interface::ComponentInfo& component,
                                const std::string& component_function,
                                const std::vector<std::string>& expected_state_interfaces,
                                const std::vector<std::string>& expected_command_interfaces) const
{
  if (!std::equal(component.command_interfaces.cbegin(),
                  component.command_interfaces.cend(),
                  expected_command_interfaces.cbegin(),
                  expected_command_interfaces.cend(),
                  [](const auto& i1, const auto& i2) { return i1.name == i2; }))
  {
    std::vector<std::string> interface_names;
    std::transform(component.command_interfaces.begin(),
                   component.command_interfaces.end(),
                   std::back_inserter(interface_names),
                   [](const auto& c) { return c.name; });
    throw std::runtime_error{
      fmt::format("{} '{}' ({}) should have {} command interfaces [{}], found [{}]",
                  component.type,
                  component.name,
                  component_function,
                  expected_command_interfaces.size(),
                  fmt::join(expected_command_interfaces, ", "),
                  fmt::join(interface_names, ", "))};
  }

  if (!std::equal(component.state_interfaces.cbegin(),
                  component.state_interfaces.cend(),
                  expected_state_interfaces.cbegin(),
                  expected_state_interfaces.cend(),
                  [](const auto& i1, const auto& i2) { return i1.name == i2; }))
  {
    std::vector<std::string> interface_names;
    std::transform(component.state_interfaces.begin(),
                   component.state_interfaces.end(),
                   std::back_inserter(interface_names),
                   [](const auto& c) { return c.name; });
    throw std::runtime_error{
      fmt::format("{} '{}' ({}) should have {} state interfaces [{}], found [{}]",
                  component.type,
                  component.name,
                  component_function,
                  expected_state_interfaces.size(),
                  fmt::join(expected_state_interfaces, ", "),
                  fmt::join(interface_names, ", "))};
  }
}

} // namespace kuka_rsi_driver
