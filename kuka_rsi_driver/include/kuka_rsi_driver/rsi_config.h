// Copyright 2025 FZI Forschungszentrum Informatik
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

/*!\file kuka_rsi_driver/rsi_config.h
 * \brief Definition of mappings between interfaces, internal structures and XML entries
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2025-03-06
 */
#ifndef KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED
#define KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED

#include <hardware_interface/hardware_info.hpp>
#include <string>
#include <vector>

namespace kuka_rsi_driver {

/*! \brief Definition of used state and command interfaces
 */
class InterfaceConfig
{
public:
  /*! \brief Create config based on interface specification
   *
   * \param info Hardware information from URDF
   */
  InterfaceConfig(const hardware_interface::HardwareInfo& info);

  /*! \brief Get the fully qualified names of all joint state position interfaces
   */
  const std::vector<std::string>& jointStatePositionInterfaces() const;

  /*! \brief Get the fully qualified names of all joint state effort interfaces
   */
  const std::vector<std::string>& jointStateEffortInterfaces() const;

  /*! \brief Get the fully qualified names of all joint command position interfaces
   */
  const std::vector<std::string>& jointCommandPositionInterfaces() const;

  /*! \brief Get the fully qualified names of all sensor interfaces for the TCP pose
   */
  const std::vector<std::string>& tcpSensorInterfaces() const;

  /*! \brief Get the fully qualified name of the robot state state interface
   */
  const std::string& robotStateInterface() const;

  /*! \brief Get the fully qualified names of the speed scaling state interface
   */
  const std::string& speedScalingInterface() const;

private:
  void verifyComponent(const hardware_interface::ComponentInfo& component,
                       const std::string& component_function,
                       const std::vector<std::string>& expected_state_interfaces,
                       const std::vector<std::string>& expected_command_interfaces) const;

  std::vector<std::string> m_joint_state_position_interfaces;
  std::vector<std::string> m_joint_state_effort_interfaces;
  std::vector<std::string> m_joint_command_position_interfaces;

  std::vector<std::string> m_tcp_sensor_interfaces;

  std::string m_robot_state_interface;
  std::string m_speed_scaling_interface;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED
