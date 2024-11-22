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

/*!\file kuka_rsi_controllers/status_broadcaster.h
 * \brief Broadcaster for robot-specific state information
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-10-23
 */
#ifndef KUKA_RSI_CONTROLLERS_STATUS_BROADCASTER_H_INCLUDED
#define KUKA_RSI_CONTROLLERS_STATUS_BROADCASTER_H_INCLUDED

#include <controller_interface/controller_interface.hpp>
#include <kuka_rsi_interfaces/msg/robot_state.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/msg/float64.hpp>

namespace kuka_rsi_controllers {

class StatusBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  std::unique_ptr<realtime_tools::RealtimePublisher<kuka_rsi_interfaces::msg::RobotState>>
    m_robot_state_pub;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>> m_speed_scaling_pub;
};

} // namespace kuka_rsi_controllers

#endif // KUKA_RSI_CONTROLLERS_STATUS_BROADCASTER_H_INCLUDED
