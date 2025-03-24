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

/*!\file kuka_rsi_driver/hardware_interface.h
 * \brief ros2_control hardware interface for controlling robots through RSI
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-21
 */
#ifndef KUKA_RSI_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
#define KUKA_RSI_DRIVER_HARDWARE_INTERFACE_H_INCLUDED

#include "control_buffer.h"
#include "control_thread.h"
#include "rsi.h"
#include "rsi_config.h"
#include "rsi_factory.h"
#include "tracing.h"

#include <hardware_interface/system_interface.hpp>
#include <optional>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

namespace kuka_rsi_driver {

class KukaRsiHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo& system_info) override;
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

private:
  void setState(const RsiState& state);

  std::optional<RsiConfig> m_rsi_config;
  std::optional<RsiFactory> m_rsi_factory;
  std::optional<ControlBuffer> m_control_buf;
  std::optional<ControlThread> m_control_thread;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
