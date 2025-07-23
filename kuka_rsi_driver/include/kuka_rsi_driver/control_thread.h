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

/*!\file kuka_rsi_driver/control_thread.h
 * \brief Thread to handle robot communication independent of ros2_control cycle
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-30
 */
#ifndef KUKA_RSI_DRIVER_CONTROL_THREAD_H_INCLUDED
#define KUKA_RSI_DRIVER_CONTROL_THREAD_H_INCLUDED

#include "rsi.h"
#include "rsi_config.h"
#include "rsi_factory.h"
#include "rsi_parser.h"
#include "rsi_writer.h"
#include "udp_server.h"

#include <atomic>
#include <rclcpp/logger.hpp>
#include <string>
#include <thread>

namespace kuka_rsi_driver {

class ControlBuffer;

class ControlThread
{
public:
  ControlThread(const RsiConfig& config,
                ControlBuffer* control_buf,
                RsiFactory* rsi_factory,
                rclcpp::Logger log);
  ~ControlThread();

  ControlThread(const ControlThread&)           = delete;
  ControlThread& operator=(const ControlThread) = delete;

  void start();
  void stop();

  bool running() const;

private:
  void run();

  rclcpp::Logger m_log;

  UdpServer m_udp_server;
  RsiParser m_rsi_parser;

  RsiCommand m_rsi_cmd;
  RsiWriter m_rsi_writer;

  ControlBuffer* m_control_buf;

  RsiCommand m_initial_cmd;

  std::optional<JointArray> m_cmd_offset;

  std::thread m_thread;
  std::atomic<bool> m_thread_stop;

  std::vector<char> m_write_buf;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_CONTROL_THREAD_H_INCLUDED
