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

/*!\file control_thread.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-30
 */
#include "kuka_rsi_driver/control_thread.h"

#include "kuka_rsi_driver/control_buffer.h"
#include "kuka_rsi_driver/phase_estimator.h"
#include "kuka_rsi_driver/tracing.h"

#include <rclcpp/rclcpp.hpp>

namespace kuka_rsi_driver {

ControlThread::ControlThread(const RsiConfig& config,
                             const std::string& sentype,
                             const std::string& listen_address,
                             unsigned short listen_port,
                             ControlBuffer* control_buf,
                             RsiFactory* rsi_factory,
                             rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_udp_server{listen_address, listen_port, std::chrono::milliseconds{1}}
  , m_rsi_parser{config.receiveTransmissionConfig(), rsi_factory, m_log}
  , m_rsi_writer{sentype, m_log}
  , m_control_buf{control_buf}
  , m_initial_cmd{rsi_factory->createCommand()}
  , m_rsi_cmd{rsi_factory->createCommand()}
{
  m_write_buf.resize(1024);

  RCLCPP_INFO(m_log, "Created UDP server listening on %s:%hu", listen_address.c_str(), listen_port);
}

ControlThread::~ControlThread()
{
  m_thread_stop.store(true);
  m_thread.join();
}

void ControlThread::start()
{
  m_thread_stop = false;
  m_thread      = std::thread{&ControlThread::run, this};

  RCLCPP_INFO(m_log, "Control thread started");
}

void ControlThread::stop()
{
  m_thread_stop.store(true);

  RCLCPP_INFO(m_log, "Control thread stopped");
}

bool ControlThread::running() const
{
  return !m_thread_stop.load();
}

void ControlThread::run()
{
  while (!m_thread_stop.load())
  {
    // Get current command
    m_control_buf->syncCommands();

    if (const auto bytes_received =
          m_udp_server.receive(std::as_writable_bytes(m_rsi_parser.buffer()));
        bytes_received.has_value())
    {
      m_control_buf->syncCommands();
      m_control_buf->phaseEstimator().signalPacket();

      const auto phase_estimate = m_control_buf->phaseEstimator().estimate();
      KUKA_RSI_DRIVER_TRACEPOINT(rsi_packet_received, phase_estimate.phase, phase_estimate.offset);

      // Parse RSI state
      const auto rsi_state = m_rsi_parser.parseBuffer(*bytes_received);

      if (!m_cmd_offset)
      {
        m_cmd_offset = rsi_state->axis_setpoint_pos;
      }

      // Check out initial command
      RsiCommand* cmd = nullptr;
      if (const auto interpolated_cmd = m_control_buf->interpolateCommand(phase_estimate.phase);
          interpolated_cmd)
      {
        cmd = interpolated_cmd.get();
      }
      else
      {
        m_initial_cmd.axis_command_pos = rsi_state->axis_setpoint_pos;
        cmd                            = &m_initial_cmd;
      }

      if (!m_control_buf->pushState(rsi_state))
      {
        RCLCPP_ERROR(m_log, "Hardware interface is not keeping up with RSI states");
        /*
         * TODO What can be done here?
         *
         * Apparently hardware activation can create larger gaps in the read/write cycle
         */
        /*
        m_thread_stop.store(true);
        return;
        */
      }

      // Apply offset to command
      for (std::size_t i = 0; i < 6; ++i)
      {
        m_rsi_cmd.axis_command_pos[i] = cmd->axis_command_pos[i] - (*m_cmd_offset)[i];
      }

      KUKA_RSI_DRIVER_TRACEPOINT(rsi_packet_sent,
                                 rsi_state->ipoc,
                                 m_rsi_cmd.axis_command_pos[0],
                                 m_rsi_cmd.axis_command_pos[1],
                                 m_rsi_cmd.axis_command_pos[2],
                                 m_rsi_cmd.axis_command_pos[3],
                                 m_rsi_cmd.axis_command_pos[4],
                                 m_rsi_cmd.axis_command_pos[5]);

      // Send back command
      const auto response_size = m_rsi_writer.writeCommand(m_rsi_cmd, rsi_state->ipoc, m_write_buf);
      m_udp_server.send(std::as_bytes(std::span<const char>{m_write_buf.data(), response_size}));
    }
  }
}

} // namespace kuka_rsi_driver
