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

/*!\file kuka_rsi_driver/rsi_writer.h
 * \brief Writer for RSI command messages
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-30
 */
#ifndef KUKA_RSI_DRIVER_RSI_WRITER_H_INCLUDED
#define KUKA_RSI_DRIVER_RSI_WRITER_H_INCLUDED

#include "rsi.h"
#include "rsi_config.h"

#include <cstdint>
#include <rclcpp/logger.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace kuka_rsi_driver {

/*! \brief
 *
 */
class TextWriter
{
public:
  explicit TextWriter(rclcpp::Logger log);

  void reset(std::span<char> target);

  [[nodiscard]] std::size_t bytesWritten() const;

  void writeText(std::string_view text);
  void writeNumber(std::size_t v);
  void writeNumber(double v);

private:
  rclcpp::Logger m_log;

  std::span<char> m_target;
  char* m_head;
};

class RsiWriter
{
public:
  RsiWriter(const std::string& sentype,
            const TransmissionConfig& config,
            rclcpp::Logger log,
            std::size_t buf_size = 1024);

  [[nodiscard]] std::size_t
  writeCommand(const RsiCommand& cmd, std::size_t ipoc, std::span<char> target);

private:
  rclcpp::Logger m_log;

  std::string m_sentype;
  TransmissionConfig m_config;

  TextWriter m_writer;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_RSI_WRITER_H_INCLUDED
