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

/*!\file rsi_writer.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-30
 */
#include "kuka_rsi_driver/rsi_writer.h"

#include <charconv>
#include <cstring>

namespace kuka_rsi_driver {

TextWriter::TextWriter(rclcpp::Logger log)
  : m_log{std::move(log)}
{
}

void TextWriter::reset(std::span<char> target)
{
  m_target = target;
  m_head   = &m_target[0];
}

std::size_t TextWriter::bytesWritten() const
{
  return m_head - m_target.data();
}

void TextWriter::writeText(std::string_view text)
{
  if (text.size() > static_cast<std::size_t>(m_target.size() - (m_head - &m_target[0])))
  {
    throw std::runtime_error{"Invalid buffer space"};
  }

  std::memcpy(m_head, text.data(), text.size());
  m_head += text.size();
}

void TextWriter::writeNumber(std::size_t v)
{
  const auto [ptr, ec] = std::to_chars(m_head, &m_target[m_target.size() - 1], v);

  if (ec != std::errc())
  {
    throw std::runtime_error{"Could not format size_t within buffer space"};
  }

  m_head = ptr;
}

void TextWriter::writeNumber(double v)
{
  const auto [ptr, ec] =
    std::to_chars(m_head, &m_target[m_target.size() - 1], v, std::chars_format::fixed, 6);

  if (ec != std::errc())
  {
    throw std::runtime_error{"Could not format double within buffer space"};
  }

  m_head = ptr;
}

RsiWriter::RsiWriter(const std::string& sentype,
                     const TransmissionConfig& config,
                     rclcpp::Logger log,
                     std::size_t buf_size)
  : m_log{std::move(log)}
  , m_config{config}
  , m_sentype{sentype}
  , m_writer{m_log}
{
}

std::size_t RsiWriter::writeCommand(const RsiCommand& cmd, std::size_t ipoc, std::span<char> target)
{
  m_writer.reset(target);
  m_writer.writeText("<Sen Type=\"");
  m_writer.writeText(m_sentype);
  m_writer.writeText("\">");

  m_writer.writeText("<AK ");
  for (std::size_t i = 0; i < cmd.axis_command_pos.size(); ++i)
  {
    m_writer.writeText("A");
    m_writer.writeNumber(i + 1);
    m_writer.writeText("=\"");
    m_writer.writeNumber(cmd.axis_command_pos[i]);
    m_writer.writeText("\" ");
  }

  for (const auto& tag : m_config.tags)
  {
    m_writer.writeText("/><");
    m_writer.writeText(tag.name);
    m_writer.writeText(" ");

    for (const auto& index : tag.indices)
    {
      m_writer.writeText(index.name);
      m_writer.writeText("=\"");

      switch (index.type)
      {
        case DataType::BOOL:
          m_writer.writeText(cmd.passthrough.values_bool[index.index] ? "1" : "0");
          break;

        case DataType::DOUBLE:
          m_writer.writeNumber(cmd.passthrough.values_double[index.index]);
          break;

        case DataType::LONG:
          m_writer.writeNumber(cmd.passthrough.values_long[index.index]);
          break;

        default:
          throw std::runtime_error{"Invalid data type"};
      }

      m_writer.writeText("\" ");
    }
  }

  m_writer.writeText("/><IPOC>");
  m_writer.writeNumber(ipoc);
  m_writer.writeText("</IPOC></Sen>");

  return m_writer.bytesWritten();
}

} // namespace kuka_rsi_driver
