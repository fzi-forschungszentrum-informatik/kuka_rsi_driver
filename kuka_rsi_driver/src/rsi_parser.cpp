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

/*!\file rsi_parser.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-23
 */
#include "kuka_rsi_driver/rsi_parser.h"

#include <array>
#include <charconv>
#include <fmt/format.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <utility>

template <typename T>
T parseNumber(std::string_view str)
{
  T result;
  const auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);

  if (ec == std::errc())
  {
    return result;
  }
  else if (ec == std::errc::invalid_argument)
  {
    throw std::runtime_error{fmt::format("'{}' is not a number", str)};
  }
  else if (ec == std::errc::result_out_of_range)
  {
    throw std::runtime_error{fmt::format("'{}' is out of range for given data type", str)};
  }

  throw std::runtime_error{fmt::format("Could not parse number '{}'", str)};
}

namespace kuka_rsi_driver {

XmlMemoryHandler* XmlMemoryHandler::m_handler = nullptr;

XmlMemoryHandler::XmlMemoryHandler()
  : m_pool_small{128, 128}
  , m_pool_large{4096, 32}
{
  activate();
}

void XmlMemoryHandler::activate()
{
  m_handler = this;
}

void* XmlMemoryHandler::malloc(std::size_t size)
{
  if (!m_handler)
  {
    throw std::bad_alloc{};
  }

  auto& handler = *m_handler;
  if (size < handler.m_pool_small.get_requested_size())
  {
    return handler.m_pool_small.malloc();
  }
  else if (size < handler.m_pool_large.get_requested_size())
  {
    return handler.m_pool_large.malloc();
  }
  else
  {
    throw std::bad_alloc{};
  }
}

void* XmlMemoryHandler::realloc(void* ptr, std::size_t size)
{
  throw std::bad_alloc{};
}

void XmlMemoryHandler::free(void* ptr)
{
  if (!m_handler)
  {
    throw std::bad_alloc{};
  }

  auto& handler = *m_handler;
  if (!ptr)
  {
    return;
  }
  else if (handler.m_pool_small.is_from(ptr))
  {
    handler.m_pool_small.free(ptr);
  }
  else if (handler.m_pool_large.is_from(ptr))
  {
    handler.m_pool_large.free(ptr);
  }
  else
  {
    throw std::bad_alloc{};
  }
}

XmlParser::XmlParser(const std::string& scope_filter, rclcpp::Logger log, std::size_t buf_size)
  : m_log{std::move(log)}
  , m_scope_filter{scope_filter}
  , m_memory_handling_suite{&XmlMemoryHandler::malloc,
                            &XmlMemoryHandler::realloc,
                            &XmlMemoryHandler::free}
  , m_parser{XML_ParserCreate_MM("US-ASCII", &m_memory_handling_suite, NULL)}
{
  setupParser();

  char* buffer = static_cast<char*>(XML_GetBuffer(m_parser, buf_size));
  m_buf        = std::span<char>{buffer, buf_size};

  m_character_buf.resize(64);
}

XmlParser::~XmlParser()
{
  XML_ParserFree(m_parser);
}

std::span<char> XmlParser::buffer() const
{
  return m_buf;
}

void XmlParser::parseBuffer(std::size_t len)
{
  if (const auto ret = XML_ParseBuffer(m_parser, static_cast<int>(len), XML_TRUE);
      ret == XML_STATUS_ERROR)
  {
    const auto line      = XML_GetCurrentLineNumber(m_parser);
    const auto error_str = XML_ErrorString(XML_GetErrorCode(m_parser));

    throw std::runtime_error{fmt::format("Xml error at line {}: {}", line, error_str)};
  }

  if (XML_ParserReset(m_parser, "US-ASCII") != XML_TRUE)
  {
    throw std::runtime_error{"Error resetting parser"};
  }
  setupParser();
}

void XmlParser::addElementCb(std::string_view tag_name, ElementCallback cb)
{
  m_element_cbs[tag_name] = cb;
}

void XmlParser::addAttributeCb(std::string_view tag_name, AttributeCallback cb)
{
  m_attribute_cbs[tag_name] = cb;
}

void XmlParser::characterData(void* user_data, const XML_Char* s, int len)
{
  auto& parser = *static_cast<XmlParser*>(user_data);

  if (parser.m_in_scope)
  {
    std::memcpy(&parser.m_character_buf[parser.m_character_buf_i], s, len);
    parser.m_character_buf_i += len;
  }
}

void XmlParser::startElement(void* user_data, const XML_Char* name_c, const XML_Char** atts)
{
  const std::string_view name{name_c};

  auto& parser = *static_cast<XmlParser*>(user_data);
  ++parser.m_scope_lvl;

  // Handle scope filtering
  if ((parser.m_scope_lvl == 1) && (parser.m_scope_filter == name))
  {
    parser.m_in_scope = true;
    return;
  }

  // Check attribute callbacks
  if (parser.m_in_scope)
  {
    if (const auto cb_it = parser.m_attribute_cbs.find(name); cb_it != parser.m_attribute_cbs.end())
    {
      (cb_it->second)(atts);
    }
  }

  // Bookkeeping
  parser.m_character_buf_i = 0;
}

void XmlParser::endElement(void* user_data, const XML_Char* name_c)
{
  const std::string_view name{name_c};

  auto& parser = *static_cast<XmlParser*>(user_data);
  --parser.m_scope_lvl;

  // Handle scope filter
  if ((parser.m_scope_lvl == 0) && (parser.m_scope_filter == name))
  {
    parser.m_in_scope = false;
    return;
  }

  // Check element callbacks
  if (parser.m_in_scope)
  {
    if (const auto cb_it = parser.m_element_cbs.find(name); cb_it != parser.m_element_cbs.end())
    {
      (cb_it->second)(std::string_view{&parser.m_character_buf[0], parser.m_character_buf_i});
    }
  }

  // Bookkeeping
  parser.m_character_buf_i = 0;
}

void XmlParser::setupParser()
{
  XML_SetUserData(m_parser, this);
  XML_SetElementHandler(m_parser, &XmlParser::startElement, &XmlParser::endElement);
  XML_SetCharacterDataHandler(m_parser, &XmlParser::characterData);

  m_character_buf_i = 0;

  m_scope_lvl = 0;
  m_in_scope  = false;
}

RsiParser::RsiParser(rclcpp::Logger log, RsiFactory* rsi_factory, std::size_t buf_size)
  : m_log{std::move(log)}
  , m_xml_parser{"Rob", m_log}
  , m_rsi_factory{rsi_factory}
{
  m_xml_parser.addAttributeCb(
    "ASPos", [this](const char** attrs) { onAxisElement(ValueType::POSITION_SETPOINT, attrs); });
  m_xml_parser.addAttributeCb(
    "AIPos", [this](const char** attrs) { onAxisElement(ValueType::POSITION_ACTUAL, attrs); });

  m_xml_parser.addAttributeCb("RSol", [this](const char** attrs) {
    onCartesianElement(ValueType::POSITION_SETPOINT, attrs);
  });
  m_xml_parser.addAttributeCb(
    "RIst", [this](const char** attrs) { onCartesianElement(ValueType::POSITION_ACTUAL, attrs); });

  m_xml_parser.addAttributeCb(
    "GearTorque", [this](const char** attrs) { onAxisElement(ValueType::TORQUE, attrs); });

  m_xml_parser.addAttributeCb("ProgStatus", [this](const char** attrs) { onProgramState(attrs); });
  m_xml_parser.addAttributeCb("OvPro", [this](const char** attrs) { onOverwrite(attrs); });

  m_xml_parser.addAttributeCb("Delay", [this](const char** attrs) { onDelay(attrs); });

  m_xml_parser.addElementCb("IPOC", [this](std::string_view text) { onIpoc(text); });
}

std::span<char> RsiParser::buffer() const
{
  return m_xml_parser.buffer();
}

std::shared_ptr<RsiState> RsiParser::parseBuffer(std::size_t len)
{
  m_rsi_state = m_rsi_factory->createCyclicState();
  m_xml_parser.parseBuffer(len);

  const auto result = m_rsi_state;
  m_rsi_state.reset();
  return result;
}

void RsiParser::onAxisElement(ValueType value_type, const XML_Char** atts)
{
  for (std::size_t i = 0; atts[i]; i += 2)
  {
    const char* name  = atts[i];
    const char* value = atts[i + 1];

    if ((name[0] != 'A') || (name[1] < '1') || (name[1] > '6') || (name[2] != '\0'))
    {
      throw std::runtime_error{fmt::format("Invalid axis name '{}'", name)};
    }

    const int n  = name[1] - '0';
    const auto v = parseNumber<double>(value);

    switch (value_type)
    {
      case ValueType::POSITION_ACTUAL:
        m_rsi_state->axis_actual_pos[n - 1] = v;
        break;

      case ValueType::POSITION_SETPOINT:
        m_rsi_state->axis_setpoint_pos[n - 1] = v;
        break;

      case ValueType::TORQUE:
        m_rsi_state->axis_eff[n - 1] = v;
    }
  }
}

void RsiParser::onCartesianElement(ValueType value_type, const XML_Char** atts)
{
  auto& pose = [&]() -> CartesianPose& {
    switch (value_type)
    {
      case ValueType::POSITION_ACTUAL:
        return m_rsi_state->cartesian_actual_pos;
      case ValueType::POSITION_SETPOINT:
        return m_rsi_state->cartesian_setpoint_pos;
      default:
        throw std::runtime_error{"Invalid value type"};
    }
  }();

  for (std::size_t i = 0; atts[i]; i += 2)
  {
    const char* name  = atts[i];
    const char* value = atts[i + 1];

    const auto v = parseNumber<double>(value);

    switch (name[0])
    {
      case 'X':
        pose.x = v;
        break;

      case 'Y':
        pose.y = v;
        break;

      case 'Z':
        pose.z = v;
        break;

      case 'A':
        pose.a = v;
        break;

      case 'B':
        pose.b = v;
        break;

      case 'C':
        pose.c = v;
        break;

      default:
        throw std::runtime_error{fmt::format("Invalid axis name '{}'", name)};
    }

    if (name[1] != '\0')
    {
      throw std::runtime_error{fmt::format("Invalid axis name '{}'", name)};
    }
  }
}

void RsiParser::onOverwrite(const XML_Char** atts)
{
  for (std::size_t i = 0; atts[i]; i += 2)
  {
    if (atts[i][0] == 'R')
    {
      m_rsi_state->speed_scaling = parseNumber<double>(atts[i + 1]);
    }
  }
}

void RsiParser::onProgramState(const XML_Char** atts)
{
  for (std::size_t i = 0; atts[i]; i += 2)
  {
    if (atts[i][0] == 'R')
    {
      m_rsi_state->program_status = static_cast<ProgramStatus>(parseNumber<long>(atts[i + 1]));
    }
  }
}

void RsiParser::onDelay(const XML_Char** atts)
{
  const auto* name = atts[0];
  if (strcmp(name, "D") != 0)
  {
    throw std::runtime_error{fmt::format("Invalid Delay attribute '{}'", name)};
  }
  const auto* value = atts[1];

  if (atts[2] != nullptr)
  {
    throw std::runtime_error{"Unexpected additional Delay attribute"};
  }

  const auto v       = parseNumber<std::size_t>(value);
  m_rsi_state->delay = v;
}

void RsiParser::onIpoc(std::string_view text)
{
  m_rsi_state->ipoc = parseNumber<std::size_t>(text);
}

} // namespace kuka_rsi_driver
