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

void XmlParser::addElementCb(std::string_view name,
                             ElementCallback cb,
                             const std::vector<std::string>& attributes)
{
  const auto [_, success] = m_callbacks.emplace(name, Callback{cb, attributes});
  if (!success)
  {
    throw std::runtime_error{
      fmt::format("Cannot register multiple callbacks for element '{}'", name)};
  }

  if (attributes.size() > m_attribute_views.size())
  {
    m_attribute_views.resize(attributes.size());
  }
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
    if (const auto cb_it = parser.m_callbacks.find(name); cb_it != parser.m_callbacks.end())
    {
      auto& callback = cb_it->second;

      // Copy over attributes
      for (std::size_t i = 0; atts[i]; i += 2)
      {
        callback.copyValue(atts[i], atts[i + 1]);
      }
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

  // Check callbacks
  if (parser.m_in_scope)
  {
    if (const auto cb_it = parser.m_callbacks.find(name); cb_it != parser.m_callbacks.end())
    {
      const auto& callback = cb_it->second;

      // Set up attribute values
      for (std::size_t i = 0; i < callback.attributes.size(); ++i)
      {
        parser.m_attribute_views[i] =
          std::string_view{callback.attribute_bufs[i].data(), callback.attribute_buf_lens[i]};
      }

      // Call registered callback
      callback.cb(
        std::string_view{parser.m_character_buf.data(), parser.m_character_buf_i},
        std::span<std::string_view>{parser.m_attribute_views.data(), callback.attributes.size()});
    }
  }

  // Bookkeeping
  parser.m_character_buf_i = 0;
}

XmlParser::Callback::Callback(ElementCallback cb, const std::vector<std::string>& attributes)
  : cb{std::move(cb)}
  , attributes{attributes}
{
  attribute_bufs.resize(attributes.size());
  attribute_buf_lens.resize(attributes.size());
  std::fill(attribute_buf_lens.begin(), attribute_buf_lens.end(), -1);

  for (auto& buf : attribute_bufs)
  {
    buf.resize(64);
  }
}

void XmlParser::Callback::copyValue(std::string_view attr_name, std::string_view attr_value)
{
  // Find correct attribute index
  if (const auto it = std::find(attributes.begin(), attributes.end(), attr_name);
      it != attributes.end())
  {
    const auto i = it - attributes.begin();

    // Check if value fits into buffer
    if (attr_value.size() > attribute_bufs[i].size())
    {
      throw std::runtime_error{fmt::format(
        "Attribute value {}={} is too large for attribute buffer", attr_name, attr_value)};
    }

    // Copy over attributes
    std::memcpy(&attribute_bufs[i][0], attr_value.data(), attr_value.size());
    attribute_buf_lens[i] = attr_value.size();
  }
  else
  {
    throw std::runtime_error{fmt::format("Unexpected attribute {}={}", attr_name, attr_value)};
  }
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
  using namespace std::literals;
  const std::vector axes           = {"A1"s, "A2"s, "A3"s, "A4"s, "A5"s, "A6"s};
  const std::vector cartesian_axes = {"X"s, "Y"s, "Z"s, "A"s, "B"s, "C"s};

  m_xml_parser.addElementCb(
    "ASPos",
    [this](std::string_view, std::span<std::string_view> attributes) {
      onAxisElement(ValueType::POSITION_SETPOINT, attributes);
    },
    axes);
  m_xml_parser.addElementCb(
    "AIPos",
    [this](std::string_view, std::span<std::string_view> attributes) {
      onAxisElement(ValueType::POSITION_ACTUAL, attributes);
    },
    axes);

  m_xml_parser.addElementCb(
    "RSol",
    [this](std::string_view, std::span<std::string_view> attributes) {
      onCartesianElement(ValueType::POSITION_SETPOINT, attributes);
    },
    cartesian_axes);
  m_xml_parser.addElementCb(
    "RIst",
    [this](std::string_view, std::span<std::string_view> attributes) {
      onCartesianElement(ValueType::POSITION_ACTUAL, attributes);
    },
    cartesian_axes);

  m_xml_parser.addElementCb(
    "GearTorque",
    [this](std::string_view, std::span<std::string_view> attributes) {
      onAxisElement(ValueType::TORQUE, attributes);
    },
    axes);

  m_xml_parser.addElementCb("ProgStatus",
                            [this](std::string_view, std::span<std::string_view> attributes) {
                              onProgramState(attributes[0]);
                            },
                            {"R"});
  m_xml_parser.addElementCb("OvPro",
                            [this](std::string_view, std::span<std::string_view> attributes) {
                              onOverwrite(attributes[0]);
                            },
                            {"R"});

  m_xml_parser.addElementCb(
    "Delay",
    [this](std::string_view, std::span<std::string_view> attributes) { onDelay(attributes[0]); },
    {"D"});

  m_xml_parser.addElementCb(
    "IPOC",
    [this](std::string_view text, std::span<std::string_view> attributes) { onIpoc(text); });
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

void RsiParser::onAxisElement(ValueType value_type, std::span<std::string_view> values)
{
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    const double v = parseNumber<double>(values[i]);
    switch (value_type)
    {
      case ValueType::POSITION_ACTUAL:
        m_rsi_state->axis_actual_pos[i] = v;
        break;

      case ValueType::POSITION_SETPOINT:
        m_rsi_state->axis_setpoint_pos[i] = v;
        break;

      case ValueType::TORQUE:
        m_rsi_state->axis_eff[i] = v;
        break;
    }
  }
}

void RsiParser::onCartesianElement(ValueType value_type, std::span<std::string_view> values)
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

  pose.x = parseNumber<double>(values[0]);
  pose.y = parseNumber<double>(values[1]);
  pose.z = parseNumber<double>(values[2]);
  pose.a = parseNumber<double>(values[3]);
  pose.b = parseNumber<double>(values[4]);
  pose.c = parseNumber<double>(values[5]);
}

void RsiParser::onOverwrite(std::string_view r)
{
  m_rsi_state->speed_scaling = parseNumber<double>(r);
}

void RsiParser::onProgramState(std::string_view r)
{
  m_rsi_state->program_status = static_cast<ProgramStatus>(parseNumber<long>(r));
}

void RsiParser::onDelay(std::string_view d)
{
  m_rsi_state->delay = parseNumber<std::size_t>(d);
}

void RsiParser::onIpoc(std::string_view text)
{
  m_rsi_state->ipoc = parseNumber<std::size_t>(text);
}

} // namespace kuka_rsi_driver
