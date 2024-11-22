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
#include <gtest/gtest.h>

#include <cstring>
#include <kuka_rsi_driver/rsi_factory.h>
#include <kuka_rsi_driver/rsi_parser.h>
#include <rclcpp/logger.hpp>
#include <string>

#include <rclcpp/logging.hpp>

using namespace kuka_rsi_driver;

TEST(RsiParser, XmlParser)
{
  // clang-format off
  const std::string test_xml =
    R"(<root>)"
    R"(  <abc foo="bar" />)"
    R"(  <def>foo</def>)"
    R"(  <def a="b">foo</def>)"
    R"(</root>)";
  // clang-format on

  // Create parser
  const auto log = rclcpp::get_logger("xml_parser");
  XmlParser parser{"root", log};

  // Check callback calls
  std::size_t cb_i = 0;
  parser.addAttributeCb("abc", [&](const char** attrs) {
    ASSERT_NE(attrs, nullptr);

    switch (cb_i++)
    {
      case 0:
        EXPECT_STREQ(attrs[0], "foo");
        EXPECT_STREQ(attrs[1], "bar");
        EXPECT_EQ(attrs[2], nullptr);
        break;

      default:
        FAIL() << "Error at callback #" << (cb_i - 1);
    }
  });

  parser.addElementCb("abc", [&](std::string_view chars) {
    switch (cb_i++)
    {
      case 1:
        EXPECT_TRUE(chars.empty());
        break;

      default:
        FAIL() << "Error at callback #" << (cb_i - 1);
    }
  });

  parser.addAttributeCb("def", [&](const char** attrs) {
    ASSERT_NE(attrs, nullptr);

    switch (cb_i++)
    {
      case 2:
        EXPECT_EQ(attrs[0], nullptr);
        break;

      case 4:
        EXPECT_STREQ(attrs[0], "a");
        EXPECT_STREQ(attrs[1], "b");
        break;

      default:
        FAIL() << "Error at callback #" << (cb_i - 1);
    }
  });
  parser.addElementCb("def", [&](std::string_view chars) {
    switch (cb_i++)
    {
      case 3:
        EXPECT_EQ(chars, "foo");
        break;

      case 5:
        EXPECT_EQ(chars, "foo");
        break;

      default:
        FAIL() << "Error at callback #" << (cb_i - 1);
    }
  });

  // Set up test xml data
  const auto buf = parser.buffer();
  ASSERT_GE(buf.size(), test_xml.size());
  std::memcpy(&buf[0], test_xml.data(), test_xml.size());

  // Verify complete document is parsed
  parser.parseBuffer(test_xml.size());
  EXPECT_EQ(cb_i, 6);
}

TEST(RsiParser, ParseTestXml)
{
  // clang-format off
  const std::string test_xml =
    R"(<Rob TYPE="KUKA">)"
    R"(  <RIst X="1.0" Y="2.0" Z="3.0" A="0.5" B="1.0" C="1.5" />)"
    R"(  <RSol X="2.0" Y="4.0" Z="8.0" A="1.5" B="1.0" C="0.5" />)"
    R"(  <AIPos A1="0.1" A2="0.2" A3="0.3" A4="0.4" A5="0.5" A6="0.6" />)"
    R"(  <ASPos A1="1.0" A2="0.9" A3="0.8" A4="0.7" A5="0.6" A6="0.5" />)"
    R"(  <Delay D="42" />)"
    R"(  <IPOC>123645634563</IPOC>)"
    R"(</Rob>)";
  // clang-format on

  const auto log = rclcpp::get_logger("rsi_parser");

  RsiFactory rsi_factory;
  RsiParser rsi_parser{log, &rsi_factory};

  const auto buf = rsi_parser.buffer();
  ASSERT_GE(buf.size(), test_xml.size());
  std::memcpy(&buf[0], test_xml.data(), test_xml.size());

  const auto rsi_state = rsi_parser.parseBuffer(test_xml.size());

  ASSERT_EQ(rsi_state->axis_actual_pos.size(), 6);
  EXPECT_DOUBLE_EQ(rsi_state->axis_actual_pos[0], 0.1);
  EXPECT_DOUBLE_EQ(rsi_state->axis_actual_pos[1], 0.2);
  EXPECT_DOUBLE_EQ(rsi_state->axis_actual_pos[2], 0.3);
  EXPECT_DOUBLE_EQ(rsi_state->axis_actual_pos[3], 0.4);
  EXPECT_DOUBLE_EQ(rsi_state->axis_actual_pos[4], 0.5);
  EXPECT_DOUBLE_EQ(rsi_state->axis_actual_pos[5], 0.6);

  ASSERT_EQ(rsi_state->axis_setpoint_pos.size(), 6);
  EXPECT_DOUBLE_EQ(rsi_state->axis_setpoint_pos[0], 1.0);
  EXPECT_DOUBLE_EQ(rsi_state->axis_setpoint_pos[1], 0.9);
  EXPECT_DOUBLE_EQ(rsi_state->axis_setpoint_pos[2], 0.8);
  EXPECT_DOUBLE_EQ(rsi_state->axis_setpoint_pos[3], 0.7);
  EXPECT_DOUBLE_EQ(rsi_state->axis_setpoint_pos[4], 0.6);
  EXPECT_DOUBLE_EQ(rsi_state->axis_setpoint_pos[5], 0.5);

  EXPECT_EQ(rsi_state->cartesian_actual_pos.x, 1.0);
  EXPECT_EQ(rsi_state->cartesian_actual_pos.y, 2.0);
  EXPECT_EQ(rsi_state->cartesian_actual_pos.z, 3.0);
  EXPECT_EQ(rsi_state->cartesian_actual_pos.a, 0.5);
  EXPECT_EQ(rsi_state->cartesian_actual_pos.b, 1.0);
  EXPECT_EQ(rsi_state->cartesian_actual_pos.c, 1.5);

  EXPECT_EQ(rsi_state->cartesian_setpoint_pos.x, 2.0);
  EXPECT_EQ(rsi_state->cartesian_setpoint_pos.y, 4.0);
  EXPECT_EQ(rsi_state->cartesian_setpoint_pos.z, 8.0);
  EXPECT_EQ(rsi_state->cartesian_setpoint_pos.a, 1.5);
  EXPECT_EQ(rsi_state->cartesian_setpoint_pos.b, 1.0);
  EXPECT_EQ(rsi_state->cartesian_setpoint_pos.c, 0.5);

  EXPECT_EQ(rsi_state->delay, 42);
  EXPECT_EQ(rsi_state->ipoc, 123645634563ul);
}
