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
#include <gtest/gtest.h>

#include <kuka_rsi_driver/rsi_writer.h>

using namespace kuka_rsi_driver;

TEST(RsiWriter, WriteTestXml)
{
  const auto log = rclcpp::get_logger("rsi_writer");

  RsiWriter writer{"TestSenType", log};

  // Create test command
  RsiCommand cmd;
  for (std::size_t i = 0; i < cmd.axis_command_pos.size(); ++i)
  {
    cmd.axis_command_pos[i] = i;
  }

  // Write command to buffer
  std::vector<char> buffer;
  buffer.resize(1024);
  const auto result_size = writer.writeCommand(cmd, 123456, buffer);

  // Verify sizes
  ASSERT_GT(result_size, 0);

  // clang-format off
  const std::string expected_result =
    R"(<Sen Type="TestSenType"><AK A1="0.000000" A2="1.000000" A3="2.000000" A4="3.000000" A5="4.000000" A6="5.000000" /><IPOC>123456</IPOC></Sen>)";
  // clang-format on

  EXPECT_EQ(expected_result, (std::string_view{buffer.data(), result_size}));
}
