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

/*!\file control_buffer.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-11-02
 */
#include <gtest/gtest.h>

#include <kuka_rsi_driver/control_buffer.h>
#include <kuka_rsi_driver/rsi_factory.h>

using namespace kuka_rsi_driver;

TEST(ControlBuffer, interpolation)
{
  RsiFactory rsi_factory;
  ControlBuffer test_buf{rsi_factory};

  std::shared_ptr<RsiCommand> test_cmd_1 = rsi_factory.createCyclicCommand();
  test_cmd_1->axis_command_pos.fill(1.0);
  ASSERT_TRUE(test_buf.pushCommand(test_cmd_1));

  std::shared_ptr<RsiCommand> test_cmd_2 = rsi_factory.createCyclicCommand();
  test_cmd_2->axis_command_pos.fill(2.0);
  ASSERT_TRUE(test_buf.pushCommand(test_cmd_2));

  std::shared_ptr<RsiCommand> test_cmd_3 = rsi_factory.createCyclicCommand();
  test_cmd_3->axis_command_pos.fill(4.0);
  ASSERT_TRUE(test_buf.pushCommand(test_cmd_3));

  test_buf.syncCommands();

  std::shared_ptr<RsiCommand> result;

  // Interpolate into future: -0.5
  result = test_buf.interpolate(-0.5);
  for (std::size_t i = 0; i < result->axis_command_pos.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(4.0, result->axis_command_pos[i]);
  }

  // Interpolate first point: 0.0
  result = test_buf.interpolate(0.0);
  for (std::size_t i = 0; i < result->axis_command_pos.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(4.0, result->axis_command_pos[i]);
  }

  // Interpolate between first points: 0.5
  result = test_buf.interpolate(0.5);
  for (std::size_t i = 0; i < result->axis_command_pos.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(3.0, result->axis_command_pos[i]);
  }

  // Interpolate second point: 1.0
  result = test_buf.interpolate(1.0);
  for (std::size_t i = 0; i < result->axis_command_pos.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(2.0, result->axis_command_pos[i]);
  }

  // Interpolate between second and third points: 1.5
  result = test_buf.interpolate(1.5);
  for (std::size_t i = 0; i < result->axis_command_pos.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(1.5, result->axis_command_pos[i]);
  }

  // Interpolate last point: 2.0
  result = test_buf.interpolate(2.0);
  for (std::size_t i = 0; i < result->axis_command_pos.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(1.0, result->axis_command_pos[i]);
  }

  // Interpolate further into past: 2.5
  result = test_buf.interpolate(2.5);
  for (std::size_t i = 0; i < result->axis_command_pos.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(1.0, result->axis_command_pos[i]);
  }
}
