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

/*!\file rsi_factory.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-11-03
 */
#include "kuka_rsi_driver/rsi_factory.h"

#include <iostream>

namespace kuka_rsi_driver {

RsiFactory::RsiFactory(std::size_t cyclic_buf_size)
  : m_cmd_i{0}
  , m_state_i{0}
{
  m_cmd_buf.reserve(cyclic_buf_size);
  m_state_buf.reserve(cyclic_buf_size);

  for (std::size_t i = 0; i < cyclic_buf_size; ++i)
  {
    m_cmd_buf.emplace_back(std::make_shared<RsiCommand>());
    m_state_buf.emplace_back(std::make_shared<RsiState>());
  }
}

RsiCommand RsiFactory::createCommand() const
{
  return RsiCommand{};
}

std::shared_ptr<RsiCommand> RsiFactory::createCyclicCommand()
{
  const auto& cmd = m_cmd_buf[m_cmd_i++];
  m_cmd_i %= m_cmd_buf.size();

  if (cmd.use_count() != 1)
  {
    throw std::runtime_error{"Cyclic commands exhausted"};
  }

  return cmd;
}

std::shared_ptr<RsiState> RsiFactory::createCyclicState()
{
  const auto& state = m_state_buf[m_state_i++];
  m_state_i %= m_state_buf.size();

  if (state.use_count() != 1)
  {
    std::cerr << m_state_i << " - " << state.use_count() << std::endl;
    throw std::runtime_error{"Cyclic states exhausted"};
  }

  return state;
}

} // namespace kuka_rsi_driver
