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
 * \date    2024-10-16
 */
#include "kuka_rsi_driver/control_buffer.h"

#include "kuka_rsi_driver/tracing.h"

#include <cmath>

namespace kuka_rsi_driver {

ControlBuffer::ControlBuffer(RsiFactory& rsi_factory)
  : m_state_buf{3}
  , m_cmd_buf{3}
  , m_interpolation_buf{3}
  , m_interpolate_cmd{std::make_shared<RsiCommand>(rsi_factory.createCommand())}
{
}

void ControlBuffer::syncCommands()
{
  for (auto& cmd : m_cmd_buf.pop())
  {
    m_phase_estimator.signalWrite(cmd->write_time);
    m_interpolation_buf.push_front(std::move(cmd));
  }
}

bool ControlBuffer::pushCommand(std::shared_ptr<RsiCommand> cmd)
{
  return m_cmd_buf.push(std::move(cmd));
}

std::shared_ptr<RsiCommand> ControlBuffer::interpolateCommand(double phase)
{
  if (!m_interpolation_buf.full())
  {
    return nullptr;
  }

  const auto interp_t = phase - 1.5;
  KUKA_RSI_DRIVER_TRACEPOINT(cmd_interpolate, phase, 0, 0, interp_t);

  return interpolate(-interp_t);
}

bool ControlBuffer::pushState(std::shared_ptr<RsiState> state)
{
  return m_state_buf.push(std::move(state));
}

std::shared_ptr<RsiState> ControlBuffer::popStates()
{
  const auto states = m_state_buf.pop();
  if (states.empty())
  {
    return nullptr;
  }

  return std::move(states[states.size() - 1]);
}

void ControlBuffer::zeroOffsets()
{
  m_phase_estimator.zeroOffset();
}

PhaseEstimator& ControlBuffer::phaseEstimator()
{
  return m_phase_estimator;
}

std::shared_ptr<RsiCommand> ControlBuffer::interpolate(double t) const
{
  if (t >= (m_interpolation_buf.size() - 1))
  {
    return m_interpolation_buf.back();
  }
  else if (t <= 0)
  {
    return m_interpolation_buf.front();
  }
  else
  {
    const auto i0 = static_cast<std::size_t>(std::floor(t));
    kuka_rsi_driver::interpolate(*m_interpolation_buf[i0],
                                 *m_interpolation_buf[i0 + 1],
                                 t - static_cast<double>(i0),
                                 *m_interpolate_cmd);
    return m_interpolate_cmd;
  }
}

} // namespace kuka_rsi_driver
