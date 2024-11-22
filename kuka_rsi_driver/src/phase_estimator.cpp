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

/*!\file phase_estimator.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-09-30
 */
#include "kuka_rsi_driver/phase_estimator.h"

#include <cmath>
#include <numbers>
#include <numeric>

namespace kuka_rsi_driver {

PhaseEstimator::PhaseEstimator()
  : m_filter_buf{15}
{
  zeroOffset();
}

void PhaseEstimator::zeroOffset()
{
  SignalRecord empty_signal{};
  m_last_write  = SignalRecord{};
  m_last_packet = SignalRecord{};

  m_filter_buf.clear();

  m_last_estimate = Estimate{};
}

void PhaseEstimator::signalWrite(std::chrono::steady_clock::time_point time)
{
  ++m_last_write.count;
  m_last_write.time = time;
}

void PhaseEstimator::signalPacket()
{
  const auto time_now = std::chrono::steady_clock::now();

  const auto since_last_packet =
    std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - m_last_packet.time).count();
  const auto since_last_write =
    std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - m_last_write.time).count();

  const auto local_phase =
    (static_cast<double>(since_last_write) / static_cast<double>(since_last_packet));
  const auto offset = static_cast<std::ptrdiff_t>(m_last_write.count) -
                      static_cast<std::ptrdiff_t>(m_last_packet.count) - 1;

  m_filter_buf.push_front(local_phase);
  const auto filter_sum = std::transform_reduce(m_filter_buf.cbegin(),
                                                m_filter_buf.cend(),
                                                0.0,
                                                std::plus{},
                                                [local_phase](double d) -> double {
                                                  const double diff = d - local_phase;
                                                  if (std::fabs(diff) > 0.5)
                                                  {
                                                    return d - std::copysign(1.0, diff);
                                                  }
                                                  else
                                                  {
                                                    return d;
                                                  }
                                                });

  m_last_estimate.phase  = filter_sum / static_cast<double>(m_filter_buf.size());
  m_last_estimate.offset = offset;

  // Bookkeeping
  ++m_last_packet.count;
  m_last_packet.time = time_now;
}

PhaseEstimator::Estimate PhaseEstimator::estimate() const
{
  return m_last_estimate;
}

} // namespace kuka_rsi_driver
