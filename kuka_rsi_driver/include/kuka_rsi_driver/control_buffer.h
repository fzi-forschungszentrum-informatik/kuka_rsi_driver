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

/*!\file kuka_rsi_driver/control_buffer.h
 * \brief Interpolation buffer robot commands and states
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-10-16
 */
#ifndef KUKA_RSI_DRIVER_CONTROL_BUFFER_H_INCLUDED
#define KUKA_RSI_DRIVER_CONTROL_BUFFER_H_INCLUDED

#include "phase_estimator.h"
#include "rsi.h"
#include "rsi_factory.h"

#include <boost/circular_buffer.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <span>
#include <vector>

namespace kuka_rsi_driver {

template <typename T>
class SynchronizationBuffer
{
public:
  SynchronizationBuffer(std::size_t size);

  bool push(const T& element);
  std::span<T> pop();

private:
  boost::lockfree::spsc_queue<T> m_sync_queue;
  std::vector<T> m_buf;
};

class ControlBuffer
{
public:
  ControlBuffer(RsiFactory& rsi_factory);

  void syncCommands();

  bool pushCommand(std::shared_ptr<RsiCommand> cmd);
  std::shared_ptr<RsiCommand> interpolateCommand(double phase);

  bool pushState(std::shared_ptr<RsiState> state);
  std::shared_ptr<RsiState> popStates();

  void zeroOffsets();
  PhaseEstimator& phaseEstimator();

  /*! \brief Interpolate between stored commands
   *
   * An argument t will yield the most recently stored command. Setting t to a larger value will
   * interpolate towards older commands, and having a t larger then the buffer size will return the
   * oldest stored command.
   *
   * \param t Time before last command
   *
   * \returns Interpolation result
   */
  std::shared_ptr<RsiCommand> interpolate(double t) const;

private:
  SynchronizationBuffer<std::shared_ptr<RsiState>> m_state_buf;
  SynchronizationBuffer<std::shared_ptr<RsiCommand>> m_cmd_buf;

  boost::circular_buffer<std::shared_ptr<RsiCommand>> m_interpolation_buf;
  std::shared_ptr<RsiCommand> m_interpolate_cmd;

  PhaseEstimator m_phase_estimator;
};

} // namespace kuka_rsi_driver


namespace kuka_rsi_driver {

template <typename T>
SynchronizationBuffer<T>::SynchronizationBuffer(std::size_t size)
  : m_sync_queue{size}
{
  m_buf.resize(size);
}

template <typename T>
bool SynchronizationBuffer<T>::push(const T& element)
{
  return m_sync_queue.push(element);
}

template <typename T>
std::span<T> SynchronizationBuffer<T>::pop()
{
  const auto num_elements = m_sync_queue.pop(&m_buf[0], m_buf.size());
  return std::span<T>{&m_buf[0], num_elements};
}

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_CONTROL_BUFFER_H_INCLUDED
