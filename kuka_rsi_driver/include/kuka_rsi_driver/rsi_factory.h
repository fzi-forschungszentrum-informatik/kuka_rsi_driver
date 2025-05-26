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

/*!\file kuka_rsi_driver/rsi_factory.h
 * \brief Storage for pre-allocated RSI commands and states
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-11-03
 */
#ifndef KUKA_RSI_DRIVER_RSI_FACTORY_H_INCLUDED
#define KUKA_RSI_DRIVER_RSI_FACTORY_H_INCLUDED

#include "rsi.h"
#include "rsi_config.h"

#include <memory>
#include <vector>

namespace kuka_rsi_driver {

class RsiFactory
{
public:
  explicit RsiFactory(std::size_t cyclic_buf_size = 1024);
  explicit RsiFactory(std::shared_ptr<RsiConfig> config, std::size_t cyclic_buf_size = 1024);

  RsiCommand createCommand() const;
  std::shared_ptr<RsiCommand> createCyclicCommand();

  std::shared_ptr<RsiState> createCyclicState();

private:
  std::shared_ptr<RsiConfig> m_rsi_config;

  std::vector<std::shared_ptr<RsiCommand>> m_cmd_buf;
  std::size_t m_cmd_i;

  std::vector<std::shared_ptr<RsiState>> m_state_buf;
  std::size_t m_state_i;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_RSI_FACTORY_H_INCLUDED
