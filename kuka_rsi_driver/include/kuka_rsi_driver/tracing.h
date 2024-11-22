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

/*!\file kuka_rsi_driver/tracing.h
 * \brief Wrapper for tracing with driver tracepoints
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-09-16
 */
#ifndef KUKA_RSI_DRIVER_TRACING_H_INCLUDED
#define KUKA_RSI_DRIVER_TRACING_H_INCLUDED

#include "kuka_rsi_driver/config.h"

#ifdef TRACING_ENABLED

#  include "kuka_rsi_driver/tracepoints.h"
#  define KUKA_RSI_DRIVER_TRACEPOINT(name, ...)                                                    \
    lttng_ust_tracepoint(kuka_rsi_driver, name, __VA_ARGS__)

#else

#  define KUKA_RSI_DRIVER_TRACEPOINT(name, ...)

#endif

#endif // KUKA_RSI_DRIVER_TRACING_H_INCLUDED