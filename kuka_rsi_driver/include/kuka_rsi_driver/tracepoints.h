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

/*!\file kuka_rsi_driver/tracepoints.h
 * \brief Definitions of tracepoints used throughout the driver
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-09-16
 */
#undef LTTNG_UST_TRACEPOINT_PROVIDER
#define LTTNG_UST_TRACEPOINT_PROVIDER kuka_rsi_driver

#undef LTTNG_UST_TRACEPOINT_INCLUDE
#define LTTNG_UST_TRACEPOINT_INCLUDE "kuka_rsi_driver/tracepoints.h"

#if !defined(KUKA_RSI_DRIVER_TRACEPOINTS_H_INCLUDED) ||                                            \
  defined(LTTNG_UST_TRACEPOINT_HEADER_MULTI_READ)
#  define KUKA_RSI_DRIVER_TRACEPOINTS_H_INCLUDED

#  include <lttng/tracepoint.h>

LTTNG_UST_TRACEPOINT_EVENT(
  LTTNG_UST_TRACEPOINT_PROVIDER,
  read,
  LTTNG_UST_TP_ARGS(unsigned long, time_nsec, unsigned long, period_nsec),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer(unsigned long, time_nsec, time_nsec)
                        lttng_ust_field_integer(unsigned long, period_nsec, period_nsec)))

LTTNG_UST_TRACEPOINT_EVENT(
  LTTNG_UST_TRACEPOINT_PROVIDER,
  write,
  LTTNG_UST_TP_ARGS(unsigned long, time_nsec, unsigned long, period_nsec),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer(unsigned long, time_nsec, time_nsec)
                        lttng_ust_field_integer(unsigned long, period_nsec, period_nsec)))

LTTNG_UST_TRACEPOINT_EVENT(
  LTTNG_UST_TRACEPOINT_PROVIDER,
  rsi_packet_received,
  LTTNG_UST_TP_ARGS(double, local_phase, long, phase_offset),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_float(double, local_phase, local_phase)
                        lttng_ust_field_integer(long, phase_offset, phase_offset)))

LTTNG_UST_TRACEPOINT_EVENT(
  LTTNG_UST_TRACEPOINT_PROVIDER,
  cmd_interpolate,
  LTTNG_UST_TP_ARGS(
    double, local_phase, long, phase_offset, long, interpolation_offset, double, interpolation_t),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_float(double, local_phase, local_phase)
                        lttng_ust_field_integer(long, phase_offset, phase_offset)
                          lttng_ust_field_integer(long, interpolation_offset, interpolation_offset)
                            lttng_ust_field_float(double, interpolation_t, interpolation_t)))

LTTNG_UST_TRACEPOINT_EVENT(
  LTTNG_UST_TRACEPOINT_PROVIDER,
  rsi_packet_sent,
  LTTNG_UST_TP_ARGS(long,
                    ipoc,
                    double,
                    cmd_pos_0,
                    double,
                    cmd_pos_1,
                    double,
                    cmd_pos_2,
                    double,
                    cmd_pos_3,
                    double,
                    cmd_pos_4,
                    double,
                    cmd_pos_5),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer(long, ipoc, ipoc)
                        lttng_ust_field_float(double, cmd_pos_0, cmd_pos_0)
                          lttng_ust_field_float(double, cmd_pos_1, cmd_pos_1)
                            lttng_ust_field_float(double, cmd_pos_2, cmd_pos_2)
                              lttng_ust_field_float(double, cmd_pos_3, cmd_pos_3)
                                lttng_ust_field_float(double, cmd_pos_4, cmd_pos_4)
                                  lttng_ust_field_float(double, cmd_pos_5, cmd_pos_5)))

#endif // KUKA_RSI_DRIVER_TRACEPOINTS_H_INCLUDED

#include <lttng/tracepoint-event.h>
