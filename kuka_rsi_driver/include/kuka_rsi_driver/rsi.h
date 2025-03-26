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

/*!\file kuka_rsi_driver/rsi.h
 * \brief Data structures used for RSI communication
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-10-04
 */
#ifndef KUKA_RSI_DRIVER_RSI_H_INCLUDED
#define KUKA_RSI_DRIVER_RSI_H_INCLUDED

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

namespace kuka_rsi_driver {

class JointArray
{
public:
  JointArray();

  using Iterator      = std::array<double, 6>::iterator;
  using ConstIterator = std::array<double, 6>::const_iterator;

  std::size_t size() const;

  void fill(double v);

  double& operator[](std::size_t i);
  double operator[](std::size_t i) const;

  Iterator begin();
  ConstIterator begin() const;

  Iterator end();
  ConstIterator end() const;

  void swap(JointArray& j);

private:
  std::array<double, 6> m_values;
};

class CartesianPose
{
public:
  CartesianPose();
  CartesianPose(double x, double y, double z, double a, double b, double c);

  using Iterator      = std::array<double, 6>::iterator;
  using ConstIterator = std::array<double, 6>::const_iterator;

  double& x();
  double x() const;

  double& y();
  double y() const;

  double& z();
  double z() const;

  double& a();
  double a() const;

  double& b();
  double b() const;

  double& c();
  double c() const;

  Iterator begin();
  ConstIterator begin() const;

  Iterator end();
  ConstIterator end() const;

  void getQuaternion(double& x, double& y, double& z, double& w) const;

private:
  std::array<double, 6> m_values;
};

enum class ProgramStatus : std::uint8_t
{
  RUNNING = 3,
  STOPPED = 4
};

/*! \brief Values that are transparently tunneled between interfaces and RSI attributes
 */
struct RsiPassthrough
{
  /*! \brief Pre-Allocate passthrough storage of given size
   *
   * \param num_bool Number of boolean values to pre-allocate
   * \param num_double Number of double values to pre-allocate
   */
  explicit RsiPassthrough(std::size_t num_bool, std::size_t num_double);

  //! Passed-through boolean values
  std::vector<bool> values_bool;
  //! Passed-through double values
  std::vector<double> values_double;
};

class RsiState
{
public:
  /*! \brief Pre-Allocate a new state object
   *
   * \param num_passthrough_bool Number of boolean passthrough values to pre-allocate
   * \param num_passthrough_double Number of double passthrough values to pre-allocate
   */
  explicit RsiState(std::size_t num_passthrough_bool, std::size_t num_passthrough_double);

  JointArray axis_actual_pos;
  JointArray axis_setpoint_pos;

  JointArray axis_eff;

  CartesianPose cartesian_actual_pos;
  CartesianPose cartesian_setpoint_pos;

  std::size_t delay;
  std::size_t ipoc;

  double speed_scaling;

  ProgramStatus program_status;

  //! Additional values that are directly tunneled from RSI attributes to state interfaces
  RsiPassthrough passthrough;
};

class RsiCommand
{
public:
  /*! \brief Pre-Allocate a new command object
   *
   * \param num_passthrough_bool Number of boolean passthrough values to pre-allocate
   * \param num_passthrough_double Number of double passthrough values to pre-allocate
   */
  explicit RsiCommand(std::size_t num_passthrough_bool, std::size_t num_passthrough_double);

  std::chrono::steady_clock::time_point write_time;

  JointArray axis_command_pos;

  //! Additional values that are directly tunneled from command interfaces to RSI attributes
  RsiPassthrough passthrough;

private:
};

/*! \brief Perform linear interpolation between two commands
 *
 * The commanded position stored in dest is the result of
 *    (1-alpha) * c1 + alpha * c2
 *
 * \param c1 First interpolation point
 * \param c2 Second interpolation point
 * \param alpha Interpolation parameter
 * \param dest Command to store result to
 */
void interpolate(const RsiCommand& c1, const RsiCommand& c2, double alpha, RsiCommand& dest);

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_RSI_H_INCLUDED
