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

/*!\file rsi.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-10-04
 */
#include "kuka_rsi_driver/rsi.h"

#include <cmath>
#include <limits>

namespace kuka_rsi_driver {

JointArray::JointArray()
{
  m_values.fill(std::numeric_limits<double>::quiet_NaN());
}

std::size_t JointArray::size() const
{
  return m_values.size();
}

void JointArray::fill(double v)
{
  std::fill(m_values.begin(), m_values.end(), v);
}

double& JointArray::operator[](std::size_t i)
{
  return m_values[i];
}

double JointArray::operator[](std::size_t i) const
{
  return m_values[i];
}

void JointArray::swap(JointArray& j)
{
  std::swap(m_values, j.m_values);
}

CartesianPose::CartesianPose()
  : x{std::numeric_limits<double>::quiet_NaN()}
  , y{std::numeric_limits<double>::quiet_NaN()}
  , z{std::numeric_limits<double>::quiet_NaN()}
  , a{std::numeric_limits<double>::quiet_NaN()}
  , b{std::numeric_limits<double>::quiet_NaN()}
  , c{std::numeric_limits<double>::quiet_NaN()}
{
}

CartesianPose::CartesianPose(double x, double y, double z, double a, double b, double c)
  : x{x}
  , y{y}
  , z{z}
  , a{a}
  , b{b}
  , c{c}
{
}

void CartesianPose::getQuaternion(double& x, double& y, double& z, double& w) const
{
  const double sa = std::sin(a * M_PI / 360);
  const double ca = std::cos(a * M_PI / 360);
  const double sb = std::sin(b * M_PI / 360);
  const double cb = std::cos(b * M_PI / 360);
  const double sc = std::sin(c * M_PI / 360);
  const double cc = std::cos(c * M_PI / 360);

  x = ca * cb * sc - sa * sb * cc;
  y = ca * sb * cc + sa * cb * sc;
  z = sa * cb * cc - ca * sb * sc;
  w = ca * cb * cc + sa * sb * sc;
}

RsiState::RsiState()
  : delay{0}
  , ipoc{0}
{
}

RsiCommand::RsiCommand()
{
  axis_command_pos.fill(0.0);
}

void interpolate(const RsiCommand& c1, const RsiCommand& c2, double alpha, RsiCommand& dest)
{
  for (std::size_t i = 0; i < c1.axis_command_pos.size(); ++i)
  {
    dest.axis_command_pos[i] =
      (1 - alpha) * c1.axis_command_pos[i] + alpha * c2.axis_command_pos[i];
  }
}

} // namespace kuka_rsi_driver
