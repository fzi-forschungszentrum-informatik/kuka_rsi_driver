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

/*!\file kuka_rsi_driver/udp_server.h
 * \brief UDP server for listening for RSI requests
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-10-16
 */
#ifndef KUKA_RSI_DRIVER_UDP_SERVER_H_INCLUDED
#define KUKA_RSI_DRIVER_UDP_SERVER_H_INCLUDED

#include <chrono>
#include <cstddef>
#include <netinet/in.h>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace kuka_rsi_driver {

class UdpServer
{
public:
  UdpServer(const std::string& address, unsigned short port, std::chrono::milliseconds timeout);
  ~UdpServer();

  void setTimeout(std::chrono::milliseconds timeout);

  UdpServer(const UdpServer&)            = delete;
  UdpServer& operator=(const UdpServer&) = delete;

  std::optional<std::size_t> receive(std::span<std::byte> target);
  void send(std::span<const std::byte> data);

private:
  int m_socket_fd;
  int m_epoll_fd;

  std::vector<std::byte> m_epoll_events;
  std::chrono::milliseconds m_timeout;

  struct sockaddr_in m_client_addr;
  socklen_t m_client_addr_len;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_UDP_SERVER_H_INCLUDED
