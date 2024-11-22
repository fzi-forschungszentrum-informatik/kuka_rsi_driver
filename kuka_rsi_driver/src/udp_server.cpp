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

/*!\file udp_server.cpp
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2024-08-30
 */
#include "kuka_rsi_driver/udp_server.h"

#include <arpa/inet.h>
#include <fmt/format.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

namespace kuka_rsi_driver {

UdpServer::UdpServer(const std::string& address,
                     unsigned short port,
                     std::chrono::milliseconds timeout)
  : m_timeout{timeout}
{
  // Create socket
  if (m_socket_fd = socket(AF_INET, SOCK_DGRAM, 0); m_socket_fd == -1)
  {
    int error = errno;
    throw std::runtime_error{fmt::format("Could not open socket: {}", strerror(error))};
  }

  // Bind server address to socket
  struct sockaddr_in serveraddr;
  std::memset(&serveraddr, 0, sizeof(serveraddr));
  serveraddr.sin_family      = AF_INET;
  serveraddr.sin_addr.s_addr = inet_addr(address.c_str());
  serveraddr.sin_port        = htons(port);
  if (serveraddr.sin_addr.s_addr == INADDR_NONE)
  {
    throw std::runtime_error{fmt::format("Invalid host address '{}'", address)};
  }

  if (bind(m_socket_fd, reinterpret_cast<struct sockaddr*>(&serveraddr), sizeof(serveraddr)) == -1)
  {
    int error = errno;
    throw std::runtime_error{fmt::format("Could not bind address to socket: {}", strerror(error))};
  }

  // Create epoll fd
  if (m_epoll_fd = epoll_create(1); m_epoll_fd == -1)
  {
    int error = errno;
    throw std::runtime_error{
      fmt::format("Could not create epoll file descriptor: {}", strerror(error))};
  }

  // Set up epoll events
  struct epoll_event epoll_ev;
  epoll_ev.events = EPOLLIN | EPOLLET;
  if (epoll_ctl(m_epoll_fd, EPOLL_CTL_ADD, m_socket_fd, &epoll_ev) == -1)
  {
    int error = errno;
    throw std::runtime_error{fmt::format("Could not add epoll event: {}", strerror(error))};
  }

  // Create buffer space for epoll events
  m_epoll_events.resize(sizeof(struct epoll_event));
}

UdpServer::~UdpServer()
{
  if (m_epoll_fd >= 0)
  {
    close(m_epoll_fd);
  }

  if (m_socket_fd >= 0)
  {
    close(m_socket_fd);
  }
}

std::optional<std::size_t> UdpServer::receive(std::span<std::byte> target)
{
  if (int wait_ret = epoll_wait(m_epoll_fd,
                                reinterpret_cast<struct epoll_event*>(&m_epoll_events[0]),
                                1,
                                m_timeout.count());
      wait_ret == -1)
  {
    int error = errno;
    throw std::runtime_error{fmt::format("Error waiting for event: {}", strerror(error))};
  }
  else if (wait_ret > 0)
  {
    m_client_addr_len = sizeof(struct sockaddr_in);
    if (const auto recv_ret = recvfrom(m_socket_fd,
                                       &target[0],
                                       target.size(),
                                       MSG_DONTWAIT,
                                       reinterpret_cast<struct sockaddr*>(&m_client_addr),
                                       &m_client_addr_len);
        recv_ret == -1)
    {
      int error = errno;

      if ((error == EAGAIN) || (error == EWOULDBLOCK))
      {
        return std::nullopt;
      }

      throw std::runtime_error{fmt::format("Could not read from socket: {}", strerror(error))};
    }
    else if (recv_ret > 0)
    {
      return recv_ret;
    }
  }

  return std::nullopt;
}

void UdpServer::send(std::span<const std::byte> data)
{
  if (const auto ret = sendto(m_socket_fd,
                              &data[0],
                              data.size(),
                              0,
                              reinterpret_cast<struct sockaddr*>(&m_client_addr),
                              m_client_addr_len);
      ret == -1)
  {
    int error = errno;
    throw std::runtime_error{fmt::format("Could not send packet: {}", strerror(error))};
  }
  else if (static_cast<std::size_t>(ret) != data.size())
  {
    throw std::runtime_error{fmt::format("Only sent {} of {} bytes", ret, data.size())};
  }
}

} // namespace kuka_rsi_driver
