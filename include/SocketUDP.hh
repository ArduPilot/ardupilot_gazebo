/*
   Copyright (C) 2024 ardupilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SOCKETUDP_HH_
#define SOCKETUDP_HH_

#include <fcntl.h>
#include <unistd.h>
#ifdef _WIN32
    #include <winsock2.h>
    #include <Ws2tcpip.h>
#else
    #include <sys/ioctl.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <arpa/inet.h>
    #include <sys/select.h>
#endif

/// \brief Simple UDP socket handling class.
class SocketUDP {
public:
    /// \brief Constructor.
    SocketUDP(bool reuseaddress, bool blocking);

    /// \brief Destructor.
    ~SocketUDP();

    /// \brief Bind socket to address and port.
    bool bind(const char *address, uint16_t port);

    /// \brief Set reuse address option.
    bool set_reuseaddress();

    /// \brief Set blocking state.
    bool set_blocking(bool blocking);

    /// \brief Send data to address and port.
    ssize_t sendto(const void *buf, size_t size, const char *address, uint16_t port);

    /// \brief Receive data.
    ssize_t recv(void *pkt, size_t size, uint32_t timeout_ms);

    /// \brief Get last client address and port
    void get_client_address(const char *&ip_addr, uint16_t &port);

private:
    /// \brief File descriptor.
    struct sockaddr_in in_addr {};

    /// \brief File descriptor.
    int fd = -1;

    /// \brief Poll for incoming data with timeout.
    bool pollin(uint32_t timeout_ms);

    /// \brief Make a sockaddr_in struct from address and port.
    void make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr);
};
#endif  // SOCKETUDP_HH_
