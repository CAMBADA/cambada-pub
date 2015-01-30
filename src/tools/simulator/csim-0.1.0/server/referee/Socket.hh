/*
 *  CSim - CAMBADA Simulator
 *  Copyright (C) 2010  Universidade de Aveiro
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
 
/*
 *  Author: Rob Tougher
 *  Copyright © 2002, Rob Tougher.
 *  Source: http://tldp.org/LDP/LG/issue74/tougher.html
 */

// Definition of the Socket class

#ifndef Socket_class
#define Socket_class


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>


namespace csim {

  const int MAXHOSTNAME = 200;
  const int MAXCONNECTIONS = 5;
  const int MAXRECV = 500;

  class Socket
  {
   public:
    Socket();
    virtual ~Socket();

    // Server initialization
    bool create();
    bool bind ( const int port );
    bool listen() const;
    bool accept ( Socket& ) const;

    // Client initialization
    bool connect ( const std::string host, const int port );

    // Data Transimission
    bool send ( const std::string ) const;
    int recv ( std::string& ) const;


    void set_non_blocking ( const bool );

    bool is_valid() const { return m_sock != -1; }

   private:

    int m_sock;
    sockaddr_in m_addr;


  };


  class SocketException
  {
   public:
    SocketException ( std::string s ) : m_s ( s ) {};
    ~SocketException (){};

    std::string description() { return m_s; }

   private:

    std::string m_s;

  };
  
  // -----
  class ServerSocket : private Socket
  {
   public:

    ServerSocket ( int port );
    ServerSocket (){};
    virtual ~ServerSocket();

    const ServerSocket& operator << ( const std::string& ) const;
    const ServerSocket& operator >> ( std::string& ) const;

    void accept ( ServerSocket& );

  };
  
  // -----
  class ClientSocket : private Socket
  {
   public:

    ClientSocket ( std::string host, int port );
    virtual ~ClientSocket(){};

    const ClientSocket& operator << ( const std::string& ) const;
    const ClientSocket& operator >> ( std::string& ) const;

  };

}

#endif
