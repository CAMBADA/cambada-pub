/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA COMM
 *
 * CAMBADA COMM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA COMM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _CAMBADA_MULTICAST_COMM_
#define _CAMBADA_MULTICAST_COMM_

#define DFL_MULTICAST_PORT 2000
#define DFL_MULTICAST_GROUP "224.16.32.200"

#define RECEIVE_OUR_DATA 1

#include <arpa/inet.h>

class MulticastSocket
{
public:
	MulticastSocket();
	~MulticastSocket();

	int openSocket(const char* interface, const char* group = DFL_MULTICAST_GROUP, short port = DFL_MULTICAST_PORT);
	int closeSocket();
	int sendData(void* data, int dataSize);
	int receiveData(void* buffer, int bufferSize);

	inline int getSocket() 
	{
		return multiSocket;
	}

private:
	const char* multicast_group;
	short multicast_port;

	int multiSocket;

	struct sockaddr_in destAddress;
	
	int if_NameToIndex(const char *ifname, char *address);
};

#endif
