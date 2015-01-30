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

#include "MulticastSocket.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char* argv[])
{
	if (argc != 4)
	{
		fprintf(stderr, "USAGE: interface multicastReceiver multicast_ip port\n");
		exit(EXIT_FAILURE);
	}

	const char* interface = argv[1];
	const char* group = argv[2];
	short port = short(atoi(argv[3]));

	MulticastSocket sock;
	assert(sock.openSocket(interface, group, port) != -1);

	char buf[1501];
	int n;
	while (1)
	{
		//memset(buf, 'a', 1500);
		printf("Vou ler\n");
		n = sock.receiveData(buf, 1500);
		buf[n] = '\0';
		printf("«««««««««««««««\n");
		printf("%s\n", buf);
		printf("»»»»»»»»»»»»»»»\n");
		fflush(stdout);
	}

	sock.closeSocket();
}
