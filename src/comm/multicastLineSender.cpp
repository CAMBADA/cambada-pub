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
#include <unistd.h>

int main(int argc, char* argv[])
{
	if (argc != 5)
	{
		fprintf(stderr, "USAGE: multicastReceiver interface multicast-ip port file-to-send\n");
		exit(EXIT_FAILURE);
	}

	const char* interface = argv[1];
	const char* ip = argv[2];
	short port = short(atoi(argv[3]));
	const char* fname = argv[4];

	/* open communication socket */
	MulticastSocket sock;
	assert(sock.openSocket(interface, ip, port) != -1);

	/* open file to be sent */
	FILE *fin;
	assert((fin = fopen(fname, "r")) != NULL);

	char buf[1500];
	while (fgets(buf, 1500, fin) != NULL)
	{
		printf("«««««\n");
		printf("%s", buf);
		printf("»»»»»\n");
		sock.sendData(buf, strlen(buf));
		usleep(500 * 1000);
	}

	fclose(fin);
	sock.closeSocket();
}
