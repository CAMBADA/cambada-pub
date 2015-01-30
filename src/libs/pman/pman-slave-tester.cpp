/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA PMAN
 *
 * CAMBADA PMAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA PMAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "pman.h"
#include "pmandefs.h"

#include "SharedTimer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>

#include <iostream>

using namespace std;
using namespace cambada::util;

bool running = true;

SharedTimer timer;

int main(int argc, char* argv[])
{
    assert(argc == 2);

    /* get name of PMAN process */
    char* pname = argv[1];

	// PMAN initializations (attach to existing process table)
	int pmanstat = -1;
	while (pmanstat < 0)
	{
		if((pmanstat = PMAN_init(SHMEM_OCAM_PMAN_KEY, SEM_OCAM_PMAN_KEY, NULL, 0, PMAN_ATTACH)) < 0)
		{ 
			fprintf(stderr,"PMAN slave: [%s]: PMAN_init (PMAN_ATTACH) failed (return code %d)\n",
                    pname, pmanstat); 
			PMAN_close(PMAN_CLFREE);
			fprintf(stderr,"PMAN slave: [%s]: PMAN_init (PMAN_ATTACH) waiting one sec\n", pname); 
			sleep(1);
		} 
	}

	// attach process to process table
	if ((pmanstat = PMAN_attach(pname, getpid())) < 0) 
	{ 
		fprintf(stderr, "PMAN slave : [%s]: PMAN_attach failed (return code %d)\n", 
                pname, pmanstat); 
		exit(EXIT_FAILURE);
	}

    /* install signal handler */
    sigset_t sigusrmask;
    sigemptyset( &sigusrmask );
	sigaddset( &sigusrmask , PMAN_ACTIVATE_SIG );

    struct sigaction sigact;
	sigact.sa_flags = 0;
    sigemptyset(&sigact.sa_mask);
    void signalHandler(int sigid);
    sigact.sa_handler = signalHandler;

	sigaction(PMAN_ACTIVATE_SIG, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
    
    /* main loop */
    while (running)
	{
        /* wait for signal */
	    pause();
        fprintf(stdout, "%s: after pause: %d\n", pname, timer.elapsed());

        /* do some work */
        for (int i = 0; i < 1000000; i++) { running = running+1-1; }
        fprintf(stdout, "%s: after busy waiting: %d\n", pname, timer.elapsed());
        assert(usleep(200 * 1000) == 0);

        /* release access */
        fprintf(stdout, "%s: before epilogue: %d\n", pname, timer.elapsed());
        PMAN_epilogue(pname);
	}
	
    /* clean before quit */
    /* to be done */

    /* that's all */
    return 0;
}

void signalHandler(int sigid)
{
    switch (sigid)
    {
        case SIGINT:
            running = false;
            return;
        default:
            return;
    }
}


