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
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <assert.h>

using namespace std;
using namespace cambada::util;

bool fPman = false; // cleaning flag

bool running = true;

SharedTimer timer;

#define DEBUG 1

void Shutdown()
{
	if( fPman ) 
    {
		cout << "Cleaning PMAN" << endl;
		PMAN_close(PMAN_CLFREE);
	}
}

int main(int argc, char *argv[])
{
    /* process command line arguments */
    if (argc != 2)
    {
        fprintf(stderr, "USAGE: %s config-file\n", argv[0]);
        exit(EXIT_FAILURE);
    }

	/* PMAN master initialization */
	int pmanstat;
	if((pmanstat = PMAN_init(SHMEM_OCAM_PMAN_KEY, SEM_OCAM_PMAN_KEY,
	        (void *)linux_sched_fifo, sizeof(int), PMAN_NEW)))
    {
		fprintf( stderr, "ERROR: PMAN_init failed (return code %d)\n", 
                pmanstat );
		Shutdown();
	}

    PMAN_print();

    /* ???? */
	fPman = true; // set PMAN cleaning flag

    /* open PMAN configuration file */
    FILE *fpPman = NULL;
    char *pmanFileName = argv[1];
	if(!(fpPman = fopen(pmanFileName, "r"))) 
    {
		fprintf( stderr, "ERROR: couldn't open configuration file \"%s\" from PMAN\n",
                pmanFileName );
		Shutdown();
	}

    /* load process table */
    char pname[32];
    int pper, ppha, pddln, pprio;
    int nv;
    while ((nv = fscanf(fpPman,"%s %d %d %d %d", pname, &pper, &ppha, &pddln, &pprio)) == 5)
    {
        PMAN_procadd(pname, PMAN_NOPID, pper, ppha, pddln, &pprio, sizeof(pprio));
        #ifdef DEBUG
        fprintf(stderr, "\n[PMAN master]: process %s (Period = %d, Phase = %d"
                "Deadline = %d, Priority = %d) added\n", 
                pname, pper, ppha, pddln, pprio);
        #endif
    }
    fclose(fpPman);

    PMAN_print();

    /* define precedences */
    PMAN_prec_add((char*)"slave1", (char*)"slave2");

    /* install SIGINT Handler */
    void sigHandler(int sigId);
    struct sigaction sa;
    sa.sa_handler = sigHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    assert(sigaction(SIGINT, &sa, NULL) != -1);

    PMAN_print();

    sleep(2);

    /* main cycle */
    while(running)
    {
        /* do some work */
        fprintf(stdout, "Master: start of cycle: %d\n", timer.elapsed());
        usleep(1000 * 1000);

        /* send tick to other processes */
        fprintf(stdout, "Master: before PMAN_tick: %d\n", timer.elapsed());
        PMAN_tick();

        /* do some more work */
        for (int i = 0; i < 1000000; i++) { running = running+1-1; }
        fprintf(stdout, "Master: after busy waiting: %d\n", timer.elapsed());
        usleep(1000 * 1000);
        fprintf(stdout, "Master: end of cycle: %d\n", timer.elapsed());
    }

    /* clean before quit */
    /* to be done after install SIGINT signal handler */

    /* that's all */
    return 0;
}

void sigHandler()
{
    running = false;
}
