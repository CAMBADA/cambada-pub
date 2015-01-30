/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA AGENT
 *
 * CAMBADA AGENT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA AGENT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#define USE_PMAN 1

#include "Robot.h"

#include <signal.h>
#include <sys/types.h>
#include <pman.h>
#include "Cambada.h"
#include "pmandefs.h"
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include "rtdb.h"

using namespace std;
using namespace cambada;

Cambada* agent = NULL;
bool EXIT = false;
bool		WAIT	= true;
char		pname[64]	= "agent";

sigset_t configControlLoopSignals(void);
void controlLoop(int sig );

int main( int argc , char* argv[] )
{
	openlog("agentLog",LOG_NDELAY|LOG_CONS,LOG_USER);

	if( DB_init() == -1 ) {
		CMD_Vel_SET(0.0,0.0,0.0,false);
		CMD_Grabber_SET(0);
		fprintf(stderr,"ERROR: main: DB_INIT failed\n");
		exit(EXIT_FAILURE);
	}

#if USE_PMAN
	strcat(pname, getenv("AGENT"));
	int pmanstat = -1;

	sigset_t sig7mask = configControlLoopSignals();

	while(pmanstat < 0 && !EXIT )
	{
		// PMAN initializations (attach to existing process table)
		if((pmanstat = PMAN_init(SHMEM_OCAM_PMAN_KEY, SEM_OCAM_PMAN_KEY, NULL, 0, PMAN_ATTACH)))
		{
			CMD_Vel_SET(0.0,0.0,0.0,false);
			CMD_Grabber_SET(0);
			fprintf(stderr,"cambada_agent : [%s]: PMAN_init (PMAN_ATTACH) failed (return code %d)\n",pname, pmanstat);
			PMAN_close(PMAN_CLFREE);
			fprintf(stderr,"cambada_agent : [%s]: PMAN_init (PMAN_ATTACH) wait one sec\n",pname);
			sleep(1);
			if( EXIT == true )
			{
				cerr << "cambada_agent : Closing RtDB  " << endl;
				DB_free();
   				cerr << "cambada_agent : Finished :)" << endl;
				return EXIT_FAILURE;
			}
		}
	}


	// Fill PID in process table
	pmanstat=PMAN_attach(pname,getpid());

	if(pmanstat)
	{
		CMD_Vel_SET(0.0,0.0,0.0,false);
		CMD_Grabber_SET(0);
		fprintf(stderr, "cambada_agent : [%s]: PMAN_attach failed (return code %d)\n",pname,pmanstat);
		exit(EXIT_FAILURE);
	}
#endif

	if( !EXIT )
	{
		agent = new Cambada();

		if( !agent->parseArguments( argc , argv ) ){
			cerr << "cambada_agent : error parsing arguments" << endl;
			EXIT = true;
		}else{
			cerr << "cambada_agent : starting agent" << endl;
		}
	}

	WAIT = false;
	while( !EXIT )
	{
	   sigsuspend(&sig7mask);
	}

	CMD_Vel_SET(0.0,0.0,0.0,false);
	CMD_Grabber_SET(0);

#if USE_PMAN
	// Release PMAN resources
	PMAN_close(PMAN_CLLEAVE);
#endif

	sigset_t sigusrmask;
	sigemptyset(&sigusrmask);
	sigaddset(&sigusrmask, PMAN_ACTIVATE_SIG);
	sigaddset(&sigusrmask, SIGINT);
	sigaddset(&sigusrmask, SIGTERM);
	sigaddset(&sigusrmask, SIGHUP);
	sigprocmask(SIG_BLOCK, &sigusrmask, NULL);

	delete agent;

	DB_free();

	exit(EXIT_SUCCESS);

	return 0;
}

void controlLoop(int sig )
{

	if( sig == SIGINT ) {
		EXIT = true;
	} else if( sig == SIGTERM ) {
		EXIT = true;
	}

	if( WAIT )
		return;

	if( sig == PMAN_ACTIVATE_SIG )
	{
		agent->thinkAndAct();

#if USE_PMAN
		PMAN_epilogue(pname);
#endif
	}
	else
	if( sig == SIGHUP )
	{
		if( agent->reconfigure() )
			EXIT = false;
	}
	else
	{
		EXIT = true;
	}

}


sigset_t configControlLoopSignals(void)
{
	sigset_t sigusrmask, sigemptymask;

	// Install handler
	sigemptyset( &sigusrmask );
	sigaddset( &sigusrmask , PMAN_ACTIVATE_SIG );
	sigaddset( &sigusrmask , SIGINT );
	sigaddset( &sigusrmask , SIGTERM );
	sigaddset( &sigusrmask , SIGHUP );
	sigemptyset( &sigemptymask );

	struct sigaction sigact;
	sigact.sa_flags = 0;
	sigact.sa_mask = sigemptymask;
	sigact.sa_handler = (void (*)(int))controlLoop;

	sigaction(PMAN_ACTIVATE_SIG, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGHUP, &sigact, NULL);

	sigprocmask(SIG_UNBLOCK, &sigusrmask, NULL);

	// Get's the currently unblocked signals
	sigset_t sig7mask;
	sigprocmask(SIG_BLOCK, NULL, &sig7mask);

	return sig7mask;
}

