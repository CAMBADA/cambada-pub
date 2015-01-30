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

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>

#include <unistd.h>

#include <sys/time.h>
#include <sched.h>

#include <stdlib.h>

#include "multicast.h"

#include "rtdb_comm.h"

#include "MersenneTwister.h"


#define BUFFER_SIZE 1400

#define TTUP_US 100E3 // 10Hz -> 100E3, 20Hz -> 50E3
#define COMM_DELAY_MS 2
#define COMM_DELAY_US COMM_DELAY_MS*1E3
#define MIN_UPDATE_DELAY_US 1E3

// #define DEBUG
// #define FILEDEBUG
#define UNSYNC

#define PERRNO(txt) \
	printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

#define PERR(txt, par...) \
	printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)

#ifdef DEBUG
#define PDEBUG(txt, par...) \
	printf("DEBUG: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)
#else
#define PDEBUG(txt, par...)
#endif

#ifdef FILEDEBUG
	FILE *filedebug;
#define FDEBUG(txt, par...) \
  fprintf(filedebug, txt , ## par)
#else
#define FDEBUG(file, txt, par...)
#endif


#define NOT_RUNNING		0
#define RUNNING			1
#define INSERT			2
#define REMOVE			3
#define MAX_REMOVE_TICKS		10

#define NO	0
#define YES	1

int end;
int timer;

int MAX_DELTA;

struct timeval lastSendTimeStamp;
int delay;
int nosend;

#ifdef DEBUF
#endif

int lostPackets[MAX_AGENTS];

struct _record
{
	int id;			  // id
	int size;		  // data size
	int life;		  // life time
	void* pData;	// pointer to data
};

struct _frameHeader
{
	unsigned char number;			    // agent number
	unsigned int counter;			    // frame counter
	char stateTable[MAX_AGENTS];	// table with my vision of each agent state
	int noRecs;						        // number of records
};

struct _agent
{
	char state;							          // current state
	char dynamicID;					          // position in frame
	char received;						        // received from agent in the last Ttup?
	struct timeval receiveTimeStamp;	// last receive time stamp
	int delta;							          // delta
	unsigned int lastFrameCounter;		// frame number
	char stateTable[MAX_AGENTS];		  // vision of agents state
  int removeCounter;                // counter to move agent to not_running state
};


int myNumber;

struct _agent agent[MAX_AGENTS];

int RUNNING_AGENTS;


//	*************************
//  Signal catch
//
static void signal_catch(int sig)
{
  if (sig == SIGINT)
    end = 1;
  else
    if (sig == SIGALRM)
      timer++;
}



// RA-TDMA
int sync_ratdma(int agentNumber)
{
  int realDiff, expectedDiff;
  struct itimerval it;

  agent[agentNumber].received = YES;

  if ((agent[agentNumber].state == NOT_RUNNING) || (agent[agentNumber].state == INSERT))
  {
    PDEBUG("*****  agent %d - NOT_RUNNING or INSERT  *****", agentNumber);
    return (1);
  }

  // real difference with average medium comm delay
  realDiff = (int)((agent[agentNumber].receiveTimeStamp.tv_sec - lastSendTimeStamp.tv_sec)*1E6 + agent[agentNumber].receiveTimeStamp.tv_usec - lastSendTimeStamp.tv_usec);
  realDiff -= (int)COMM_DELAY_US; // travel time
  if (realDiff < 0)
  {
    PDEBUG("*****  realDiff to agent %d = %d  *****", agentNumber, realDiff);
    return (2);
  }

  // expected difference
  expectedDiff = (int)((agent[agentNumber].dynamicID - agent[myNumber].dynamicID) * TTUP_US / RUNNING_AGENTS);
  if (expectedDiff < 0)
    expectedDiff += (int)TTUP_US;
	
  agent[agentNumber].delta = realDiff - expectedDiff;

  // only dynamic agent 0 make adjustments
  if (agent[myNumber].dynamicID == 0)
  {
    if ((agent[agentNumber].delta > delay) && (agent[agentNumber].delta < MAX_DELTA))
    {
      // avoid small corrections
      if (agent[agentNumber].delta > (int)MIN_UPDATE_DELAY_US)
      {
        delay = agent[agentNumber].delta;
        PDEBUG("delay between %d(%d) and %d(%d) -> %d", myNumber, agent[myNumber].dynamicID, agentNumber, agent[agentNumber].dynamicID, delay);
      }
    }
  }
  else
  {
    // only sync from dynamic agent 0
    if (agent[agentNumber].dynamicID == 0)
    {
      expectedDiff = (int)(TTUP_US - expectedDiff);
      expectedDiff -= (int)COMM_DELAY_US; // travel time
      it.it_value.tv_usec = (long int)(expectedDiff % (int)1E6);
      it.it_value.tv_sec = (long int)(expectedDiff / (int)1E6);
      it.it_interval.tv_usec=(__suseconds_t)(TTUP_US);
      it.it_interval.tv_sec=0;
      setitimer (ITIMER_REAL, &it, NULL);
    }
  }
    
//	PDEBUG("Delta [%d] = %d\trealDiff = %d\texpectedDiff = %d\t delay = %d", agentNumber, agent[agentNumber].delta, realDiff, expectedDiff, delay);

//	for (i=0; i < agentNumber; i++)
//		FDEBUG(filedebug, "\t");

//  FDEBUG(filedebug, "%d\n", (int)(realDiff - COMM_DELAY));

//			FDEBUG(filedebug, "Received from %1d->%1d(%4u)-->delay=%7d realDiff=%7d expectedDiff=%7d\n", agent[agentNumber].inFramePos, agentNumber, frameHeader.counter, agent[agentNumber].delta, realDiff, expectedDiff);

  return (0);
}




void update_stateTable(void)
{
  int i, j;

  for (i=0; i<MAX_AGENTS; i++)
  {
    if ( i != myNumber)
    {
      switch (agent[i].state)
      {
        case RUNNING:
          if (agent[i].received == NO)
            agent[i].state = REMOVE;
          break;
        case NOT_RUNNING:
          if (agent[i].received == YES)
            agent[i].state = INSERT;
          break;
				case INSERT:
					if (agent[i].received == NO)
						agent[i].state = NOT_RUNNING;
					else
					{
						for (j = 0; j < MAX_AGENTS; j++)
							if ((agent[j].state == RUNNING) &&
									((agent[j].stateTable[i] == NOT_RUNNING) || (agent[j].stateTable[i] == REMOVE)))
								break;
						agent[i].state = RUNNING;
					}
					break;
				case REMOVE:
					if (agent[i].received == YES)
          {
            agent[i].removeCounter = 0;
						agent[i].state = RUNNING;
          }
					else
					{
						for (j = 0; j < MAX_AGENTS; j++)
							if ((agent[j].state == RUNNING) &&
									((agent[j].stateTable[i] == RUNNING) || (agent[j].stateTable[i] == INSERT)))
								break;
             agent[i].removeCounter ++;
             if (agent[i].removeCounter >= MAX_REMOVE_TICKS)
             {
               agent[i].state = NOT_RUNNING;
               agent[i].removeCounter = 0;
             }
					}
					break;
			}
		}
  }

  // my state
	agent[myNumber].state = RUNNING;
}



// *************************
//  Receive Thread
//
//  Input:
//    int *sckt = pointer of socket descriptor
//
void *receiveDataThread(void *arg)
{
  int recvLen;
  char recvBuffer[BUFFER_SIZE];
  int indexBuffer;
  int agentNumber;
  int i;
	RTDBconf_var rec;
	int life;
	struct _frameHeader frameHeader;

	int size;
	

	while(!end)
	{
		bzero(recvBuffer, BUFFER_SIZE);
		indexBuffer = 0;

		if((recvLen = receiveData(*(int*)arg, recvBuffer, BUFFER_SIZE)) > 0 )
		{

			memcpy (&frameHeader, recvBuffer + indexBuffer, sizeof(frameHeader));
			indexBuffer += sizeof(frameHeader);

			agentNumber = frameHeader.number;

      gettimeofday(&(agent[agentNumber].receiveTimeStamp), NULL);

      // receive from ourself
      // not supposed to occur. just to prevent!
  		if ((agentNumber == myNumber) && (nosend == 0))
				continue;

			// TODO
      // correction when frameCounter overflows
			if ((agent[agentNumber].lastFrameCounter + 1) != frameHeader.counter)
				lostPackets[agentNumber] = frameHeader.counter - (agent[agentNumber].lastFrameCounter + 1);
			agent[agentNumber].lastFrameCounter = frameHeader.counter;

      // state team view from received agent
			for (i = 0; i < MAX_AGENTS; i++)
				agent[agentNumber].stateTable[i] = frameHeader.stateTable[i];

			for(i = 0; i < frameHeader.noRecs; i++)
			{
				// id
				memcpy (&rec.id, recvBuffer + indexBuffer, sizeof(rec.id));
				indexBuffer += sizeof(rec.id);

				// size
				memcpy (&rec.size, recvBuffer + indexBuffer, sizeof(rec.size));
				indexBuffer += sizeof(rec.size);

				// life
				memcpy (&life, recvBuffer + indexBuffer, sizeof(life));
				indexBuffer += sizeof(life);

        life += COMM_DELAY_MS;

				// data
				if((size = DB_comm_put (agentNumber, rec.id, rec.size, recvBuffer + indexBuffer, life)) != (int)rec.size)
				{
					PERR("Error in frame/rtdb: from = %d, item = %d, received size = %d, local size = %d", agentNumber, rec.id, rec.size, size);
					break;
				}
				PDEBUG("Receive from %d\n", agentNumber);

				indexBuffer += rec.size;
			}

#ifndef UNSYNC
			sync_ratdma(agentNumber);
#endif

		}
	}

	return NULL;
}


void printUsage(void)
{
	printf("Usage: comm <interface_name> [nosend]\n\n");
	printf("<interface_name> - eth0, wlan0, other\n");
	printf("[nosend] - only receives data\n\n");
}


//*************************
//  Main
//
int main(int argc, char *argv[])
{
	int sckt;
	pthread_t recvThread;
	char sendBuffer[BUFFER_SIZE];
	int indexBuffer;
	int sharedRecs;
	RTDBconf_var rec[MAX_RECS];
	unsigned int frameCounter = 0;
	int i, j;
	int life;

	struct sched_param proc_sched;
	pthread_attr_t thread_attr;

	struct itimerval it;
	struct _frameHeader frameHeader;

	struct timeval tempTimeStamp;

  nosend = 0;
	if ((argc < 2) || (argc > 3))
	{
		printUsage();
		return (-1);
	}
	if (argc == 3)
	{
		if(strcmp(argv[2], "nosend") == 0)
		{
			printf("\n*** Running in listing only mode ***\n\n");
			nosend = 1;
		}
		else
		{
			printUsage();
			return (-1);
		}
	}

	/* initializations */
	delay = 0;
	timer = 0;
	end = 0;
	RUNNING_AGENTS = 1;

	/* Assign a real-time priority to process */
	proc_sched.sched_priority=60;
	if ((sched_setscheduler(getpid(), SCHED_FIFO, &proc_sched)) < 0)
	{
		PERRNO("setscheduler");
		return -1;
	}

	if(signal(SIGALRM, signal_catch) == SIG_ERR)
	{
		PERRNO("signal");
		return -1;
	}

	if(signal(SIGINT, signal_catch) == SIG_ERR)
	{
		PERRNO("signal");
		return -1;
	}

	if((sckt = openSocket(argv[1])) == -1)
	{
		PERR("openMulticastSocket");
		printf("\nUsage: comm <interface_name>\n\n");
		return -1;
	}

	if(DB_init() == -1)
	{
		PERR("DB_init");
		closeSocket(sckt);
		return -1;
	}
	
	if((sharedRecs = DB_comm_ini(rec)) < 1)
	{
		PERR("DB_comm_ini");
		DB_free();
		closeSocket(sckt);
		return -1;
	}

#ifdef FILEDEBUG
	if ((filedebug = fopen("log.txt", "w")) == NULL)
	{
		PERRNO("fopen");
		DB_free();
		closeSocket(sckt);
		return -1;
	}
#endif

	/* initializations */
	for (i=0; i<MAX_AGENTS; i++)
	{
		lostPackets[i]=0;
		agent[i].lastFrameCounter = 0;
		agent[i].state = NOT_RUNNING;
		agent[i].removeCounter = 0;
	}
	myNumber = Whoami();
	agent[myNumber].state = RUNNING;

	/* receive thread */
	pthread_attr_init (&thread_attr);
	pthread_attr_setinheritsched (&thread_attr, PTHREAD_INHERIT_SCHED);
	if ((pthread_create(&recvThread, &thread_attr, receiveDataThread, (void *)&sckt)) != 0)
	{
		PERRNO("pthread_create");
		DB_free();
		closeSocket(sckt);
		return -1;
	}

	/* Set itimer to reactivate the program */
	it.it_value.tv_usec=(__suseconds_t)(TTUP_US);
	it.it_value.tv_sec=0;
	it.it_interval.tv_usec=(__suseconds_t)(TTUP_US);
	it.it_interval.tv_sec=0;
	setitimer (ITIMER_REAL, &it, NULL);

	printf("communication: STARTED in ");
#ifdef UNSYNC
	printf("unsync mode...\n");
#else
	printf("sync mode...\n");
#endif 


	MTRand randomGenerator;
	while (!end)
	{
		//pause();
		double waitTime = randomGenerator.randNorm(0,1) * TTUP_US * 0.05 + TTUP_US;
		usleep(waitTime);

		// not timer event
		if (timer == 0)
			continue;

#ifndef UNSYNC
		// dynamic agent 0
		if ((delay > (int)MIN_UPDATE_DELAY_US) && (agent[myNumber].dynamicID == 0) && timer == 1)
		{
			it.it_value.tv_usec = (__suseconds_t)(delay - (int)MIN_UPDATE_DELAY_US/2);
			it.it_value.tv_sec = 0;
			setitimer (ITIMER_REAL, &it, NULL);
			delay = 0;
			continue;
		}
#endif

		timer = 0;

		indexBuffer = 0;
		bzero(sendBuffer, BUFFER_SIZE);

		update_stateTable();

		// update dynamicID
		j = 0;	
		for (i = 0; i < MAX_AGENTS; i++)
		{
			if ((agent[i].state == RUNNING) || (agent[i].state == REMOVE))
			{
				agent[i].dynamicID = j;
				j++;
			}
			agent[myNumber].stateTable[i] = agent[i].state;
		}
		RUNNING_AGENTS = j;

		MAX_DELTA = (int)(TTUP_US/RUNNING_AGENTS * 2/3);

		// frame header
		frameHeader.number = myNumber;
		frameHeader.counter = frameCounter;
		frameCounter ++;
		for (i = 0; i < MAX_AGENTS; i++)
			frameHeader.stateTable[i] = agent[myNumber].stateTable[i];
		frameHeader.noRecs = sharedRecs;
		memcpy(sendBuffer + indexBuffer, &frameHeader, sizeof(frameHeader));
		indexBuffer += sizeof(frameHeader);

		for(i = 0; i < sharedRecs; i++)
		{
			// id
			memcpy(sendBuffer + indexBuffer, &rec[i].id, sizeof(rec[i].id));
			indexBuffer += sizeof(rec[i].id);

			// size
			memcpy(sendBuffer + indexBuffer, &rec[i].size, sizeof(rec[i].size));
			indexBuffer += sizeof(rec[i].size);

			// life and data
			life = DB_get(myNumber, rec[i].id, sendBuffer + indexBuffer + sizeof(life));
			memcpy(sendBuffer + indexBuffer, &life, sizeof(life));
			indexBuffer = indexBuffer + sizeof(life) + rec[i].size;
		}

		if (indexBuffer > BUFFER_SIZE)
		{
			PERR("Pretended frame is bigger that the available buffer.");
			PERR("Please increase the buffer size or reduce the number of disseminated records");
			break;
		}
	
		if (nosend == 0) 
		{
			if (sendData(sckt, sendBuffer, indexBuffer) != indexBuffer)
				PERRNO("Error sending data");
		}

		gettimeofday (&tempTimeStamp, NULL);
		lastSendTimeStamp.tv_sec = tempTimeStamp.tv_sec;
		lastSendTimeStamp.tv_usec = tempTimeStamp.tv_usec;

		// reset values for next round
		for (i=0; i<MAX_AGENTS; i++)
		{
			agent[i].delta = 0;
			agent[i].received = NO;
		}
	}

	FDEBUG (filedebug, "\nLost Packets:\n");
	for (i=0; i<MAX_AGENTS; i++)
		FDEBUG (filedebug, "%d\t", lostPackets[i]);
	FDEBUG (filedebug, "\n");
	
	printf("communication: STOPED.\nCleaning process...\n");

#ifdef FILEDEBUG
	fclose (filedebug);
#endif

	closeSocket(sckt);

	pthread_join(recvThread, NULL);

	DB_free();

	printf("communication: FINISHED.\n");

	return 0;
}
