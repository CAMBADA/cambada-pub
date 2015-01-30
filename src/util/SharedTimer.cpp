/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA UTILITIES
 *
 * CAMBADA UTILITIES is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA UTILITIES is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SharedTimer.h"

#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <errno.h>

namespace cambada
{
namespace util
{

/**
 *      Se os processos morrem e o semáforo fica criado e a zero
 *      o sistema bloqueia. É preciso, neste momento eliminar o
 *      semáforo 0x777777.
 *      É preciso resolver este problema.
 */
SharedTimer::SharedTimer()
{
    /* try create the semaphore */
    assert((semid = semget(KEY, 1, 0666 | IPC_CREAT | IPC_EXCL)) != -1  ||  errno == EEXIST);

    if (semid != -1) // I'm the creator
    {
        fprintf(stderr, "I'm the creator\n");

        /* create the shared memory */
        assert((shmid = shmget(KEY, sizeof(struct shmData), 0666 | IPC_CREAT)) != -1);

        /* attach shared memory */
        assert((data = (shmData*)shmat(shmid, 0, 0)) != NULL);

        /* init shared memory */
        data->cnt = 1; // I'm the first process
        gettimeofday(&data->baseTime, NULL);
    }

    else // I'm not the creator
    {
        fprintf(stderr, "I'm not the creator\n");

        /* get semaphore id */
        assert((semid = semget(KEY, 0, 0)) != -1);

        /* gain mutual exclusion access */
        adquireAcess();

        /* get shared memory id */
        assert((shmid = shmget(KEY, 0, 0)) != -1);

        /* attach shared memory */
        assert((data = (shmData*)shmat(shmid, 0, 0)) != NULL);

        /* inc process counter */
        data->cnt++;
    }

    /* init local data */
    gettimeofday(&savedTime, NULL);

    /* print shared data */
    fprintf(stderr, "cnt = %d\n", data->cnt);

    /* release access and quit */
    releaseAccess();
}

SharedTimer::~SharedTimer()
{
    /* gain exclusive access */
    adquireAcess();

    /* decrement process count and save value for future use */
    data->cnt--;
    int cnt = data->cnt;

    /* detach memory */
    shmdt(data);

    /* mark memory to be destroyed */
    shmctl(shmid, IPC_RMID, NULL);

    /* destroy semaphore if last process */
    if (cnt == 0)
    {
        fprintf(stderr, "SharedTimer: I'm destroying the sempahore\n");
        semctl(semid, 0, IPC_RMID);
    }

    /* otherwise release access */
    else
    {
        releaseAccess();
    }
}

void SharedTimer::resetTime()
{
    /* gain exclusive access */
    adquireAcess();

    /* init times */
    gettimeofday(&data->baseTime, NULL);
    savedTime = data->baseTime;

    /* release exclusive access */
    releaseAccess();
}

void SharedTimer::saveCurrentTime()
{
    gettimeofday(&savedTime, NULL);
}

int SharedTimer::getLongTermTime()
{
    /* gain exclusive access */
    adquireAcess();

    /* get time */
    struct timeval t;
    gettimeofday(&t, NULL);
    int ret= (t.tv_sec - data->baseTime.tv_sec)*1000 + 
            rint((t.tv_usec - data->baseTime.tv_usec)/1000.0);

    /* release exclusive access */
    releaseAccess();

    /* return time */
    return ret;
}

int SharedTimer::getShortTermTime()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec - savedTime.tv_sec)*1000 + rint((t.tv_usec - savedTime.tv_usec)/1000.0);
}

void SharedTimer::adquireAcess()
{
    struct sembuf sb = { 0, -1, 0 };
    assert(semop(semid, &sb, 1) != -1);
}

void SharedTimer::releaseAccess()
{
    struct sembuf sb = { 0, +1, 0 };
    assert(semop(semid, &sb, 1) != -1);
}

}
}
