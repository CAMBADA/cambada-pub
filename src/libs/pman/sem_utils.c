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

#include <sys/types.h>
#include <sys/sem.h>

#include <sem_utils.h>

/*
 * Semaphore wait/signal primitives 
 */
int sem_pwait(int sem_id)
{
  struct sembuf sem_b;
  
  sem_b.sem_num=0;
  sem_b.sem_op=-1;
  sem_b.sem_flg=SEM_UNDO;
  return semop(sem_id,&sem_b,1);
}

int sem_psignal(int sem_id)
{
  struct sembuf sem_b;
  
  sem_b.sem_num=0;
  sem_b.sem_op=1;
  sem_b.sem_flg=SEM_UNDO;
  return semop(sem_id,&sem_b,1);
}
