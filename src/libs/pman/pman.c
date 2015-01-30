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

#include <stdlib.h>
#include <stdio.h> 
#include <string.h>

// For POSIX support flags
#include <unistd.h>

#include <time.h>

#include <sys/types.h>
#include <sys/time.h>

#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/ipc.h>

#include <signal.h>
#include <sched.h>

#include <errno.h>

#include <sem_utils.h>
#include <pman.h>

#include <assert.h>

//#define DEBUG_PMAN

#ifdef DEBUG_PMAN
#define PMAN_DBG(string,args...) printf(string, ##args)
#else
#define PMAN_DBG(string, args...)
#endif

/* 
 * Global vars
 */
int pman_sem_id;                // Semaphore ID (mutual exclusion)
int pman_shmem_id;              // Shared memory ID
PROC_TABLE_TYPE * p_table; // Process table

int pamn_sem_id_LUT[ MAX_MASTER_INSTANCE ];                 // sempahore id look up table
int pamn_shmem_id_LUT[ MAX_MASTER_INSTANCE ];               // shared memory id look up table
PROC_TABLE_TYPE* pman_p_table_LUT[ MAX_MASTER_INSTANCE ];   // proc table address look up table


/*
 * Initializes the process table
 *
 *   Input args: (global var) *p_table : pointer to the process table data structure
 *               shmem_pman_key      : shared memory key
 *               sem_pman_key        : sempahore key (mutual exclusion)
 *               QoSfun                : pointer to QoS manager function (only "manager" process)
 *               QoSdata_sz            : size of QoS data structure
 *               create_flags          : PMAN_NEW    - Create a new process table 
 *                                       PMAN_ATTACH - Attach to an existing process table
 *
 *   Returns:    0 : Success
 *              -1 : Error creating shared memory region 
 *              -2 : Failed to create semaphore 
 *              -3 : Error setting the priority
 *              -4 : Error allocating memory (QoS data)
 */
int PMAN_init(key_t shmem_pman_key, key_t sem_pman_key, void * QoSfun, int QoSdata_sz, int create_flags)
{
	char* agent;
	assert((agent = getenv("AGENT")) != NULL);
	int agent_no = atoi(agent);
	return PMAN_init2(shmem_pman_key + 2*agent_no, sem_pman_key + 2*agent_no, QoSfun,QoSdata_sz, create_flags);
}

int PMAN_init2(key_t shmem_pman_key, key_t sem_pman_key, void * QoSfun, int QoSdata_sz, int create_flags)
{
	int i,j,sstat;
	union dsemun sem_union;
	struct sched_param proc_sched;
	struct timeval tv;

	PMAN_DBG("\n [PMAN_init]: shmem_pman_key %x / sem_pman_key %x, QoSfun:%p, QoSdata_sz:%d, create_flags:%d",
			shmem_pman_key, sem_pman_key, QoSfun, QoSdata_sz, create_flags);

	switch (create_flags)
	{
	case PMAN_NEW:

		/* Get shared memory region */
		pman_shmem_id = shmget((key_t) shmem_pman_key, sizeof(PROC_TABLE_TYPE)+PROC_TABLE_SIZE*QoSdata_sz, 0666 | IPC_CREAT);
		if(pman_shmem_id == -1){
			fprintf(stderr, "\n [PMAN_init (PMAN_NEW)]: PMAN shmget failed");
			return -11;
		}

		p_table=(PROC_TABLE_TYPE *)shmat(pman_shmem_id, (void *)0, 0);

		if(p_table == (void *)-1){
			fprintf(stderr, "\n [PMAN_init (PMAN_NEW)]: PMAN shmat failed");
			return -12;
		}

		//   printf("\n sizeof: PROC_TABLE_TYPE:%d / int: %d / long:%d", sizeof(PROC_TABLE_TYPE),sizeof(int), sizeof(long));

		PMAN_DBG("\n [PMAN_init (PMAN_NEW)]: PMAN shared memory attached at [%p,%x] (%d bytes)\n",
				p_table,(void*)p_table+sizeof(PROC_TABLE_TYPE)+PROC_TABLE_SIZE*QoSdata_sz,sizeof(PROC_TABLE_TYPE)+PROC_TABLE_SIZE*QoSdata_sz);
		PMAN_DBG("\n [PMAN_init (PMAN_NEW)]: PMAN process data [%p,%x]",p_table,(void*)p_table+sizeof(PROC_TABLE_TYPE)-1);
		PMAN_DBG("\n [PMAN_init (PMAN_NEW)]: QoS process data [%x,%x]",
				(void*)p_table+sizeof(PROC_TABLE_TYPE),(void*)p_table+sizeof(PROC_TABLE_TYPE)+PROC_TABLE_SIZE*QoSdata_sz-1);


		/* Init process table */
		p_table->nprocs = 0;
		p_table->ticks = 0;
		p_table->QoSupd = QoSfun;
		p_table->DdlnExcpt = NULL;

		for(i=0;i<PROC_TABLE_SIZE;i++)
		{
			p_table->proc[i].PROC_name[0]=0;
			p_table->proc[i].PROC_id = PMAN_NOPID;
			p_table->proc[i].PROC_period = 0;
			p_table->proc[i].PROC_phase = 0;
			p_table->proc[i].PROC_deadline = 0;

			for(j=0;j<PMAN_MAX_PRED;j++)
				p_table->proc[i].PROC_pred_name[j][0]=0;
			for(j=0;j<PMAN_MAX_SUCC;j++)
				p_table->proc[i].PROC_succ_index[j]=PMAN_NOINDEX;
			p_table->proc[i].PROC_pred_mask=0;
			p_table->proc[i].PROC_pred_met=0;

			p_table->proc[i].PROC_qosdata= sizeof(PROC_TABLE_TYPE)+i*QoSdata_sz;

			p_table->proc[i].PROC_qosupdflag=0;

			tv.tv_sec=tv.tv_usec =0;
			p_table->proc[i].PROC_last_start = tv;
			p_table->proc[i].PROC_last_finish = tv;

			p_table->proc[i].PROC_status = PROC_S_EMPTY;
			p_table->proc[i].PROC_nact = 0;
			p_table->proc[i].PROC_ndm = 0;

#ifdef PMAN_TRACE
			p_table->evt_firstindex = 0;
			p_table->evt_lastindex = 0;
#endif
		}


		/* Create and init semaphore */
		pman_sem_id = semget((key_t)sem_pman_key,1,0666 | IPC_CREAT);
		if(pman_sem_id == -1)
			return -13;

		PMAN_DBG("\n     [PMAN_init CREATE flag]: semaphore id: %d",pman_sem_id);

		sem_union.val=1;
		sstat=semctl(pman_sem_id,0,SETVAL,sem_union);
		if(sstat == -1)
			return -14;

#if _POSIX_PRIORITY_SCHEDULING > 0
		/* Set the process priority and scheduling policy */
		proc_sched.sched_priority=sched_get_priority_max(SCHED_FIFO);
		if( sched_setscheduler(0,SCHED_FIFO,&proc_sched) == -1)
		{
			fprintf(stderr,"\n [PMAN_init (PMAN_NEW)]: Error setting the priority (sched_setscheduler)!");
			return -15;
		}
#endif

		break;


	case PMAN_ATTACH:

		/* Create shared memory region */
		pman_shmem_id = shmget((key_t) shmem_pman_key, sizeof(PROC_TABLE_TYPE), 0666 | IPC_CREAT);
		if(pman_shmem_id == -1){
			fprintf(stderr, "\n [PMAN_init (PMAN_ATTACH)]: PMAN shmget failed");
			return -16;
		}

		p_table=(PROC_TABLE_TYPE *)shmat(pman_shmem_id, (void *)0, 0);
		if(p_table == (void *)-1){
			fprintf(stderr, "\n [PMAN_init (PMAN_ATTACH)]: PMAN shmat failed");
			return -17;
		}

		PMAN_DBG("\n [PMAN_init (PMAN_ATTACH)]: PMAN shared memory attached at [%p,%x] (%d bytes)\n", p_table,(void*)p_table+sizeof(PROC_TABLE_TYPE),sizeof(PROC_TABLE_TYPE));

		/* Get semaphore ID*/
		pman_sem_id = semget((key_t)sem_pman_key,1,0666);
		if(pman_sem_id == -1)
			return -18;

		PMAN_DBG("\n     [PMAN_init ATTACH flag]: semaphore id: %d",pman_sem_id);

		break;
	}



	return 0;
}


/*
 * Releases resources allocated by PCSHED lib 
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *             close_flags           : PMAN_CLFREE  - Completely removes all resources (shared mem,
 *                                                      semaphore) and kill all registered "client" processes 
 *                                   : PMAN_CLLEAVE - Just deattaches shared mem region 
 *              
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 *             -2 : error removing shared memory
 *             -3 : error deleting semaphore
 *             -4 : invalid close flag
 */
int PMAN_close(int close_flags)
{
	int i;
	int sstat;
	union dsemun sem_union;

	PMAN_DBG("\n PMAN_close called (p_table:%p)", p_table);

	if(p_table == NULL)
		return -111;

	switch(close_flags)
	{
	case PMAN_CLFREE:

		/* Wait for any ongoing op */
		sem_pwait(pman_sem_id);

		/* Kills every registered processes */
		for(i=0;i<PROC_TABLE_SIZE;i++)
			if(p_table->proc[i].PROC_id != PMAN_NOPID)
			{
				sstat=kill(p_table->proc[i].PROC_id, SIGINT);
				//printf(" \n INT signal sent to process %s ,pid=%d, returned %d",p_table->proc[i].PROC_name, p_table->proc[i].PROC_id, sstat);
			}

		/* Deletes shared mem */
		if(shmdt((void *)p_table) == -1) {
			fprintf(stderr,"[PMAN_close (PMAN_CLFREE)]: shmdt ptable failed!\n");
			return -112;
		}
		if(shmctl(pman_shmem_id, IPC_RMID,0) == -1) {
			fprintf(stderr,"[PMAN_close (PMAN_CLFREE)]: pman shmctl(IPC_RMID) failed!\n");
			return -113;
		}

		/* Deletes semaphore */
		sstat=semctl(pman_sem_id,0,IPC_RMID,sem_union);
		if(sstat == -1) {
			fprintf(stderr,"[PMAN_close (PMAN_CLFREE)]: semctl (IPC_RMID) failed!\n");
			return -114;
		}

		return 0;
		break;


	case PMAN_CLLEAVE:
		/* Deattaches shared mem */
		if(shmdt((void *)p_table) == -1) {
			fprintf(stderr,"[PMAN_close (PMAN_CLLEAVE)]: shmdt ptable failed!\n");
			return -115;
		}
		break;
	}

	return -116;
}


/*
 * Adds a process to the process table
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *              p_name                : process name (string)
 *              p_id                  : process id (OS PID)
 *              p_period              : process period (ticks)
 *              p_phase               : process initial phase (ticks)
 *              p_deadline            : process deadline (ticks)
 *              p_QoSdata             : pointer to process QoS data
 *              p_QoSdata_sz          : size of QoS data sttructure
 *
 * Returns:      0 : success
 *              -1 : invalid process table pointer (null) 
 *              -2 : process table full
 *              -3 : duplicated process id
 */
int PMAN_procadd(char * p_name, int p_id,int p_period, int p_phase, int p_deadline, void * p_QoSdata, int p_QoSdata_sz)
{
	int i;
	int free_index;
	//unused: char dup_flag;

	PMAN_DBG("\n PMAN_procadd called (ptable:%p, p_name:%s, p_id:%d, p_period:%d, p_phase:%d,p_deadline:%d, \
              p_QoSdata:%p, p_QoSdata_sz:%d)",p_table,p_name, p_id, p_period, p_phase, p_deadline, p_QoSdata, p_QoSdata_sz);  


	//printf("\n priority:%d",*((int *)(p_QoSdata)));
	//return 0;

	if(p_table == NULL)
		return -1;

	if(p_table->nprocs == PROC_TABLE_SIZE)
		return -2;

	sem_pwait(pman_sem_id);

	/* Search for a free slot in the PT and check against duplicated process ids */
	free_index = -1;
	//unused: dup_flag = 0;

	for(i=0;i<PROC_TABLE_SIZE;i++)
	{
		if(p_table->proc[i].PROC_name[0] == 0 && free_index == -1)
			free_index = i;
		else
		{
			if( strcmp(p_table->proc[i].PROC_name,p_name) == 0)
			{
				sem_psignal(pman_sem_id);
				return -3;
			}
		}
	}

	if(free_index == -1) // Shouldn't happen !
	{
		sem_psignal(pman_sem_id);
		return -2;
	}


	/* Valid proc id and free slot found; add process data*/
	strcpy(p_table->proc[free_index].PROC_name,p_name);
	p_table->proc[free_index].PROC_id=p_id;
	p_table->proc[free_index].PROC_period = p_period;
	p_table->proc[free_index].PROC_phase = p_phase;
	p_table->proc[free_index].PROC_deadline = p_deadline;

	//printf("\n QoS address: %p QoS (priority):%d QoS size:%d ",p_QoSdata, *((int *)(p_QoSdata)), p_QoSdata_sz);
	//printf("\n ptable address:%p  ptable index:%d ptable QoS address: %x ",p_table,free_index, p_table->proc[free_index].PROC_qosdata + (int)p_table);
	memcpy((void *)(p_table->proc[free_index].PROC_qosdata+(void*)p_table),p_QoSdata,p_QoSdata_sz);

	p_table->proc[free_index].PROC_qosupdflag=1;

	p_table->proc[free_index].PROC_status = PROC_S_IDLE;
	p_table->proc[free_index].PROC_nact = 0;
	p_table->proc[free_index].PROC_ndm = 0;

	(p_table->nprocs)++;


	sem_psignal(pman_sem_id);
	return 0;
}

/*
 * Removes a process from the process table
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *             p_name                : process name to delete
 *
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 *             -2 : process not found
 */
int PMAN_procdel(char *p_name)
{
	int i;

	PMAN_DBG("\n PMAN_procdel called (p_table:%p  p_name:%s)",p_table, p_name);

	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	for(i=0;i<PROC_TABLE_SIZE;i++)

		if( strcmp(p_table->proc[i].PROC_name,p_name) == 0)
		{
			if(p_table->proc[i].PROC_id != PMAN_NOPID) /* If procees running, kill it */
				kill(p_table->proc[i].PROC_id, SIGINT);

			p_table->proc[i].PROC_name[0] = 0;
			p_table->proc[i].PROC_id = PMAN_NOPID;

			(p_table->nprocs)--;

			sem_psignal(pman_sem_id);
			return 0;
		}

	sem_psignal(pman_sem_id);
	return -2;
}


/*
 * Attaches the process PID of an already registered process 
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *              p_name               : process name (string)
 *              p_id                 : process id (OS PID)
 *
 * Returns:      0 : success
 *              -1 : invalid process table pointer (null) 
 *              -2 : process not found
 */
int PMAN_attach(char *p_name, pid_t p_id)
{
	int i;

	PMAN_DBG("\n PMAN_attach called (p_table:%p  p_name:%s p_id:%d)",p_table, p_name, p_id);

	if(p_table == NULL)
	{
		return -1;
	}

	sem_pwait(pman_sem_id);

	for (i=0; i<PROC_TABLE_SIZE; i++)
	{
		if (strcmp(p_table->proc[i].PROC_name, p_name) == 0)
		{
			p_table->proc[i].PROC_id = p_id;
			p_table->proc[i].PROC_qosupdflag=1; // Update process QoS on next activation

			sem_psignal(pman_sem_id);
			return 0;
		}
	}

	sem_psignal(pman_sem_id);
	return -2;
}


/*
 * Deattaches a process PID from an registered process 
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *             p_name                : process name (string)
 *
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 *              -2 : process not found
 */
int PMAN_deattach(char *p_name)
{
	int i;

	PMAN_DBG("\n PMAN_deattach called (p_table: %p  p_name:%s)",p_table, p_name);


	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	for(i=0;i<PROC_TABLE_SIZE;i++)

		if( strcmp(p_table->proc[i].PROC_name,p_name) == 0)
		{
			p_table->proc[i].PROC_id = PMAN_NOPID;

			sem_psignal(pman_sem_id);
			return 0;
		}

	sem_psignal(pman_sem_id);
	return -2;
}


/*
 * Defines a precedence constraint for a given process 
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *              p_name_pred          : process name (predecessor)
 *              p_name_succ          : process name (successor)
 *
 * Returns:      0 : success
 *              -1 : invalid process table pointer (null) 
 *              -2 : predecessor process not found
 *              -3 : successor process not found
 *              -4 : number of predecessors exceeded
 */
int PMAN_prec_add(char *p_name_pred, char *p_name_succ)
{
	int i, pred_index, succ_index;

	PMAN_DBG("\n PMAN_prec_define called (p_table:%p  p_name_pred:%s p_name_succ:%s)",p_table, p_name_pred, p_name_succ);

	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	/* Look for sucessor position on PMAN table */
	succ_index=PMAN_NOINDEX;
	for(i=0;i<PROC_TABLE_SIZE;i++)
		if( strcmp(p_table->proc[i].PROC_name,p_name_succ) == 0) {
			succ_index=i;
			break;
		}

	if(succ_index==PMAN_NOINDEX) { // Successor process not found !
		sem_psignal(pman_sem_id);
		return -3;
	}
	/* Look for predecessor in PMAN table */
	pred_index=PMAN_NOINDEX;
	for(i=0;i<PROC_TABLE_SIZE;i++)
		if( strcmp(p_table->proc[i].PROC_name,p_name_pred) == 0) {
			pred_index=i;
			break;
		}

	if(pred_index==PMAN_NOINDEX) { // Predecessor process not found !
		sem_psignal(pman_sem_id);
		return -2;
	}

	/* Update pred_name and succ_index arrays */
	for(i=0;i<PMAN_MAX_PRED;i++)
		if(p_table->proc[succ_index].PROC_pred_name[i][0] == 0) {
			strcpy(p_table->proc[succ_index].PROC_pred_name[i],p_name_pred);
			break;
		}

	if(i >= PMAN_MAX_PRED) { // Did not found an empty entry to add the predecessor name
		sem_psignal(pman_sem_id);
		return -4;
	}

	for(i=0;i<PMAN_MAX_SUCC;i++)
		if(p_table->proc[pred_index].PROC_succ_index[i] == PMAN_NOINDEX) {
			p_table->proc[pred_index].PROC_succ_index[i]=succ_index;
			break;
		}

	if(i >= PMAN_MAX_SUCC) { // Did not found an empty entry to add the successor index
		sem_psignal(pman_sem_id);
		return -4;
	}

	/* Update precedence mask of successor process ...  */
	p_table->proc[succ_index].PROC_pred_met = 0;
	p_table->proc[succ_index].PROC_pred_mask |= (1 << (pred_index));


	/* Done */
	sem_psignal(pman_sem_id);
	return -2;
}


/*
 * Updates the QoS data of a specified process 
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *             p_name                : process name (string)
 *             QoSdata               : pointer to QoS data structure
 *             QoSdata_sz            : size of QoS data structure
 *             when_flag             : when QoS update shoukd occur (immediate/on next activation)
 *
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 *             -2 : process not found
 *             -3 : invalid when_flag
 */
int PMAN_QoSupd(char * p_name, void * QoSdata, int QoSdata_sz, char when_flag)
{
	int i;

	PMAN_DBG("\n PMAN_QoSupd called! p_table:%p  p_name:%s *QoSdata:%p QoSdata_sz:%d when_flags:%d",p_table, \
			p_name,QoSdata, QoSdata_sz, when_flag);

	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	for(i=0;i<PROC_TABLE_SIZE;i++)

		if( strcmp(p_table->proc[i].PROC_name,p_name) == 0)
		{
			memcpy((void *)(p_table->proc[i].PROC_qosdata+(void*)p_table),QoSdata,QoSdata_sz);
			switch(when_flag)
			{
			case  PMAN_UPD_IMEDIATE:
				(*p_table->QoSupd)(i); // Update process's QoS
				p_table->proc[i].PROC_qosupdflag=0;
				break;

			case  PMAN_UPD_ONNEXTACT:
				p_table->proc[i].PROC_qosupdflag=1; // Update process QoS on next activation
				break;

			default:
				sem_psignal(pman_sem_id);
				return -3; // Invalid when_flag

			}

			sem_psignal(pman_sem_id);
			return 0;
		}

	sem_psignal(pman_sem_id);
	return -2;
}



/*
 * Updates the temporal atributes of a specified process 
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *             p_name                : process name (string)
 *             p_period              : process period
 *             p_phase               : process initial phase
 *             p_deadline            : process deadline
 *             which_flags           : which atributes to update
 *
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 *             -2 : process not found
 *             -3 : invalid atribute
 */
int PMAN_msgupd(char * p_name, int p_period, int p_phase, int p_deadline, char which_flag)
{
	int i;

	PMAN_DBG("\n PMAN_msgupd called! p_table:%p  p_name:%s p_period:%d p_phase:%d p_deadline:%d which_flags:%x",\
			p_table, p_name, p_period, p_phase, p_deadline, which_flag);

	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	for(i=0;i<PROC_TABLE_SIZE;i++)
		if( strcmp(p_table->proc[i].PROC_name,p_name) == 0)
		{
			if((which_flag & PMAN_UPD_PERIOD) && (p_period > 0))
				p_table->proc[i].PROC_period = p_period;

			if((which_flag & PMAN_UPD_PHASE) && (p_phase >= 0))
				p_table->proc[i].PROC_phase = p_phase;

			if((which_flag & PMAN_UPD_DEADLINE) && (p_deadline > 0))
				p_table->proc[i].PROC_deadline = p_deadline;

			sem_psignal(pman_sem_id);
			return 0;
		}

	sem_psignal(pman_sem_id);
	return -2;

}



/*
 * Signals that the process has fisnished its current instance 
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *             p_name                : process name (string)
 *              
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 *             -2 : process not found
 */
int PMAN_epilogue(char *p_name)
{
	int i,j, succ_index, check_preced_flag=0;

	PMAN_DBG("\n PMAN_epilogue called (p_table:%p  p_name:%s)", p_table, p_name);

	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	for(i=0;i<PROC_TABLE_SIZE;i++)

		if( strcmp(p_table->proc[i].PROC_name,p_name) == 0)
		{
			/* Update process status and finish time */
			p_table->proc[i].PROC_status = PROC_S_IDLE; //
			gettimeofday(&p_table->proc[i].PROC_last_finish, NULL);

#ifdef PMAN_TRACE
			p_table->evt_trace[p_table->evt_lastindex].pindex=i;
			p_table->evt_trace[p_table->evt_lastindex].etype='E';
			p_table->evt_trace[p_table->evt_lastindex].etime=p_table->proc[i].PROC_last_finish;
			p_table->evt_lastindex=(p_table->evt_lastindex+1) % PMAN_TRACE_SIZE;
			if(p_table->evt_lastindex == p_table->evt_firstindex)
				p_table->evt_firstindex+=1;
#endif

			/* Check for successors */
			for(j=0;j<PMAN_MAX_SUCC;j++)
				if( (succ_index = p_table->proc[i].PROC_succ_index[j]) != PMAN_NOINDEX) // Dependent process found!
				{
					p_table->proc[succ_index].PROC_pred_met |=  (1 << i); // Mark precedence as met
					check_preced_flag = 1; // Must check if successor process can be released
				}

			break; // Done
		}


	/* Done */
	sem_psignal(pman_sem_id);

	/* Check if successor processes can be released */
	if(check_preced_flag)
		PMAN_release();

	if(i == PROC_TABLE_SIZE)
		return -2; // Process not found
	else
		return 0;
}


/*
 * Queries the contents of the process table 
 * 
 * Input args: (global var) *p_table : pointer to the process table data structure
 *                            *pdata : pointer to a process data structure
 *                        reset_flag : if true, restart query from beggining of table
 *              
 * Returns:     0 : success
 *              1 : end of table reached
 *             -1 : invalid process table pointer (null) 
 *             -2 : invalid process data pointer 
 */
int PMAN_query(PROC_TYPE *pdata, int reset_flag)
{
	int i;
	static int pti=0; /* index of current process */

	PMAN_DBG("\n PMAN_query called (ptable: %p)",p_table);

	if(reset_flag) /* Restart from beginning */
		pti=0;

	if(pdata == NULL)
		return -2;

	if(p_table == NULL)
		return -1;

	if(pti >= PROC_TABLE_SIZE)
		return 1;

	for(i=pti;i<PROC_TABLE_SIZE;i++)
		if( p_table->proc[i].PROC_name[0] != 0)
		{
			//printf("(found at %d)",i);
			*pdata = p_table->proc[i];
			pti=i+1;
			return 0;

		}

	return 1;

}


/*
 * Prints the current contents of the process table 
 * 
 * Input args: (global var) *p_table : pointer to the process table data structure
 *              
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 */
int PMAN_print(void)
{
	int i;

	PMAN_DBG("\n PMAN_print called (ptable: %p)",p_table);

	if(p_table == NULL)
		return -1;

	printf("\n         name    ID    Per   Ph     Ddln  *QoSdta    QoSflg Stat  #Act #Dmiss  Start       Finish");
	for(i=0;i<PROC_TABLE_SIZE;i++)

		if( p_table->proc[i].PROC_name[0] != 0)
		{
			printf("\n [%d] : %5s %5d  %5d %5d %9d %9p %5d %4x %5u %5u %5ld:%5ld %5ld:%5ld",i,\
					p_table->proc[i].PROC_name,\
					p_table->proc[i].PROC_id,\
					p_table->proc[i].PROC_period,\
					p_table->proc[i].PROC_phase,\
					p_table->proc[i].PROC_deadline,\
					(void *)(p_table->proc[i].PROC_qosdata+(void*)p_table),\
					p_table->proc[i].PROC_qosupdflag,\
					p_table->proc[i].PROC_status,\
					p_table->proc[i].PROC_nact,\
					p_table->proc[i].PROC_ndm,\
					p_table->proc[i].PROC_last_start.tv_sec,\
					p_table->proc[i].PROC_last_start.tv_usec,\
					p_table->proc[i].PROC_last_finish.tv_sec,\
					p_table->proc[i].PROC_last_finish.tv_usec);
		}
		else
			printf("\n [%d] : Free slot",i);

	printf("\n");

	return 0;
}


#ifdef PMAN_TRACE
/*
 * Saves the current trace data 
 * 
 * Input args: (global var) *p_table : pointer to the process table data structure
 *              
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 *             -2 : no tarce data to save
 */
int PMAN_trace_save(void)
{
	int i, retval=0;
	char fname[80];
	FILE *flog;

	PMAN_DBG("\n PMAN_trace_save called (ptable: %p)",p_table);

	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	/* Save trace buffer */
	if(p_table->evt_firstindex != p_table->evt_lastindex) {


		sprintf(fname,"%ld-trace.txt",time(NULL));

		if( !(flog=fopen(fname,"wr")) ) {
			printf("[PMAN_trace_Save] Can't create log file (%s)",fname);

		}
		else {
			fprintf(flog,"\n    PName , PInd,EVT,      Instant\n");
			for(i=p_table->evt_firstindex;i != p_table->evt_lastindex;i=(i+1) % PMAN_TRACE_SIZE) {
				fprintf(flog,"%10s,%5d, %c ,%5ld,%5ld\n",			\
						p_table->proc[p_table->evt_trace[i].pindex].PROC_name,	\
						p_table->evt_trace[i].pindex,				\
						p_table->evt_trace[i].etype,				\
						p_table->evt_trace[i].etime.tv_sec,
						p_table->evt_trace[i].etime.tv_usec);
			}

			fclose(flog);
		}

		/* Reset indexes */
		p_table->evt_firstindex=0;
		p_table->evt_lastindex=0;
	}
	else
		retval = -2;


	/* Done */
	sem_psignal(pman_sem_id);
	return retval;

}
#endif


/*
 * Prints the current precedence status of the process table 
 * 
 * Input args: (global var) *p_table : pointer to the process table data structure
 *              
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 */
int PMAN_print_prec(void)
{
	int i,j;

	PMAN_DBG("\n PMAN_print_prec called (ptable: %p)",p_table);

	if(p_table == NULL)
		return -1;

	printf("\n [index] : name  {   process name 1,2,...,n    } { suc ind 1,2,...,n }    pred_mask   pred_met Status");
	for(i=0;i<PROC_TABLE_SIZE;i++)
		if( p_table->proc[i].PROC_name[0] != 0) {
			printf("\n [%5d] : %5s {",i, p_table->proc[i].PROC_name);

			/* Update pred_name and succ_index arrays */

			for(j=0;j<PMAN_MAX_PRED;j++) {
				if(p_table->proc[i].PROC_pred_name[j][0] != 0)
					printf("%5s",p_table->proc[i].PROC_pred_name[j]);
				else
					printf("     ");
				if(j!=(PMAN_MAX_PRED-1))
					printf(",");
			}

			printf("} {");

			for(j=0;j<PMAN_MAX_SUCC;j++) {
				if(p_table->proc[i].PROC_succ_index[j] != PMAN_NOINDEX)
					printf("%3d",p_table->proc[i].PROC_succ_index[j]);
				else
					printf("   ");

				if(j!=(PMAN_MAX_SUCC-1))
					printf(",");
			}

			printf("} %8lx    %8lx   %5d",p_table->proc[i].PROC_pred_mask, \
					p_table->proc[i].PROC_pred_met, p_table->proc[i].PROC_status);

		}
	printf("\n");

	/* Done */
	return 0;
}


/*
 * "System tick". Should be called every basic time unit (whatever it is).
 *                Checks for process activations; sends activation signals
 *                Checks for missed deadlines (TODO)
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 */
int PMAN_tick(void)
{
	int i, check_release_flag=0;

	PMAN_DBG("\n PMAN_tick called (ptable: %p ). Activated processes:",p_table);

	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	/* Scans the process table and activates processes*/
	for(i=0;i<PROC_TABLE_SIZE;i++)
	{
		if(p_table->proc[i].PROC_id != PMAN_NOPID)
		{ /* Filled PT slot: check if process ready */
			if( ( (p_table->ticks - p_table->proc[i].PROC_phase) % p_table->proc[i].PROC_period) == 0)
			{
				if(p_table->proc[i].PROC_qosupdflag) // Check for pending QoS update requests with PMAN_ONNEXTACT flag
				{
					(*p_table->QoSupd)(i); // Update process's QoS
					p_table->proc[i].PROC_qosupdflag=0; // Reset QoS update flag
					PMAN_DBG("(QoS update on process %s)", p_table->proc[i].PROC_name);

					//printf("**(QoS update on process %s)**", p_table->proc[i].PROC_name);
				}

				//printf("[PMAN_tick] process %s ACTIV");
				p_table->proc[i].PROC_status = PROC_S_ACTIV;
				check_release_flag = 1; // Signals that processes have become ready
			}
		}
	}

	/* Increment tick counter */
	(p_table->ticks)++;

	/* Release mutex region */
	sem_psignal(pman_sem_id);

	/* Release processes that became ready */
	if(check_release_flag)
		PMAN_release();

	return 0;
}



/*
 * Process release: Checks for process activations and precedences and sends activation signals when appropriate
 *               
 *
 * Input args: (global var) *p_table : pointer to the process table data structure
 *
 * Returns:     0 : success
 *             -1 : invalid process table pointer (null) 
 */
int PMAN_release(void)
{
	int i;

	int p_killed_index[PROC_TABLE_SIZE];
	int npid_killed=0;

	PMAN_DBG("\n PMAN_release called (ptable: %p ). Activated processes:",p_table);

	if(p_table == NULL)
		return -1;

	sem_pwait(pman_sem_id);

	/* Scans the process table and activates processes*/
	for(i=0;i<PROC_TABLE_SIZE;i++)
		if( p_table->proc[i].PROC_id != PMAN_NOPID)
			if( (p_table->proc[i].PROC_status == PROC_S_ACTIV) || (p_table->proc[i].PROC_status == PROC_S_PEND) ) {
				/* Activate process if activated and precedences met */
				if( p_table->proc[i].PROC_pred_mask == p_table->proc[i].PROC_pred_met) {
					p_table->proc[i].PROC_pred_met = 0; // Reset precedence bitmap

					if(kill(p_table->proc[i].PROC_id, PMAN_ACTIVATE_SIG))
						if(errno == ESRCH) /* Process no longer exists */
							p_killed_index[npid_killed++] = i;

					gettimeofday(&p_table->proc[i].PROC_last_start, NULL);

#ifdef PMAN_TRACE
					p_table->evt_trace[p_table->evt_lastindex].pindex=i;
					p_table->evt_trace[p_table->evt_lastindex].etype='R';
					p_table->evt_trace[p_table->evt_lastindex].etime=p_table->proc[i].PROC_last_start;
					p_table->evt_lastindex=(p_table->evt_lastindex+1) % PMAN_TRACE_SIZE;
					if(p_table->evt_lastindex == p_table->evt_firstindex)
						p_table->evt_firstindex+=1;
#endif

					p_table->proc[i].PROC_status = PROC_S_READY;
					p_table->proc[i].PROC_nact++;

					PMAN_DBG(" activated [%s (%d)] ",p_table->proc[i].PROC_name, p_table->proc[i].PROC_id);
				}
				else {
					p_table->proc[i].PROC_status = PROC_S_PEND;
					PMAN_DBG(" pending [%s (%d)] ",p_table->proc[i].PROC_name, p_table->proc[i].PROC_id);
				}

			}

	/* Exit mutex */
	sem_psignal(pman_sem_id);

	/* Detach killed processes */
	for(i=0;i<npid_killed;i++)
		PMAN_deattach(p_table->proc[p_killed_index[i]].PROC_name);

	return 0;
}

int linux_sched_fifo(int pindex)
{
	struct sched_param proc_sched;
	int prio;
	pid_t pid;
	PROC_TYPE procdata;

	procdata=p_table->proc[pindex];
	prio = *( (int *)((int)procdata.PROC_qosdata+(void*)p_table));
	pid=procdata.PROC_id;

	fprintf(stderr, "\nMaster : <QoS, pindex=%d pid=%d prio=%d>\n",pindex, pid, prio);

#if _POSIX_PRIORITY_SCHEDULING > 0
	// Set the process priority and scheduling policy
	proc_sched.sched_priority=prio;
	if( sched_setscheduler(pid,SCHED_FIFO,&proc_sched) == -1)
	{
		fprintf(stderr,"linux_cshed: [QoS]: Error setting the priority (sched_setscheduler)!");
		return -3;
	}
#endif

	return 0;
}


