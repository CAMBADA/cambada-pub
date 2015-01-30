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

#ifndef PMAN_H_INCLUDED
#define PMAN_H_INCLUDED

#include <sys/types.h>
#include <sys/time.h>

/*
 * Defines
 *
 */
#define PMAN_ACTIVATE_SIG	SIGCONT		// Signal used to activate processes

#define PMAN_ATTACH			1			// PMAN_init option: attach to an existing process table
#define PMAN_NEW			0			// PMAN_init option: initialize a new process table

#define PMAN_CLFREE			0			// Flag for PMAN_close. Release resources. Called by "scheduler" process 
										//    (shared mem, semaphore and kills active processes). 
#define PMAN_CLLEAVE		1			// Flag for PMAN_close. Deattaches from shared mem, only. Called by "client" processes 

#define PMAN_UPD_IMEDIATE	0x01		// QoS update carried out immediately
#define PMAN_UPD_ONNEXTACT	0x02		// QoS update carried out on next activation

#define PMAN_UPD_PERIOD		0x04		// Update period
#define PMAN_UPD_PHASE		0x08		// Update initial phase
#define PMAN_UPD_DEADLINE	0x10		// Update deadline


#define PROC_TABLE_SIZE		  20		// Maximum number of processes managed
#define PNAME_LEN		  32		// Maximum length of the process name string

#define PMAN_NOPID		   0		// Process id undefined 
#define PMAN_NOINDEX		  -1		// Process index in PMAN is undefined 

#define PMAN_MAX_PRED              5    // Maximum number of predecessors that a process may have             
#define PMAN_MAX_SUCC              5    // Maximum number of successors that a process may have             

#define PMAN_NOPENDACT             0    // No pending activation on process
#define PMAN_PENDACT               1    // Pending activation on process

/* Process status */
#define PROC_S_EMPTY            0       // Process entry not filled up  
#define PROC_S_ACTIV            1       // Process was activated (but not yet released)
#define PROC_S_READY		2	// Process ready to execute (i.e. active ) flag  
#define PROC_S_IDLE		3	// Process waiting for next instance
#define PROC_S_PEND		4	// Process activated but with unsatisfied precedences  


/* TRACE parameters */
#define PMAN_TRACE
#define PMAN_TRACE_SIZE     10000     // Number of events to record during trace ( events/sec = FPS*SUM(1/Period_i)*2 )


/*
 * Data types
 *
 */
#ifdef PMAN_TRACE
typedef struct {
  int pindex;            // Process index within PMAN table
  int etype;             // Event type {[R]elease;[E]pilog}
  struct timeval etime;  // Event time 
} PROC_TRACE_DATA;
#endif

typedef struct {
  char PROC_name[PNAME_LEN+1];  // Process id string
  pid_t PROC_id;                // Process id
  
  /* generic temporal properties */
  int    PROC_period;           // Process period
  int    PROC_phase;            // Process initial phase
  int    PROC_deadline;         // Process deadline (in us)

  /* precedence constraints */
  char   PROC_pred_name[PMAN_MAX_PRED][PNAME_LEN+1]; // Array with the names of the predecessor processes
  int    PROC_succ_index[PMAN_MAX_SUCC];             // Array with the PMAN table indexes of the successors of the process
  unsigned long   PROC_pred_mask;                    // bitmap with the index of all the predecessor processes set  
  unsigned long   PROC_pred_met;                     // bitmap with the index of predecessor processes that had an instance

  /* QoS data */
  int    PROC_qosdata;            // Offset to data structure containing QoS specific data
  char   PROC_qosupdflag;         // Flag to signal if qos data has been updated

  /* Status & statistics data */
  struct timeval PROC_last_start;  // Activation instant (last instance)
  struct timeval PROC_last_finish; // Finish instant (last instance)

  int  PROC_status;            // Process status {PROC_S_READY}
  
  unsigned int PROC_nact;      // Number of activations
  unsigned int PROC_ndm;       // Number of deadline misses
 
} PROC_TYPE;

typedef struct {
  int nprocs;        // Number of active processes registered
  int ticks;         // System "tick" counter
  int (*QoSupd)();   // QoS update function hook
  int (*DdlnExcpt)();// Deadline exception handling hook
  PROC_TYPE proc[PROC_TABLE_SIZE];
#ifdef PMAN_TRACE
  int evt_lastindex;  // Index of last event recorded
  int evt_firstindex; // Index of first event recorded
  PROC_TRACE_DATA evt_trace[PMAN_TRACE_SIZE]; // Trace data
#endif
} PROC_TABLE_TYPE;



/* 
 * Global vars
 */
extern int pman_sem_id;                // Semaphore ID (mutual exclusion)
extern int pman_shmem_id;              // Shared memory ID
extern PROC_TABLE_TYPE * p_table; // Process table

/************************************************************/
/*    To be used when more than a master exists             */
/************************************************************/
#define MAX_MASTER_INSTANCE 40

extern int pamn_sem_id_LUT[ MAX_MASTER_INSTANCE ];
extern int pamn_shmem_id_LUT[ MAX_MASTER_INSTANCE ];
extern PROC_TABLE_TYPE* pman_p_table_LUT[ MAX_MASTER_INSTANCE ];

/**
 *  Save current IPCs IDs of the current PMAN Master
 *
 *  @param _master Master id.
 *
 */
inline void pman_save_ids(int _master){
  if ( _master >= MAX_MASTER_INSTANCE ) return;
  
  pamn_sem_id_LUT[_master]   = pman_sem_id;
  pamn_shmem_id_LUT[_master] = pman_shmem_id;
  pman_p_table_LUT[_master]  = p_table;
}

/**
 *  Switch current IPCs IDs to the IPCs IDs of another PMAN Master.
 *
 *  @param _master Master id.
 *
 */
inline void pman_switch_id(int _master){
  if ( _master >= MAX_MASTER_INSTANCE ) return;
  
  pman_sem_id   = pamn_sem_id_LUT[_master];
  pman_shmem_id = pamn_shmem_id_LUT[_master];
  p_table       = pman_p_table_LUT[_master];
}

/************************************************************/

/*
 * Function prototypes
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  
/**
 * \brief Initializes the process table
 *
 * \param shmem_pman_key shared memory key
 * \param sem_pman_key sempahore key (mutual exclusion)
 * \param QoSfun pointer to QoS manager function (only "manager" process)
 * \param QoSdata_sz size of QoS data structure
 * \param create_flags PMAN_NEW - Create a new process table 
 *                     PMAN_ATTACH - Attach to an existing process table
 *
 * \return  0 : Success
 *         -1 : Error creating shared memory region 
 *         -2 : Failed to create semaphore 
 *         -3 : Error setting the priority
 *         -4 : Error allocating memory (QoS data)
 */
int PMAN_init(key_t shmem_pman_key, key_t sem_pman_key, void * QoSfun, int QoSdata_sz, int create_flags);
int PMAN_init2(key_t shmem_pman_key, key_t sem_pman_key, void * QoSfun, int QoSdata_sz, int create_flags);


/**
 * \brief Releases resources allocated by PCSHED lib 
 *
 * \param close_flags PMAN_CLFREE  - Completely removes all resources (shared mem,
 *                             semaphore) and kill all registered "client" processes 
 *                    PMAN_CLLEAVE - Just deattaches shared mem region 
 *              
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : error removing shared memory
 *         -3 : error deleting semaphore
 *         -4 : invalid close flag
 */
int PMAN_close(int close_flags);


/**
 * \brief Process release
 *
 * Checks for process activations and precedences and sends activation signals when appropriate
 *               
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 */
int PMAN_release(void);
 

/**
 * \brief Adds a process to the process table
 *
 * \param p_name process name (string)
 * \param p_id  process id (OS PID)
 * \param p_period  process period (ticks)
 * \param p_phase process initial phase (ticks)
 * \param p_deadline process deadline (ticks)
 * \param p_QoSdata  pointer to process QoS data
 * \param p_QoSdata_sz size of QoS data sttructure
 *
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : process table full
 *         -3 : duplicated process id
 */
int PMAN_procadd(char * p_name, int p_id, int p_period, int p_phase, int p_deadline, void * p_QoSdata, int p_QoSdata_sz);


/**
 * \brief Removes a process from the process table
 *
 * \param p_name  process name to delete
 *
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : process not found
 */
int PMAN_procdel(char * p_name);
  

/**
 * \brief Attaches the process PID of an already registered process 
 *
 * \param p_name  process name (string)
 * \param p_id    process id (OS PID)
 *
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : process not found
 */
int PMAN_attach(char * p_name, pid_t p_id);


/**
 * \brief Detaches a process PID from a registered process 
 *
 * \param p_name process name (string)
 *
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : process not found
 */
int PMAN_deattach(char * p_name);


/**
 * \brief Defines a precedence constraint for a given process 
 *
 * \param p_name_pred          : process name (predecessor)
 * \param p_name_succ          : process name (successor)
 *
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : predecessor process not found
 *         -3 : successor process not found
 *         -4 : number of predecessors exceeded
 */
int PMAN_prec_add(char *p_name_pred, char *p_name_succ);


/**
 * \brief Updates the QoS data of a specified process 
 *
 * \param p_name                : process name (string)
 * \param QoSdata               : pointer to QoS data structure
 * \param QoSdata_sz            : size of QoS data structure
 * \param when_flag             : when QoS update shoukd occur (immediate/on next activation)
 *
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : process not found
 *         -3 : invalid when_flag
 */
int PMAN_QoSupd(char * p_name, void * QoSdata, int QoSdata_sz, char when_flag);


/**
 * \brief Updates the temporal atributes of a specified process 
 *
 * \param p_name                : process name (string)
 * \param p_period              : process period
 * \param p_phase               : process initial phase
 * \param p_deadline            : process deadline
 * \param which_flags           : which atributes to update
 *
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : process not found
 *         -3 : invalid atribute
 */
int PMAN_msgupd(char * p_name, int p_period, int p_phase, int p_deadline, char which_flag);


/**
 * \brief Signals that the process has fisnished its current instance 
 *
 * \param p_name                : process name (string)
 *              
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 *         -2 : process not found
 */
int PMAN_epilogue(char *p_name);


/**
 * \brief Queries the contents of the process table 
 * 
 * \param pdata : pointer to a process data structure
 * \param reset_flag : if true, restart query from beggining of table
 *              
 * \return  0 : success
 *          1 : end of table reached
 *         -1 : invalid process table pointer (null) 
 *         -2 : invalid process data pointer 
 */
int PMAN_query(PROC_TYPE *pdata, int reset_flag);


/**
 * \brief Prints the current contents of the process table 
 * 
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 */
int PMAN_print(void);


/**
 * \brief Prints the current precedence status of the process table 
 * 
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 */
int PMAN_print_prec(void);


#ifdef PMAN_TRACE
  int PMAN_trace_save(void);
#endif


/**
 * \brief "System tick". 
 *
 * Should be called every basic time unit (whatever it is).
 *  Checks for process activations; sends activation signals
 *  Checks for missed deadlines (TODO)
 *
 * \return  0 : success
 *         -1 : invalid process table pointer (null) 
 */
int PMAN_tick(void);  


/** \todo Is function linux_sched_fifo public? */
int linux_sched_fifo(int pindex);

  
#ifdef __cplusplus
}
#endif

#endif
