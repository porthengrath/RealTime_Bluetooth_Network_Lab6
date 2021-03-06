// os.c
// Runs on LM4F120/TM4C123/MSP432
// Lab 3 starter file.
// Daniel Valvano
// March 24, 2016

#include <stdint.h>
#include "os.h"
#include "CortexM.h"
#include "BSP.h"

// function definitions in osasm.s
void StartOS(void);
static void runperiodicevents(void);

#define NUMTHREADS  6        // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
  int32_t *blocked;  // nonzero if blocked on this semaphore
  uint32_t sleep;    // nonzero if this thread is sleeping
  
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];

struct event_tcb_t
{
  void    (*funcp)(void);
  uint32_t timer_ms;
  uint32_t period_ms;
};

#define NUM_EVT_THREADS 2
struct event_tcb_t event_tcbs[NUM_EVT_THREADS];

// ******** OS_Init ************
// Initialize operating system, disable interrupts
// Initialize OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initialize OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void){
  DisableInterrupts();
  BSP_Clock_InitFastest();// set processor clock to fastest speed
  // perform any initializations needed
  BSP_PeriodicTask_Init(runperiodicevents, 
                        1000,
                        0);
}

void SetInitialStack(int i){
  Stacks[i][STACKSIZE-1] = 0x01000000; // Thumb bit
  // reserved Stacks[i][STACKSIZE - 2] for PC
  Stacks[i][STACKSIZE-3] = 0x14141414; // R14
  Stacks[i][STACKSIZE-4] = 0x12121212; // R12
  Stacks[i][STACKSIZE-5] = 0x03030303; // R3
  Stacks[i][STACKSIZE-6] = 0x02020202; // R2
  Stacks[i][STACKSIZE-7] = 0x01010101; // R1
  Stacks[i][STACKSIZE-8] = 0x00000000; // R0
  Stacks[i][STACKSIZE-9] = 0x11111111; // R11
  Stacks[i][STACKSIZE-10] = 0x10101010; // R10
  Stacks[i][STACKSIZE-11] = 0x09090909; // R9
  Stacks[i][STACKSIZE-12] = 0x08080808; // R8
  Stacks[i][STACKSIZE-13] = 0x07070707; // R7
  Stacks[i][STACKSIZE-14] = 0x06060606; // R6
  Stacks[i][STACKSIZE-15] = 0x05050505; // R5
  Stacks[i][STACKSIZE-16] = 0x04040404; // R4
  
  // thread stack pointer
  tcbs[i].sp = &Stacks[i][STACKSIZE - 16];
}

//******** OS_AddThreads ***************
// Add six main threads to the scheduler
// Inputs: function pointers to six void/void main threads
// Outputs: 1 if successful, 0 if this thread can not be added
// This function will only be called once, after OS_Init and before OS_Launch
int OS_AddThreads(void(*thread0)(void),
                  void(*thread1)(void),
                  void(*thread2)(void),
                  void(*thread3)(void),
                  void(*thread4)(void),
                  void(*thread5)(void)){
  // save PRIMASK                    
  int32_t status = StartCritical();
  
  // initialize TCB circular list
  tcbs[0].next = &tcbs[1];
  tcbs[1].next = &tcbs[2];
  tcbs[2].next = &tcbs[3];
  tcbs[3].next = &tcbs[4];
  tcbs[4].next = &tcbs[5];
  
  // set the last index to wrap around
  tcbs[5].next = &tcbs[0];

  // initialize RunPt
  RunPt = &tcbs[0];

  // initialize four stacks, including initial PC
  SetInitialStack(0); Stacks[0][STACKSIZE - 2] = (int32_t)(thread0);
  SetInitialStack(1); Stacks[1][STACKSIZE - 2] = (int32_t)(thread1);
  SetInitialStack(2); Stacks[2][STACKSIZE - 2] = (int32_t)(thread2);
  SetInitialStack(3); Stacks[3][STACKSIZE - 2] = (int32_t)(thread3);
  SetInitialStack(4); Stacks[4][STACKSIZE - 2] = (int32_t)(thread4);
  SetInitialStack(5); Stacks[5][STACKSIZE - 2] = (int32_t)(thread5);
  
  // set as non-blocked and non-sleeping
  for (uint8_t i = 0; i < sizeof(tcbs)/sizeof(tcbs[0]); i++)
  {
    tcbs[i].blocked = 0;
    tcbs[i].sleep   = 0;
  }

  // return PRIMASK
  EndCritical(status);
  return 1;               // successful
}

//******** OS_AddPeriodicEventThread ***************
// Add one background periodic event thread
// Typically this function receives the highest priority
// Inputs: pointer to a void/void event thread function
//         period given in units of OS_Launch (Lab 3 this will be msec)
// Outputs: 1 if successful, 0 if this thread cannot be added
// It is assumed that the event threads will run to completion and return
// It is assumed the time to run these event threads is short compared to 1 msec
// These threads cannot spin, block, loop, sleep, or kill
// These threads can call OS_Signal
// In Lab 3 this will be called exactly twice
int OS_AddPeriodicEventThread(void(*thread)(void), uint32_t period)
{
  uint8_t i = 0;
  for (i = 0; i < NUM_EVT_THREADS; i++)
  {
    if (event_tcbs[i].period_ms == 0)
    {
      // found an empty slot
      break;
    }
  }
  if (i < NUM_EVT_THREADS)
  {
    event_tcbs[i].funcp     = thread;
    event_tcbs[i].period_ms = period;
    event_tcbs[i].timer_ms  = 0;
    return 1;
  }
  else
  {
    // no slots found
    return 0;
  }
}

void static runperiodicevents(void){
  // RUN PERIODIC THREADS, DECREMENT SLEEP COUNTERS
  for (uint8_t n = 0; n < sizeof(event_tcbs)/sizeof(event_tcbs[0]); n++)
  {
    event_tcbs[n].timer_ms++;
    
    if (event_tcbs[n].timer_ms >= event_tcbs[n].period_ms)
    {
      // reset
      event_tcbs[n].timer_ms = 0;

      // invoke callback
      event_tcbs[n].funcp();
    }
  }
  
  for (uint8_t i = 0; i < sizeof(tcbs)/sizeof(tcbs[0]); i++)
  {
    if (tcbs[i].sleep > 0)
    {
      // decrement the sleep time
      tcbs[i].sleep = tcbs[i].sleep - 1;
    }
  }
}

//******** OS_Launch ***************
// Start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
// Errors: theTimeSlice must be less than 16,777,216
void OS_Launch(uint32_t theTimeSlice){
  STCTRL = 0;                  // disable SysTick during setup
  STCURRENT = 0;               // any write to current clears it
  SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
  STRELOAD = theTimeSlice - 1; // reload value
  STCTRL = 0x00000007;         // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
// runs every ms
void Scheduler(void){ // every time slice
// ROUND ROBIN, skip blocked and sleeping threads
  RunPt = RunPt->next;
  while (RunPt->blocked || RunPt->sleep)
  {
    RunPt = RunPt->next;
  }
}

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void){
  STCURRENT = 0;        // any write to current clears it
  INTCTRL = 0x04000000; // trigger SysTick
// next thread gets a full time slice
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  // set sleep parameter in TCB
  RunPt->sleep = sleepTime;
  
  // suspend, stops running
  OS_Suspend();
}

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
  *semaPt = value;
}

// ******** OS_Wait ************
// Decrement semaphore and block if less than zero
// Lab2 spinlock (does not suspend while spinning)
// Lab3 block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt){
 DisableInterrupts();
 (*semaPt) = (*semaPt) - 1;
  
  // check if thread is to be blocked
  if ((*semaPt) < 0)
  {
    // suspend this thread and blocked it from running
    RunPt->blocked = semaPt;
    EnableInterrupts();
    OS_Suspend();
  }
 EnableInterrupts();
}

// ******** OS_Signal ************
// Increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt){
  DisableInterrupts();
  (*semaPt) = (*semaPt) + 1;

  if ((*semaPt) <= 0)
  {
    tcbType * pt = RunPt->next;
    while (pt->blocked != semaPt)
    {
      pt = pt->next;
    }

    // wake up the thread
    pt->blocked = 0;
  }
  EnableInterrupts();
}

#define FSIZE 10    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t  CurrentSize;// 0 means FIFO empty, FSIZE means full
int32_t  FifoMutex;
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  
// One event thread producer, one main thread consumer
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){
  PutI = 0;
  GetI = 0;
  for (uint32_t i = 0; i < FSIZE; i++)
  {
    Fifo[i] = 0;
  }
  OS_InitSemaphore(&CurrentSize, 0);
  OS_InitSemaphore(&FifoMutex, 1);
  LostData    = 0;
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  
// Exactly one event thread puts,
// do not block or spin if full
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
  if (CurrentSize == FSIZE)
  {
    LostData++;
    return -1;
  }
  else
  {
    Fifo[PutI] = data;

    // increment the index
    PutI = (PutI + 1) % FSIZE;

    // signal the consumer
    OS_Signal(&CurrentSize);
    return 0;
  }
}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.   
// Exactly one main thread get,
// do block if empty
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
  OS_Wait(&CurrentSize);
  // OS_Wait(&FifoMutex);
  data = Fifo[GetI];
  GetI = (GetI + 1) % FSIZE;
  // OS_Signal(&FifoMutex);
  return data;
}



