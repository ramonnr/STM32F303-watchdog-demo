

#ifndef __OS_THREAD_HELPER_FUNCTION
#define __OS_THREAD_HELPER_FUNCTION


#include "main.h"
#include "cmsis_os.h"

typedef struct _osThread_def_UD threadDef;

//typedef struct _os_thread_def_UD{
//  char                   *name;        /* Thread name                                               */
//  os_pthread             pthread;      /* start address of thread function                          */
//  osPriority             tpriority;    /* initial thread priority                                   */
//  uint32_t               instances;    /* maximum number of instances of that thread function       */
//  uint32_t               stacksize;    /* stack size requirements in bytes; 0 is default stack size */
//}threadDef;



void* copy_osThreadDef_UD_t(threadDef* destination, osThreadDef_t* source);

#endif