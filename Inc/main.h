#ifndef __MAINH
#define __MAINH

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
//#include "os_thread_helper_function.h"

typedef struct _threadArgs{
  uint16_t              led;
  uint8_t               id;
  uint32_t              runningTime;
}threadArgs;


typedef struct _genericArg{
  char*                 type;
  void**                 arg;
}genericArg;

/*bruteforce every-fucking-thing
or this is defined as a goddamn const in cmsis_os.h
So i redefine it for reasons
*/
typedef struct _os_thread_def_UD{
  char*                 name;        /* Thread name                                               */
  os_pthread            pthread;      /* start address of thread function                          */
  osPriority            tpriority;    /* initial thread priority                                   */
  uint32_t              instances;    /* maximum number of instances of that thread function       */
  uint32_t              stacksize;    /* stack size requirements in bytes; 0 is default stack size */
}threadDef;

//typedef _os_thread_def_UD threadDef;

typedef struct _threadInfo{
  uint32_t size;
  osThreadId*           ID;
  threadDef*            tDef;
  void**                 arg;
}threadInfo;

void* copy_osThreadDef_UD_t(threadDef* destination, osThreadDef_t* source);


#endif