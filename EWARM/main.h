
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

typedef struct _threadArgs{
  uint16_t led;
  uint8_t  id;
  uint32_t runningTime;
}threadArgs;

typedef struct _threadInfo{
  uint32_t size;
  osThreadId* ID;
  osThreadDef_t* ThreadDef;
}threadInfo;

