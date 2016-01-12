/**
  *A spelendid example of a simple watchdog
  *
  **/
/* Includes ------------------------------------------------------------------*/
//#include "stm32f3xx_hal.h"
//#include "cmsis_os.h"
//#include <string.h>
#include "main.h"


/* Private variables ---------------------------------------------------------*/
//global for checkback
uint8_t gCheckback=0;


IWDG_HandleTypeDef hiwdg;
#define KICK_WATCHDOG hiwdg.Instance->KR=0xAAAA


osThreadId defaultTaskHandle;
osThreadId buttonTaskHandle;



//4 test threads for WD verification
osThreadId worker1TaskHandle;
osThreadId worker2TaskHandle;
osThreadId worker3TaskHandle;
osThreadId worker4TaskHandle;
osThreadId loadHandle;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void const * argument);
void StartButtonTask(void const* argument);
void WorkerThread(void const* argument);
void loadHandleThread(void const* argument);

#include <stdio.h>

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  
  //printf("reset :0x%x\n",rcc.CSR);
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  RCC_TypeDef rcc;
  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)!=RESET){
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    HAL_Delay(2000);
  }else{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }
  
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
  MX_IWDG_Init();

  
  osThreadDef(buttonTask, StartButtonTask, osPriorityRealtime,0,128);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask),NULL);
  
  
  /*
  osThreadId worker1TaskHandle;
  osThreadId worker2TaskHandle;
  osThreadId worker3TaskHandle;
  osThreadId worker4TaskHandle;
  
  */
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);
  static threadArgs args0;
  args0.led=GPIO_PIN_9;
  args0.id=0;
  args0.runningTime = 100;
  static threadArgs args1;
  args1.led=GPIO_PIN_11;
  args1.id  = 1;
  args1.runningTime=100;
  static threadArgs args2;
  args2.led = GPIO_PIN_13;
  args2.id  = 2;
  args2.runningTime=100;
  static threadArgs args3;
  args3.led = GPIO_PIN_15;
  args3.id  =3;
  args3.runningTime=100;
  
  
  osThreadDef(worker1, WorkerThread, osPriorityNormal,0,128);
  worker1TaskHandle= osThreadCreate(osThread(worker1),(void*)&args0);
  
  
  
  osThreadDef(worker2, WorkerThread, osPriorityNormal,0,128);
  worker2TaskHandle= osThreadCreate(osThread(worker2),(void*)&args1);


  
  osThreadDef(worker3, WorkerThread, osPriorityNormal,0,128);
  worker3TaskHandle= osThreadCreate(osThread(worker3),(void*)&args2);

  
  osThreadDef(worker4, WorkerThread, osPriorityNormal,0,128);
  worker4TaskHandle= osThreadCreate(osThread(worker4),(void*)&args3);
  
  //struct for the handler thread
  
  /*
  
typedef struct _threadInfo{
  uint32_t size;
  osThreadId* ID;
  void* ThreadDef;
}threadInfo;
  */
  
  threadInfo* tInfo     = pvPortMalloc(sizeof(threadInfo));
  tInfo->size           = 5;
  tInfo->ID             = pvPortMalloc(sizeof(osThreadId)*5);
  tInfo->tDef           = pvPortMalloc(sizeof(threadDef)*5); 
  tInfo->arg            = (void*)pvPortMalloc(sizeof(void*)*5);
  
  //save thread handles. Take not that the order is the same 
  //as the order they ack the loadHandler thread through gCheckback
  tInfo->ID[0]          = worker1TaskHandle;
  tInfo->ID[1]          = worker2TaskHandle;
  tInfo->ID[2]          = worker3TaskHandle;
  tInfo->ID[3]          = worker4TaskHandle;
  tInfo->ID[4]          = buttonTaskHandle;
 
  //copy thread data, note the order
  copy_osThreadDef_UD_t(&(tInfo->tDef[0]),(struct os_thread_def const*)&(os_thread_def_worker1));
  copy_osThreadDef_UD_t(&(tInfo->tDef[1]),(struct os_thread_def const*)&(os_thread_def_worker2));
  copy_osThreadDef_UD_t(&(tInfo->tDef[2]),(struct os_thread_def const*)&(os_thread_def_worker3));
  copy_osThreadDef_UD_t(&(tInfo->tDef[3]),(struct os_thread_def const*)&(os_thread_def_worker4));
  copy_osThreadDef_UD_t(&(tInfo->tDef[4]),(struct os_thread_def const*)&(os_thread_def_buttonTask));
  
  
  //copy thread arguments
  tInfo->arg[0]         = (void*)&args0;
  tInfo->arg[1]         = (void*)&args1;
  tInfo->arg[2]         = (void*)&args2;
  tInfo->arg[3]         = (void*)&args3;
  tInfo->arg[4]         = (void*)NULL;
  //creation of watchdog kicker
  osThreadDef(load, loadHandleThread, osPriorityRealtime,0,128);
  loadHandle = osThreadCreate(osThread(load),(void*)tInfo);
 

  /* Start scheduler */
  osKernelStart(NULL, NULL);
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  //do not put code here
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  HAL_IWDG_Init(&hiwdg);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PA11   ------> USB_DM
     PA12   ------> USB_DP
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin 
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin 
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 SPI1_MISO_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
void StartButtonTask(void const* argument){
  int dummy=-1;
  while(1){
    while((int)HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)){
      HAL_Delay(1000);
    }
    gCheckback|=1<<4;
    osDelay(100);
  }

}


//worker thread 
void WorkerThread(void const* argument){
  
  threadArgs* args = (threadArgs*)argument;
  uint16_t led  = args->led;
  uint8_t threadId = args->id;
  uint32_t runningTime = args->runningTime;
  
  gCheckback|=1<<threadId;
  while(1){
    HAL_GPIO_WritePin(GPIOE,led,GPIO_PIN_SET);
    HAL_Delay(runningTime);
    HAL_GPIO_WritePin(GPIOE,led,GPIO_PIN_RESET);
    HAL_Delay(runningTime);
    gCheckback |= 1<<threadId;
  }
  
}

void loadHandleThread(void const* arguments){
  threadInfo* tInfo = (threadInfo*)arguments;
  size_t size = tInfo->size;
  
 osDelay(400);
  while(1){
  
  uint8_t ack = gCheckback;
  if(ack == 0x1F){
    gCheckback=0;
    KICK_WATCHDOG;
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
  }
  //shit probally hit the fan
  else{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
    
    for(int i = 0; i < size; i++){
      if( ((ack >> i)&0x1)  ==  0) {
        vTaskDelete(tInfo->ID[i]);
        osThreadCreate((osThreadDef_t*)&(tInfo->tDef[i]),*(tInfo->arg));
        break;
      }
    }    
  }
  osDelay(400);
  }
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
  osDelay(500);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
    osDelay(100);
  }
  /* USER CODE END 5 */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
