/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for eventCreator */
osThreadId_t eventCreatorHandle;
const osThreadAttr_t eventCreator_attributes = {
  .name = "eventCreator",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for dispatcher */
osThreadId_t dispatcherHandle;
const osThreadAttr_t dispatcher_attributes = {
  .name = "dispatcher",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for police */
osThreadId_t policeHandle;
const osThreadAttr_t police_attributes = {
  .name = "police",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ambulance */
osThreadId_t ambulanceHandle;
const osThreadAttr_t ambulance_attributes = {
  .name = "ambulance",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for fire */
osThreadId_t fireHandle;
const osThreadAttr_t fire_attributes = {
  .name = "fire",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for corona */
osThreadId_t coronaHandle;
const osThreadAttr_t corona_attributes = {
  .name = "corona",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for queue1 */
osMessageQueueId_t queue1Handle;
const osMessageQueueAttr_t queue1_attributes = {
  .name = "queue1"
};
/* Definitions for semPolice */
osSemaphoreId_t semPoliceHandle;
const osSemaphoreAttr_t semPolice_attributes = {
  .name = "semPolice"
};
/* Definitions for semAmbulance */
osSemaphoreId_t semAmbulanceHandle;
const osSemaphoreAttr_t semAmbulance_attributes = {
  .name = "semAmbulance"
};
/* Definitions for semCorona */
osSemaphoreId_t semCoronaHandle;
const osSemaphoreAttr_t semCorona_attributes = {
  .name = "semCorona"
};
/* Definitions for semFire */
osSemaphoreId_t semFireHandle;
const osSemaphoreAttr_t semFire_attributes = {
  .name = "semFire"
};
/* USER CODE BEGIN PV */

osThreadId_t policeHandle1;
const osThreadAttr_t police_attributes1 = {
  .name = "police1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t policeHandle2;
const osThreadAttr_t police_attributes2 = {
  .name = "police2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t policeHandle3;
const osThreadAttr_t police_attributes3 = {
  .name = "police3",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t ambulanceHandle1;
const osThreadAttr_t ambulance_attributes1 = {
  .name = "ambulance1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t ambulanceHandle2;
const osThreadAttr_t ambulance_attributes2 = {
  .name = "ambulance2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t ambulanceHandle3;
const osThreadAttr_t ambulance_attributes3 = {
  .name = "ambulance3",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t ambulanceHandle4;
const osThreadAttr_t ambulance_attributes4 = {
  .name = "ambulance4",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t fireHandle1;
const osThreadAttr_t fire_attributes1 = {
  .name = "fire1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t fireHandle2;
const osThreadAttr_t fire_attributes2 = {
  .name = "fire2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t coronaHandle1;
const osThreadAttr_t corona_attributes1 = {
  .name = "corona1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t coronaHandle2;
const osThreadAttr_t corona_attributes2 = {
  .name = "corona2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t coronaHandle3;
const osThreadAttr_t corona_attributes3 = {
  .name = "corona3",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t coronaHandle4;
const osThreadAttr_t corona_attributes4 = {
  .name = "corona4",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void startEventCreator(void *argument);
void srartDispatcher(void *argument);
void StartPolice(void *argument);
void StartAmbulance(void *argument);
void StartFire(void *argument);
void StartCorona(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 1000);
	return len;
}
void InitializePoliceThreads(void);
void InitializeAmbulanceTreads(void);
void InitializeFireTreads(void);
void InitializeCoronaTreads(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semPolice */
  semPoliceHandle = osSemaphoreNew(50, 0, &semPolice_attributes);

  /* creation of semAmbulance */
  semAmbulanceHandle = osSemaphoreNew(50, 0, &semAmbulance_attributes);

  /* creation of semCorona */
  semCoronaHandle = osSemaphoreNew(50, 0, &semCorona_attributes);

  /* creation of semFire */
  semFireHandle = osSemaphoreNew(50, 0, &semFire_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of queue1 */
  queue1Handle = osMessageQueueNew (256, sizeof(uint8_t), &queue1_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of eventCreator */
  eventCreatorHandle = osThreadNew(startEventCreator, NULL, &eventCreator_attributes);

  /* creation of dispatcher */
  dispatcherHandle = osThreadNew(srartDispatcher, NULL, &dispatcher_attributes);

  /* creation of police */
  //policeHandle = osThreadNew(StartPolice, NULL, &police_attributes);

  /* creation of ambulance */
  //ambulanceHandle = osThreadNew(StartAmbulance, NULL, &ambulance_attributes);

  /* creation of fire */
  //fireHandle = osThreadNew(StartFire, NULL, &fire_attributes);

  /* creation of corona */
  //coronaHandle = osThreadNew(StartCorona, NULL, &corona_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  InitializePoliceThreads();
  InitializeAmbulanceTreads();
  InitializeFireTreads();
  InitializeCoronaTreads();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void InitializePoliceThreads(void){
	policeHandle1 = osThreadNew(StartPolice, NULL, &police_attributes1);
	policeHandle2 = osThreadNew(StartPolice, NULL, &police_attributes2);
	policeHandle3 = osThreadNew(StartPolice, NULL, &police_attributes3);
}
void InitializeAmbulanceTreads(void){
	ambulanceHandle1 = osThreadNew(StartAmbulance, NULL, &ambulance_attributes1);
	ambulanceHandle2 = osThreadNew(StartAmbulance, NULL, &ambulance_attributes2);
	ambulanceHandle3 = osThreadNew(StartAmbulance, NULL, &ambulance_attributes3);
	ambulanceHandle4 = osThreadNew(StartAmbulance, NULL, &ambulance_attributes4);
}
void InitializeFireTreads(void){
	fireHandle1 = osThreadNew(StartFire, NULL, &fire_attributes1);
	fireHandle2 = osThreadNew(StartFire, NULL, &fire_attributes2);
}
void InitializeCoronaTreads(void){
	coronaHandle1 = osThreadNew(StartCorona, NULL, &corona_attributes1);
	coronaHandle2 = osThreadNew(StartCorona, NULL, &corona_attributes2);
	coronaHandle3 = osThreadNew(StartCorona, NULL, &corona_attributes3);
	coronaHandle4 = osThreadNew(StartCorona, NULL, &corona_attributes4);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startEventCreator */
/**
* @brief Function implementing the eventCreator thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startEventCreator */
void startEventCreator(void *argument)
{
  /* USER CODE BEGIN startEventCreator */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	uint8_t call = rand()%4;
	uint8_t *pCall = &call;
	//printf("generated call = %d\n\r", call);
	int eventTime = rand()%3000 + 500;
	//printf("%d\n\r", eventTime);
	osMessageQueuePut(queue1Handle, pCall, 0, portMAX_DELAY);
    osDelay(eventTime);
  }
  /* USER CODE END startEventCreator */
}

/* USER CODE BEGIN Header_srartDispatcher */
/**
* @brief Function implementing the dispatcher thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_srartDispatcher */
void srartDispatcher(void *argument)
{
  /* USER CODE BEGIN srartDispatcher */
  /* Infinite loop */
  for(;;)
  {
	  uint8_t recivedCall;
	  osMessageQueueGet(queue1Handle, &recivedCall, 0, portMAX_DELAY);
	  printf("call received #%u\n\r", recivedCall);

	  if 		(recivedCall == 0) {
		osSemaphoreRelease(semPoliceHandle);
	  }	else if (recivedCall == 1) {
		osSemaphoreRelease(semAmbulanceHandle);
	  }	else if (recivedCall == 2) {
		osSemaphoreRelease(semFireHandle);
	  }	else if (recivedCall == 3) {
		osSemaphoreRelease(semCoronaHandle);
	  }
  }
  /* USER CODE END srartDispatcher */
}

/* USER CODE BEGIN Header_StartPolice */
/**
* @brief Function implementing the police thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPolice */
void StartPolice(void *argument)
{
  /* USER CODE BEGIN StartPolice */
	  osThreadId id = osThreadGetId();
	  const char *name = NULL;
	  name = osThreadGetName(id);
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(semPoliceHandle, portMAX_DELAY);
	  printf("%s is out for a mission!!!\n\r", name);
	  uint32_t delayTime = rand()%5000+10000;
	  osDelay(delayTime);
	  printf("%s is back from mission!\n\r", name);
  }
  /* USER CODE END StartPolice */
}

/* USER CODE BEGIN Header_StartAmbulance */
/**
* @brief Function implementing the ambulance thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAmbulance */
void StartAmbulance(void *argument)
{
  /* USER CODE BEGIN StartAmbulance */
	osThreadId id = osThreadGetId();
	const char *name = NULL;
	name = osThreadGetName(id);
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(semAmbulanceHandle, portMAX_DELAY);
	  printf("%s is out for a mission!!!\n\r", name);
	  uint32_t delayTime = rand()%5000+10000;
	  osDelay(delayTime);
	  printf("%s is back from mission!\n\r", name);
  }
  /* USER CODE END StartAmbulance */
}

/* USER CODE BEGIN Header_StartFire */
/**
* @brief Function implementing the fire thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFire */
void StartFire(void *argument)
{
  /* USER CODE BEGIN StartFire */
	osThreadId id = osThreadGetId();
	const char *name = NULL;
	name = osThreadGetName(id);
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(semFireHandle, portMAX_DELAY);
	  printf("%s is out for a mission!!!\n\r", name);
	  uint32_t delayTime = rand()%5000+10000;
	  osDelay(delayTime);
	  printf("%s is back from mission!\n\r", name);
  }
  /* USER CODE END StartFire */
}

/* USER CODE BEGIN Header_StartCorona */
/**
* @brief Function implementing the corona thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCorona */
void StartCorona(void *argument)
{
  /* USER CODE BEGIN StartCorona */
	osThreadId id = osThreadGetId();
	const char *name = NULL;
	name = osThreadGetName(id);
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(semCoronaHandle, portMAX_DELAY);
	  printf("%s is out for a mission!!!\n\r", name);
	  uint32_t delayTime = rand()%5000+10000;
	  osDelay(delayTime);
	  printf("%s is back from mission!\n\r", name);
  }
  /* USER CODE END StartCorona */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
