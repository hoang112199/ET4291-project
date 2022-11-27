/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "string.h"
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
uint32_t NormalTaskExcutionTime_1 = 0;
uint32_t NormalTaskExcutionTime_2 = 0;
uint32_t AboveTaskExcutionTime = 0;
osThreadId NormalTask1Handle;
osThreadId NormalTask2Handle;
osThreadId AboveNormalTaskHandle;
/* USER CODE BEGIN PV */
osMutexId myMutex01Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartNormalTask1(void const * argument);
void StartNormalTask2(void const * argument);
void StartAboveNormalTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	osMutexDef(myMutex01);
	myMutex01Handle = osMutexCreate(osMutex(myMutex01));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of NormalTask1 */
  osThreadDef(NormalTask1, StartNormalTask1, osPriorityNormal, 0, 128);
  NormalTask1Handle = osThreadCreate(osThread(NormalTask1), NULL);

  /* definition and creation of NormalTask2 */
  osThreadDef(NormalTask2, StartNormalTask2, osPriorityNormal, 0, 128);
  NormalTask2Handle = osThreadCreate(osThread(NormalTask2), NULL);

  /* definition and creation of AboveNormalTask */
  osThreadDef(AboveNormalTask, StartAboveNormalTask, osPriorityAboveNormal, 0, 128);
  AboveNormalTaskHandle = osThreadCreate(osThread(AboveNormalTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartNormalTask1 */
/**
  * @brief  Function implementing the NormalTask1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartNormalTask1 */
void StartNormalTask1(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t data[80];
	uint32_t timeStart;
	uint32_t timeEnd;
  /* Infinite loop */
  for(;;)
  {
		
		osMutexWait(myMutex01Handle, osWaitForever);
		timeStart = osKernelSysTick();
		sprintf((char*)data, "Normal task 1 start at %d ms\n",(int)timeStart);
		HAL_UART_Transmit(&huart2, data, strlen((char*)data),10);
		timeEnd = osKernelSysTick();
		NormalTaskExcutionTime_1 += (timeEnd-timeStart);
		sprintf((char*)data, "Normal task 1 running time %d ms\n",(int)NormalTaskExcutionTime_1);
		HAL_UART_Transmit(&huart2, data, strlen((char*)data),10);
		osMutexRelease(myMutex01Handle);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartNormalTask2 */
/**
* @brief Function implementing the NormalTask2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNormalTask2 */
void StartNormalTask2(void const * argument)
{
  /* USER CODE BEGIN StartNormalTask2 */
	uint8_t data[80];
	uint32_t timeStart;
	uint32_t timeEnd;
  /* Infinite loop */
  for(;;)
  {
		
		osMutexWait(myMutex01Handle, osWaitForever);
		timeStart = osKernelSysTick();
		sprintf((char*)data, "Normal task 2 start at %d ms\n",(int)timeStart);
		HAL_UART_Transmit(&huart2, data, strlen((char*)data),10);
		timeEnd = osKernelSysTick();
		NormalTaskExcutionTime_2 += (timeEnd-timeStart);
		sprintf((char*)data, "Normal task 2 running time %d ms\n",(int)NormalTaskExcutionTime_2);
		HAL_UART_Transmit(&huart2, data, strlen((char*)data),10);
		osMutexRelease(myMutex01Handle);
    osDelay(1000);
  }
  /* USER CODE END StartNormalTask2 */
}

/* USER CODE BEGIN Header_StartAboveNormalTask */
/**
* @brief Function implementing the AboveNormalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAboveNormalTask */
void StartAboveNormalTask(void const * argument)
{
  /* USER CODE BEGIN StartAboveNormalTask */
	uint8_t data[80];
	uint32_t timeStart;
	uint32_t timeEnd;
	uint32_t count = 0;
  /* Infinite loop */
  for(;;)
  {
		timeStart = osKernelSysTick();
		osMutexWait(myMutex01Handle, osWaitForever);
		sprintf((char*)data, "Normal Task 1 excution time %d in %d minute\n",(int)NormalTaskExcutionTime_1, count);
		HAL_UART_Transmit(&huart2, data, strlen((char*)data),10);
		sprintf((char*)data, "Normal Task 2 excution time %d in %d minute\n",(int)NormalTaskExcutionTime_2, count);
		HAL_UART_Transmit(&huart2, data, strlen((char*)data),10);
		timeEnd = osKernelSysTick();
		AboveTaskExcutionTime += (timeEnd-timeStart);
		sprintf((char*)data, "Above Task excution time %d in %d minute\n",(int)AboveTaskExcutionTime, count);
		HAL_UART_Transmit(&huart2, data, strlen((char*)data),10);
		count++;
		osMutexRelease(myMutex01Handle);
		osDelay(60000);
  }
  /* USER CODE END StartAboveNormalTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
