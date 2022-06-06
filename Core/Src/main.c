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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//enum commandSelection{spr1on,spr2on,spr3on,spr1off,spr2off,spr3off, wait5};

char UARTinput[10] = {0};

char scene1[6][10] = {"spr1on","wait5","spr1off","spr2on","wait5","spr2off"};
char scene2[7][10] = {"spr3on","wait5","spr3off","wait5","spr3on","wait5","spr3off"};
char scene3[6][10] = {"spr3on","spr2on","wait5","spr3off","wait5","spr2off"};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for scene1 */
osThreadId_t scene1Handle;
const osThreadAttr_t scene1_attributes = {
  .name = "scene1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for scene2 */
osThreadId_t scene2Handle;
const osThreadAttr_t scene2_attributes = {
  .name = "scene2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for scene3 */
osThreadId_t scene3Handle;
const osThreadAttr_t scene3_attributes = {
  .name = "scene3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for scene1Queue */
osMessageQueueId_t scene1QueueHandle;
const osMessageQueueAttr_t scene1Queue_attributes = {
  .name = "scene1Queue"
};
/* Definitions for scene2Queue */
osMessageQueueId_t scene2QueueHandle;
const osMessageQueueAttr_t scene2Queue_attributes = {
  .name = "scene2Queue"
};
/* Definitions for scene3Queue */
osMessageQueueId_t scene3QueueHandle;
const osMessageQueueAttr_t scene3Queue_attributes = {
  .name = "scene3Queue"
};
/* Definitions for sprinkler1Mutex */
osMutexId_t sprinkler1MutexHandle;
const osMutexAttr_t sprinkler1Mutex_attributes = {
  .name = "sprinkler1Mutex"
};
/* Definitions for sprinkler2Mutex */
osMutexId_t sprinkler2MutexHandle;
const osMutexAttr_t sprinkler2Mutex_attributes = {
  .name = "sprinkler2Mutex"
};
/* Definitions for sprinkler3Mutex */
osMutexId_t sprinkler3MutexHandle;
const osMutexAttr_t sprinkler3Mutex_attributes = {
  .name = "sprinkler3Mutex"
};
/* Definitions for scene1Mutex */
osMutexId_t scene1MutexHandle;
const osMutexAttr_t scene1Mutex_attributes = {
  .name = "scene1Mutex"
};
/* Definitions for scene2Mutex */
osMutexId_t scene2MutexHandle;
const osMutexAttr_t scene2Mutex_attributes = {
  .name = "scene2Mutex"
};
/* Definitions for scene3Mutex */
osMutexId_t scene3MutexHandle;
const osMutexAttr_t scene3Mutex_attributes = {
  .name = "scene3Mutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

/* USER CODE BEGIN PFP */
void InitLED()
{
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	//led PA0
	GPIOA -> MODER |= GPIO_MODER_MODER0_0;
	GPIOA -> MODER &= ~(GPIO_MODER_MODER0_1);
	//

	//led PA1
	GPIOA -> MODER |= GPIO_MODER_MODER1_0;
	GPIOA -> MODER &= ~(GPIO_MODER_MODER1_1);
	//

	//led PA4
	GPIOA -> MODER |= GPIO_MODER_MODER4_0;
	GPIOA -> MODER &= ~(GPIO_MODER_MODER4_1);
	//
}

void sprinkler1On()
{
	GPIOA->ODR |= GPIO_ODR_0;
}
void sprinkler2On()
{
	GPIOA->ODR |= GPIO_ODR_1;
}
void sprinkler3On()
{
	GPIOA->ODR |= GPIO_ODR_4;
}
void sprinkler1Off()
{
	GPIOA->ODR &= ~GPIO_ODR_0;
}
void sprinkler2Off()
{
	GPIOA->ODR &= ~GPIO_ODR_1;
}
void sprinkler3Off()
{
	GPIOA->ODR &= ~GPIO_ODR_4;
}
void executeScene(int sceneNum)
{
	int itNum = 0;
	char scene[10];
	osMessageQueueId_t handler;
	switch (sceneNum)
	{
	case 1:
		handler = scene1QueueHandle;
		itNum = 6;
		osMutexAcquire(scene1MutexHandle, osWaitForever);
		break;
	case 2:
		handler = scene2QueueHandle;
		itNum = 7;
		osMutexAcquire(scene2MutexHandle, osWaitForever);
		break;
	case 3:
		handler = scene3QueueHandle;
		itNum = 6;
		osMutexAcquire(scene3MutexHandle, osWaitForever);
		break;
	default:
		return;
		break;
	}

	for (int i = 0; i<itNum; i++)
			 {
		osMessageQueueGet(handler, (void *)scene, 1, osWaitForever);
			    if (strcmp(scene,"spr1on")==0)
			    {
			    	osMutexAcquire(sprinkler1MutexHandle, osWaitForever);
			  	    sprinkler1On();
			    }
			  	if (strcmp(scene,"spr1off")==0)
			  	{
			  		sprinkler1Off();
			  		osMutexRelease(sprinkler1MutexHandle);
			  	}
			  	if (strcmp(scene,"spr2on")==0)
			  	{
			  		osMutexAcquire(sprinkler2MutexHandle, osWaitForever);
			  		sprinkler2On();
			  	}
			  	if (strcmp(scene,"spr2off")==0)
			  	{
			  		sprinkler2Off();
			  		osMutexRelease(sprinkler2MutexHandle);
			  	}
			  	if (strcmp(scene,"spr3on")==0)
			  	{
			  		osMutexAcquire(sprinkler3MutexHandle, osWaitForever);
			  		sprinkler3On();
			  	}
			  	if (strcmp(scene,"spr3off")==0)
			  	{
			    	sprinkler3Off();
			        osMutexRelease(sprinkler3MutexHandle);
			   	}
			  	if (strcmp(scene,"wait5")==0)
			  	{
			  	  	  	  osDelay(5000);
			  	}
	  }


	 switch (sceneNum)
	 	{
	 	case 1:
	 		osMutexRelease(scene1MutexHandle);
	 		break;
	 	case 2:
	 		osMutexRelease(scene2MutexHandle);
	 		break;
	 	case 3:
	 		osMutexRelease(scene3MutexHandle);
	 		break;
	 	}
}


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
  InitLED();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of sprinkler1Mutex */
  sprinkler1MutexHandle = osMutexNew(&sprinkler1Mutex_attributes);

  /* creation of sprinkler2Mutex */
  sprinkler2MutexHandle = osMutexNew(&sprinkler2Mutex_attributes);

  /* creation of sprinkler3Mutex */
  sprinkler3MutexHandle = osMutexNew(&sprinkler3Mutex_attributes);

  /* creation of scene1Mutex */
  scene1MutexHandle = osMutexNew(&scene1Mutex_attributes);

  /* creation of scene2Mutex */
  scene2MutexHandle = osMutexNew(&scene2Mutex_attributes);

  /* creation of scene3Mutex */
  scene3MutexHandle = osMutexNew(&scene3Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of scene1Queue */
  scene1QueueHandle = osMessageQueueNew (16, sizeof(char[10]), &scene1Queue_attributes);

  /* creation of scene2Queue */
  scene2QueueHandle = osMessageQueueNew (16, sizeof(char[10]), &scene2Queue_attributes);

  /* creation of scene3Queue */
  scene3QueueHandle = osMessageQueueNew (16, sizeof(char[10]), &scene3Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of scene1 */
  //scene1Handle = osThreadNew(StartTask02, NULL, &scene1_attributes);

  /* creation of scene2 */
  //scene2Handle = osThreadNew(StartTask03, NULL, &scene2_attributes);

  /* creation of scene3 */
  //scene3Handle = osThreadNew(StartTask04, NULL, &scene3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //osThreadTerminate(scene1Handle);
  //osThreadTerminate(scene2Handle);
  //osThreadTerminate(scene3Handle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart2.Init.BaudRate = 38400;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
	memset(UARTinput, 0, 10);
    HAL_UART_Receive(&huart2, (void *)UARTinput, 10, 100);
    if(strcmp(UARTinput,"1")==0)
    {
    	for(int i = 0; i<6; i++)
    	{
    	osMessageQueuePut(scene1QueueHandle, scene1[i], 1, osWaitForever);
    	}
		scene1Handle = osThreadNew(StartTask02, NULL, &scene1_attributes);
    }
    if(strcmp(UARTinput,"2")==0)
     {
    	for(int i = 0; i<7; i++)
    	    	{
    	    	osMessageQueuePut(scene2QueueHandle, scene2[i], 1, osWaitForever);
    	    	}
    	scene2Handle = osThreadNew(StartTask03, NULL, &scene2_attributes);
     }
    if(strcmp(UARTinput,"3")==0)
     {
    	for(int i = 0; i<6; i++)
    	    	{
    	    	osMessageQueuePut(scene3QueueHandle, scene3[i], 1, osWaitForever);
    	    	}
        scene3Handle = osThreadNew(StartTask04, NULL, &scene3_attributes);
     }
    HAL_UART_Transmit(&huart2, (void *)UARTinput, strlen(UARTinput), 100);
    osDelay(10);
  }
  osThreadTerminate(osThreadGetId());
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the spr1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	  executeScene(1);
	  osThreadTerminate(osThreadGetId());
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the spr2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	  executeScene(2);
	  osThreadTerminate(osThreadGetId());
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the scene3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	executeScene(3);
	osThreadTerminate(osThreadGetId());
  /* USER CODE END StartTask04 */
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

