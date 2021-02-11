/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for UART1_to_Queue */
osThreadId_t UART1_to_QueueHandle;
const osThreadAttr_t UART1_to_Queue_attributes = {
  .name = "UART1_to_Queue",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Queue_to_Motor */
osThreadId_t Queue_to_MotorHandle;
const osThreadAttr_t Queue_to_Motor_attributes = {
  .name = "Queue_to_Motor",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for UART3_to_Queue */
osThreadId_t UART3_to_QueueHandle;
const osThreadAttr_t UART3_to_Queue_attributes = {
  .name = "UART3_to_Queue",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Queue_to_SH */
osThreadId_t Queue_to_SHHandle;
const osThreadAttr_t Queue_to_SH_attributes = {
  .name = "Queue_to_SH",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Input_UART */
osMessageQueueId_t Input_UARTHandle;
const osMessageQueueAttr_t Input_UART_attributes = {
  .name = "Input_UART"
};
/* Definitions for UART1_Bytes */
osSemaphoreId_t UART1_BytesHandle;
const osSemaphoreAttr_t UART1_Bytes_attributes = {
  .name = "UART1_Bytes"
};
/* Definitions for UART3_Bytes */
osSemaphoreId_t UART3_BytesHandle;
const osSemaphoreAttr_t UART3_Bytes_attributes = {
  .name = "UART3_Bytes"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void Start_UART1_to_Queue(void *argument);
void StartQueue_to_Motor(void *argument);
void StartUART3_to_Queue(void *argument);
void StartQueue_to_SH(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RxUART1[5] = "";
uint8_t RxUART3[5] = "";
_Bool first_TIM2_ISR = 0;
_Bool Time_over = 0;
_Bool UART1_active = 0;
_Bool UART3_active = 0;
uint8_t wait_to_start = 0;

uint32_t ADC_Read[11] = {0,0,0,0,0,0,0,0,0,0,0};
uint8_t ADC_Val[1] = "\0";

uint8_t Speed_Data [201][9] = 	{{172,48,130,1,100,252,24,154,173},
								{172,48,130,1,100,252,34,90,173},
								{172,48,130,1,100,252,44,69,173},
								{172,48,130,1,100,252,54,166,173},
								{172,48,130,1,100,252,64,131,173},
								{172,48,130,1,100,252,74,253,173},
								{172,48,130,1,100,252,84,127,173},
								{172,48,130,1,100,252,94,1,173},
								{172,48,130,1,100,252,104,98,173},
								{172,48,130,1,100,252,114,129,173},
								{172,48,130,1,100,252,124,158,173},
								{172,48,130,1,100,252,134,148,173},
								{172,48,130,1,100,252,144,212,173},
								{172,48,130,1,100,252,154,170,173},
								{172,48,130,1,100,252,164,11,173},
								{172,48,130,1,100,252,174,117,173},
								{172,48,130,1,100,252,184,53,173},
								{172,48,130,1,100,252,194,179,173},
								{172,48,130,1,100,252,203,47,173},
								{172,48,130,1,100,252,214,79,173},
								{172,48,130,1,100,252,224,44,173},
								{172,48,130,1,100,252,234,82,173},
								{172,48,130,1,100,252,244,208,173},
								{172,48,130,1,100,252,254,174,173},
								{172,48,130,1,100,253,8,195,173},
								{172,48,130,1,100,253,18,32,173},
								{172,48,130,1,100,253,28,63,173},
								{172,48,130,1,100,253,38,255,173},
								{172,48,130,1,100,253,48,191,173},
								{172,48,130,1,100,253,58,193,173},
								{172,48,130,1,100,253,68,38,173},
								{172,48,130,1,100,253,78,88,173},
								{172,48,130,1,100,253,88,24,173},
								{172,48,130,1,100,253,98,216,173},
								{172,48,130,1,100,253,108,199,173},
								{172,48,130,1,100,253,118,36,173},
								{172,48,130,1,100,253,128,141,173},
								{172,48,130,1,100,253,138,243,173},
								{172,48,130,1,100,253,148,113,173},
								{172,48,130,1,100,253,158,15,173},
								{172,48,130,1,100,253,168,108,173},
								{172,48,130,1,100,253,178,143,173},
								{172,48,130,1,100,253,188,144,173},
								{172,48,130,1,100,253,198,22,173},
								{172,48,130,1,100,253,208,86,173},
								{172,48,130,1,100,253,218,40,173},
								{172,48,130,1,100,253,228,137,173},
								{172,48,130,1,100,253,238,247,173},
								{172,48,130,1,100,253,248,183,173},
								{172,48,130,1,100,254,2,232,173},
								{172,48,130,1,100,254,12,247,173},
								{172,48,130,1,100,254,22,20,173},
								{172,48,130,1,100,254,32,119,173},
								{172,48,130,1,100,254,42,9,173},
								{172,48,130,1,100,254,52,139,173},
								{172,48,130,1,100,254,62,245,173},
								{172,48,130,1,100,254,72,208,173},
								{172,48,130,1,100,254,82,51,173},
								{172,48,130,1,100,254,92,44,173},
								{172,48,130,1,100,254,102,236,173},
								{172,48,130,1,100,254,111,112,173},
								{172,48,130,1,100,254,122,210,173},
								{172,48,130,1,100,254,132,185,173},
								{172,48,130,1,100,254,142,199,173},
								{172,48,130,1,100,254,152,135,173},
								{172,48,130,1,100,254,162,71,173},
								{172,48,130,1,100,254,171,219,173},
								{172,48,130,1,100,254,182,187,173},
								{172,48,130,1,100,254,192,158,173},
								{172,48,130,1,100,254,202,224,173},
								{172,48,130,1,100,254,212,98,173},
								{172,48,130,1,100,254,222,28,173},
								{172,48,130,1,100,254,232,127,173},
								{172,48,130,1,100,254,242,156,173},
								{172,48,130,1,100,254,252,131,173},
								{172,48,130,1,100,255,6,77,173},
								{172,48,130,1,100,255,16,13,173},
								{172,48,130,1,100,255,26,115,173},
								{172,48,130,1,100,255,36,210,173},
								{172,48,130,1,100,255,45,78,173},
								{172,48,130,1,100,255,56,236,173},
								{172,48,130,1,100,255,66,106,173},
								{172,48,130,1,100,255,76,117,173},
								{172,48,130,1,100,255,86,150,173},
								{172,48,130,1,100,255,96,245,173},
								{172,48,130,1,100,255,106,139,173},
								{172,48,130,1,100,255,116,9,173},
								{172,48,130,1,100,255,126,119,173},
								{172,48,130,1,100,255,136,222,173},
								{172,48,130,1,100,255,146,61,173},
								{172,48,130,1,100,255,156,34,173},
								{172,48,130,1,100,255,166,226,173},
								{172,48,130,1,100,255,176,162,173},
								{172,48,130,1,100,255,186,220,173},
								{172,48,130,1,100,255,196,59,173},
								{172,48,130,1,100,255,206,69,173},
								{172,48,130,1,100,255,216,5,173},
								{172,48,130,1,100,255,226,197,173},
								{172,48,130,1,100,255,236,218,173},
								{172,48,130,1,100,255,246,57,173},
								{172,48,130,1,100,0,0,17,173},
								{172,48,130,1,100,0,10,111,173},
								{172,48,130,1,100,0,20,237,173},
								{172,48,130,1,100,0,30,147,173},
								{172,48,130,1,100,0,40,240,173},
								{172,48,130,1,100,0,50,19,173},
								{172,48,130,1,100,0,60,12,173},
								{172,48,130,1,100,0,70,138,173},
								{172,48,130,1,100,0,80,202,173},
								{172,48,130,1,100,0,90,180,173},
								{172,48,130,1,100,0,100,21,173},
								{172,48,130,1,100,0,110,107,173},
								{172,48,130,1,100,0,120,43,173},
								{172,48,130,1,100,0,130,33,173},
								{172,48,130,1,100,0,140,62,173},
								{172,48,130,1,100,0,150,221,173},
								{172,48,130,1,100,0,160,190,173},
								{172,48,130,1,100,0,170,192,173},
								{172,48,130,1,100,0,180,66,173},
								{172,48,130,1,100,0,190,60,173},
								{172,48,130,1,100,0,200,25,173},
								{172,48,130,1,100,0,210,250,173},
								{172,48,130,1,100,0,220,229,173},
								{172,48,130,1,100,0,230,37,173},
								{172,48,130,1,100,0,240,101,173},
								{172,48,130,1,100,0,250,27,173},
								{172,48,130,1,100,1,4,180,173},
								{172,48,130,1,100,1,14,202,173},
								{172,48,130,1,100,1,24,138,173},
								{172,48,130,1,100,1,34,74,173},
								{172,48,130,1,100,1,44,85,173},
								{172,48,130,1,100,1,54,182,173},
								{172,48,130,1,100,1,64,147,173},
								{172,48,130,1,100,1,74,237,173},
								{172,48,130,1,100,1,84,111,173},
								{172,48,130,1,100,1,94,17,173},
								{172,48,130,1,100,1,104,114,173},
								{172,48,130,1,100,1,114,145,173},
								{172,48,130,1,100,1,124,142,173},
								{172,48,130,1,100,1,134,132,173},
								{172,48,130,1,100,1,144,196,173},
								{172,48,130,1,100,1,154,186,173},
								{172,48,130,1,100,1,164,27,173},
								{172,48,130,1,100,1,174,101,173},
								{172,48,130,1,100,1,184,37,173},
								{172,48,130,1,100,1,194,163,173},
								{172,48,130,1,100,1,204,188,173},
								{172,48,130,1,100,1,214,95,173},
								{172,48,130,1,100,1,224,60,173},
								{172,48,130,1,100,1,234,66,173},
								{172,48,130,1,100,1,244,192,173},
								{172,48,130,1,100,1,254,190,173},
								{172,48,130,1,100,2,8,66,173},
								{172,48,130,1,100,2,18,161,173},
								{172,48,130,1,100,2,28,190,173},
								{172,48,130,1,100,2,38,126,173},
								{172,48,130,1,100,2,48,62,173},
								{172,48,130,1,100,2,58,64,173},
								{172,48,130,1,100,2,68,167,173},
								{172,48,130,1,100,2,78,217,173},
								{172,48,130,1,100,2,88,153,173},
								{172,48,130,1,100,2,98,89,173},
								{172,48,130,1,100,2,108,70,173},
								{172,48,130,1,100,2,118,165,173},
								{172,48,130,1,100,2,128,12,173},
								{172,48,130,1,100,2,138,114,173},
								{172,48,130,1,100,2,148,240,173},
								{172,48,130,1,100,2,158,142,173},
								{172,48,130,1,100,2,168,237,173},
								{172,48,130,1,100,2,178,14,173},
								{172,48,130,1,100,2,188,17,173},
								{172,48,130,1,100,2,198,151,173},
								{172,48,130,1,100,2,208,215,173},
								{172,48,130,1,100,2,218,169,173},
								{172,48,130,1,100,2,228,8,173},
								{172,48,130,1,100,2,238,118,173},
								{172,48,130,1,100,2,248,54,173},
								{172,48,130,1,100,3,2,248,173},
								{172,48,130,1,100,3,12,231,173},
								{172,48,130,1,100,3,22,4,173},
								{172,48,130,1,100,3,32,103,173},
								{172,48,130,1,100,3,42,25,173},
								{172,48,130,1,100,3,52,155,173},
								{172,48,130,1,100,3,62,229,173},
								{172,48,130,1,100,3,72,192,173},
								{172,48,130,1,100,3,82,35,173},
								{172,48,130,1,100,3,92,60,173},
								{172,48,130,1,100,3,102,252,173},
								{172,48,130,1,100,3,112,188,173},
								{172,48,130,1,100,3,122,194,173},
								{172,48,130,1,100,3,132,169,173},
								{172,48,130,1,100,3,142,215,173},
								{172,48,130,1,100,3,152,151,173},
								{172,48,130,1,100,3,162,87,173},
								{172,48,130,1,100,3,174,244,173},
								{172,48,130,1,100,3,182,171,173},
								{172,48,130,1,100,3,192,142,173},
								{172,48,130,1,100,3,202,240,173},
								{172,48,130,1,100,3,212,114,173},
								{172,48,130,1,100,3,222,12,173},
								{172,48,130,1,100,3,232,111,173}};


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{

		HAL_UART_Receive_IT(&huart1, RxUART1, 1);
		osSemaphoreRelease(UART1_BytesHandle);

		//Timer aktuell nur für besseres Debugging
		/*if(__HAL_TIM_GET_COUNTER(&htim2) == 0) //Falls Timer nicht aktiv -> aktivieren
		{
			MX_TIM2_Init();
			first_TIM2_ISR = 0;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		if(__HAL_TIM_GET_COUNTER(&htim2) != 0) //Falls Timer aktiv -> zurücksetzen
		{
			__HAL_TIM_SET_COUNTER(&htim2, 1);
		}*/

	}
	else if (huart->Instance == USART2)
	{

	}
	else if (huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3, RxUART3, 1);
		osSemaphoreRelease(UART3_BytesHandle);

		//Timer aktuell nur für besseres Debugging
		/*if(__HAL_TIM_GET_COUNTER(&htim2) == 0) //Falls Timer nicht aktiv -> aktivieren
		{
			MX_TIM2_Init();
			first_TIM2_ISR = 0;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		if(__HAL_TIM_GET_COUNTER(&htim2) != 0) //Falls Timer aktiv -> zurücksetzen
		{
			__HAL_TIM_SET_COUNTER(&htim2, 1);
		}*/
	}
}


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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //Start des Programms erst nach mehr als 5 Mikrosekunden Ruhe auf dem Empfangspin des UART1
  /*uint8_t Text[10] = "Boot";
  HAL_UART_Transmit(&huart2, Text, sizeof(Text),10);
  //for(wait_to_start = 0; wait_to_start < ; wait_to_start++)
  //{
	  MX_TIM2_Init();
	  HAL_TIM_Base_Start_IT(&htim2);
	  while(!Time_over)
	  {
		  if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
		  {
			  __HAL_TIM_SET_COUNTER(&htim2, 1);
			  wait_to_start = 0;
		  }
	  }
	  Time_over = 0;
  //}
  //Start UART1 Interrupt*/
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, RxUART1, 1);
  HAL_UART_Receive_IT(&huart3, RxUART1, 1);
  uint8_t Text1[10] = "Start";
  HAL_UART_Transmit(&huart2, Text1, sizeof(Text1),10);

  HAL_UART_Receive_IT(&huart1, RxUART1, 1);
  HAL_UART_Receive_IT(&huart3, RxUART1, 1);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of UART1_Bytes */
  UART1_BytesHandle = osSemaphoreNew(1, 1, &UART1_Bytes_attributes);

  /* creation of UART3_Bytes */
  UART3_BytesHandle = osSemaphoreNew(1, 1, &UART3_Bytes_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Input_UART */
  Input_UARTHandle = osMessageQueueNew (50, sizeof(uint8_t), &Input_UART_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART1_to_Queue */
  UART1_to_QueueHandle = osThreadNew(Start_UART1_to_Queue, NULL, &UART1_to_Queue_attributes);

  /* creation of Queue_to_Motor */
  Queue_to_MotorHandle = osThreadNew(StartQueue_to_Motor, NULL, &Queue_to_Motor_attributes);

  /* creation of UART3_to_Queue */
  UART3_to_QueueHandle = osThreadNew(StartUART3_to_Queue, NULL, &UART3_to_Queue_attributes);

  /* creation of Queue_to_SH */
  Queue_to_SHHandle = osThreadNew(StartQueue_to_SH, NULL, &Queue_to_SH_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* 1 Millisekunde pro Intervall*/
  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_UART1_to_Queue */
/**
* @brief Function implementing the UART1_to_Queue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART1_to_Queue */
void Start_UART1_to_Queue(void *argument)
{
  /* USER CODE BEGIN Start_UART1_to_Queue */
  /* Warte auf neues Byte von UART1 und transferiere es in die MessageQueue*/

  /* Infinite loop */
  for(;;)
  {

	  osSemaphoreAcquire(UART1_BytesHandle, osWaitForever);
	  UART3_active = 0;
	  UART1_active = 1;
	  osMessageQueuePut(Input_UARTHandle, &RxUART1, 0U, 0);
  }
  /* USER CODE END Start_UART1_to_Queue */
}

/* USER CODE BEGIN Header_StartQueue_to_Motor */
/**
* @brief Function implementing the Queue_to_Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartQueue_to_Motor */
void StartQueue_to_Motor(void *argument)
{
  /* USER CODE BEGIN StartQueue_to_Motor */
  /* Sobald CPU-Recourcen frei sind und ein neues Byte in der MessageQueue steht, lese es.
   * Schreibe es in einen Buffer
   * Sende bei gefundenem ":" eine Meldung an UART2*/
  uint8_t Buffer[10] = "\0";
  uint8_t Debug[1] = "\0";

  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGetCount(Input_UARTHandle))
	  {

		  for(int i = 1; i <= 9; i++)
		  {
			  Buffer[i-1] = Buffer[i];
		  }
		  osMessageQueueGet(Input_UARTHandle, &Buffer[9], NULL, 0U);
		  Debug[0] = Buffer[9];

		  if(	Buffer[8] == 100 &&
				Buffer[7] == 1 &&
				Buffer[6] == 130 &&
				Buffer[5] == 48 &&
				Buffer[4] == 172	)
		  {
			  HAL_UART_Transmit(&huart2, ADC_Val, sizeof(ADC_Val), 1);
		  }


		  if(UART1_active)
		  {
			  HAL_UART_Transmit(&huart3, Debug, sizeof(Debug), 1);
		  }
		  if(UART3_active)
		  {
			  HAL_UART_Transmit(&huart1, Debug, sizeof(Debug), 1);
		  }

	  }
  }
  /* USER CODE END StartQueue_to_Motor */
}

/* USER CODE BEGIN Header_StartUART3_to_Queue */
/**
* @brief Function implementing the UART3_to_Queue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART3_to_Queue */
void StartUART3_to_Queue(void *argument)
{
  /* USER CODE BEGIN StartUART3_to_Queue */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(UART3_BytesHandle, osWaitForever);
	  UART1_active = 0;
	  UART3_active = 1;
	  osMessageQueuePut(Input_UARTHandle, &RxUART3, 0U, 0);
  }
  /* USER CODE END StartUART3_to_Queue */
}

/* USER CODE BEGIN Header_StartQueue_to_SH */
/**
* @brief Function implementing the Queue_to_SH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartQueue_to_SH */
void StartQueue_to_SH(void *argument)
{
  /* USER CODE BEGIN StartQueue_to_SH */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartQueue_to_SH */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /*Dient aktuell nur zum Start des Controllers und zu Debuggingzwecken für die Übersichtlichkeit auf UART2
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2)
  {
	 HAL_ADC_Start_DMA(&hadc1, &ADC_Read, 11);
	 ADC_Val[0] = ADC_Read[9]*200/4294967296;
	 /*if(!first_TIM2_ISR)
	 {
		 first_TIM2_ISR = 1;
	 }
	 else
	 {
		 Time_over = 1;
		 //uint8_t Text2[1] = "\n";
		 //osMessageQueuePut(Input_UART1Handle, &Text2[0], 0U, 0);
		 //HAL_UART_Transmit(&huart2, Text2, sizeof(Text2),10);
		 //osMessageQueueReset(Input_UART1Handle);
		 HAL_TIM_Base_Stop_IT(&htim2);
		 first_TIM2_ISR = 0;
	 }*/

  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
