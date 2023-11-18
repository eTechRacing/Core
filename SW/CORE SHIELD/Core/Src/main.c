/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 	3				//ADC buffer length
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart4;

/* Definitions for Maquina_Estats */
osThreadId_t Maquina_EstatsHandle;
const osThreadAttr_t Maquina_Estats_attributes = {
  .name = "Maquina_Estats",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for CAN_TX */
osThreadId_t CAN_TXHandle;
const osThreadAttr_t CAN_TX_attributes = {
  .name = "CAN_TX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for USB_TASK */
osThreadId_t USB_TASKHandle;
const osThreadAttr_t USB_TASK_attributes = {
  .name = "USB_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//CAN variables
CAN_RxHeaderTypeDef 	RxHeader; 					//CAN Bus Transmit Header
CAN_TxHeaderTypeDef 	TxHeader; 					//CAN Bus Receive Header
uint8_t 				TxData[8];  				//CAN Bus Receive Buffer
uint8_t 				RxData[8];  				//CAN Bus Receive Buffer
CAN_FilterTypeDef 		canfil; 					//CAN Bus Filter
uint32_t 				TxMailbox; 					//CAN Bus Mail box variable.


//ADC variables
uint16_t LECTURES_ADC[ADC_BUF_LEN];					//Where the data from the ADC is located, is the name of the buffer used by the ADC
													//The number inside the claudators is the amount of channels used by the ADC
char USB_TX[10];									//Buffer where the USB data is stored to be sent
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
void Maquina_estats(void *argument);
void CAN_Transmit(void *argument);
void usb_data(void *argument);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)LECTURES_ADC, ADC_BUF_LEN); //ADC with DMA initialization.
  	  	  	  	  	  	  	  	  	  	  	  	  		//(&hadc1): The function uses the ADC 1
	  	  	  	  										//LECTURES_ADC: It saves all the read data in the buffer
  	  	  	  	  	  	  	  	  	  	  	  	  		//"ADC_BUF_LEN": Number of read channels by the ADC..
//Receiving filter configuration
  canfil.FilterBank = 0;								//This refers to which filter is being configured. On this case is the filter number 0
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;			//FilterMode: How are we filtering the incoming messages. Only the messages that coincide with the mask and the filter are accepted
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;			//Defines at which FIFO is this filter being configured to. In this case is the FIFO 0.
  canfil.FilterIdHigh = 0x0000;							//MSB: Most Significant Bit. When it's in 0, accepts all the messages
  canfil.FilterIdLow = 0x0000;							//LSB: Least Significant Bit. When it's in 0, accepts all the messages
  canfil.FilterMaskIdHigh = 0x0000;						//Most Significant Bit of the mask. When it's in 0, accepts all the messages
  canfil.FilterMaskIdLow = 0x0000;						//Least Significant Bit of the mask. When it's in 0, accepts all the messages
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;			//Defines the Filter Scale. It will use the 32 bits of the mask and the identifier
  canfil.FilterActivation = ENABLE;						//This activates the filter as it is enable
  canfil.SlaveStartFilterBank = 14;						//Indicates the first filter slave number. In this case it is the principal filter.

//CAN controller configuration
  if(HAL_CAN_ConfigFilter(&hcan1,&canfil) != HAL_OK) 	//Configures the CAN controller filter
  {
  	Error_Handler();									//If the action goes wrong, call the Error_Handler function
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)					//Initiates the CAN controller
  {
  	Error_Handler();									//If the action goes wrong, call the Error_Handler function
  }

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) //It activates the FIFO pending messages and empty mailboxes interruptions
      {
      	Error_Handler();								//If the action goes wrong, call the Error_Handler function
      }

//Transmission configuration
  TxHeader.DLC = 8; 									//Number of bites to be transmitted max- 8. DLC: Data Length Code
  TxHeader.IDE = CAN_ID_STD;							//IDE: Identifier Extension. ID_STD: Standard Identifier. Dominant(0) = 11 bit ID, Recessive(1) = 29 bit ID
  TxHeader.RTR = CAN_RTR_DATA;							//RTR: Remote Transmission Request, Dominant(0) = Data frame, Recessive (1) = Remote Frame. Type of trace
  TxHeader.StdId = 0x321;								//Standard identifier ID
  TxHeader.TransmitGlobalTime = DISABLE;				//A temporal mark in the CAN message is not added
  TxData[0] = 1;										//Sent data. The TxData is the buffer where the data is saved
  TxData[1] = 2;										//Sent data. The TxData is the buffer where the data is saved
  TxData[2] = 3;										//Sent data. The TxData is the buffer where the data is saved
  TxData[3] = 4;										//Sent data. The TxData is the buffer where the data is saved
  TxData[4] = 5;										//Sent data. The TxData is the buffer where the data is saved
  TxData[5] = 6;										//Sent data. The TxData is the buffer where the data is saved
  TxData[6] = 7;										//Sent data. The TxData is the buffer where the data is saved
  TxData[7] = 8;										//Sent data. The TxData is the buffer where the data is saved
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
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
  /* creation of Maquina_Estats */
  Maquina_EstatsHandle = osThreadNew(Maquina_estats, NULL, &Maquina_Estats_attributes);

  /* creation of CAN_TX */
  CAN_TXHandle = osThreadNew(CAN_Transmit, NULL, &CAN_TX_attributes);

  /* creation of USB_TASK */
  USB_TASKHandle = osThreadNew(usb_data, NULL, &USB_TASK_attributes);

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
  while (1)												//Main while
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)		//Completed CAN transmission function
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);								//It changes the LED state
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)			//Received CAN function
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);		//Takes the data from the CAN channel
																		//&hadc1: The function uses the ADC 1
																		//CAN_RX_FIFO0: Where the received data is located, a FIFO, specifically the FIFO 0
																		//&RxHeader: CAN Bus Transmit Header
																		//RxData: The CAN Bus Receive Buffer
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);								//It changes the LED state
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)					//Function called each time the ADC finished the conversions
	  	  {

	  	    sprintf(USB_TX, "%d %d %d\n", LECTURES_ADC[0], LECTURES_ADC[1],LECTURES_ADC[2]); 		//Creates an string from each read value
	  	    																						//USB_TX: Is the buffer where the read and operated data is stored
	  	    																						//%d\: Defines the type of variable, one for each saved variable
	  	    																						//"LECTURES_ADC[X]": Defines each variable, it is defined by the position in the buffer.
	  	    																						//Vbat channel works between 1.65V and 3.6V
	  	    CDC_Transmit_FS((uint8_t*)USB_TX, strlen(USB_TX));										//Sends the USB_TX buffer through USB
	  	  }
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Maquina_estats */
/**
  * @brief  Function implementing the Maquina_Estats thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Maquina_estats */
void Maquina_estats(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)												//Infinite loop
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CAN_Transmit */
/**
* @brief Function implementing the CAN_TX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Transmit */
void CAN_Transmit(void *argument)
{
  /* USER CODE BEGIN CAN_Transmit */
  /* Infinite loop */
  for(;;)															//Infinite loop
  {
	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);	//Adds a message to the sending queue of the CAN controller
	  osDelay(500);													//Freertos delay of 500ms
	  TxData[7] = TxData[7] + 1;									//Adds 1 to the seventh element of the TxData buffer each time the loop is repeated
	  osDelay(1);													//Freertos delay of 1ms
  }
  /* USER CODE END CAN_Transmit */
}

/* USER CODE BEGIN Header_usb_data */
/**
* @brief Function implementing the USB_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usb_data */
void usb_data(void *argument)
{
  /* USER CODE BEGIN usb_data */
  /* Infinite loop */
  for(;;)												//Infinite loop
  {
    osDelay(1);
  }
  /* USER CODE END usb_data */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
