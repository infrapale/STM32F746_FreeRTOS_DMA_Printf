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
#include "task.h"
#include "sms.h"
#include "msg.h"
#include "retarget.h"
#include "stdio.h"
#include "string.h"
#include "globals.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define CONSOLE_UART huart3
#define SMS_UART     huart2

#define WAIT_UART_READY(__UART__) while(HAL_UART_GetState(&__UART__) != HAL_UART_STATE_READY) osThreadYield();

#define UART_DMA_TRANSMIT(__UART__, __BPTR__)  	if(HAL_UART_Transmit_DMA(&(__UART__), (uint8_t*)(__BPTR__), strlen((char *)(__BPTR__))) != HAL_OK) \
		{\
		}


#define SMS_DMA_TRANSMIT(__BPTR__)  \
		WAIT_UART_READY(SMS_UART); \
		if(HAL_UART_Transmit_DMA(&(SMS_UART), (uint8_t*)(__BPTR__), strlen((char *)(__BPTR__))) != HAL_OK) \
		{\
		}\
		WAIT_UART_READY(SMS_UART);

#define SET_TICK_LIMIT(__CNTR__, __TICKS__) (__CNTR__) = HAL_GetTick() + (__TICKS__)


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ProcessSmsTask */
osThreadId_t ProcessSmsTaskHandle;
const osThreadAttr_t ProcessSmsTask_attributes = {
  .name = "ProcessSmsTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for ConsoleTxTask */
osThreadId_t ConsoleTxTaskHandle;
const osThreadAttr_t ConsoleTxTask_attributes = {
  .name = "ConsoleTxTask",
  .priority = (osPriority_t) osPriorityAboveNormal1,
  .stack_size = 256 * 4
};
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for GsmTxTask */
osThreadId_t GsmTxTaskHandle;
const osThreadAttr_t GsmTxTask_attributes = {
  .name = "GsmTxTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for GsmRxTask */
osThreadId_t GsmRxTaskHandle;
const osThreadAttr_t GsmRxTask_attributes = {
  .name = "GsmRxTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for smsRxQueue */
osMessageQueueId_t smsRxQueueHandle;
const osMessageQueueAttr_t smsRxQueue_attributes = {
  .name = "smsRxQueue"
};
/* Definitions for ConsoleTxQueue */
osMessageQueueId_t ConsoleTxQueueHandle;
const osMessageQueueAttr_t ConsoleTxQueue_attributes = {
  .name = "ConsoleTxQueue"
};
/* Definitions for SmsRxTimeout */
osTimerId_t SmsRxTimeoutHandle;
const osTimerAttr_t SmsRxTimeout_attributes = {
  .name = "SmsRxTimeout"
};
/* Definitions for consoleSema */
osSemaphoreId_t consoleSemaHandle;
const osSemaphoreAttr_t consoleSema_attributes = {
  .name = "consoleSema"
};
/* USER CODE BEGIN PV */


/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
struct msg_handle_struct *msg_handle_ptr;
char debug_str[1000];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartProcessSmsTask(void *argument);
void StartConsoleTxTask(void *argument);
void StartDebugTask(void *argument);
void StartGsmTxTask(void *argument);
void StartGsmRxTask(void *argument);
void SmsRxTimeout_Callback(void *argument);

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    // Initilaize message repository
    msg_initialize();
    RetargetInit(&huart3);
    //------------------------------------------------------------------------------------------------
    HAL_Delay(2000);
    printf("%s %s","STM32F746_FreeRTOS_DMA_Printf", &newline_str);


    HAL_Delay(100);
    //------------------------------------------------------------------------------------------------

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of consoleSema */
  consoleSemaHandle = osSemaphoreNew(1, 1, &consoleSema_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of SmsRxTimeout */
  SmsRxTimeoutHandle = osTimerNew(SmsRxTimeout_Callback, osTimerOnce, NULL, &SmsRxTimeout_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of smsRxQueue */
  smsRxQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &smsRxQueue_attributes);

  /* creation of ConsoleTxQueue */
  ConsoleTxQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &ConsoleTxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ProcessSmsTask */
  ProcessSmsTaskHandle = osThreadNew(StartProcessSmsTask, NULL, &ProcessSmsTask_attributes);

  /* creation of ConsoleTxTask */
  ConsoleTxTaskHandle = osThreadNew(StartConsoleTxTask, NULL, &ConsoleTxTask_attributes);

  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

  /* creation of GsmTxTask */
  GsmTxTaskHandle = osThreadNew(StartGsmTxTask, NULL, &GsmTxTask_attributes);

  /* creation of GsmRxTask */
  GsmRxTaskHandle = osThreadNew(StartGsmRxTask, NULL, &GsmRxTask_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Turn LED2 on: transmission DMA transfer completed */
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    //BSP_LED_On(LD2);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  huart: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Turn LED4 on: reception DMA transfer completed */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	//BSP_LED_On(LED4);
}

/**
  * @brief  UART error callbacks
  * @param  huart: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Turn LED3 on: Transfer error in reception/transmission process */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    //BSP_LED_On(LED3);
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
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

#define SEND_AT_COMMAND(__AT_CMND_IDX__, __AT_STR__) \
		memset((__AT_STR__), 0x00,sizeof((__AT_STR__))); \
		sms_get_at_cmnd((__AT_CMND_IDX__), (__AT_STR__)); \
		SMS_DMA_TRANSMIT(__AT_STR__)


/* USER CODE BEGIN Header_StartProcessSmsTask */
/**
* @brief Function implementing the ProcessSmsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessSmsTask */
void StartProcessSmsTask(void *argument)
{
  /* USER CODE BEGIN StartProcessSmsTask */
    osStatus_t retvalue;
    uint16_t task_state;
    // Msg_StatusTypeDef msg_ret_val;
    static struct msg_handle_struct *sms_rx_msg_handle;
    uint8_t       msg_indx;
    uint16_t      nbr_sub_rows;
    at_cmnd_type  at_cmnd;
    char          at_cmnd_str[20];

	printf("%s %s",osThreadGetName(ProcessSmsTaskHandle), &newline_str);
    task_state = 0;
    at_cmnd = AT_AT;
    /* Infinite loop */
    for(;;)
    {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    	switch(task_state)
    	{

    	case 0:    // HW reset
    		task_state = 100;
    		osDelay(200);
    		break;

    	case 100:
    		at_cmnd = AT_CHIP_INFO;
    		SEND_AT_COMMAND(at_cmnd, at_cmnd_str);
    		task_state++;
    		break;
    	case 101:
    		retvalue = osMessageQueueGet(smsRxQueueHandle,(uint8_t*)&msg_indx, (uint8_t *)osPriorityNormal, 4000);
     		if (retvalue == osOK)
     		{
    	        if (msg_get_handle( (struct msg_handle_struct **) &sms_rx_msg_handle, msg_indx) == MSG_OK)
    		    {
					printf("%s %s",sms_rx_msg_handle->buf, &newline_str);
					nbr_sub_rows =  msg_row_split(sms_rx_msg_handle);
					sms_debug(at_cmnd, sms_rx_msg_handle);
					printf("SMS check = %u%s",(uint8_t) sms_check(at_cmnd, sms_rx_msg_handle), &newline_str);
					msg_release(sms_rx_msg_handle);
					//printf("%s %s","SMS raw message received", &newline_str);
					//printf("%s %s","...end\r\n", &newline_str);
  			    }
   			}
    		else
    		{
    		   osThreadYield();
    		}
     		osDelay(2000);
     		task_state = 100;
    		break;
    	case 200:
    		break;
    	}

  }


  /* USER CODE END StartProcessSmsTask */
}

/* USER CODE BEGIN Header_StartConsoleTxTask */
/**
* @brief Function implementing the ConsoleTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartConsoleTxTask */
void StartConsoleTxTask(void *argument)
{
  /* USER CODE BEGIN StartConsoleTxTask */
	 osStatus_t retvalue;
	 uint8_t    msg_indx;
	 struct msg_handle_struct *console_tx_msg_handle;
    /* Infinite loop */
    for(;;)
    {
    	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	    retvalue = osMessageQueueGet(ConsoleTxQueueHandle,(uint8_t*)&msg_indx, (uint8_t*)osPriorityNormal, 1000);
	    if (retvalue == osOK){
	    	if (msg_get_handle( (struct msg_handle_struct **) &console_tx_msg_handle, msg_indx) == MSG_OK){
	    		//osSemaphoreAcquire(consoleSemaHandle,osWaitForever);
	    		printf("%s %s",console_tx_msg_handle->buf, &newline_str);
	    	   	msg_release(console_tx_msg_handle);
	    		//osSemaphoreRelease(consoleSemaHandle);
	    	}
	    }
	    else
	    {
	    	osThreadYield();
	    }

        osDelay(100);
    }
  /* USER CODE END StartConsoleTxTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
* @brief Function implementing the DebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartDebugTask */
  /* Infinite loop */
  for(;;)
  {

	if( HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)  == GPIO_PIN_SET){
		printf("---debug--\r\n");
		printf("Task name       state          stack \r\n");
		vTaskList( debug_str );
		printf(debug_str);
		printf("X=running, B=Blocked, R=Ready, D=Deleted, S=Suspended \r\n");
		//osDelay(1000);
		//vTaskGetRunTimeStats(debug_str);
		//printf(debug_str);
		osDelay(1000);
	}
    osDelay(100);
  }
  /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_StartGsmTxTask */
/**
* @brief Function implementing the GsmTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGsmTxTask */
void StartGsmTxTask(void *argument)
{
  /* USER CODE BEGIN StartGsmTxTask */
	printf("%s %s",osThreadGetName(GsmTxTaskHandle), &newline_str);

    /* Infinite loop */
    for(;;)
    {
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		//SMS_DMA_TRANSMIT("ATI\r\n");

        osDelay(2000);
    }
  /* USER CODE END StartGsmTxTask */
}

/* USER CODE BEGIN Header_StartGsmRxTask */
/**
* @brief Function implementing the GsmRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGsmRxTask */
void StartGsmRxTask(void *argument)
{
  /* USER CODE BEGIN StartGsmRxTask */
	static uint16_t  nbr_rx_chr = 0;
	static struct msg_handle_struct *rx_msg_handle;
	static uint8_t   indx_buf[2];
	static uint32_t  tick_limit;
	static uint8_t   free_buffer;


	//uint8_t  *end_ptr;
	printf("%s %s",osThreadGetName(GsmRxTaskHandle), &newline_str);

  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		free_buffer = 0;
		nbr_rx_chr = 0;
        SET_TICK_LIMIT(tick_limit,100);
		if (msg_reserve( (struct msg_handle_struct **) &rx_msg_handle, msg_type_gsm_rx) == MSG_OK  )
		{

			if (HAL_UART_Receive_DMA(&SMS_UART, (uint8_t *)rx_msg_handle->buf, 80) != HAL_OK)
			{
				printf("%s %s","SMS UART DMA Receive error", &newline_str);
			    //Error_Handler();  /* Transfer error in reception process */
			}
			while ((HAL_UART_GetState(&SMS_UART) != HAL_UART_STATE_READY) &&  (HAL_GetTick() < tick_limit) )
			{
				if (strlen((char *) rx_msg_handle->buf) > (size_t) nbr_rx_chr )
				{
			        SET_TICK_LIMIT(tick_limit,100);
			        nbr_rx_chr = strlen((char *)rx_msg_handle->buf);
				}

			}
			if (nbr_rx_chr > 0)
			{
				printf("<%u>%s %s",nbr_rx_chr, rx_msg_handle->buf, &newline_str);
					indx_buf[0] = rx_msg_handle->index;
					osMessageQueuePut(smsRxQueueHandle, (uint8_t *) indx_buf, (uint8_t)osPriorityNormal,200 );
			}
			else
			{
					 free_buffer = 1;
					 //printf("%s %s","SMS Rx timeout", &newline_str);
			}

			if (HAL_UART_DMAStop(&SMS_UART) != HAL_OK )
			{
				printf("%s %s","HAL_UART_DMAStop != HAL_OK", &newline_str);

			}
		}
		else
		{
  		    printf("%s %s","Message repository is full", &newline_str);
		}

		if (free_buffer )
		{
     	     msg_release(rx_msg_handle);
		}
        osDelay(5);

    }
  /* USER CODE END StartGsmRxTask */
}

/* SmsRxTimeout_Callback function */
void SmsRxTimeout_Callback(void *argument)
{
  /* USER CODE BEGIN SmsRxTimeout_Callback */

  /* USER CODE END SmsRxTimeout_Callback */
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
