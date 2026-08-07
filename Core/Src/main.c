/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "jsmn.h"
#include "modbus.h"
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
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t modbusSwitchCase = 0;
ModbusMaster mb2;
machineSetting MachineSetting;
uint32_t currentTime = 0;
uint32_t rand = 0;

uint8_t IOTestArray[18];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MachineSettingInit(void);
static void StateMachineRunning(void);
static void StateMachineRunningTest(void);
static void SlaveCom(void);
static void WriteOutput(uint8_t PinNo, GPIO_PinState IsSet);
static GPIO_PinState ReadInput(uint8_t PinNo);
static GPIO_PinState ReadOutput(uint8_t PinNo);
static void CheckTCP(void);
static void IOListUpdate(void);
static void ResetAllIO(void);
static int parse_master_request(const char *json, masterRequest *req);
static void parse_task_object(const char *json, jsmntok_t *task_tok, masterRequest *req);
static int jsoneq(const char *json, jsmntok_t *tok, const char *s);
static uint32_t parse_int(const char *json, jsmntok_t *tok);
static float parse_float(const char *json, jsmntok_t *tok);
static float powhere10(int exp);
static void executeCommand(void);
static uint8_t TaskInfoFIFO_Push(TaskInfoFIFO *fifo, const TaskInfo *task);
static void TaskInfoFIFO_RemoveFirst(TaskInfoFIFO *fifo);
static void TaskInfoFIFO_RemoveAll(TaskInfoFIFO *fifo);
static int build_status_json(char *buffer, size_t buf_size);
static const char* state_to_string(machineState state);
static machineState string_to_state(const char *str);
static int float_to_string(float value, char *str);
static void AddErrorCode(uint16_t errorcode);
static void ClearErrorCode(void);
static void ModbusTransmitSwitchDevice(void);
static void TaskHistoryFIFO_Push(TaskHistoryFIFO *fifo, const TaskHistory *taskHistory);
static void StateHistoryFIFO_Push(StateHistoryFIFO *fifo, const StateHistory *stateHistory);
static void AppendStateHistory(uint16_t exitcode);
static void CheckWarningCondition(void);
static void MoveAllTasksToTaskHistory(void);
static void MoveTaskToTaskHistory(uint8_t* taskid);
static void TaskInfoFIFO_Remove(TaskInfoFIFO * fifo, uint8_t* taskid);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == MachineSetting.SlaveCom.huart){
		MachineSetting.SlaveCom.timeStamp = HAL_GetTick();
		MachineSetting.SlaveCom.bufferCount = 0;
		HAL_GPIO_WritePin(MachineSetting.SlaveCom.DirectionalPin.Port, MachineSetting.SlaveCom.DirectionalPin.Pin, GPIO_PIN_RESET);
		HAL_UART_Receive_IT(MachineSetting.SlaveCom.huart, &MachineSetting.SlaveCom.Buffer[MachineSetting.SlaveCom.bufferCount], 1);
		MachineSetting.SlaveCom.state = SLAVE_COM_PRE_RX;
	}
	else if(huart->Instance == USART3){
		ModbusMaster_UART_TxCpltCallback(&mb2);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == MachineSetting.TCPCom.huart){
		if(MachineSetting.TCPCom.BufferCount < TCP_Buffer_MAX_Count-1){//The last byte must be 0, so we cant fill the last byte
			HAL_UART_Receive_IT(MachineSetting.TCPCom.huart, &MachineSetting.TCPCom.Buffer[++MachineSetting.TCPCom.BufferCount], 1);
			MachineSetting.TCPCom.TimeStamp = HAL_GetTick();
		}

	}
	else if (huart == MachineSetting.SlaveCom.huart){
		MachineSetting.SlaveCom.timeStamp = HAL_GetTick();
		HAL_UART_Receive_IT(MachineSetting.SlaveCom.huart, &MachineSetting.SlaveCom.Buffer[++MachineSetting.SlaveCom.bufferCount], 1);
		MachineSetting.SlaveCom.state = SLAVE_COM_RX;
	}
	else if (huart->Instance == USART3){
		ModbusMaster_UART_RxCpltCallback(&mb2);
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
  MX_RTC_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MachineSettingInit();
  ModbusMaster_Init(&mb2,&huart3, GPIOC, GPIO_PIN_8);
  ModbusMaster_AddReadQueue(&mb2, 1, 1, 1, 300, 300, 300);
  ModbusMaster_AddReadQueue(&mb2, 2, 10, 2, 300, 300, 300);
  ModbusMaster_AddReadQueue(&mb2, 3, 10, 2, 300, 300, 300);
  ModbusMaster_UpdateReadTransaction(&mb2);
  memset(IOTestArray,0, sizeof(IOTestArray));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //TestingIO();
	  uint32_t current = HAL_GetTick();
	  MachineSetting.scanTime = current - MachineSetting.currentTick;
	  MachineSetting.currentTick = current;
	  StateMachineRunningTest();
	  ModbusTransmitSwitchDevice();
	  CheckTCP();
	  IOListUpdate();
	  SlaveCom();
	  CheckWarningCondition();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC4
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4
                           PB5 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void MachineSettingInit(void){
	IOLocalPin outputsPin[LOCAL_OUT] = {
		{GPIOA, GPIO_PIN_11}, {GPIOA, GPIO_PIN_12}, {GPIOA, GPIO_PIN_8},
		{GPIOC, GPIO_PIN_9},  {GPIOB, GPIO_PIN_3},  {GPIOB, GPIO_PIN_4},
		{GPIOB, GPIO_PIN_5},  {GPIOB, GPIO_PIN_6},  {GPIOB, GPIO_PIN_7},
		{GPIOB, GPIO_PIN_8},  {GPIOB, GPIO_PIN_9},  {GPIOC, GPIO_PIN_13},
		{GPIOC, GPIO_PIN_14}, {GPIOC, GPIO_PIN_15}, {GPIOB, GPIO_PIN_0},
		{GPIOB, GPIO_PIN_1}
	};
	IOLocalPin inputsPin[LOCAL_IN] = {
		{GPIOD, GPIO_PIN_8},{GPIOD, GPIO_PIN_9},  {GPIOD, GPIO_PIN_10}, {GPIOD, GPIO_PIN_11},
		{GPIOD, GPIO_PIN_12},{GPIOD, GPIO_PIN_13}, {GPIOD, GPIO_PIN_14},{GPIOD, GPIO_PIN_15},
		{GPIOD, GPIO_PIN_0},{GPIOD, GPIO_PIN_1}, {GPIOD, GPIO_PIN_2},{GPIOD, GPIO_PIN_3},
		{GPIOD, GPIO_PIN_4},{GPIOD, GPIO_PIN_5},{GPIOD, GPIO_PIN_6},  {GPIOD, GPIO_PIN_7}
	};
	for (uint8_t i = 0; i < LOCAL_OUT; i++) {
		MachineSetting.generalSetting.Outputs.LocalPin[i].Port = outputsPin[i].Port;
		MachineSetting.generalSetting.Outputs.LocalPin[i].Pin  = outputsPin[i].Pin;
	}
	for (uint8_t i = 0; i < LOCAL_IN; i++) {
		MachineSetting.generalSetting.Inputs.LocalPin[i].Port = inputsPin[i].Port;
		MachineSetting.generalSetting.Inputs.LocalPin[i].Pin  = inputsPin[i].Pin;
	}
	MachineSetting.generalSetting.Outputs.BasicOut[WaterReservoirPump] = 22;
	MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump] = 23;
	MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource] = 8;
	MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix1] = 9;
	MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1] = 10;
	MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix2] = 11;
	MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix2] = 12;
	MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FILL_VALVE] = 13;
	MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE] = 0;
	MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP] = 4;
	MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_PROPELLER] = 6;
	MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FILL_VALVE] = 14;
	MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE] = 1;
	MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP] = 5;
	MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_PROPELLER] = 7;
	MachineSetting.generalSetting.ValveOut[0] = 15;
	MachineSetting.generalSetting.ValveOut[1] = 16;
	MachineSetting.generalSetting.ValveOut[2] = 17;

	MachineSetting.generalSetting.Outputs.RealPinState[0].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[1].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[4].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[5].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[6].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[7].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[8].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[9].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[10].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[11].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[12].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[13].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[14].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[15].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[16].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[17].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[22].IsBind = 1;
	MachineSetting.generalSetting.Outputs.RealPinState[23].IsBind = 1;

	MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow] = 0;
	MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirWarning] = 1;
	MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirHigh] = 2;
	MachineSetting.generalSetting.Inputs.BasicIn[Mix1Low] = 3;
	MachineSetting.generalSetting.Inputs.BasicIn[Mix1High] = 4;
	MachineSetting.generalSetting.Inputs.BasicIn[Mix1Limit] = 5;
	MachineSetting.generalSetting.Inputs.BasicIn[Mix2Low] = 6;
	MachineSetting.generalSetting.Inputs.BasicIn[Mix2High] = 7;
	MachineSetting.generalSetting.Inputs.BasicIn[Mix2Limit] = 8;
	MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_LOW] = 9;
	MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_WARNING] = 10;
	MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_HIGH] = 11;
	MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_LOW] = 12;
	MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_WARNING] = 13;
	MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_HIGH] = 14;
	MachineSetting.generalSetting.Inputs.BasicIn[Flow] = 15;

	MachineSetting.generalSetting.Inputs.RealPinState[0].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[1].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[2].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[3].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[4].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[5].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[6].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[7].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[8].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[9].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[10].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[11].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[12].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[13].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[14].IsBind = 1;
	MachineSetting.generalSetting.Inputs.RealPinState[15].IsBind = 1;

	MachineSetting.TimeOut[STATE_IDLE] = 0;
	MachineSetting.TimeOut[STATE_WATER_RESERVOIR_REFILL] = 1800000;
	MachineSetting.TimeOut[STATE_CLEAN_WATER_WATERING] = 1800000;
	MachineSetting.TimeOut[STATE_MIXED_WATER_WATERING] = 1800000;
	MachineSetting.TimeOut[STATE_CLEAN_WATER_FILLING_LOWER_EC] = 1800000;
	MachineSetting.TimeOut[STATE_WATER_FILLING_TO_MIX1_HIGHSENSOR] = 1800000;
	MachineSetting.TimeOut[STATE_FERTILIZER_ADDING] = 1800000;
	MachineSetting.TimeOut[STATE_OVERFLOW_RECOVERY] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_WATER_RESERVOIR_REFILL] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_CLEAN_WATER_WATERING] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_MIXED_WATER_WATERING] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_FERTILIZER_ADDING] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_OVERFLOW_RECOVERY] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_FILL_FERTILIZER_TANK] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_FILL_PESTICIDE_TANK] = 1800000;
	MachineSetting.TimeOut[STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER] = 1800000;
	MachineSetting.TimeOut[STATE_ABORT] = 0;
	MachineSetting.TimeOut[STATE_ERROR] = 0;

//	MachineSetting.Errortypearr[TIMEOUT_MIXED_WATER_WATERING] = 100;
//	MachineSetting.Errortypearr[TIMEOUT_WATER_RESERVOIR_REFILL] = 101;
//	MachineSetting.Errortypearr[TIMEOUT_CLEAN_WATER_WATERING] = 102;
//	MachineSetting.Errortypearr[TIMEOUT_CLEAN_WATER_FILLING_EC] = 103;
//	MachineSetting.Errortypearr[TIMEOUT_WATER_FILLING_HIGH_SENSOR] = 104;
//	MachineSetting.Errortypearr[TIMEOUT_ADDING_FERTILIZER] = 105;
//	MachineSetting.Errortypearr[TIMEOUT_OVERFLOW_RECOVERY] = 106;
//	MachineSetting.Errortypearr[SENSOR_WATER_RESERVOIR_LOW] = 200;
//	MachineSetting.Errortypearr[SENSOR_FERT_TANK_LOW] = 201;
//	MachineSetting.Errortypearr[SENSOR_LimTrig_HiNotTrig] = 202;
//	MachineSetting.Errortypearr[SENSORDATA_EC] = 300;
//	MachineSetting.Errortypearr[SENSORDATA_FLOWA] = 301;
//	MachineSetting.Errortypearr[SENSORDATA_FLOWB] = 302;
//	MachineSetting.Errortypearr[SLAVE_COM_FAIL] = 401;
//	MachineSetting.Errortypearr[E_STOP] = 500;

	MachineSetting.TCPCom.huart = &huart6;
	MachineSetting.TCPCom.txTimeOut = 1000;
	MachineSetting.TCPCom.maxTimeDisconnectFromPC = 10000;
	MachineSetting.TCPCom.DirectionalPin.Port = GPIOB;
	MachineSetting.TCPCom.DirectionalPin.Pin = GPIO_PIN_9;
	MachineSetting.TCPCom.intercharTimeOut = 10;
	MachineSetting.TCPCom.BufferCount = 0;
	HAL_UART_Receive_IT(MachineSetting.TCPCom.huart, MachineSetting.TCPCom.Buffer, 1);

	MachineSetting.manualEvent.isManualNew = 1;
	MachineSetting.manualEvent.isManual = 1;
	MachineSetting.manualEvent.ManualResetTimeOut = 300000;
	uint8_t slaveAddress = 0;
	for(int i = 0; i<4;i++){
	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0 << i) == GPIO_PIN_RESET){
		  slaveAddress |= 1<<i;
	  }
	}
	if(slaveAddress != 0){
		MachineSetting.SlaveCom.isSlave = 1;
		MachineSetting.SlaveCom.currentSlave = slaveAddress;
		MachineSetting.SlaveCom.intercharTimeOut = 10;
		MachineSetting.SlaveCom.txTimeOut = 100;
		MachineSetting.SlaveCom.huart = &huart2;
		MachineSetting.SlaveCom.DirectionalPin.Pin = GPIO_PIN_4;
		MachineSetting.SlaveCom.DirectionalPin.Port = GPIOC;
		HAL_UART_Receive_IT(MachineSetting.SlaveCom.huart, &MachineSetting.SlaveCom.Buffer[MachineSetting.SlaveCom.bufferCount], 1);
	}
	else{
		MachineSetting.SlaveCom.slaveCount = 1;
		MachineSetting.SlaveCom.intercharTimeOut = 10;
		MachineSetting.SlaveCom.txTimeOut = 1000;
		MachineSetting.SlaveCom.rxTimeOut = 1000;
		MachineSetting.SlaveCom.huart = &huart2;
		MachineSetting.SlaveCom.DirectionalPin.Pin = GPIO_PIN_4;
		MachineSetting.SlaveCom.DirectionalPin.Port = GPIOC;
		//HAL_UART_Receive_IT(MachineSetting.SlaveCom.huart, &MachineSetting.SlaveCom.Buffer[MachineSetting.SlaveCom.bufferCount], 1);
	}
	MachineSetting.generalSetting.Fert.FertCount = 2;
	MachineSetting.generalSetting.ValveOutCount = 3;
	MachineSetting.generalSetting.readingEC.modbusSetting.factor = 1;
	MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.factor = 1;
	MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.factor = 1;

}

static void StateMachineRunning(void){

	static uint32_t NoFlowTimeStamp = 0;
	switch(MachineSetting.MachineState){
	case STATE_IDLE:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.manualEvent.isManual != MachineSetting.manualEvent.isManualNew){
			MachineSetting.manualEvent.isManual = MachineSetting.manualEvent.isManualNew;
		}
		MachineSetting.TimeStamp = HAL_GetTick();
		if(MachineSetting.isAbort != 0){
			MachineSetting.MachineState = STATE_ABORT;

			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.manualEvent.isManual != 0){
			switch (MachineSetting.manualEvent.triggerEvent){
			case STATE_MANUAL_WATER_RESERVOIR_REFILL:
				MachineSetting.MachineState = STATE_MANUAL_WATER_RESERVOIR_REFILL;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_CLEAN_WATER_WATERING:
				MachineSetting.MachineState = STATE_MANUAL_CLEAN_WATER_WATERING;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_MIXED_WATER_WATERING:
				MachineSetting.MachineState = STATE_MANUAL_MIXED_WATER_WATERING;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC:
				MachineSetting.MachineState = STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR:
				MachineSetting.MachineState = STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_FERTILIZER_ADDING:
				MachineSetting.MachineState = STATE_MANUAL_FERTILIZER_ADDING;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_OVERFLOW_RECOVERY:
				MachineSetting.MachineState = STATE_MANUAL_OVERFLOW_RECOVERY;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_FILL_FERTILIZER_TANK:
				MachineSetting.MachineState = STATE_MANUAL_FILL_FERTILIZER_TANK;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_FILL_PESTICIDE_TANK:
				MachineSetting.MachineState = STATE_MANUAL_FILL_PESTICIDE_TANK;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER:
				MachineSetting.MachineState = STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			default:
				break;
			}
			MachineSetting.manualEvent.triggerEvent = STATE_IDLE;
			break;
		}
		if(MachineSetting.TaskProxyBuffer.InfoFIFO.count != 0){
			if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime >= MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration){
				TaskHistory taskHistory = {0};
				memcpy(taskHistory.taskID, MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].taskID, TASK_ID_BYTE_COUNT);

				if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration> MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime){
					taskHistory.incompleteDuration = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration -  MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				}
				else{
					taskHistory.incompleteDuration = 0;
				}
				taskHistory.end = HAL_GetTick();
				taskHistory.start = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].timeStamp;
				TaskHistoryFIFO_Push(&MachineSetting.TaskProxyBuffer.HistoryFIFO, &taskHistory);
				TaskInfoFIFO_RemoveFirst(&MachineSetting.TaskProxyBuffer.InfoFIFO);
				break;
			}
			else{
				if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].timeStamp == 0){
					MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].timeStamp = HAL_GetTick();
				}
				if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].TargetEC < 500){
					if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
						MachineSetting.MachineState = STATE_WATER_RESERVOIR_REFILL;
						MachineSetting.TimeStamp = HAL_GetTick();
						break;
					}
					else{
						MachineSetting.MachineState = STATE_CLEAN_WATER_WATERING;
						MachineSetting.TimeStamp = HAL_GetTick();
						break;
					}
				}
				else{
					if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Low]) == GPIO_PIN_SET){
						if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow] == GPIO_PIN_SET)){
							MachineSetting.MachineState = STATE_WATER_RESERVOIR_REFILL;
							MachineSetting.TimeStamp = HAL_GetTick();
							break;
						}
						else{
							MachineSetting.MachineState = STATE_WATER_FILLING_TO_MIX1_HIGHSENSOR;
							MachineSetting.TimeStamp = HAL_GetTick();
							break;
						}
					}
					else{
						if(MachineSetting.generalSetting.readingEC.MeanEC < MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].TargetEC + MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].Hysterisis && MachineSetting.generalSetting.readingEC.MeanEC > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].TargetEC-MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].Hysterisis){
							MachineSetting.MachineState = STATE_MIXED_WATER_WATERING;
							MachineSetting.TimeStamp = HAL_GetTick();
							break;
						}
						else{
							if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Limit]) == GPIO_PIN_RESET){
								if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix2Limit]) == GPIO_PIN_RESET){
									AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_MIXTANKSFULL + 10000);
									MachineSetting.MachineState = STATE_ERROR;
									MachineSetting.TimeStamp = HAL_GetTick();
									break;
								}
								else{
									MachineSetting.MachineState = STATE_OVERFLOW_RECOVERY;
									MachineSetting.TimeStamp = HAL_GetTick();
									break;
								}
							}
							else{
								if(MachineSetting.generalSetting.readingEC.MeanEC > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].TargetEC + MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].Hysterisis){
									if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
										MachineSetting.MachineState = STATE_WATER_RESERVOIR_REFILL;
										MachineSetting.TimeStamp = HAL_GetTick();
										break;
									}
									else{
										MachineSetting.MachineState = STATE_CLEAN_WATER_FILLING_LOWER_EC;
										MachineSetting.TimeStamp = HAL_GetTick();
										break;
									}

								}
								else if(MachineSetting.generalSetting.readingEC.MeanEC < MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].TargetEC - MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].Hysterisis){
									MachineSetting.MachineState = STATE_FERTILIZER_ADDING;
									MachineSetting.TimeStamp = HAL_GetTick();
									break;
								}
							}
						}
					}
				}
			}
		}
		else if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirWarning]) == GPIO_PIN_SET){
			MachineSetting.MachineState = STATE_WATER_RESERVOIR_REFILL;
			break;
		}
		break;
	}
	case STATE_WATER_RESERVOIR_REFILL:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()- MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[WaterReservoirPump], GPIO_PIN_SET);
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirHigh]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_WATER_RESERVOIR_HIGH);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp > MachineSetting.TimeOut[STATE_WATER_RESERVOIR_REFILL]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		break;
	}
	case STATE_CLEAN_WATER_WATERING:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp >DELAY_FOR_VALVE){
			MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp - DELAY_FOR_VALVE;
		}
		else{
			MachineSetting.ElapsedTime = 0;
		}
		MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp + MachineSetting.ElapsedTime;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_SET);
		if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].plotID >= MachineSetting.generalSetting.ValveOutCount){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_PLOTOUTOFRANGE + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_PLOTOUTOFRANGE + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		else{
			WriteOutput(MachineSetting.generalSetting.ValveOut[MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].plotID], GPIO_PIN_SET);
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp < DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);

		if( MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_ELAPSED_TIME_GREATER_THAN_DURATION);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_WATER_RESERVOIR_LOW);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		if(MachineSetting.ElapsedTime > MachineSetting.TimeOut[STATE_CLEAN_WATER_WATERING]){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		break;
	}
	case STATE_MIXED_WATER_WATERING:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp >DELAY_FOR_VALVE){
			MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp - DELAY_FOR_VALVE;
		}
		else{
			MachineSetting.ElapsedTime = 0;
		}
		MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp + MachineSetting.ElapsedTime;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
		if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].plotID >= MachineSetting.generalSetting.ValveOutCount){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_PLOTOUTOFRANGE + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_PLOTOUTOFRANGE + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		else{
			WriteOutput(MachineSetting.generalSetting.ValveOut[MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].plotID], GPIO_PIN_SET);
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp < DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);

		if( MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_ELAPSED_TIME_GREATER_THAN_DURATION);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Low]) == GPIO_PIN_SET){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_LOW);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		if(MachineSetting.ElapsedTime > MachineSetting.TimeOut[STATE_MIXED_WATER_WATERING]){
			MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		break;
	}
	case STATE_CLEAN_WATER_FILLING_LOWER_EC:{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()- MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix1], GPIO_PIN_SET);

		if (MachineSetting.generalSetting.readingEC.modbusSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ECREADING + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ECREADING + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.ElapsedTime < DELAY_FOR_VALVE){
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);

		static uint32_t TimeStamp = 0;
		static uint8_t ECLowerThanTarget = 0;
		uint8_t currentECLowerThanTarget = (MachineSetting.generalSetting.readingEC.MeanEC < MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].TargetEC + MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].Hysterisis);

		if(ECLowerThanTarget != currentECLowerThanTarget){
			ECLowerThanTarget = currentECLowerThanTarget;
			TimeStamp = HAL_GetTick();
		}
		if (ReadOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1]) == GPIO_PIN_RESET){
			if(ECLowerThanTarget == 1){
				WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_RESET);
				WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
				TimeStamp = HAL_GetTick();
			}
		}
		else{
			if(ECLowerThanTarget != 0){
				if(HAL_GetTick()-TimeStamp >60000){
					AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_EC_REACH_TARGET);
					MachineSetting.MachineState = STATE_ABORT;
					TimeStamp = HAL_GetTick();
					break;
				}
			}
			else{
				if(HAL_GetTick()-TimeStamp >10000){
					WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_SET);
					WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_RESET);
					TimeStamp = HAL_GetTick();
				}
			}
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp > MachineSetting.TimeOut[STATE_CLEAN_WATER_FILLING_LOWER_EC]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Limit]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_LIMIT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_WATER_RESERVOIR_LOW);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		break;
	}
	case STATE_WATER_FILLING_TO_MIX1_HIGHSENSOR:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix1], GPIO_PIN_SET);
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix2Low]) == GPIO_PIN_SET){
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_RESET);
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix2], GPIO_PIN_SET);
		}
		else{
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_SET);
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix2], GPIO_PIN_RESET);
		}
		if(MachineSetting.ElapsedTime < DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);
		if(HAL_GetTick()-MachineSetting.TimeStamp > MachineSetting.TimeOut[STATE_WATER_FILLING_TO_MIX1_HIGHSENSOR]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1High]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_HIGH);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Limit]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_LIMIT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_WATER_RESERVOIR_LOW);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		break;
	}
	case STATE_FERTILIZER_ADDING:{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		static uint32_t TimeStamp = 0;
		static uint8_t ECHigherThanTarget = 0;
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix1], GPIO_PIN_SET);
		if(MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_FLOWAREADING + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_FLOWAREADING + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_FLOWBREADING + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_FLOWBREADING + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if (MachineSetting.generalSetting.readingEC.modbusSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ECREADING + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ECREADING + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp < DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
			MachineSetting.generalSetting.Fert.Fert[0].StampValue = MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.value;
			MachineSetting.generalSetting.Fert.Fert[1].StampValue = MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.value;
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);
		uint8_t currentECHigherThanTarget = (MachineSetting.generalSetting.readingEC.MeanEC > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].TargetEC);
		if(ECHigherThanTarget != currentECHigherThanTarget){
			ECHigherThanTarget = currentECHigherThanTarget;
			TimeStamp = HAL_GetTick();
		}
		if (ReadOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET || ReadOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET) {
			if(ECHigherThanTarget == 1){
				WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP],GPIO_PIN_RESET);
				WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE],GPIO_PIN_RESET);
				WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP],GPIO_PIN_RESET);
				WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE],GPIO_PIN_RESET);
				TimeStamp = HAL_GetTick();
			}
			else{
				if(ReadOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET && ReadOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET){
					if(MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume > MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume +50){
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP],GPIO_PIN_RESET);
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE],GPIO_PIN_RESET);
					}
					else if(MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume > MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume +50){
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP],GPIO_PIN_RESET);
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE],GPIO_PIN_RESET);
					}
				}
				else if(ReadOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_RESET && ReadOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET){
					if(MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume < MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume){
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP],GPIO_PIN_SET);
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE],GPIO_PIN_SET);
					}
				}
				else if(ReadOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET && ReadOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_RESET){
					if(MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume < MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume){
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP],GPIO_PIN_SET);
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE],GPIO_PIN_SET);
					}
				}
			}
		}
		else{
			if(ECHigherThanTarget != 0){
				if(HAL_GetTick()-TimeStamp >60000){
					AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_EC_REACH_TARGET);
					MachineSetting.MachineState = STATE_ABORT;
					TimeStamp = HAL_GetTick();

					break;
				}
			}
			else{
				if(HAL_GetTick()-TimeStamp >5000){
					WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP],GPIO_PIN_SET);
					WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE],GPIO_PIN_SET);
					WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP],GPIO_PIN_SET);
					WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE],GPIO_PIN_SET);
					TimeStamp = HAL_GetTick();
				}
			}
		}
		if(HAL_GetTick()- MachineSetting.TimeStamp> MachineSetting.TimeOut[STATE_FERTILIZER_ADDING]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Limit]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_LIMIT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_LOW]) == GPIO_PIN_SET){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ALOW + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ALOW + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_LOW]) == GPIO_PIN_SET){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_BLOW + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_BLOW + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}


		MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume = MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.value - MachineSetting.generalSetting.Fert.Fert[0].StampValue;
		MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume = MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.value - MachineSetting.generalSetting.Fert.Fert[1].StampValue;
		break;
	}
	case STATE_OVERFLOW_RECOVERY:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix2], GPIO_PIN_SET);
		if(HAL_GetTick()-MachineSetting.TimeStamp < DELAY_FOR_VALVE){
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1High]) == GPIO_PIN_SET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_HIGH);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix2Limit]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK2_LIMIT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(MachineSetting.ElapsedTime >MachineSetting.TimeOut[STATE_OVERFLOW_RECOVERY]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET  && (HAL_GetTick()-MachineSetting.TimeStamp > DELAY_FOR_VALVE+5000)){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}


		break;
	}
	case STATE_MANUAL_WATER_RESERVOIR_REFILL:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()- MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[WaterReservoirPump], GPIO_PIN_SET);
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirHigh]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_WATER_RESERVOIR_HIGH);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp > MachineSetting.TimeOut[STATE_MANUAL_WATER_RESERVOIR_REFILL]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		break;
	}
	case STATE_MANUAL_CLEAN_WATER_WATERING:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp >DELAY_FOR_VALVE){
			MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp - DELAY_FOR_VALVE;
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
			MachineSetting.ElapsedTime = 0;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_SET);
		if(MachineSetting.manualEvent.plotID < MachineSetting.generalSetting.ValveOutCount){
			WriteOutput(MachineSetting.generalSetting.ValveOut[MachineSetting.manualEvent.plotID], GPIO_PIN_SET);
		}
		else{
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_PLOTOUTOFRANGE + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_PLOTOUTOFRANGE + 10000);
			MachineSetting.MachineState = STATE_ERROR;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp < DELAY_FOR_VALVE){
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp > MachineSetting.TimeOut[MachineSetting.MachineState]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_WATER_RESERVOIR_LOW);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(MachineSetting.ElapsedTime > MachineSetting.manualEvent.TimeOut){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_ELAPSED_TIME_GREATER_THAN_DURATION);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		break;
	}
	case STATE_MANUAL_MIXED_WATER_WATERING:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp >DELAY_FOR_VALVE){
			MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp - DELAY_FOR_VALVE;
		}
		else{
			MachineSetting.ElapsedTime = 0;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
		if(MachineSetting.manualEvent.plotID < MachineSetting.generalSetting.ValveOutCount){
			WriteOutput(MachineSetting.generalSetting.ValveOut[MachineSetting.manualEvent.plotID], GPIO_PIN_SET);
		}
		else{
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_PLOTOUTOFRANGE + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_PLOTOUTOFRANGE + 10000);
			MachineSetting.MachineState = STATE_ERROR;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp < DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);


		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Low]) == GPIO_PIN_SET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_LOW);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(MachineSetting.ElapsedTime > MachineSetting.manualEvent.TimeOut){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_ELAPSED_TIME_GREATER_THAN_DURATION);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp > MachineSetting.TimeOut[MachineSetting.MachineState]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		break;
	}
	case STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC:{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()- MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix1], GPIO_PIN_SET);

		if (MachineSetting.generalSetting.readingEC.modbusSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ECREADING + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ECREADING + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.ElapsedTime < DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);

		static uint32_t TimeStamp = 0;
		static uint8_t ECLowerThanTarget = 0;
		uint8_t currentECLowerThanTarget = (MachineSetting.generalSetting.readingEC.MeanEC < MachineSetting.manualEvent.TargetEC);

		if(ECLowerThanTarget != currentECLowerThanTarget){
			ECLowerThanTarget = currentECLowerThanTarget;
			TimeStamp = HAL_GetTick();
		}
		if (ReadOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1]) == GPIO_PIN_RESET){
			if(ECLowerThanTarget == 1){
				WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_RESET);
				WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
				TimeStamp = HAL_GetTick();
			}
		}
		else{
			if(ECLowerThanTarget != 0){
				if(HAL_GetTick()-TimeStamp >60000){
					AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_EC_REACH_TARGET);
					MachineSetting.MachineState = STATE_ABORT;
					TimeStamp = HAL_GetTick();
					break;
				}
			}
			else{
				if(HAL_GetTick()-TimeStamp >10000){
					WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_SET);
					WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_RESET);
					TimeStamp = HAL_GetTick();
				}
			}
		}
		if(MachineSetting.ElapsedTime > MachineSetting.TimeOut[STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Limit]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_LIMIT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_WATER_RESERVOIR_LOW);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		break;
	}
	case STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix1], GPIO_PIN_SET);
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix2Low]) == GPIO_PIN_SET){
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_RESET);
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix2], GPIO_PIN_SET);
		}
		else{
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_SET);
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix2], GPIO_PIN_RESET);
		}
		if(MachineSetting.ElapsedTime < DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);
		if(MachineSetting.ElapsedTime > MachineSetting.TimeOut[STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1High]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_HIGH);
			MachineSetting.MachineState = STATE_ABORT;;
			break;
		}
		else if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Limit]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_LIMIT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_WATER_RESERVOIR_LOW);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW +10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		break;
	}
	case STATE_MANUAL_FERTILIZER_ADDING:{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		static uint32_t TimeStamp = 0;
		static uint8_t ECHigherThanTarget = 0;
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix1], GPIO_PIN_SET);
		if(MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_FLOWAREADING + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_FLOWAREADING + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_FLOWBREADING +10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_FLOWBREADING + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if (MachineSetting.generalSetting.readingEC.modbusSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ECREADING + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ECREADING + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp < DELAY_FOR_VALVE){
			WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
			MachineSetting.generalSetting.Fert.Fert[0].StampValue = MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.value;
			MachineSetting.generalSetting.Fert.Fert[1].StampValue = MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.value;
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);
		uint8_t currentECHigherThanTarget = (MachineSetting.generalSetting.readingEC.MeanEC>MachineSetting.manualEvent.TargetEC);
		if(ECHigherThanTarget != currentECHigherThanTarget){
			ECHigherThanTarget = currentECHigherThanTarget;
			TimeStamp = HAL_GetTick();
		}
		if (ReadOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET || ReadOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET) {
			if(ECHigherThanTarget == 1 && HAL_GetTick()){
				WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP],GPIO_PIN_RESET);
				WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE],GPIO_PIN_RESET);
				WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP],GPIO_PIN_RESET);
				WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE],GPIO_PIN_RESET);
				TimeStamp = HAL_GetTick();
			}
			else{
				if(ReadOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET && ReadOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET){
					if(MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume > MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume +50){
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP],GPIO_PIN_RESET);
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE],GPIO_PIN_RESET);
					}
					else if(MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume > MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume +50){
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP],GPIO_PIN_RESET);
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE],GPIO_PIN_RESET);
					}
				}
				else if(ReadOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_RESET && ReadOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET){
					if(MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume < MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume){
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP],GPIO_PIN_SET);
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE],GPIO_PIN_SET);
					}
				}
				else if(ReadOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_SET && ReadOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP]) == GPIO_PIN_RESET){
					if(MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume < MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume){
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP],GPIO_PIN_SET);
						WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE],GPIO_PIN_SET);
					}
				}
			}
		}
		else{
			if(ECHigherThanTarget != 0){
				if(HAL_GetTick()-TimeStamp >60000){
					AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_EC_REACH_TARGET);
					MachineSetting.MachineState = STATE_ABORT;
					TimeStamp = HAL_GetTick();

					break;
				}
			}
			else{
				if(HAL_GetTick()-TimeStamp >5000){
					WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_PUMP],GPIO_PIN_SET);
					WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FERT_VALVE],GPIO_PIN_SET);
					WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_PUMP],GPIO_PIN_SET);
					WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FERT_VALVE],GPIO_PIN_SET);
					TimeStamp = HAL_GetTick();
				}
			}
		}
		if(HAL_GetTick()- MachineSetting.TimeStamp> MachineSetting.TimeOut[STATE_MANUAL_FERTILIZER_ADDING]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1Limit]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_LIMIT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_LOW]) == GPIO_PIN_SET){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ALOW + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ALOW + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_LOW]) == GPIO_PIN_SET){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_BLOW +10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_BLOW + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}

		MachineSetting.generalSetting.Fert.Fert[0].CurrentVolume = MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.value - MachineSetting.generalSetting.Fert.Fert[0].StampValue;
		MachineSetting.generalSetting.Fert.Fert[1].CurrentVolume = MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.value - MachineSetting.generalSetting.Fert.Fert[1].StampValue;
		break;
	}
	case STATE_MANUAL_OVERFLOW_RECOVERY:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveOutMix1], GPIO_PIN_SET);
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveInMix2], GPIO_PIN_SET);
		if(HAL_GetTick()-MachineSetting.TimeStamp < DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix1High]) == GPIO_PIN_SET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK1_HIGH);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Mix2Limit]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_MIX_TANK2_LIMIT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp >MachineSetting.TimeOut[STATE_MANUAL_OVERFLOW_RECOVERY]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTimeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}
		break;
	}
	case STATE_MANUAL_FILL_FERTILIZER_TANK:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[ValveWaterSource], GPIO_PIN_SET);

		if(ReadInput(MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_HIGH]) == GPIO_PIN_RESET){
			WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FILL_VALVE], GPIO_PIN_RESET);
		}
		else{
			WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_FILL_VALVE], GPIO_PIN_SET);
		}

		if(ReadInput(MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_HIGH]) == GPIO_PIN_RESET){
			WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FILL_VALVE], GPIO_PIN_RESET);
		}
		else{
			WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_FILL_VALVE], GPIO_PIN_SET);
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp< DELAY_FOR_VALVE){
			NoFlowTimeStamp = HAL_GetTick();
			break;
		}
		WriteOutput(MachineSetting.generalSetting.Outputs.BasicOut[FertigationPump], GPIO_PIN_SET);

		if(ReadInput(MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_HIGH]) == GPIO_PIN_RESET && ReadInput(MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_HIGH]) == GPIO_PIN_RESET){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_FERT_TANKS_HIGH);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_RESERVOIRLOW + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_RESERVOIRLOW + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(MachineSetting.ElapsedTime > MachineSetting.TimeOut[STATE_MANUAL_FILL_FERTILIZER_TANK]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT +10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[Flow]) == GPIO_PIN_RESET){
			if((HAL_GetTick()-NoFlowTimeStamp) > NO_FLOW_DELAY){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_NOFLOW + 10000);
				MachineSetting.MachineState = STATE_ERROR;
				break;
			}
		}
		else{
			NoFlowTimeStamp = HAL_GetTick();
		}
		break;
	}
	case STATE_MANUAL_FILL_PESTICIDE_TANK:
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.MachineState = STATE_ABORT;
		break;
	case STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.isAbort != 0){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_RESET);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		WriteOutput(MachineSetting.generalSetting.Fert.Fert[0].out[FERTOUT_PROPELLER], GPIO_PIN_SET);
		WriteOutput(MachineSetting.generalSetting.Fert.Fert[1].out[FERTOUT_PROPELLER], GPIO_PIN_SET);
		if(MachineSetting.ElapsedTime>MachineSetting.manualEvent.TimeOut){
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + EXIT_ELAPSED_TIME_GREATER_THAN_DURATION);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp > MachineSetting.TimeOut[MachineSetting.MachineState]){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_TIMEOUT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		break;
	}
	case STATE_ABORT:
	{
		if(MachineSetting.SlaveCom.ErrorCount > 5){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_CONTROLBOARDFAIL + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.manualEvent.isManual != MachineSetting.manualEvent.isManualNew){
			MachineSetting.manualEvent.isManual = MachineSetting.manualEvent.isManualNew;
		}
		if(MachineSetting.isAbort != 0){
			MachineSetting.isAbort = 0;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		ClearErrorCode();
		ResetAllIO();



		MachineSetting.MachineState = STATE_IDLE;
		break;
	}
	case STATE_ERROR:
	{
		if(MachineSetting.isAbort != 0){
			MachineSetting.MachineState = STATE_ABORT;
			//break;
		}
		if(MachineSetting.isError != 0){
			MachineSetting.isError = 0;
		}
		ResetAllIO();
		break;
	}
	case STATE_COUNT:
		MachineSetting.MachineState = STATE_ABORT;
		break;
	}
}

static void StateMachineRunningTest(void){
	switch(MachineSetting.MachineState){
	case STATE_IDLE:
		if(MachineSetting.isAbort != 0){
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(MachineSetting.manualEvent.isManual != 0){

			switch (MachineSetting.manualEvent.triggerEvent){
			case STATE_MANUAL_MIXED_WATER_WATERING:
				MachineSetting.MachineState = STATE_MANUAL_MIXED_WATER_WATERING;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_WATER_RESERVOIR_REFILL:
				MachineSetting.MachineState = STATE_MANUAL_WATER_RESERVOIR_REFILL;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_CLEAN_WATER_WATERING:
				MachineSetting.MachineState = STATE_MANUAL_CLEAN_WATER_WATERING;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC:
				MachineSetting.MachineState = STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR:
				MachineSetting.MachineState = STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_FERTILIZER_ADDING:
				MachineSetting.MachineState = STATE_MANUAL_FERTILIZER_ADDING;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_OVERFLOW_RECOVERY:
				MachineSetting.MachineState = STATE_MANUAL_OVERFLOW_RECOVERY;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_FILL_FERTILIZER_TANK:
				MachineSetting.MachineState = STATE_MANUAL_FILL_FERTILIZER_TANK;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_FILL_PESTICIDE_TANK:
				MachineSetting.MachineState = STATE_MANUAL_FILL_PESTICIDE_TANK;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			case STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER:
				MachineSetting.MachineState = STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER;
				MachineSetting.TimeStamp = HAL_GetTick();
				break;
			default:
				break;
			}
			MachineSetting.manualEvent.triggerEvent = STATE_IDLE;
			break;
		}
		if(MachineSetting.TaskProxyBuffer.InfoFIFO.count != 0){
			if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime >= MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration){
				TaskHistory taskHistory = {0};
				memcpy(taskHistory.taskID, MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].taskID, TASK_ID_BYTE_COUNT);

				if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration> MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime){
					taskHistory.incompleteDuration = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration -  MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
				}
				else{
					taskHistory.incompleteDuration = 0;
				}
				taskHistory.end = HAL_GetTick();
				taskHistory.start = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].timeStamp;
				TaskHistoryFIFO_Push(&MachineSetting.TaskProxyBuffer.HistoryFIFO, &taskHistory);
				TaskInfoFIFO_RemoveFirst(&MachineSetting.TaskProxyBuffer.InfoFIFO);
				break;

			}
			else{
				if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].timeStamp == 0){
					MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].timeStamp = HAL_GetTick();
				}
				MachineSetting.MachineState = STATE_MIXED_WATER_WATERING;
				MachineSetting.TimeStamp = HAL_GetTick();
				//MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].timeStamp = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime;
			}
		}
		break;
	case STATE_WATER_RESERVOIR_REFILL:
		break;
	case STATE_CLEAN_WATER_WATERING:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(MachineSetting.generalSetting.Inputs.RealPinState[0].PinStatus == GPIO_PIN_SET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
		}
		if( MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].timeStamp + MachineSetting.ElapsedTime;
		break;
	case STATE_MIXED_WATER_WATERING:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(MachineSetting.generalSetting.Inputs.RealPinState[0].PinStatus == GPIO_PIN_SET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
		}
		if( MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].duration){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head].elapsedTime = MachineSetting.ElapsedTime;
		break;

	case STATE_CLEAN_WATER_FILLING_LOWER_EC:
		break;
	case STATE_WATER_FILLING_TO_MIX1_HIGHSENSOR:
		break;
	case STATE_FERTILIZER_ADDING:
		break;
	case STATE_OVERFLOW_RECOVERY:
		break;
	case STATE_MANUAL_WATER_RESERVOIR_REFILL:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp>10000){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_MANUAL_MIXED_WATER_WATERING:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp>MachineSetting.manualEvent.TimeOut){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_MANUAL_CLEAN_WATER_WATERING:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp>MachineSetting.manualEvent.TimeOut){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp>10000){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp>10000){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_MANUAL_FERTILIZER_ADDING:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp>10000){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_MANUAL_OVERFLOW_RECOVERY:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp>10000){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_MANUAL_FILL_FERTILIZER_TANK:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp>10000){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_MANUAL_FILL_PESTICIDE_TANK:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GetTick()-MachineSetting.TimeStamp>10000){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		break;
	case STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER:
		if(MachineSetting.isAbort != 0){
			uint32_t rand = HAL_GetTick();
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
			MachineSetting.MachineState = STATE_ABORT;
			break;
		}
		if(MachineSetting.isError != 0){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET){
			uint32_t rand = HAL_GetTick();
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		if(HAL_GetTick()-MachineSetting.TimeStamp>MachineSetting.manualEvent.TimeOut){
			uint32_t rand = HAL_GetTick();
			if(rand%100<30){
				AddErrorCode((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%ERROR_TYPECOUNT + 10000);
				MachineSetting.MachineState = STATE_ERROR;

			}
			else{
				AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + rand%EXIT_TYPECOUNT);
				MachineSetting.MachineState = STATE_ABORT;
			}
			return;
		}
		MachineSetting.ElapsedTime = HAL_GetTick()-MachineSetting.TimeStamp;
		break;
	case STATE_ABORT:
		if(MachineSetting.manualEvent.isManual != MachineSetting.manualEvent.isManualNew){
			MachineSetting.manualEvent.isManual = MachineSetting.manualEvent.isManualNew;
		}
		if(MachineSetting.isAbort != 0){
			MachineSetting.isAbort = 0;
		}
		if(MachineSetting.isError != 0){
			AddErrorCode((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			AppendStateHistory((uint16_t)MachineSetting.MachineState*100 + ERROR_ESTOP + 10000);
			MachineSetting.MachineState = STATE_ERROR;
			break;
		}
		ClearErrorCode();
		ResetAllIO();
		MachineSetting.MachineState = STATE_IDLE;
		break;
	case STATE_ERROR:
		if(MachineSetting.isAbort != 0){
			MachineSetting.MachineState = STATE_ABORT;
			//break;
		}
		if(MachineSetting.isError != 0){
			MachineSetting.isError = 0;
		}

		ResetAllIO();
		break;
	case STATE_COUNT:
		break;
	}
}

static void WriteOutput(uint8_t PinNo, GPIO_PinState IsSet){
	MachineSetting.generalSetting.Outputs.RealPinState[PinNo].PinStatus = IsSet;
}

static GPIO_PinState ReadOutput(uint8_t PinNo){
	return MachineSetting.generalSetting.Outputs.RealPinState[PinNo].PinStatus;
}

static GPIO_PinState ReadInput(uint8_t PinNo){
	return MachineSetting.generalSetting.Inputs.RealPinState[PinNo].PinStatus;
}

static void ResetAllIO(void){
	for(int i = 0; i< LOCAL_OUT+OUT_PER_SLAVE*MAX_SLAVE ; i++){
		WriteOutput(i, GPIO_PIN_RESET);
	}
}

static void IOListUpdate(void){
	for(int i = 0; i<LOCAL_OUT;i++){
		if(MachineSetting.generalSetting.Outputs.RealPinState[i].IsBind != 0 || MachineSetting.SlaveCom.isSlave != 0){
			HAL_GPIO_WritePin(MachineSetting.generalSetting.Outputs.LocalPin[i].Port, MachineSetting.generalSetting.Outputs.LocalPin[i].Pin, !MachineSetting.generalSetting.Outputs.RealPinState[i].PinStatus);
		}
		else{
			HAL_GPIO_WritePin(MachineSetting.generalSetting.Outputs.LocalPin[i].Port, MachineSetting.generalSetting.Outputs.LocalPin[i].Pin, GPIO_PIN_SET);
		}
	}
	for(int i = 0; i < LOCAL_IN;i++){
		if(MachineSetting.generalSetting.Inputs.RealPinState[i].IsBind != 0 || MachineSetting.SlaveCom.isSlave != 0){
			GPIO_PinState IsSet = !HAL_GPIO_ReadPin(MachineSetting.generalSetting.Inputs.LocalPin[i].Port, MachineSetting.generalSetting.Inputs.LocalPin[i].Pin);
			if(IsSet != MachineSetting.generalSetting.Inputs.RealPinState[i].PinStatus){

				if(MachineSetting.generalSetting.Inputs.RealPinState[i].AccumulatedChange > INPUT_CHANGE_THRESHOLD){
					MachineSetting.generalSetting.Inputs.RealPinState[i].PinStatus = IsSet;
					MachineSetting.generalSetting.Inputs.RealPinState[i].AccumulatedChange = 0;
				}
				else{
					MachineSetting.generalSetting.Inputs.RealPinState[i].AccumulatedChange +=1;
				}
			}
			else{
				MachineSetting.generalSetting.Inputs.RealPinState[i].AccumulatedChange = 0;
			}
		}
		else{
			MachineSetting.generalSetting.Inputs.RealPinState[i].PinStatus = GPIO_PIN_RESET;
		}

	}
}

static void SlaveCom(void){
	if(MachineSetting.SlaveCom.isSlave != 0){
		if(MachineSetting.SlaveCom.bufferCount != 0 && HAL_GetTick()-MachineSetting.SlaveCom.timeStamp > MachineSetting.SlaveCom.intercharTimeOut){
			uint8_t bufferCount = 0;
			HAL_UART_AbortReceive(MachineSetting.SlaveCom.huart);
			if(MachineSetting.SlaveCom.bufferCount == 1+ (LOCAL_OUT+BITS_PER_BYTE-1)/BITS_PER_BYTE +(LOCAL_IN+BITS_PER_BYTE-1)/BITS_PER_BYTE && MachineSetting.SlaveCom.Buffer[bufferCount++] == MachineSetting.SlaveCom.currentSlave){
				for(int i = 0; i<(LOCAL_OUT-1+BITS_PER_BYTE)/BITS_PER_BYTE; i++){
					uint8_t currentByte = MachineSetting.SlaveCom.Buffer[bufferCount++];
					for (int j = 0; j< BITS_PER_BYTE; j++){
						if(i*BITS_PER_BYTE + j  < LOCAL_OUT){
							if(((currentByte>>(BITS_PER_BYTE-j-1))& 1) == 1){
								MachineSetting.generalSetting.Outputs.RealPinState[i*BITS_PER_BYTE + j].PinStatus =GPIO_PIN_SET;
							}
							else{
								MachineSetting.generalSetting.Outputs.RealPinState[i*BITS_PER_BYTE + j].PinStatus =GPIO_PIN_RESET;
							}
						}
						else{
							break;
						}
					}
				}
				MachineSetting.SlaveCom.bufferCount = 1 + (LOCAL_IN+BITS_PER_BYTE-1)/BITS_PER_BYTE;
				for(int i = 0; i < (LOCAL_IN+BITS_PER_BYTE-1)/BITS_PER_BYTE;i++){
					uint8_t currentByte = 0;
					for(int j = 0; i<BITS_PER_BYTE;j++){
						if(i*BITS_PER_BYTE + j  < LOCAL_OUT){
							if(MachineSetting.generalSetting.Inputs.RealPinState[i*BITS_PER_BYTE + j].PinStatus == GPIO_PIN_SET){
								currentByte |= (1<<(BITS_PER_BYTE-1-j));
							}
						}
						else{
							break;
						}
					}
					MachineSetting.SlaveCom.Buffer[MachineSetting.SlaveCom.bufferCount++] = currentByte;
				}
				HAL_GPIO_WritePin(MachineSetting.SlaveCom.DirectionalPin.Port, MachineSetting.SlaveCom.DirectionalPin.Pin, GPIO_PIN_SET);
				HAL_UART_Transmit(MachineSetting.SlaveCom.huart, MachineSetting.SlaveCom.Buffer, MachineSetting.SlaveCom.bufferCount, MachineSetting.SlaveCom.txTimeOut);
				HAL_GPIO_WritePin(MachineSetting.SlaveCom.DirectionalPin.Port, MachineSetting.SlaveCom.DirectionalPin.Pin, GPIO_PIN_RESET);
				MachineSetting.SlaveCom.bufferCount = 0;
				HAL_UART_Receive_IT(MachineSetting.SlaveCom.huart, &MachineSetting.SlaveCom.Buffer[MachineSetting.SlaveCom.bufferCount], 1);
			}
			else{
				MachineSetting.SlaveCom.bufferCount = 0;
				HAL_UART_Receive_IT(MachineSetting.SlaveCom.huart, &MachineSetting.SlaveCom.Buffer[MachineSetting.SlaveCom.bufferCount], 1);
			}
		}
		else if(HAL_GetTick()-MachineSetting.SlaveCom.timeStamp > MAX_SLAVE_NO_CALL_FROM_MASTER_TIME){
			for(int i = 0; i<LOCAL_OUT; i++){
				MachineSetting.generalSetting.Outputs.RealPinState[i].PinStatus =GPIO_PIN_SET;
			}
		}
		return;
	}
	if(MachineSetting.SlaveCom.slaveCount == 0){
		return;
	}
	switch(MachineSetting.SlaveCom.state){
	case SLAVE_COM_IDLE:
		MachineSetting.SlaveCom.bufferCount = 0;
		MachineSetting.SlaveCom.Buffer[MachineSetting.SlaveCom.bufferCount++] = MachineSetting.SlaveCom.currentSlave+1;
		for(int i = 0; i < (OUT_PER_SLAVE+BITS_PER_BYTE-1)/BITS_PER_BYTE; i++){
			uint8_t currentByte = 0;
			for(int j = 0; j<BITS_PER_BYTE; j++){
				if(i*BITS_PER_BYTE+j >= OUT_PER_SLAVE){
					break;
				}
				if(MachineSetting.generalSetting.Outputs.RealPinState[LOCAL_OUT + OUT_PER_SLAVE*MachineSetting.SlaveCom.currentSlave + i*BITS_PER_BYTE+j].PinStatus == GPIO_PIN_SET){
					currentByte |= 1<<(BITS_PER_BYTE-j-1);
				}
			}
			MachineSetting.SlaveCom.Buffer[MachineSetting.SlaveCom.bufferCount++] = currentByte;
		}
		MachineSetting.SlaveCom.bufferCount += (IN_PER_SLAVE+BITS_PER_BYTE-1)/BITS_PER_BYTE;
		HAL_GPIO_WritePin(MachineSetting.SlaveCom.DirectionalPin.Port, MachineSetting.SlaveCom.DirectionalPin.Pin, GPIO_PIN_SET);
		MachineSetting.SlaveCom.timeStamp = HAL_GetTick();
		HAL_UART_Transmit_IT(MachineSetting.SlaveCom.huart, MachineSetting.SlaveCom.Buffer, MachineSetting.SlaveCom.bufferCount);
		MachineSetting.SlaveCom.state = SLAVE_COM_TX;
		break;
	case SLAVE_COM_TX:
		if(HAL_GetTick() - MachineSetting.SlaveCom.timeStamp > MachineSetting.SlaveCom.txTimeOut){
			MachineSetting.SlaveCom.ErrorCount ++;
			HAL_UART_AbortTransmit(MachineSetting.SlaveCom.huart);
			HAL_GPIO_WritePin(MachineSetting.SlaveCom.DirectionalPin.Port, MachineSetting.SlaveCom.DirectionalPin.Pin, GPIO_PIN_RESET);
			if(MachineSetting.SlaveCom.ErrorCount >= 5 ){
				MachineSetting.SlaveCom.state = SLAVE_COM_ERROR;
			}
			else{
				MachineSetting.SlaveCom.state = SLAVE_COM_IDLE;
			}
		}
		break;
	case SLAVE_COM_PRE_RX:
		if(HAL_GetTick()- MachineSetting.SlaveCom.timeStamp > MachineSetting.SlaveCom.rxTimeOut){
			MachineSetting.SlaveCom.ErrorCount ++;
			HAL_UART_AbortReceive(MachineSetting.SlaveCom.huart);
			if(MachineSetting.SlaveCom.ErrorCount >= 5 ){
				MachineSetting.SlaveCom.state = SLAVE_COM_ERROR;
			}
			else{
				MachineSetting.SlaveCom.state = SLAVE_COM_IDLE;
			}
		}
		break;
	case SLAVE_COM_RX:
		if((HAL_GetTick()-MachineSetting.SlaveCom.timeStamp > MachineSetting.SlaveCom.intercharTimeOut)){
			HAL_UART_AbortReceive(MachineSetting.SlaveCom.huart);
			uint8_t bufferCount = 0;
			if((MachineSetting.SlaveCom.bufferCount == (OUT_PER_SLAVE+BITS_PER_BYTE-1)/BITS_PER_BYTE + (IN_PER_SLAVE+BITS_PER_BYTE-1)/BITS_PER_BYTE + 1) && (MachineSetting.SlaveCom.Buffer[bufferCount++] == MachineSetting.SlaveCom.currentSlave+1) ){
				MachineSetting.SlaveCom.ErrorCount = 0;
				bufferCount += (OUT_PER_SLAVE+BITS_PER_BYTE-1)/BITS_PER_BYTE;
				for(int i = 0; i<(IN_PER_SLAVE-1+BITS_PER_BYTE)/BITS_PER_BYTE; i++){
					uint8_t currentByte = MachineSetting.SlaveCom.Buffer[bufferCount++];
					for (int j = 0; j< BITS_PER_BYTE; j++){
						if(i*BITS_PER_BYTE + j  < IN_PER_SLAVE){
							if(((currentByte>>(BITS_PER_BYTE-j-1))& 1) == 1){
								if(MachineSetting.generalSetting.Inputs.RealPinState[LOCAL_IN + MachineSetting.SlaveCom.currentSlave*IN_PER_SLAVE + i*BITS_PER_BYTE + j].PinStatus !=GPIO_PIN_SET){
									MachineSetting.generalSetting.Inputs.RealPinState[LOCAL_IN + MachineSetting.SlaveCom.currentSlave*IN_PER_SLAVE + i*BITS_PER_BYTE + j].PinStatus =GPIO_PIN_SET;
								}
							}
							else{
								if(MachineSetting.generalSetting.Inputs.RealPinState[LOCAL_IN + MachineSetting.SlaveCom.currentSlave*IN_PER_SLAVE + i*BITS_PER_BYTE + j].PinStatus !=GPIO_PIN_RESET){
									MachineSetting.generalSetting.Inputs.RealPinState[LOCAL_IN + MachineSetting.SlaveCom.currentSlave*IN_PER_SLAVE + i*BITS_PER_BYTE + j].PinStatus =GPIO_PIN_RESET;
								}
							}
						}
						else{
							break;
						}
					}
				}

			}
			else{
				MachineSetting.SlaveCom.ErrorCount ++;
			}

			if(MachineSetting.SlaveCom.ErrorCount >= 5 ){
				MachineSetting.SlaveCom.state = SLAVE_COM_ERROR;
			}
			else{
				MachineSetting.SlaveCom.state = SLAVE_COM_RX_DONE;
			}

		}
		break;
	case SLAVE_COM_RX_DONE:
		MachineSetting.SlaveCom.ErrorCount = 0;
		MachineSetting.SlaveCom.bufferCount = 0;
		if(MachineSetting.SlaveCom.slaveCount != 0){
			MachineSetting.SlaveCom.currentSlave = (MachineSetting.SlaveCom.currentSlave+1)%MachineSetting.SlaveCom.slaveCount;
		}
		MachineSetting.SlaveCom.state = SLAVE_COM_IDLE;
		break;
	case SLAVE_COM_ERROR:
		MachineSetting.SlaveCom.state = SLAVE_COM_IDLE;
		break;
	}

}

static void CheckTCP(void){
	if(MachineSetting.TCPCom.MasterRequest.isNewCommandComing != 0){
		if(MachineSetting.TCPCom.MasterRequest.isNewCommandComing < 2){
			HAL_GPIO_WritePin(MachineSetting.TCPCom.DirectionalPin.Port, MachineSetting.TCPCom.DirectionalPin.Pin, GPIO_PIN_SET);
			//HAL_UART_Transmit(MachineSetting.TCPCom.huart, MachineSetting.TCPCom.Buffer, MachineSetting.TCPCom.BufferCount, MachineSetting.TCPCom.txTimeOut);
		//		MachineSetting.TCPCom.BufferCount = 0;
		//		memset(&MachineSetting.TCPCom.Buffer, 0, sizeof(TCP_Buffer_MAX_Count));
			int jsonParseSuccess = 0;
			jsonParseSuccess = build_status_json((char*)MachineSetting.TCPCom.Buffer, TCP_Buffer_MAX_Count);
			if(jsonParseSuccess == -1){

			}
			else{
				MachineSetting.TCPCom.BufferCount = jsonParseSuccess;
				HAL_UART_Transmit(MachineSetting.TCPCom.huart,MachineSetting.TCPCom.Buffer, MachineSetting.TCPCom.BufferCount, MachineSetting.TCPCom.txTimeOut);
			}
			HAL_GPIO_WritePin(MachineSetting.TCPCom.DirectionalPin.Port, MachineSetting.TCPCom.DirectionalPin.Pin, GPIO_PIN_RESET);
			MachineSetting.TCPCom.BufferCount = 0;
			memset(MachineSetting.TCPCom.Buffer, 0, sizeof(MachineSetting.TCPCom.Buffer));
			MachineSetting.TCPCom.TimeStamp = HAL_GetTick();
			HAL_UART_Receive_IT(MachineSetting.TCPCom.huart, MachineSetting.TCPCom.Buffer, 1);
			MachineSetting.TCPCom.MasterRequest.isNewCommandComing = 0;
		}
	}
	if ((HAL_GetTick()-MachineSetting.TCPCom.TimeStamp > MachineSetting.TCPCom.intercharTimeOut) && MachineSetting.TCPCom.BufferCount>0){
		HAL_UART_AbortReceive(MachineSetting.TCPCom.huart);
		parse_master_request((char*)MachineSetting.TCPCom.Buffer, &MachineSetting.TCPCom.MasterRequest);
		executeCommand();
		memset(&MachineSetting.TCPCom.Buffer,0, sizeof(MachineSetting.TCPCom.Buffer));
		MachineSetting.TCPCom.BufferCount = 0;
		MachineSetting.TCPCom.MasterRequest.isNewCommandComing = 1;

	}
	else if(HAL_GetTick()-MachineSetting.TCPCom.TimeStamp > MachineSetting.TCPCom.maxTimeDisconnectFromPC){
		HAL_UART_AbortReceive(MachineSetting.TCPCom.huart);
		memset(&MachineSetting.TCPCom.Buffer, 0, sizeof(MachineSetting.TCPCom.Buffer));
		MachineSetting.TCPCom.BufferCount = 0;
		HAL_UART_Receive_IT(MachineSetting.TCPCom.huart, MachineSetting.TCPCom.Buffer, 1);
		MachineSetting.TCPCom.TimeStamp = HAL_GetTick();
	}
}

static int parse_master_request(const char *json, masterRequest *req) {
    jsmn_parser parser;
    static jsmntok_t tokens[32];
    jsmn_init(&parser);
    int num = jsmn_parse(&parser, json, strlen(json), tokens, sizeof(tokens)/sizeof(tokens[0]));
    if (num < 0) return -1;

    req->cmd = CMD_UNKNOWN;
    memset(&req->task, 0, sizeof(TaskInfo));
    memset(&req->manual, 0, sizeof(ManualEvent));

    jsmntok_t *task_tok = NULL;
    jsmntok_t *manual_tok = NULL;
    char event_buf[32] = {0};

    // First pass: find top-level keys
    for (int i = 1; i < num; i++) {
        if (jsoneq(json, &tokens[i], "event") == 0) {
            jsmntok_t *val = &tokens[i+1];
            snprintf(event_buf, sizeof(event_buf), "%.*s", val->end - val->start, json + val->start);
            i++; // skip value
        }
        else if (jsoneq(json, &tokens[i], "task") == 0) {
            task_tok = &tokens[i+1];
            i++;
        }
        else if (jsoneq(json, &tokens[i], "manual") == 0) {
            manual_tok = &tokens[i+1];
            i++;
        }
    }

    // Determine command based on event string
    if (strcmp(event_buf, "heartbeat") == 0) req->cmd = CMD_HEARTBEAT;
    else if (strcmp(event_buf, "reset") == 0) req->cmd = CMD_RESET;
    else if (strcmp(event_buf, "clear_task") == 0) req->cmd = CMD_CLEAR_TASK;
    else if (strcmp(event_buf, "clear_tasks") == 0) req->cmd = CMD_CLEAR_TASKS;
    else if (strcmp(event_buf, "estop") == 0) req->cmd = CMD_ESTOP;
    else if (strcmp(event_buf, "write_task") == 0) req->cmd = CMD_WRITE_TASK;
    else if(strcmp(event_buf, "calibrate_ec") == 0) req->cmd = CMD_CALIBRATE_EC;
    else if (strcmp(event_buf, "change_to_manual") == 0) req->cmd = CMD_MANUAL_ENTER;
    else if (strcmp(event_buf, "change_to_auto") == 0) req->cmd = CMD_MANUAL_EXIT;
    else if (strcmp(event_buf, "manual_trigger") == 0) req->cmd = CMD_MANUAL_TRIGGER;

    // Parse additional data based on command
    if ((req->cmd == CMD_WRITE_TASK || req->cmd == CMD_CLEAR_TASK) && task_tok != NULL && task_tok->type == JSMN_OBJECT) {
        parse_task_object(json, task_tok, req);
    }
    else if (req->cmd == CMD_MANUAL_TRIGGER && manual_tok != NULL && manual_tok->type == JSMN_OBJECT) {
        // Parse the manual object
        int pair_count = manual_tok->size;
        int idx = 1; // relative to manual_tok
        for (int k = 0; k < pair_count; k++) {
            jsmntok_t *key = &manual_tok[idx];
            jsmntok_t *val = &manual_tok[idx+1];
            if (jsoneq(json, key, "state") == 0) {
                char state_buf[64];
                snprintf(state_buf, sizeof(state_buf), "%.*s", val->end - val->start, json + val->start);
                req->manual.triggerEvent = string_to_state(state_buf);
            } else if (jsoneq(json, key, "timeout") == 0) {
                req->manual.TimeOut = (uint32_t)parse_int(json, val);
            } else if (jsoneq(json, key, "targetEC") == 0) {
                req->manual.TargetEC = parse_float(json, val);
            } else if (jsoneq(json, key, "plotID") == 0) {
				req->manual.plotID = (uint8_t)parse_int(json, val);
			} else if(jsoneq(json, key, "taskID")== 0){
				snprintf((char*)req->manual.taskID, TASK_ID_BYTE_COUNT, "%.*s", val->end - val->start, json + val->start);
			}

            idx += 2;
        }
        req->manual.isManual = 1;  // mark that a manual trigger is requested
    }
    else if (req->cmd == CMD_CALIBRATE_EC && manual_tok != NULL && manual_tok->type == JSMN_OBJECT){
    	int pair_count = manual_tok->size;
		int idx = 1; // relative to manual_tok
		for (int k = 0; k < pair_count; k++) {
			jsmntok_t *key = &manual_tok[idx];
			jsmntok_t *val = &manual_tok[idx+1];
			if (jsoneq(json, key, "targetEC") == 0) {
				req->manual.TargetEC = parse_float(json, val);
			}
			idx += 2;
		}
    }

    return 0;
}

//static int parse_master_request_old(const char *json, masterRequest *req) {
//    jsmn_parser parser;
//    jsmntok_t tokens[64];
//    jsmn_init(&parser);
//    int num = jsmn_parse(&parser, json, strlen(json), tokens, sizeof(tokens)/sizeof(tokens[0]));
//    if (num < 0) return -1;
//
//    req->cmd = CMD_UNKNOWN;
//    memset(&req->task, 0, sizeof(TaskInfo));
//    jsmntok_t *task_tok = NULL;
//
//    for (int i = 1; i < num; i++) {
//        if (jsoneq(json, &tokens[i], "event") == 0) {
//            jsmntok_t *val = &tokens[i+1];
//            char buf[16];
//            snprintf(buf, sizeof(buf), "%.*s", val->end - val->start, json + val->start);
//            if (strcmp(buf, "heartbeat") == 0) req->cmd = CMD_HEARTBEAT;
//            else if (strcmp(buf, "reset") == 0) req->cmd = CMD_RESET;
//            else if (strcmp(buf, "clear_tasks") == 0) req->cmd = CMD_CLEAR_TASKS;
//            else if (strcmp(buf, "estop") == 0) req->cmd = CMD_ESTOP;
//            else if (strcmp(buf, "write_task") == 0) req->cmd = CMD_WRITE_TASK;
//            i++; // skip value
//        }
//        else if (jsoneq(json, &tokens[i], "task") == 0) {
//            // Remember the token for the task object (the value after "task")
//            task_tok = &tokens[i+1];
//            i++; // skip value
//        }
//    }
//
//    // Now handle the task if needed
//    if (req->cmd == CMD_WRITE_TASK && task_tok != NULL && task_tok->type == JSMN_OBJECT) {
//        // Parse the task object into req->task
//        parse_task_object(json, task_tok, req);
//    }
//
//    return 0;
//}

static void parse_task_object(const char *json, jsmntok_t *task_tok, masterRequest *req) {
    TaskInfo temp = {0};   // zero-initialized
    int pair_count = task_tok->size;
    int idx = 1; // skip the object token itself

    for (int i = 0; i < pair_count; i++) {
        jsmntok_t *key = &task_tok[idx];
        jsmntok_t *val = &task_tok[idx+1];
        if (jsoneq(json, key, "plotID") == 0) {
            temp.plotID = (uint8_t)parse_int(json, val);
        } else if(jsoneq(json, key, "taskID") == 0){
        	int len = val->end - val->start;
        	if (len > 0 && len < TASK_ID_BYTE_COUNT) {
        	    snprintf((char*)temp.taskID, sizeof(temp.taskID), "%.*s", len, json + val->start);
        	} else {
        	    temp.taskID[0] = '\0'; // Empty string
        	}
        } else if (jsoneq(json, key, "duration") == 0) {
            temp.duration = (uint32_t)parse_int(json, val);
        } else if (jsoneq(json, key, "TargetEC") == 0) {
            temp.TargetEC = parse_float(json, val);
        } else if (jsoneq(json, key, "Hysteresis") == 0) {
            temp.Hysterisis = parse_float(json, val);
        } else if (jsoneq(json, key, "ECRatio") == 0 && val->type == JSMN_ARRAY) {
            int arr_len = val->size;
            if (arr_len > MAX_FERT_TYPE) arr_len = MAX_FERT_TYPE;
            // Calculate starting index of array elements
            int arr_start = (int)(val - task_tok) + 1;
            for (int j = 0; j < arr_len; j++) {
                jsmntok_t *elem = &task_tok[arr_start + j];
                temp.ECRatio[j] = (uint8_t)parse_int(json, elem);
            }
        }
        idx += 2;
    }

    // After parsing, copy the whole struct to req->task
    // (You may want to validate fields here, e.g., plotID != 0, duration > 0)
    if (temp.plotID != 0 && temp.duration > 0) {
        memcpy(&req->task, &temp, sizeof(TaskInfo));
    } else if(req->cmd == CMD_CLEAR_TASK){
    	memcpy(&req->task, &temp, sizeof(TaskInfo));
    }
    else
    {
        // Optionally set req->cmd to CMD_UNKNOWN or handle error
        req->cmd = CMD_UNKNOWN;
    }
}


static int jsoneq(const char *json, jsmntok_t *tok, const char *s){
	if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
		strncmp(json + tok->start, s, tok->end - tok->start) == 0)
		return 0;
	return -1;
}

static uint32_t parse_int(const char *json, jsmntok_t *tok)
{
    uint32_t val = 0;
    const char *p = json + tok->start;
    const char *end = json + tok->end;
    if (*p == '-') {p++; }
    for (; p < end; p++) {
        if (*p >= '0' && *p <= '9')
            val = val * 10 + (*p - '0');
        else
            break;
    }
    return val;
}

static float parse_float(const char *json, jsmntok_t *tok) {
    const char *p = json + tok->start;
    const char *end = json + tok->end;
    float result = 0.0f;
    float sign = 1.0f;
    float fraction = 0.0f;
    int frac_digits = 0;
    int exp_sign = 1;
    int exponent = 0;

    // sign
    if (*p == '-') { sign = -1.0f; p++; }
    else if (*p == '+') p++;

    // integer part
    while (p < end && *p >= '0' && *p <= '9') {
        result = result * 10.0f + (*p - '0');
        p++;
    }

    // fractional part
    if (p < end && *p == '.') {
        p++;
        while (p < end && *p >= '0' && *p <= '9') {
            fraction = fraction * 10.0f + (*p - '0');
            frac_digits++;
            p++;
        }
    }
    result += fraction / powhere10(frac_digits); // need a helper

    // exponent (e/E)
    if (p < end && (*p == 'e' || *p == 'E')) {
        p++;
        if (*p == '-') { exp_sign = -1; p++; }
        else if (*p == '+') p++;
        while (p < end && *p >= '0' && *p <= '9') {
            exponent = exponent * 10 + (*p - '0');
            p++;
        }
    }

    result *= sign;
    if (exponent != 0) {
        result *= powhere10(exp_sign * exponent);
    }
    return result;
}

// Helper: integer power of 10 (for fraction denominator and exponent)
static float powhere10(int exp) {
    float r = 1.0f;
    if (exp >= 0) {
        for (int i = 0; i < exp; i++) r *= 10.0f;
    } else {
        for (int i = 0; i < -exp; i++) r /= 10.0f;
    }
    return r;
}


static void executeCommand(void){
	switch(MachineSetting.TCPCom.MasterRequest.cmd){
	case CMD_HEARTBEAT:
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		break;
	case CMD_RESET:
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		MachineSetting.isAbort = 1;
		break;
	case CMD_CLEAR_TASK:
		MachineSetting.isAbort = 1;
		MoveTaskToTaskHistory(MachineSetting.TCPCom.MasterRequest.task.taskID);
		TaskInfoFIFO_Remove(&MachineSetting.TaskProxyBuffer.InfoFIFO,MachineSetting.TCPCom.MasterRequest.task.taskID);
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		break;
	case CMD_CLEAR_TASKS:
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		MoveAllTasksToTaskHistory();
		TaskInfoFIFO_RemoveAll(&MachineSetting.TaskProxyBuffer.InfoFIFO);
		MachineSetting.isAbort = 1;
		break;
	case CMD_ESTOP:
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		MachineSetting.isError = 1;
		break;
	case CMD_WRITE_TASK:
		MachineSetting.TCPCom.MasterRequest.task.Hysterisis = 150;
		MachineSetting.TCPCom.MasterRequest.task.plotID -= 1;
		MachineSetting.TCPCom.MasterRequest.task.timeStamp = 0;
		TaskInfoFIFO_Push(&MachineSetting.TaskProxyBuffer.InfoFIFO, &MachineSetting.TCPCom.MasterRequest.task);
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		break;
	case CMD_CALIBRATE_EC:
		if(MachineSetting.generalSetting.readingEC.modbusSetting.isError != 0 && MachineSetting.generalSetting.readingEC.MeanEC >10.0){
			MachineSetting.generalSetting.readingEC.modbusSetting.factor = MachineSetting.TCPCom.MasterRequest.manual.TargetEC/MachineSetting.generalSetting.readingEC.MeanEC;
		}
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		break;
	case CMD_MANUAL_ENTER:
		if(MachineSetting.manualEvent.isManual != 1){
			MachineSetting.manualEvent.isManualNew = 1;
			MachineSetting.isAbort = 1;
		}
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		break;
	case CMD_MANUAL_EXIT:
		if(MachineSetting.manualEvent.isManual != 0){
			MachineSetting.manualEvent.isManualNew = 0;
			MachineSetting.isAbort = 1;
		}
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		break;
	case CMD_MANUAL_TRIGGER:
		if(MachineSetting.manualEvent.isManual!= 0 && MachineSetting.MachineState == STATE_IDLE){
			switch (MachineSetting.TCPCom.MasterRequest.manual.triggerEvent){
			case STATE_MANUAL_MIXED_WATER_WATERING:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_MIXED_WATER_WATERING;
				MachineSetting.manualEvent.plotID = MachineSetting.TCPCom.MasterRequest.manual.plotID-1;
				MachineSetting.manualEvent.TimeOut = MachineSetting.TCPCom.MasterRequest.manual.TimeOut;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_WATER_RESERVOIR_REFILL:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_WATER_RESERVOIR_REFILL;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_CLEAN_WATER_WATERING:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_CLEAN_WATER_WATERING;
				MachineSetting.manualEvent.plotID = MachineSetting.TCPCom.MasterRequest.manual.plotID-1;
				MachineSetting.manualEvent.TimeOut = MachineSetting.TCPCom.MasterRequest.manual.TimeOut;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC;
				MachineSetting.manualEvent.TargetEC = MachineSetting.TCPCom.MasterRequest.manual.TargetEC;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_FERTILIZER_ADDING:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_FERTILIZER_ADDING;
				MachineSetting.manualEvent.TargetEC = MachineSetting.TCPCom.MasterRequest.manual.TargetEC;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_OVERFLOW_RECOVERY:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_OVERFLOW_RECOVERY;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_FILL_FERTILIZER_TANK:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_FILL_FERTILIZER_TANK;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_FILL_PESTICIDE_TANK:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_FILL_PESTICIDE_TANK;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			case STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER:
				MachineSetting.manualEvent.triggerEvent = STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER;
				MachineSetting.manualEvent.TimeOut = MachineSetting.TCPCom.MasterRequest.manual.TimeOut;
				memcpy(MachineSetting.manualEvent.taskID, MachineSetting.TCPCom.MasterRequest.manual.taskID, TASK_ID_BYTE_COUNT);
				break;
			default:
				break;
			}
		}
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		break;
	case CMD_UNKNOWN:
		memset(&MachineSetting.TCPCom.MasterRequest.task, 0,  sizeof(TaskInfo));
		memset(&MachineSetting.TCPCom.MasterRequest.manual, 0,  sizeof(ManualEvent));
		break;
	}
	MachineSetting.TCPCom.MasterRequest.cmd = CMD_HEARTBEAT;
}

static void ModbusTransmitSwitchDevice(void){
	switch(modbusSwitchCase){
		case 0:
			if (mb2.modbusWriteQueue.count != 0){
				ModbusMaster_UpdateWriteTransaction(&mb2);
				ModbusMaster_WriteMultipleRegisters(&mb2);
				switch(mb2.modbusTransaction.slaveAddress){
					case 1:
						modbusSwitchCase = 20;
						break;
					case 2:
						modbusSwitchCase = 12;
						break;
				}
			}
			else{
				uint32_t currentTime = HAL_GetTick();
				if (currentTime-mb2.lastOperationTimeStamp>=mb2.modbusTransaction.delayTime){
					ModbusMaster_ReadHoldingRegister(&mb2);
					switch(mb2.modbusReadQueue.head){
						case 0:
							modbusSwitchCase = 10;
							break;
						case 1:
							modbusSwitchCase = 11;
							break;
						case 2:
							modbusSwitchCase = 12;
							break;
					}
				}
			}
			break;
		case 10:
			if (mb2.modbusState == MB_STATE_ERROR){
				if (mb2.modbusReadQueue.requestQueue[mb2.modbusReadQueue.head].errorCount>5){
					MachineSetting.generalSetting.readingEC.modbusSetting.isError = 1;
				}
				mb2.modbusReadQueue.head = (mb2.modbusReadQueue.head +1)%(mb2.modbusReadQueue.count);
				modbusSwitchCase = 100;
			}
			if(mb2.modbusState == MB_STATE_RX_COMPLETE){
				uint32_t slave1EC32 = mb2.modbusTransaction.data[0];
				MachineSetting.generalSetting.readingEC.modbusSetting.value = slave1EC32;
				float slave1EC = 0;

				slave1EC = MachineSetting.generalSetting.readingEC.modbusSetting.factor*slave1EC32;

				if(MachineSetting.generalSetting.readingEC.modbusSetting.isError != 0){
					MachineSetting.generalSetting.readingEC.MeanEC = slave1EC;
				}
				else{
					MachineSetting.generalSetting.readingEC.MeanEC = MachineSetting.generalSetting.readingEC.MeanEC*0+ slave1EC*1;
				}
				MachineSetting.generalSetting.readingEC.modbusSetting.isError = 0;
				mb2.modbusReadQueue.head = (mb2.modbusReadQueue.head +1)%(mb2.modbusReadQueue.count);
				modbusSwitchCase = 100;
			}
			break;
		case 11:
			if (mb2.modbusState == MB_STATE_ERROR){
				if (mb2.modbusReadQueue.requestQueue[mb2.modbusReadQueue.head].errorCount>5){
					MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.isError = 1;
				}
				mb2.modbusReadQueue.head = (mb2.modbusReadQueue.head +1)%(mb2.modbusReadQueue.count);
				modbusSwitchCase = 100;
			}
			if(mb2.modbusState == MB_STATE_RX_COMPLETE){
				float mb2slave2AccumulateVolumeA = (float)((uint32_t)mb2.modbusTransaction.data[0]  | mb2.modbusTransaction.data[1] << 16);
				mb2slave2AccumulateVolumeA = mb2slave2AccumulateVolumeA*MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.factor;
				MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.value = mb2slave2AccumulateVolumeA;
				mb2.modbusReadQueue.head = (mb2.modbusReadQueue.head +1)%(mb2.modbusReadQueue.count);
				modbusSwitchCase = 100;
				MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.isError = 0;
			}
			break;
		case 12:
			if (mb2.modbusState == MB_STATE_ERROR){
				if (mb2.modbusReadQueue.requestQueue[mb2.modbusReadQueue.head].errorCount>5){
					MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.isError = 1;
				}
				mb2.modbusReadQueue.head = (mb2.modbusReadQueue.head +1)%(mb2.modbusReadQueue.count);
				modbusSwitchCase = 100;
			}
			if(mb2.modbusState == MB_STATE_RX_COMPLETE){
				float mb2slave3AccumulateVolumeB = ((uint32_t)mb2.modbusTransaction.data[0]<<16 | mb2.modbusTransaction.data[1]);
				mb2slave3AccumulateVolumeB = mb2slave3AccumulateVolumeB*MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.factor;
				MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.value = mb2slave3AccumulateVolumeB;
				mb2.modbusReadQueue.head = (mb2.modbusReadQueue.head +1)%(mb2.modbusReadQueue.count);
				modbusSwitchCase = 100;
				MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.isError = 0;
			}
			break;
		case 20:
			if (mb2.modbusState == MB_STATE_TIMEOUT || mb2.modbusState == MB_STATE_ERROR){
				ModbusMaster_DeleteWriteQueue(&mb2);
				modbusSwitchCase = 100;
			}
			if(mb2.modbusState == MB_STATE_RX_COMPLETE){
				ModbusMaster_DeleteWriteQueue(&mb2);
				modbusSwitchCase = 100;
			}
			break;
		case 21:
		  break;
		case 22:
		  break;
		case 100:
			ModbusMaster_UpdateReadTransaction(&mb2);
			modbusSwitchCase = 0;
			break;
		}
		ModbusMaster_MonitorTransceive(&mb2);
}

static const char* state_to_string(machineState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_WATER_RESERVOIR_REFILL: return "WATER_RESERVOIR_REFILL";
        case STATE_CLEAN_WATER_WATERING: return "CLEAN_WATER_WATERING";
        case STATE_MIXED_WATER_WATERING: return "MIXED_WATER_WATERING";
        case STATE_CLEAN_WATER_FILLING_LOWER_EC: return "CLEAN_WATER_FILLING_LOWER_EC";
        case STATE_WATER_FILLING_TO_MIX1_HIGHSENSOR: return "WATER_FILLING_TO_MIX1_HIGHSENSOR";
        case STATE_FERTILIZER_ADDING: return "FERTILIZER_ADDING";
        case STATE_OVERFLOW_RECOVERY: return "OVERFLOW_RECOVERY";
        case STATE_MANUAL_CLEAN_WATER_WATERING: return "MANUAL_CLEAN_WATER_WATERING";
        case STATE_MANUAL_WATER_RESERVOIR_REFILL: return "MANUAL_WATER_RESERVOIR_REFILL";
        case STATE_MANUAL_MIXED_WATER_WATERING: return "MANUAL_MIXED_WATER_WATERING";
        case STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC: return "MANUAL_CLEAN_WATER_FILLING_LOWER_EC";
        case STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR: return "MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR";
        case STATE_MANUAL_FERTILIZER_ADDING: return "MANUAL_FERTILIZER_ADDING";
        case STATE_MANUAL_OVERFLOW_RECOVERY: return "MANUAL_OVERFLOW_RECOVERY";
        case STATE_MANUAL_FILL_FERTILIZER_TANK: return "MANUAL_FILL_FERTILIZER_TANK";
        case STATE_MANUAL_FILL_PESTICIDE_TANK: return "MANUAL_FILL_PESTICIDE_TANK";
        case STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER: return "MANUAL_DISSOLVE_SOLID_FERTILIZER";
        case STATE_ABORT: return "ABORT";
        case STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

static machineState string_to_state(const char *str) {
    if (strcmp(str, "IDLE") == 0) return STATE_IDLE;
    if (strcmp(str, "CLEAN_WATER_WATERING") == 0) return STATE_CLEAN_WATER_WATERING;
    if (strcmp(str, "MIXED_WATER_WATERING") == 0) return STATE_MIXED_WATER_WATERING;
    if (strcmp(str, "CLEAN_WATER_FILLING_LOWER_EC") == 0) return STATE_CLEAN_WATER_FILLING_LOWER_EC;
    if (strcmp(str, "WATER_FILLING_TO_MIX1_HIGHSENSOR") == 0) return STATE_WATER_FILLING_TO_MIX1_HIGHSENSOR;
    if (strcmp(str, "FERTILIZER_ADDING") == 0) return STATE_FERTILIZER_ADDING;
    if (strcmp(str, "OVERFLOW_RECOVERY") == 0) return STATE_OVERFLOW_RECOVERY;
    if(strcmp(str, "WATER_RESERVOIR_REFILL") == 0) return STATE_WATER_RESERVOIR_REFILL;
    if(strcmp(str, "MANUAL_WATER_RESERVOIR_REFILL") == 0) return STATE_MANUAL_WATER_RESERVOIR_REFILL;
    if (strcmp(str, "MANUAL_CLEAN_WATER_WATERING") == 0) return STATE_MANUAL_CLEAN_WATER_WATERING;
    if (strcmp(str, "MANUAL_MIXED_WATER_WATERING") == 0) return STATE_MANUAL_MIXED_WATER_WATERING;
    if (strcmp(str, "MANUAL_CLEAN_WATER_FILLING_LOWER_EC") == 0) return STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC;
    if (strcmp(str, "MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR") == 0) return STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR;
    if (strcmp(str, "MANUAL_FERTILIZER_ADDING") == 0) return STATE_MANUAL_FERTILIZER_ADDING;
    if (strcmp(str, "MANUAL_OVERFLOW_RECOVERY") == 0) return STATE_MANUAL_OVERFLOW_RECOVERY;
    if (strcmp(str, "MANUAL_FILL_FERTILIZER_TANK") == 0) return STATE_MANUAL_FILL_FERTILIZER_TANK;
    if (strcmp(str, "MANUAL_FILL_PESTICIDE_TANK") == 0) return STATE_MANUAL_FILL_PESTICIDE_TANK;
    if (strcmp(str, "MANUAL_DISSOLVE_SOLID_FERTILIZER") == 0) return STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER;
    if (strcmp(str, "ABORT") == 0) return STATE_ABORT;
    if (strcmp(str, "ERROR") == 0) return STATE_ERROR;
    // Default fallback
    return STATE_IDLE;
}

static int float_to_string(float value, char *str) {
    int int_part;
    int frac_part;
    char *p = str;
    if (value < 0) {
        *p++ = '-';
        value = -value;
    }
    int_part = (int)value;
    frac_part = (int)((value - int_part) * 1000 + 0.5f);
    p += sprintf(p, "%d", int_part);
    *p++ = '.';
    p += sprintf(p, "%03d", frac_part);
    return p - str;
}

static int build_status_json(char *buffer, size_t buf_size) {
    if (!buffer || buf_size == 0) return -1;

    TaskInfoFIFO *fifo = &MachineSetting.TaskProxyBuffer.InfoFIFO;
    uint8_t total_tasks = fifo->count;
    TaskHistoryFIFO *historyfifo = &MachineSetting.TaskProxyBuffer.HistoryFIFO;
    uint8_t total_taskhistorys = historyfifo->count;
    StateHistoryFIFO *statehistoryfifo = &MachineSetting.stateHistoryFIFO;
    uint8_t total_statehistorys = statehistoryfifo->count;
    char ec_str[16];
    int len = 0;
    float_to_string(MachineSetting.generalSetting.readingEC.MeanEC, ec_str);

    len += snprintf(buffer + len, buf_size - len,
                    "{\"state\":\"%s\",\"stateElapsedTime\":%u,\"currentTime\":%u,\"scanTime\":%u,\"manualMode\":%d,\"taskCount\":%d,\"currentEC\":%s,",
                    state_to_string(MachineSetting.MachineState),
					(unsigned int)MachineSetting.ElapsedTime,
					(unsigned int)MachineSetting.currentTick,
					(unsigned int)MachineSetting.scanTime,
                    MachineSetting.manualEvent.isManual ? 1 : 0,
                    total_tasks,
                    ec_str);
    if (len >= buf_size) return -1;
    if(total_tasks != 0){
		float_to_string(fifo->FIFO[fifo->head].TargetEC, ec_str);
		len += snprintf(buffer + len, buf_size - len,
				"\"currentTask\":{\"taskID\":\"%s\",\"plotID\":%u,\"duration\":%u,\"elapsedTime\":%u,\"targetEC\":%s,\"ecRatio\":[",
							(char*) fifo->FIFO[fifo->head].taskID,
							(unsigned int)fifo->FIFO[fifo->head].plotID,
							(unsigned int)fifo->FIFO[fifo->head].duration,
							(unsigned int)fifo->FIFO[fifo->head].elapsedTime,
							ec_str);
		if (len >= buf_size) return -1;
		uint8_t fert_count = MachineSetting.generalSetting.Fert.FertCount;
		for (int j = 0; j < fert_count; j++) {
			len += snprintf(buffer + len, buf_size - len, "%d", fifo->FIFO[fifo->head].ECRatio[j]);
			if (j < fert_count - 1) {
				len += snprintf(buffer + len, buf_size - len, ",");
				if (len >= buf_size) return -1;
			}
		}
		// If fert_count < MAX_FERT_TYPE, remaining ratios are omitted (they are zero anyway)
		len += snprintf(buffer + len, buf_size - len, "]},");
		if (len >= buf_size) return -1;
    }
    len += snprintf(buffer + len, buf_size - len,
					"\"tasks\":[");
    if (len >= buf_size) return -1;

    uint8_t idx = fifo->head;

    for (int i = 0; i < total_tasks; i++) {
        TaskInfo *t = &fifo->FIFO[idx];

        // Convert TargetEC to string without %f
        char ec_str[16];
        float_to_string(t->TargetEC, ec_str);

        len += snprintf(buffer + len, buf_size - len,
                        "\"%s\"",
                        (char*)(t->taskID));
        if (len >= buf_size) return -1;

        // Use active fertilizer count (not MAX_FERT_TYPE)
//        uint8_t fert_count = MachineSetting.generalSetting.Fert.FertCount;
//        for (int j = 0; j < fert_count; j++) {
//            len += snprintf(buffer + len, buf_size - len, "%d", t->ECRatio[j]);
//            if (j < fert_count - 1) {
//                len += snprintf(buffer + len, buf_size - len, ",");
//                if (len >= buf_size) return -1;
//            }
//        }
//        // If fert_count < MAX_FERT_TYPE, remaining ratios are omitted (they are zero anyway)
//        len += snprintf(buffer + len, buf_size - len, "]}");
//        if (len >= buf_size) return -1;

        if (i < total_tasks- 1) {
            len += snprintf(buffer + len, buf_size - len, ",");
            if (len >= buf_size) return -1;
        }
        idx = (idx + 1) % MAX_TASK_COUNT;
    }
    len += snprintf(buffer +len, buf_size-len, "],\"stateHistory\":[");
    for(int i = 0; i< total_statehistorys; i++){
    	len += snprintf(buffer + len, buf_size - len,
			"{\"taskID\":\"%s\",\"elapsedTime\":%u,\"exitTimeStamp\":%u,\"exitCode\":%u}",
						(char*) statehistoryfifo->FIFO[(statehistoryfifo->head +i)%MAX_STATE_HISTORY_COUNT].taskID,
						(unsigned int)statehistoryfifo->FIFO[(statehistoryfifo->head +i)%MAX_STATE_HISTORY_COUNT].elapsedTime,
						(unsigned int)statehistoryfifo->FIFO[(statehistoryfifo->head +i)%MAX_STATE_HISTORY_COUNT].exitTimeStamp,
						(unsigned int)statehistoryfifo->FIFO[(statehistoryfifo->head +i)%MAX_STATE_HISTORY_COUNT].exitCode);
    	if (len >= buf_size) return -1;
    	if(i < total_statehistorys-1){
    		len += snprintf(buffer + len, buf_size - len, ",");
    		if (len >= buf_size) return -1;
    	}
    }
    len += snprintf(buffer +len, buf_size-len, "],\"taskHistory\":[");
	for(int i = 0; i< total_taskhistorys; i++){
		len += snprintf(buffer + len, buf_size - len,
			"{\"taskID\":\"%s\",\"incompleteDuration\":%u,\"start\":%u,\"end\":%u}",
						(char*) historyfifo->FIFO[(historyfifo->head +i)%MAX_TASK_HISTORY_COUNT].taskID,
						(unsigned int)historyfifo->FIFO[(historyfifo->head +i)%MAX_TASK_HISTORY_COUNT].incompleteDuration,
						(unsigned int)historyfifo->FIFO[(historyfifo->head +i)%MAX_TASK_HISTORY_COUNT].start,
						(unsigned int)historyfifo->FIFO[(historyfifo->head +i)%MAX_TASK_HISTORY_COUNT].end);
		if (len >= buf_size) return -1;
		if(i < total_taskhistorys-1){
			len += snprintf(buffer + len, buf_size - len, ",");
			if (len >= buf_size) return -1;
		}
	}
    len += snprintf(buffer + len, buf_size - len, "],\"warnings\":[");
    for(int i = 0; i < WARN_COUNT;i++){
    	if(MachineSetting.WarningArray[i] != 0){
    		len += snprintf(buffer+len, buf_size-len, "1");
    	}
    	else{
    		len += snprintf(buffer+len, buf_size-len, "0");
    	}
    	if(len>=buf_size) return -1;
    	if(i < WARN_COUNT-1){
    		len += snprintf(buffer + len, buf_size - len, ",");
    		if (len >= buf_size) return -1;
    	}
    }
    len += snprintf(buffer + len, buf_size - len, "],\"errors\":");
	if (len >= buf_size) return -1;
	len += snprintf(buffer + len, buf_size - len, "%u}", (unsigned int)MachineSetting.ErrorCode);
	if (len >= buf_size) return -1;
	len += snprintf(buffer + len, buf_size - len, "\r\n");
	if (len >= buf_size) return -1;
//    len += snprintf(buffer + len, buf_size - len, ",\"warnings\":[]}");
//    if (len >= buf_size) return -1;

    return len;
}

static uint8_t TaskInfoFIFO_Push(TaskInfoFIFO *fifo, const TaskInfo *task) {

    for(int i = 0; i< fifo->count; i++){
    	if(memcmp(fifo->FIFO[(fifo->head + i)% MAX_TASK_COUNT].taskID,task->taskID, TASK_ID_BYTE_COUNT) == 0 && i != 0){
    		fifo->FIFO[(fifo->head + i)%MAX_TASK_COUNT] = *task;
    		return 1;
    	}
    }
    if (fifo->count >= MAX_TASK_COUNT) return 0;
    fifo->FIFO[fifo->tail] = *task;
    fifo->tail = (fifo->tail + 1) % MAX_TASK_COUNT;
    fifo->count++;
    return 1;
}

static void TaskInfoFIFO_RemoveFirst(TaskInfoFIFO *fifo) {
    if (fifo->count == 0) return;
    fifo->head = (fifo->head + 1) % MAX_TASK_COUNT;
    fifo->count--;
}

static void TaskInfoFIFO_Remove(TaskInfoFIFO * fifo, uint8_t* taskid){
	if(fifo->count == 0) return;
	uint8_t found = 0;
	for(int i = 0; i < fifo->count; i++){
		if(found == 0){
			if(memcmp(fifo->FIFO[(fifo->head +i)%(MAX_TASK_COUNT)].taskID, taskid,TASK_ID_BYTE_COUNT)== 0){
				found = 1;
			}
		}
		else{
			memcpy(&fifo->FIFO[(fifo->head +i-1)%(MAX_TASK_COUNT)], &fifo->FIFO[(fifo->head +i)%(MAX_TASK_COUNT)], sizeof(TaskInfo));
		}
	}
	if(found == 1){
		fifo->count = fifo->count -1;
		fifo->tail  = (fifo->tail + MAX_TASK_COUNT-1) % MAX_TASK_COUNT;
	}
}

static void TaskInfoFIFO_RemoveAll(TaskInfoFIFO *fifo) {
    if (fifo->count == 0) return;
    fifo->count = 0;
    fifo->head = fifo -> tail;
}

static void MoveTaskToTaskHistory(uint8_t* taskid){
	for(int i = 0; i < MachineSetting.TaskProxyBuffer.InfoFIFO.count; i++){
		TaskHistory taskHistory = {0};
		if(memcmp(taskid, MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].taskID, TASK_ID_BYTE_COUNT) == 0){
			if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].duration > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].elapsedTime){
				taskHistory.incompleteDuration = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].duration - MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].elapsedTime;

			}
			else{
				taskHistory.incompleteDuration = 0;
			}
			taskHistory.start = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].duration > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].timeStamp;
			taskHistory.end = HAL_GetTick();
			memcpy(taskHistory.taskID, &MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].taskID, TASK_ID_BYTE_COUNT);
			TaskHistoryFIFO_Push(&MachineSetting.TaskProxyBuffer.HistoryFIFO, &taskHistory);
			break;
		}
	}
}

static void MoveAllTasksToTaskHistory(void){
	for(int i = 0; i < MachineSetting.TaskProxyBuffer.InfoFIFO.count; i++){
		TaskHistory taskHistory = {0};
		if(MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].duration > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].elapsedTime){
			taskHistory.incompleteDuration = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].duration - MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].elapsedTime;

		}
		else{
			taskHistory.incompleteDuration = 0;
		}
		taskHistory.start = MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].duration > MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].timeStamp;
		taskHistory.end = HAL_GetTick();
		memcpy(taskHistory.taskID, &MachineSetting.TaskProxyBuffer.InfoFIFO.FIFO[MachineSetting.TaskProxyBuffer.InfoFIFO.head + i].taskID, TASK_ID_BYTE_COUNT);
		TaskHistoryFIFO_Push(&MachineSetting.TaskProxyBuffer.HistoryFIFO, &taskHistory);
	}
}

static void TaskHistoryFIFO_Push(TaskHistoryFIFO *fifo, const TaskHistory *taskHistory){
	fifo->FIFO[fifo->tail] = *taskHistory;
	fifo->tail = (fifo->tail + 1) % MAX_TASK_HISTORY_COUNT;
	if(fifo->count < MAX_TASK_HISTORY_COUNT){
		fifo->count ++;
	}
	else{
		fifo-> head = (fifo->head + 1) % MAX_TASK_HISTORY_COUNT;
	}
}

static void StateHistoryFIFO_Push(StateHistoryFIFO *fifo, const StateHistory *stateHistory){
	fifo->FIFO[fifo->tail] = *stateHistory;
	fifo->tail = (fifo->tail + 1) % MAX_STATE_HISTORY_COUNT;
	if(fifo->count < MAX_TASK_HISTORY_COUNT){
		fifo->count ++;
	}
	else{
		fifo-> head = (fifo->head + 1) % MAX_STATE_HISTORY_COUNT;
	}
}

static void AppendStateHistory(uint16_t exitcode){
	StateHistory stateHistory = {0};
	stateHistory.elapsedTime = MachineSetting.ElapsedTime;
	if(MachineSetting.manualEvent.isManual != 0){
		memcpy(stateHistory.taskID, MachineSetting.manualEvent.taskID, TASK_ID_BYTE_COUNT);
	}
	stateHistory.exitTimeStamp = HAL_GetTick();
	stateHistory.isManual = MachineSetting.manualEvent.isManual;
	stateHistory.exitCode = exitcode;
	StateHistoryFIFO_Push(&MachineSetting.stateHistoryFIFO, &stateHistory);
}

static void AddErrorCode(uint16_t errorcode){
	MachineSetting.ErrorCode = errorcode;
	return;
}

static void ClearErrorCode(void){
	MachineSetting.ErrorCode = 0;
	return;
}

static void CheckWarningCondition(void){
	if(ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirLow]) == GPIO_PIN_SET || ReadInput(MachineSetting.generalSetting.Inputs.BasicIn[WaterResevoirWarning]) == GPIO_PIN_SET){
		MachineSetting.WarningArray[WARN_WATER_RESERVOIR_NEED_REFILL] = 1;
	}
	else{
		MachineSetting.WarningArray[WARN_WATER_RESERVOIR_NEED_REFILL] = 0;
	}

	if(ReadInput(MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_WARNING]) == GPIO_PIN_SET || ReadInput(MachineSetting.generalSetting.Fert.Fert[0].in[FERTIN_LOW]) == GPIO_PIN_SET){
		MachineSetting.WarningArray[WARN_TANK_A_NEED_REFILL] = 1;
	}
	else{
		MachineSetting.WarningArray[WARN_TANK_A_NEED_REFILL] = 0;
	}

	if(ReadInput(MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_WARNING]) == GPIO_PIN_SET || ReadInput(MachineSetting.generalSetting.Fert.Fert[1].in[FERTIN_LOW]) == GPIO_PIN_SET){
		MachineSetting.WarningArray[WARN_TANK_B_NEED_REFILL] = 1;
	}
	else{
		MachineSetting.WarningArray[WARN_TANK_B_NEED_REFILL] = 0;
	}

	if(MachineSetting.generalSetting.readingEC.modbusSetting.isError == 1){
		MachineSetting.WarningArray[WARN_EC_SENSOR_FAULT] = 1;
	}
	else{
		MachineSetting.WarningArray[WARN_EC_SENSOR_FAULT] = 0;
	}

	if(MachineSetting.generalSetting.Fert.Fert[0].modbusSetting.isError != 0){
		MachineSetting.WarningArray[WARN_FLOW_METER_A_FAULT] = 1;
	}
	else{
		MachineSetting.WarningArray[WARN_FLOW_METER_A_FAULT] = 0;
	}

	if(MachineSetting.generalSetting.Fert.Fert[1].modbusSetting.isError != 0){
		MachineSetting.WarningArray[WARN_FLOW_METER_B_FAULT] = 1;
	}
	else{
		MachineSetting.WarningArray[WARN_FLOW_METER_B_FAULT] = 0;
	}
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
