/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define BITS_PER_BYTE 8
#define LOCAL_OUT 16
#define LOCAL_IN 16
#define OUT_PER_SLAVE 16
#define IN_PER_SLAVE 16
#define MAX_SLAVE 5
#define OUT_PER_FERT 4
#define IN_PER_FERT 4
#define EC_SAMPLE_COUNT 30
#define MAX_TASK_COUNT 32
#define MAX_FERT_TYPE 5
#define MAX_PESTICIDE_TANK 5
#define INPUT_CHANGE_THRESHOLD 30
#define DELAY_FOR_VALVE 18000
#define DEFAULT_EC_HYSTERISIS 100
#define SLAVE_TX_TIMEOUT 100
#define SLAVE_RX_TIMEOUT 100
#define SLAVE_INTERCHAR_TIMEOUT 10
#define TCP_Buffer_MAX_Count 1024
#define ERROR_ARRAY_COUNT 100

//typedef enum{
//	TIMEOUT_MIXED_WATER_WATERING = 0,
//	TIMEOUT_WATER_RESERVOIR_REFILL,
//	TIMEOUT_CLEAN_WATER_WATERING,
//	TIMEOUT_CLEAN_WATER_FILLING_EC,
//	TIMEOUT_WATER_FILLING_HIGH_SENSOR,
//	TIMEOUT_ADDING_FERTILIZER,
//	TIMEOUT_OVERFLOW_RECOVERY,
//	SENSOR_WATER_RESERVOIR_LOW,
//	SENSOR_FERT_TANK_LOW,
//	SENSOR_LimTrig_HiNotTrig,
//	SENSORDATA_EC,
//	SENSORDATA_FLOWA,
//	SENSORDATA_FLOWB,
//	SLAVE_COM_FAIL,
//	E_STOP,
//	ERROR_TYPE_COUNT
//}ERROR_CODE_ENUM;

typedef enum{
	WaterReservoirPump,
	FertigationPump,
	ValveWaterSource,
	ValveInMix1,
	ValveOutMix1,
	ValveInMix2,
	ValveOutMix2,
	BaseOutCount
}OutputEnum;

typedef enum{
	WaterResevoirLow = 0,
	WaterResevoirWarning,
	WaterResevoirHigh,
	Mix1Limit,
	Mix1High,
	Mix1Low,
	Mix2Limit,
	Mix2High,
	Mix2Low,
	Flow,
	BaseInCount
}InputEnum;

typedef enum {
    MODBUS_UINT16 = 0,
    MODBUS_INT16,
    MODBUS_UINT32,
    MODBUS_INT32,
    MODBUS_FLOAT
} ModbusDataType;

typedef enum{
	FERTOUT_FILL_VALVE = 0,
	FERTOUT_WATER_VALVE,
	FERTOUT_FERT_VALVE,
	FERTOUT_FERT_PUMP,
	FERTOUT_PROPELLER,
	FERTOUT_COUNT
}FERT_OUT;

typedef enum{
	FERTIN_LOW = 0,
	FERTIN_WARNING,
	FERTIN_HIGH,
	FERTIN_COVER,
	FERTIN_COUNT
}FERT_IN;

typedef enum{
	ERROR_ESTOP = 0,
	ERROR_TIMEOUT,
	ERROR_MIXTANKSFULL,
	ERROR_RESERVOIRLOW,
	ERROR_PLOTOUTOFRANGE,
	ERROR_ECREADING,
	ERROR_FLOWAREADING,
	ERROR_FLOWBREADING,
	ERROR_ALOW,
	ERROR_BLOW,
	ERROR_TYPECOUNT
}ERROR_TYPE;

typedef struct{
	uint8_t ValveInMapToRealOutputPin;
	uint8_t HighSensorMapToRealInputPin;
}pesticideIO;

typedef struct{
	pesticideIO pesticideArray[MAX_PESTICIDE_TANK];
	uint8_t count;
}PesticideControl;

typedef struct{
	uint8_t modbusAdr;
	uint8_t registerAdr;
	ModbusDataType type;
	uint8_t smallEndian;
	uint8_t isError;
	float factor;
	float offset;
	float value;
}modbusData;

typedef struct{
	uint8_t isActive;
	modbusData modbusSetting;
	float StampValue;
	float TargetVolume;
	float TargetIncrement;
	float CurrentVolume;
	uint8_t out[FERTOUT_COUNT];//0 is Water In Valve, 1 is small Water In Valve, 2 is fertilizer in valve, 3 is pump.
	uint8_t in[FERTIN_COUNT];//0 is High, 1 is Warning, 2 is Low.

}FertPinAssignment;

typedef struct{
	uint8_t FertCount;
	uint32_t FertInterDelay;
	uint32_t ExitConditionDelay;
	FertPinAssignment Fert[MAX_FERT_TYPE];
}FertSetting;

typedef struct{
	GPIO_TypeDef* Port;
	uint16_t Pin;
}IOLocalPin;

typedef struct{
	GPIO_PinState PinStatus;
	uint8_t AccumulatedChange;
	uint8_t IsBind;
}RealPin;

typedef struct{
	IOLocalPin LocalPin[LOCAL_OUT];
	uint8_t BasicOut[BaseOutCount];
	RealPin RealPinState[LOCAL_OUT+MAX_SLAVE*OUT_PER_SLAVE];
	uint8_t SlaveCount;
}OutputsPinState;

typedef struct{
	IOLocalPin LocalPin[LOCAL_IN];
	uint8_t BasicIn[BaseInCount];
	RealPin RealPinState[LOCAL_IN+MAX_SLAVE*IN_PER_SLAVE];
	uint8_t SlaveCount;
}InputsPinState;

typedef struct{
	modbusData modbusSetting;
	float deltaEC;
	float MeanEC;
	float ReadingECSample[EC_SAMPLE_COUNT];
	uint8_t ReadingError;
}ReadingEC;

typedef struct{
	OutputsPinState Outputs;
	InputsPinState Inputs;
	uint8_t ValveOut[LOCAL_OUT+MAX_SLAVE*OUT_PER_SLAVE-BaseOutCount];
	uint8_t ValveOutCount;
	FertSetting Fert;
	ReadingEC readingEC;
}GeneralSetting;

typedef struct{
	uint8_t plotID;
	uint32_t duration;
	uint32_t elapsedTime;
	uint32_t elapsedTimeStamp;
	uint32_t timeStamp;
	float TargetEC;
	float Hysterisis;
	uint8_t ECRatio[MAX_FERT_TYPE];
}TaskInfo;

typedef struct{
	TaskInfo FIFO[MAX_TASK_COUNT];
	uint8_t count;
	uint8_t head;
	uint8_t tail;
}TaskInfoFIFO;

typedef struct{
	uint8_t plotID;
	uint32_t start;
	uint32_t end;
	uint32_t incompleteDuration;
}TaskHistory;

typedef struct{
	TaskHistory FIFO[MAX_TASK_COUNT];
	uint8_t count;
	uint8_t head;
	uint8_t tail;
}TaskHistoryFIFO;

typedef struct{
	uint8_t currentTask; //Equal to MAX_TASK_COUNT when it is not running any task.
	TaskInfoFIFO InfoFIFO;
	TaskHistoryFIFO HistoryFIFO;
}TaskProxy;

typedef enum{
	SLAVE_COM_IDLE=0,
	SLAVE_COM_TX,
	SLAVE_COM_PRE_RX,
	SLAVE_COM_RX,
	SLAVE_COM_RX_DONE,
	SLAVE_COM_ERROR
}SlaveComState;

typedef struct{
	uint8_t isSlave;
	UART_HandleTypeDef* huart;
	IOLocalPin DirectionalPin;
	uint32_t txTimeOut;
	uint32_t rxTimeOut;
	uint32_t intercharTimeOut;
	uint32_t timeStamp;
	SlaveComState state;
	uint8_t isError;
	uint8_t ErrorCount;
	uint8_t currentSlave;
	uint8_t slaveCount;
	uint8_t Buffer[128];
	uint8_t bufferCount;
}SlaveCommunication;


typedef enum{
	STATE_IDLE = 0,
	STATE_WATER_RESERVOIR_REFILL,
	STATE_CLEAN_WATER_WATERING,
	STATE_MIXED_WATER_WATERING,
	STATE_CLEAN_WATER_FILLING_LOWER_EC,
	STATE_WATER_FILLING_TO_MIX1_HIGHSENSOR,
	STATE_FERTILIZER_ADDING,
	STATE_OVERFLOW_RECOVERY,//Overflow Recovery will suck out the excess water in mixing tank to fertigation.
	STATE_MANUAL_WATER_RESERVOIR_REFILL,
	STATE_MANUAL_CLEAN_WATER_WATERING,
	STATE_MANUAL_MIXED_WATER_WATERING,
	STATE_MANUAL_CLEAN_WATER_FILLING_LOWER_EC,
	STATE_MANUAL_WATER_FILLING_TO_MIX1_HIGHSENSOR,
	STATE_MANUAL_FERTILIZER_ADDING,
	STATE_MANUAL_OVERFLOW_RECOVERY,
	STATE_MANUAL_FILL_FERTILIZER_TANK,
	STATE_MANUAL_FILL_PESTICIDE_TANK,
	STATE_MANUAL_DISSOLVE_SOLID_FERTILIZER,
	STATE_ABORT,
	STATE_ERROR,
	STATE_COUNT
}machineState;

typedef enum {
    CMD_HEARTBEAT,
    CMD_RESET,
    CMD_CLEAR_TASKS,
    CMD_ESTOP,
    CMD_WRITE_TASK,
	CMD_CALIBRATE_EC,
	CMD_MANUAL_ENTER,      // new
	CMD_MANUAL_EXIT,       // new
	CMD_MANUAL_TRIGGER,    // new
    CMD_UNKNOWN
} MasterCommand;

typedef struct{
	uint8_t isManual;
	machineState triggerEvent;
	uint8_t plotID;
	uint32_t TimeOut;
	float TargetEC;
	uint32_t ManualResetTimeOut;
}ManualEvent;

typedef struct {
	uint8_t isNewCommandComing;
	uint8_t isParseTaskSuccess;
    MasterCommand cmd;
    TaskInfo task;   // valid only if cmd == CMD_WRITE_TASK
    ManualEvent manual;
} masterRequest;

typedef struct{
	UART_HandleTypeDef* huart;
	IOLocalPin DirectionalPin;
	uint8_t Buffer[TCP_Buffer_MAX_Count];
	uint16_t BufferCount;
	uint32_t txTimeOut;
	uint32_t TimeStamp;
	uint32_t intercharTimeOut;
	uint32_t maxTimeDisconnectFromPC;
	masterRequest MasterRequest;
}TCPContext;



typedef struct{
	uint8_t isAbort;
	uint8_t isError;
	ManualEvent manualEvent;
	PesticideControl pesticideControl;
	TCPContext TCPCom;
	machineState MachineState;
	SlaveCommunication SlaveCom;
	uint32_t TimeOut[STATE_COUNT];
	uint32_t TimeStamp;
	uint32_t AddFertTimeStamp;
	uint32_t ElapsedTime;
	uint32_t currentTick;
	TaskProxy TaskProxyBuffer;
	GeneralSetting generalSetting;
	uint16_t ErrorArray[ERROR_ARRAY_COUNT];
	float TargetEC;
	uint8_t ErrorCount;
}machineSetting;




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
