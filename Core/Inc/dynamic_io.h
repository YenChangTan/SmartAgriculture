/*
 * dynamic_io.h
 *
 *  Created on: Aug 7, 2026
 *      Author: Tan Yen Chang
 */

#ifndef INC_DYNAMIC_IO_H_
#define INC_DYNAMIC_IO_H_

#include "stm32f4xx_hal.h"

typedef struct {
	GPIO_TypeDef *Port;
	uint16_t Pin;
}IOLocalPin;

typedef struct {
	GPIO_PinState PinStatus;
	uint8_t AccumulatedChange;
	uint8_t ChangeThreshold;
	uint8_t IsBind;
}InputPin;

typedef struct {
	GPIO_PinState PinStatus;
	uint8_t AccumulatedChange;
	uint8_t ChangeThreshold;
	uint8_t IsBind;
}OutputPin;

typedef struct{
	IOLocalPin LocalPin[LOCAL_OUT];
	RealOutputPin RealPinState[LOCAL_OUT + MAX_SLAVE*OUT_PER_SLAVE];
}OutputsPinState;

typedef struct{
	IOLocalPin LocalPin[LOCAL_IN];
	InputPin RealPinState[LOCAL_IN + MAX_SLAVE*IN_PER_SLAVE];
};


typedef enum{
	SLAVE_COM_IDLE = 0,
	SLAVE_COM_TX,
	SLAVE_COM_PRE_RX,
	SLAVE_COM_RX,
	SLAVE_COM_RX_DONE,
	SLAVE_COM_ERROR
}SlaveComState;

typedef struct{
	uint8_t isSlave;
	UART_HandleTypeDef *huart;
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
	uint8_t Buffer[]
};

#endif /* INC_DYNAMIC_IO_H_ */
