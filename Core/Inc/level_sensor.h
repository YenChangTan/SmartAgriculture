/*
 * level_sensor.h
 *
 *  Created on: Nov 2, 2026
 *      Author: Tan Yen Chang
 *
 *  Analog water level, one LevelSensor object per tank.
 *  The ADC scans every rank continuously into a circular DMA buffer of LEVEL_SENSOR_SAMPLES scans.
 *  Each time the DMA finishes half of the buffer, those scans are added to a running sum per rank,
 *  so every sample counts and each tank reading is the mean of everything since the previous one.
 *  Reading never waits for a conversion.
 *  CubeMX: ADC1 Scan = Enable, Continuous = Enable, DMA Continuous Requests = Enable,
 *  DMA2 Stream0 circular, half word to half word, DMA2 Stream0 interrupt enabled.
 *  Call LevelSensorBus_HalfComplete / LevelSensorBus_Complete from HAL_ADC_ConvHalfCpltCallback /
 *  HAL_ADC_ConvCpltCallback.
 */

#ifndef INC_LEVEL_SENSOR_H_
#define INC_LEVEL_SENSOR_H_
#include "stm32f4xx_hal.h"

#define LEVEL_SENSOR_MAX_PER_BUS 6
#define LEVEL_SENSOR_SAMPLES 16		//DMA buffer depth in scans. Must be even: each half is summed when the DMA finishes it.
#define LEVEL_SENSOR_RAW_EMPTY 0xFFFF
#define LEVEL_SENSOR_WINDOW_MAX 32	//Most readings the depth's moving average can hold (32 x 100 ms = 3.2 s)
#define LEVEL_SENSOR_VREF 3.3f
#define LEVEL_SENSOR_ADC_FULL 4095.0f

//Reservoir and fertilizer tanks: their Warning switch maps to HIGH and their High switch maps to LIMIT.
typedef enum{
	LEVEL_SENSOR_LOW = 0,
	LEVEL_SENSOR_HIGH,
	LEVEL_SENSOR_LIMIT,
	LEVEL_SENSOR_POINT_COUNT
}LevelSensorPoint;

typedef struct{
	uint8_t rank;		//Position in the ADC scan, 0 = CubeMX rank 1.
	uint32_t samplePeriod;
	uint16_t rawMin;	//Raw value outside rawMin..rawMax counts as a failed reading (open or shorted wire).
	uint16_t rawMax;
	uint16_t zeroRaw;	//12-bit reading at depth 0 (4 mA), e.g. 620 for 0.5 V
	uint16_t fullRaw;	//12-bit reading at fullDepth (20 mA), e.g. 3723 for 3.0 V
	float fullDepth;	//Sensor's rated range, e.g. 2000 (mm)
	uint8_t windowSize;	//Readings in the depth's moving average, 1..LEVEL_SENSOR_WINDOW_MAX; x samplePeriod = averaging time.
	uint8_t failThreshold;	//Bad readings in a row before the sensor is in error.
	uint8_t recoverThreshold;	//Good readings in a row needed to clear an error.
	uint32_t restartTime;	//A fault longer than this (ms) restarts the depth average; a shorter one keeps it.
	uint8_t isPointUsed[LEVEL_SENSOR_POINT_COUNT];
	float pointDepth[LEVEL_SENSOR_POINT_COUNT];
	float hysteresis;	//A point is reached at pointDepth and released below pointDepth - hysteresis.
}LevelSensorConfig;

typedef struct{
	LevelSensorConfig config;
	float a;			//depth = a * raw + b, worked out from zeroRaw, fullRaw and fullDepth
	float b;
	uint16_t raw;
	float depth;
	uint16_t window[LEVEL_SENSOR_WINDOW_MAX];	//Last windowSize good raw readings (ring buffer)
	uint32_t windowSum;	//Sum of the readings in window
	uint8_t windowCount;	//Readings in window so far (fills up after start or restart)
	uint8_t windowIndex;	//Next slot to write; the oldest reading once the window is full
	uint8_t hasDepth;
	uint8_t isError;
	uint8_t failCount;
	uint8_t goodCount;	//Good readings in a row while in error
	uint32_t dueTimeStamp;
	uint32_t lastGoodTimeStamp;
	GPIO_PinState pointState[LEVEL_SENSOR_POINT_COUNT];
}LevelSensor;

typedef struct{
	ADC_HandleTypeDef *hadc;
	uint8_t rankCount;
	volatile uint16_t buffer[LEVEL_SENSOR_SAMPLES * LEVEL_SENSOR_MAX_PER_BUS];
	volatile uint32_t sum[LEVEL_SENSOR_MAX_PER_BUS];	//Sum of the samples since the rank was last read, added in the DMA callbacks
	volatile uint32_t count[LEVEL_SENSOR_MAX_PER_BUS];	//Number of samples in sum
	LevelSensor *sensorList[LEVEL_SENSOR_MAX_PER_BUS];
	uint8_t sensorCount;
	uint32_t restartCount;
}LevelSensorBus;

void LevelSensor_Init(LevelSensor *sensor, const LevelSensorConfig *config);
void LevelSensor_SetCalibrationPoints(LevelSensor *sensor, uint16_t raw0, float depth0, uint16_t raw1, float depth1);
float LevelSensor_GetDepth(const LevelSensor *sensor);
uint16_t LevelSensor_GetRaw(const LevelSensor *sensor);
uint8_t LevelSensor_IsError(const LevelSensor *sensor);
uint8_t LevelSensor_IsReady(const LevelSensor *sensor);

//Same meaning as a digital level switch input: GPIO_PIN_SET when the water is below the point.
GPIO_PinState LevelSensor_ReadPoint(const LevelSensor *sensor, LevelSensorPoint point);

void LevelSensorBus_Init(LevelSensorBus *bus, ADC_HandleTypeDef *hadc, uint8_t rankCount);
uint8_t LevelSensorBus_Add(LevelSensorBus *bus, LevelSensor *sensor);
HAL_StatusTypeDef LevelSensorBus_Start(LevelSensorBus *bus);
void LevelSensorBus_Service(LevelSensorBus *bus);

//Call from the DMA interrupt: HAL_ADC_ConvHalfCpltCallback and HAL_ADC_ConvCpltCallback.
void LevelSensorBus_HalfComplete(LevelSensorBus *bus);
void LevelSensorBus_Complete(LevelSensorBus *bus);

//Latest single 12-bit sample of one rank, straight from the DMA buffer (no averaging).
uint16_t LevelSensorBus_GetLatestRaw(const LevelSensorBus *bus, uint8_t rank);

#endif /* INC_LEVEL_SENSOR_H_ */
