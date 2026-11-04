/*
 * level_sensor.c
 *
 *  Created on: Nov 2, 2026
 *      Author: Tan Yen Chang
 */

#include "level_sensor.h"
#include <string.h>

_Static_assert(LEVEL_SENSOR_SAMPLES % 2 == 0, "LEVEL_SENSOR_SAMPLES must be even");

//Samples in a rank's running sum before it is started again; 0x80000 x 4095 stays below 2^32.
#define LEVEL_SENSOR_COUNT_LIMIT 0x80000u

static void _clear_buffer(LevelSensorBus *bus);
static void _accumulate(LevelSensorBus *bus, uint32_t firstScan, uint32_t scanCount);
static uint8_t _take_rank(LevelSensorBus *bus, uint8_t rank, uint16_t *raw);
static void _process_sample(LevelSensor *sensor, uint16_t raw);
static void _record_failure(LevelSensor *sensor);
static void _update_points(LevelSensor *sensor, uint8_t isFirstSample);

void LevelSensor_Init(LevelSensor *sensor, const LevelSensorConfig *config){
	memset(sensor, 0, sizeof(LevelSensor));
	memcpy(&sensor->config, config, sizeof(LevelSensorConfig));
	if(sensor->config.failThreshold == 0){
		sensor->config.failThreshold = 1;
	}
	if(sensor->config.recoverThreshold == 0){
		sensor->config.recoverThreshold = 1;
	}
	if(sensor->config.windowSize == 0){
		sensor->config.windowSize = 1;
	}
	if(sensor->config.windowSize > LEVEL_SENSOR_WINDOW_MAX){
		sensor->config.windowSize = LEVEL_SENSOR_WINDOW_MAX;
	}
	for(int i = 0; i < LEVEL_SENSOR_POINT_COUNT; i++){
		sensor->pointState[i] = GPIO_PIN_RESET;
	}
	LevelSensor_SetCalibrationPoints(sensor, sensor->config.zeroRaw, 0.0f, sensor->config.fullRaw, sensor->config.fullDepth);
}

//Two known points (raw count, depth) set the straight line. Also used to recalibrate with real water.
void LevelSensor_SetCalibrationPoints(LevelSensor *sensor, uint16_t raw0, float depth0, uint16_t raw1, float depth1){
	if(raw1 == raw0){
		return;
	}
	sensor->a = (depth1 - depth0) / ((float)raw1 - (float)raw0);
	sensor->b = depth0 - sensor->a * raw0;
	sensor->hasDepth = 0;
}

float LevelSensor_GetDepth(const LevelSensor *sensor){
	return sensor->depth;
}

uint16_t LevelSensor_GetRaw(const LevelSensor *sensor){
	return sensor->raw;
}

uint8_t LevelSensor_IsError(const LevelSensor *sensor){
	return sensor->isError;
}

uint8_t LevelSensor_IsReady(const LevelSensor *sensor){
	return sensor->hasDepth;
}

GPIO_PinState LevelSensor_ReadPoint(const LevelSensor *sensor, LevelSensorPoint point){
	if(point >= LEVEL_SENSOR_POINT_COUNT){
		return GPIO_PIN_RESET;
	}
	return sensor->pointState[point];
}

void LevelSensorBus_Init(LevelSensorBus *bus, ADC_HandleTypeDef *hadc, uint8_t rankCount){
	memset(bus, 0, sizeof(LevelSensorBus));
	bus->hadc = hadc;
	bus->rankCount = (rankCount > LEVEL_SENSOR_MAX_PER_BUS) ? LEVEL_SENSOR_MAX_PER_BUS : rankCount;
	_clear_buffer(bus);
}

uint8_t LevelSensorBus_Add(LevelSensorBus *bus, LevelSensor *sensor){
	if(bus->sensorCount >= LEVEL_SENSOR_MAX_PER_BUS || sensor->config.rank >= bus->rankCount){
		return 0;
	}
	bus->sensorList[bus->sensorCount++] = sensor;
	//First reading after one full period, so it averages as many samples as the later ones.
	sensor->dueTimeStamp = HAL_GetTick() + sensor->config.samplePeriod;
	return 1;
}

HAL_StatusTypeDef LevelSensorBus_Start(LevelSensorBus *bus){
	_clear_buffer(bus);
	for(int r = 0; r < LEVEL_SENSOR_MAX_PER_BUS; r++){
		bus->sum[r] = 0;
		bus->count[r] = 0;
	}
	return HAL_ADC_Start_DMA(bus->hadc, (uint32_t *)(uintptr_t)bus->buffer, LEVEL_SENSOR_SAMPLES * bus->rankCount);
}

void LevelSensorBus_HalfComplete(LevelSensorBus *bus){
	_accumulate(bus, 0, LEVEL_SENSOR_SAMPLES / 2);
}

void LevelSensorBus_Complete(LevelSensorBus *bus){
	_accumulate(bus, LEVEL_SENSOR_SAMPLES / 2, LEVEL_SENSOR_SAMPLES / 2);
}

void LevelSensorBus_Service(LevelSensorBus *bus){
	if(__HAL_ADC_GET_FLAG(bus->hadc, ADC_FLAG_OVR)){
		//Overrun stops the DMA requests; restart the scan and read again on a later pass.
		HAL_ADC_Stop_DMA(bus->hadc);
		__HAL_ADC_CLEAR_FLAG(bus->hadc, ADC_FLAG_OVR);
		LevelSensorBus_Start(bus);
		bus->restartCount++;
		return;
	}

	uint32_t now = HAL_GetTick();
	for(int i = 0; i < bus->sensorCount; i++){
		LevelSensor *sensor = bus->sensorList[i];
		if((int32_t)(now - sensor->dueTimeStamp) < 0){
			continue;
		}
		uint16_t raw;
		if(_take_rank(bus, sensor->config.rank, &raw) == 0){
			continue;
		}
		sensor->dueTimeStamp += sensor->config.samplePeriod;
		if((int32_t)(now - sensor->dueTimeStamp) >= 0){
			//More than one period behind: restart the schedule instead of catching up in a burst.
			sensor->dueTimeStamp = now + sensor->config.samplePeriod;
		}
		_process_sample(sensor, raw);
	}
}

uint16_t LevelSensorBus_GetLatestRaw(const LevelSensorBus *bus, uint8_t rank){
	if(rank >= bus->rankCount || bus->hadc->DMA_Handle == NULL){
		return LEVEL_SENSOR_RAW_EMPTY;
	}
	//The DMA counter tells how far into the buffer it has written; take the last complete scan before that.
	uint32_t total = LEVEL_SENSOR_SAMPLES * bus->rankCount;
	uint32_t written = total - __HAL_DMA_GET_COUNTER(bus->hadc->DMA_Handle);
	uint32_t scan = (written / bus->rankCount + LEVEL_SENSOR_SAMPLES - 1) % LEVEL_SENSOR_SAMPLES;
	return bus->buffer[scan * bus->rankCount + rank];
}

static void _clear_buffer(LevelSensorBus *bus){
	for(int i = 0; i < LEVEL_SENSOR_SAMPLES * LEVEL_SENSOR_MAX_PER_BUS; i++){
		bus->buffer[i] = LEVEL_SENSOR_RAW_EMPTY;
	}
}

//Runs in the DMA interrupt: adds scans firstScan .. firstScan + scanCount - 1 to each rank's running sum.
//One pass in buffer order and no multiply per sample, to keep the interrupt short.
static void _accumulate(LevelSensorBus *bus, uint32_t firstScan, uint32_t scanCount){
	uint32_t sum[LEVEL_SENSOR_MAX_PER_BUS] = {0};
	uint8_t rankCount = bus->rankCount;
	const volatile uint16_t *sample = &bus->buffer[firstScan * rankCount];
	for(uint32_t s = 0; s < scanCount; s++){
		for(uint8_t r = 0; r < rankCount; r++){
			sum[r] += *sample++;
		}
	}
	for(uint8_t r = 0; r < rankCount; r++){
		if(bus->count[r] >= LEVEL_SENSOR_COUNT_LIMIT){
			//Rank not read for minutes (no sensor on it): start again before the sum can overflow.
			bus->sum[r] = 0;
			bus->count[r] = 0;
		}
		bus->sum[r] += sum[r];
		bus->count[r] += scanCount;
	}
}

//Mean of every sample of one rank since the last call. Returns 0 if none have arrived yet.
static uint8_t _take_rank(LevelSensorBus *bus, uint8_t rank, uint16_t *raw){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint32_t sum = bus->sum[rank];
	uint32_t count = bus->count[rank];
	bus->sum[rank] = 0;
	bus->count[rank] = 0;
	__set_PRIMASK(primask);
	if(count == 0){
		return 0;
	}
	*raw = (uint16_t)((sum + count / 2) / count);
	return 1;
}

static void _process_sample(LevelSensor *sensor, uint16_t raw){
	sensor->raw = raw;
	if(raw < sensor->config.rawMin || raw > sensor->config.rawMax){
		_record_failure(sensor);
		return;
	}
	sensor->failCount = 0;
	uint32_t now = HAL_GetTick();
	if(sensor->isError){
		//Stay in error until enough good readings in a row; the depth keeps its last value meanwhile.
		if(++sensor->goodCount < sensor->config.recoverThreshold){
			return;
		}
		sensor->isError = 0;
		if((now - sensor->lastGoodTimeStamp) > sensor->config.restartTime){
			//Long fault: the old depth may be far off, so start the average again.
			sensor->hasDepth = 0;
		}
	}
	sensor->goodCount = 0;
	uint8_t isFirstSample = (sensor->hasDepth == 0);
	if(isFirstSample){
		//Start (or restart after a long fault) with an empty window; until it fills, average what is there.
		sensor->windowSum = 0;
		sensor->windowCount = 0;
		sensor->windowIndex = 0;
		sensor->hasDepth = 1;
	}
	//Moving average of the last windowSize readings, kept as an integer sum of raw values so it never drifts.
	if(sensor->windowCount == sensor->config.windowSize){
		sensor->windowSum -= sensor->window[sensor->windowIndex];
	}
	else{
		sensor->windowCount++;
	}
	sensor->window[sensor->windowIndex] = raw;
	sensor->windowSum += raw;
	sensor->windowIndex = (uint8_t)((sensor->windowIndex + 1) % sensor->config.windowSize);
	//depth is a straight line of raw, so the depth of the mean raw equals the mean depth.
	sensor->depth = sensor->a * ((float)sensor->windowSum / sensor->windowCount) + sensor->b;
	sensor->lastGoodTimeStamp = now;
	_update_points(sensor, isFirstSample);
}

//The depth is not touched here: a short fault keeps the average, a long one restarts it on recovery.
static void _record_failure(LevelSensor *sensor){
	sensor->goodCount = 0;
	if(sensor->failCount < 0xFF){
		sensor->failCount++;
	}
	if(sensor->failCount >= sensor->config.failThreshold){
		sensor->isError = 1;
	}
}

static void _update_points(LevelSensor *sensor, uint8_t isFirstSample){
	for(int i = 0; i < LEVEL_SENSOR_POINT_COUNT; i++){
		if(sensor->config.isPointUsed[i] == 0){
			continue;
		}
		if(sensor->depth >= sensor->config.pointDepth[i]){
			sensor->pointState[i] = GPIO_PIN_RESET;
		}
		else if(isFirstSample || sensor->depth < sensor->config.pointDepth[i] - sensor->config.hysteresis){
			sensor->pointState[i] = GPIO_PIN_SET;
		}
	}
}
