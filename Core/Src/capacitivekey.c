/*
 * capacitivekey.c
 *
 *  Created on: Oct 13, 2023
 *      Author: dillon
 */

#include "capacitivekey.h"

int grabKeys(capkey_t *keys,TIM_HandleTypeDef *htim)
{


	return -1; // stub or error lol
}

int getKey(capkey_t *key,TIM_HandleTypeDef *htim)
{
	const uint32_t timeOut = 4000;
	// start, stop, and delta time
	uint32_t startTime = 0;
	uint32_t stopTime = 0;
	uint32_t deltaTime = 0;

	// set the driver pin to high and do the thing
	startTime = __HAL_TIM_GET_COUNTER(htim);
	HAL_GPIO_WritePin(driverpin_GPIO_Port,driverpin_Pin,1);

	// after we set the pin to high, or we get a time out, which ever comes first
	while(HAL_GPIO_ReadPin(key->gpioport,key->gpiopin) == 0)
	{
		deltaTime = __HAL_TIM_GET_COUNTER(htim) - startTime;
		if(deltaTime > timeOut)
		{
			break;
		}
	}

	// return the data
	HAL_GPIO_WritePin(driverpin_GPIO_Port,driverpin_Pin,0);
	return deltaTime;

	//const uint32_t timeOut = 4000;
	// start, stop, and delta time
//	uint32_t startTime = 0;
//	uint32_t stopTime = 0;
//	uint32_t deltaTime = 0;

//	startTime = __HAL_TIM_GET_COUNTER(htim);
//	// set passed key pin to high
//	HAL_GPIO_WritePin(key->gpioport,key->gpiopin,1);
	// then wait for the sensor pin to go high
//	while(HAL_GPIO_ReadPin(sensor_pin_GPIO_Port,sensor_pin_Pin) == 0)
//	{
//		deltaTime = __HAL_TIM_GET_COUNTER(htim) - startTime;
//		if(deltaTime > timeOut)
//		{
//			break;
//		}
//	}

//	HAL_GPIO_WritePin(key->gpioport,key->gpiopin,0);
	return deltaTime;
}
