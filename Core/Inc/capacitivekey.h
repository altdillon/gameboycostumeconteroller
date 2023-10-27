/*
 * capacitivekey.h
 *
 *  Created on: Oct 13, 2023
 *      Author: dillon
 */

#ifndef INC_CAPACITIVEKEY_H_
#define INC_CAPACITIVEKEY_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "main.h"

// struct for holding some basic infomation about keys
// register and bank being used
typedef struct
{
	bool state; // current state of the button
	bool lastState; // last state of the button
	uint8_t pinIndex; // index for the pin  =
	//char pinName[8]; // name of the pin
	GPIO_TypeDef *gpioport; // register
	uint16_t gpiopin; // pin name
	uint16_t triggerTime; // max time
	uint32_t upTime; // how long the input has been on for
	uint8_t keyCode; // keycode for what the keypress is
}capkey_t;

// grab a key
int grabKeys(capkey_t *keys,TIM_HandleTypeDef *htim);
int getKey(capkey_t *key,TIM_HandleTypeDef *htim);

#endif /* INC_CAPACITIVEKEY_H_ */
