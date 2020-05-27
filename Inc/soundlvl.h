/*
 * soundlvl.h
 *
 *  Created on: May 27, 2020
 *      Author: anolp
 */

#ifndef SOUNDLVL_H_
#define SOUNDLVL_H_

#include "stdint.h"
#include "stdbool.h"

typedef struct {
	uint16_t err_t;
	uint16_t sound_lvl;
	int16_t temperature;
	int16_t vref;
} SOUNDLVL_t;

void SL_Initialize(void * handler);

void SL_Calibration(void);

void SL_Enable(bool en);

uint16_t SL_StartMeasure(SOUNDLVL_t* val);

#endif /* SOUNDLVL_H_ */
