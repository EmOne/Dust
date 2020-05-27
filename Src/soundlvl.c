/*
 * soundlvl.c
 *
 *  Created on: May 19, 2020
 *      Author: anolp
 */

#include "soundlvl.h"
#include "main.h"

ADC_HandleTypeDef *hadc;
SOUNDLVL_t sl_sensor;

void SL_Initialize(void *handler) {
	uint8_t buf[6];

	if (handler != NULL)
		hadc = handler;
	else {
		SL_Enable(false);
		return;
	}

	SL_Enable(true);
}

void SL_Enable(bool en) {
//	GPIO_PinState state = en;
//	HAL_GPIO_WritePin(CO2_EN_GPIO_Port, CO2_EN_Pin, state);
	if (hadc != NULL) {
		(en) ? __HAL_ADC_ENABLE(hadc) : __HAL_ADC_DISABLE(hadc);
	}
	HAL_Delay(50);
}

void SL_Calibration(void) {

}

uint16_t SL_StartMeasure(SOUNDLVL_t *val) {
	ADC_ChannelConfTypeDef adcConf = { 0 };
	float adcvalue, vref;
	// Enable HSI
	__HAL_RCC_HSI_ENABLE();

	// Wait till HSI is ready
	while ( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET) {
	}

	__HAL_RCC_ADC1_CLK_ENABLE();

	adcConf.Channel = ADC_CHANNEL_TEMPSENSOR;
	adcConf.Rank = 1;
	adcConf.SamplingTime = ADC_SAMPLETIME_480CYCLES;

	HAL_ADC_ConfigChannel(hadc, &adcConf);

	// Enable ADC1
	SL_Enable(true);
	// Start ADC Software Conversion
	HAL_ADC_Start(hadc);

	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

	val->temperature = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);

	adcConf.Channel = ADC_CHANNEL_VREFINT;
	adcConf.Rank = 1;
	adcConf.SamplingTime = ADC_SAMPLETIME_480CYCLES;

	HAL_ADC_ConfigChannel(hadc, &adcConf);

	// Enable ADC1
	SL_Enable(true);

	// Start ADC Software Conversion
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	vref = HAL_ADC_GetValue(hadc);
	vref /= (4094.0f);
	vref *= 3.3f;
	val->vref = vref * 1000;

	HAL_ADC_Stop(hadc);

	adcConf.Channel = ADC_CHANNEL_3;
	adcConf.Rank = 1;
	adcConf.SamplingTime = ADC_SAMPLETIME_480CYCLES;

	HAL_ADC_ConfigChannel(hadc, &adcConf);

	// Enable ADC1
	SL_Enable(true);
	// Start ADC Software Conversion
	HAL_ADC_Start(hadc);

	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

	adcvalue = HAL_ADC_GetValue(hadc);
	adcvalue /= (4094.0f);
	adcvalue *= vref;
	val->sound_lvl = adcvalue * 50;

	HAL_ADC_Stop(hadc);

	__HAL_RCC_ADC1_CLK_DISABLE();

	// Disable HSI
	__HAL_RCC_HSI_DISABLE();

	return 0;
}

