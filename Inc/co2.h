/*
 * co2.h
 *
 *  Created on: May 19, 2020
 *      Author: anolp
 */

#ifndef CO2_H_
#define CO2_H_

#include "stdint.h"
#include "stdbool.h"

typedef struct {
	uint16_t err_t;

	int16_t co2_f_pc_val;
	int16_t temperature;

	uint8_t measure_count;
	uint16_t measure_cycle;
	int16_t co2_uf_pc_val;
	int16_t co2_f_val;
	int16_t co2_uf_val;

	uint16_t barometric;

	uint16_t fw_ver;
	uint32_t sensor_id;
} CO2_t;

#define I2C_SLAVE_ADDR 				0x68

//Read only registor
#define CO2_ERR_REG					0x00
#define CO2_FILTER_VAL_PC_REG		0x06
#define CO2_TEMPERATURE_VAL_REG		0x08

#define CO2_MEASURE_COUNT_REG		0x0D
#define CO2_MEASURE_CYCLE_REG		0x0E
#define CO2_UNFILTER_VAL_PC_REG		0x10
#define CO2_FILTER_VAL_REG			0x12
#define CO2_UNFILTER_VAL_REG		0x14

#define CO2_FW_REV_REG				0x38
#define CO2_SENSOR_ID_REG			0x3A

//Read/write registor
#define CO2_CALIBRATE_STATUS_REG	0x81
#define CO2_CALIBRATE_CMD_REG		0x82

#define CO2_MEASUREMENT_MODE_REG	0x95	//Default 0:Continuous mode
#define CO2_MEASUREMENT_PERIOD_REG	0x96	//Default 16 samples
#define CO2_NUMBER_OF_SAMPLES_REG	0x98	//Default 8 samples
#define CO2_ABC_PERIOR_REG			0x9A	//Default 180 hours
#define CO2_CLR_ERR_STATUS_REG		0x9D
#define CO2_ABC_TARGET_REG			0x9E	//Default 400
#define CO2_IIR_FILTER_REG			0xA1
#define CO2_SCR_REG					0xA3	//Reset/Restart
#define CO2_METER_CONTROL_REG		0xA5	//0: nRDY, 1: ABC, 2: Static IIR, 3: Dynamic IIR, 4: Pressure
#define CO2_MB_ADDR_REG				0xA7	//Default 104 (0x68) Slave id setting

#define CO2_BIO_AIR_PRESSURE_REG	0xDC

void CO2_Initialize(void * handler);

void CO2_Calibration(void);

void CO2_Enable(bool en);

uint16_t CO2_StartMeasure(uint16_t* val, uint16_t* temperature);

#endif /* CO2_H_ */
