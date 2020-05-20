/*
 * co2.c
 *
 *  Created on: May 19, 2020
 *      Author: anolp
 */

#include "co2.h"
#include "main.h"

I2C_HandleTypeDef *hi2c;
CO2_t sensor;

static void I2C_Read(uint8_t regAddr, uint8_t *val, size_t len);
static void I2C_Write(uint8_t regAddr, uint8_t *val, size_t len);

void CO2_Initialize(void * handler)
{
	uint8_t buf[6];

	if(handler != NULL)
		hi2c = handler;
	else {
		CO2_Enable(false);
		return;
	}

	CO2_Enable(true);

	//Read err status
//	I2C_Read(CO2_ERR_REG, (uint8_t*) &sensor.err_t, sizeof(sensor.err_t));
	//Read measurement properties
//	I2C_Read(CO2_ERR_REG, &sensor.measure_count, 9);
	CO2_StartMeasure(NULL, NULL);

	//Read FW version. / ID
	I2C_Read(CO2_ERR_REG, (uint8_t *) &buf[0], 6);
	sensor.fw_ver = buf[0]<<8 | buf[1];
	sensor.sensor_id = buf[2] << 24 | buf[3] << 16  | buf[4] << 8  | buf[5] << 0;
}

void CO2_Enable(bool en)
{
	GPIO_PinState state = en;
	HAL_GPIO_WritePin(CO2_EN_GPIO_Port, CO2_EN_Pin, state);

	HAL_Delay(50);
}

void CO2_Calibration(void)
{
	uint16_t calib = 0;
	uint8_t status = 0;

	if(hi2c == NULL) return;

	//Start calibration
	I2C_Write(CO2_CALIBRATE_STATUS_REG, &status, sizeof(status));
	I2C_Read(CO2_CALIBRATE_CMD_REG, (uint8_t *) &calib, sizeof(calib));
	calib = 0x027C; //Restore factory calibration
	I2C_Write(CO2_CALIBRATE_CMD_REG, (uint8_t*) &calib, sizeof(calib));

	HAL_Delay(100);

	I2C_Read(CO2_CALIBRATE_STATUS_REG, &status, sizeof(status));
	//I2C_Write(CO2_CALIBRATE_STATUS_REG, &calib, 1);

}

uint16_t CO2_StartMeasure(uint16_t* val, uint16_t* temperature)
{
	uint8_t buf[16];
	if(hi2c == NULL) return 0xFFFF;

	//Read err status
	I2C_Read(CO2_ERR_REG, (uint8_t*) &buf[0], sizeof(sensor.err_t));
	sensor.err_t = buf[0] << 8 | buf[1];

	//Read value
	I2C_Read(CO2_FILTER_VAL_PC_REG, (uint8_t*) &buf[0], 4);

	sensor.co2_f_pc_val = buf[0] << 8 | buf[1];
	sensor.temperature = buf[2] << 8 | buf[3];

	I2C_Read(CO2_MEASURE_COUNT_REG, (uint8_t*) &buf[0], 9);
	sensor.measure_count = buf[0];
	sensor.measure_cycle = buf[1] << 8 | buf[2];
	sensor.co2_uf_pc_val = buf[3] << 8 | buf[4];
	sensor.co2_f_val = buf[5] << 8 | buf[6];
	sensor.co2_uf_val = buf[7] << 8 | buf[8];

	I2C_Read(CO2_BIO_AIR_PRESSURE_REG, (uint8_t*) &buf[0], 2);
	sensor.barometric = buf[0] << 8 | buf[1];

	*val = sensor.co2_f_pc_val;
	*temperature = sensor.temperature;

	return 0;
}

static void I2C_Read(uint8_t regAddr, uint8_t * val, size_t len)
{
	HAL_StatusTypeDef ret;
	uint16_t slave_id = I2C_SLAVE_ADDR << 1;
	//Wake up
	ret = HAL_I2C_IsDeviceReady(hi2c, slave_id, 1, 25);

	ret = HAL_I2C_Master_Transmit(hi2c, slave_id, &regAddr, 1, 500);

	ret = HAL_I2C_Master_Receive(hi2c, slave_id, val, len, 500);
	if(ret == HAL_OK)
	{
		//Read registor
//		printf("CO2 comm. ok\r\n");
	} else {
		printf("CO2 comm. err\r\n");
	}
}

static void I2C_Write(uint8_t regAddr, uint8_t *val, size_t len)
{
	HAL_StatusTypeDef ret;
	uint16_t slave_id = I2C_SLAVE_ADDR << 1;
	//Wake up
	ret = HAL_I2C_IsDeviceReady(hi2c, slave_id, 1, 25);

	ret = HAL_I2C_Mem_Write(hi2c, slave_id, regAddr, I2C_MEMADD_SIZE_8BIT, val, len, 500);

	if(ret == HAL_OK)
	{//Write reg. address val
		printf("CO2 comm. ok\r\n");
	} else {
		printf("CO2 comm. err\r\n");
	}
}

