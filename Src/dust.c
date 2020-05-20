/*
 * dust.c
 *
 *  Created on: Jan 9, 2020
 *      Author: anolp
 */
#include "dust.h"

#include "main.h"

//static uint8_t Particle_Measure_Data[32];
static uint16_t _PM2_5 = 0;
static uint16_t _PM10 = 0;

eDUST_STATE state = INITIALIZING;

static UART_HandleTypeDef* huart;

static void SendCMD( uint8_t* data, size_t len );
static int ReadCmdResp (uint8_t cmdType, uint8_t * dataBuf, size_t dataBufSize);

static int CheckSum(uint8_t* payload, size_t len);

void Initialize( void* hHandler )
{
	printf("Initializing...\r\n");
	if (hHandler != NULL) {
		huart = (UART_HandleTypeDef *) hHandler;
		state = INITIALIZED;
		printf("Initialized...\r\n");
		HAL_Delay(100);
		StartMeasurement();
		HAL_Delay(100);
		StopMeasurement();
		DisableAutoSend();
	}
}

bool ReadParticle( uint16_t* PM2_5, uint16_t* PM10 )
{
	uint8_t meas_read_data[4] = { HPM_CMD_SEND_HEAD, 0x01, READ_PARTICLE_MEASURMENT, 0x93 };
	static unsigned char dataBuf[HPM_READ_PARTICLE_MEASURMENT_LEN - 1];

	SendCMD(meas_read_data, 4);

	//Read response
	if (ReadCmdResp(READ_PARTICLE_MEASURMENT, dataBuf, sizeof(dataBuf))
			== HAL_OK) {
		_PM2_5 = dataBuf[0] * 256 + dataBuf[1];
		_PM10 = dataBuf[2] * 256 + dataBuf[3];
		*PM2_5 = _PM2_5;
		*PM10 = _PM10;
		return true;
	} else {
		printf("NACK\r\n");
		return false;
	}
}


void StartMeasurement ( void )
{
	uint8_t meas_start_data[] = { HPM_CMD_SEND_HEAD, 0x01, START_PARTICLE_MEASURMENT, 0x96 };
	static unsigned char dataBuf[2] = {};

	printf("StartMeasurement : ");
	SendCMD(meas_start_data, 4);

	if (ReadCmdResp(START_PARTICLE_MEASURMENT, dataBuf, sizeof(dataBuf))
				== HAL_OK) {
		printf("ACK\r\n");
	} else {
		printf("NACK\r\n");
	}
}

void StopMeasurement ( void )
{
	const uint8_t meas_stop_data[] = { HPM_CMD_SEND_HEAD, 0x01, STOP_PARTICLE_MEASURMENT, 0x95 };
	static unsigned char dataBuf[2] = {};

	printf("StopMeasurement : ");
	SendCMD(meas_stop_data, 4);

	if (ReadCmdResp(STOP_PARTICLE_MEASURMENT, dataBuf, sizeof(dataBuf))
			== HAL_OK) {
		printf("ACK\r\n");
	} else {
		printf("NACK\r\n");
	}
}

void SetCoefficient (uint8_t coeff)
{
	uint8_t coeff_set_data[] = { HPM_CMD_SEND_HEAD, 0x02 ,SET_ADJUSTMENT_COEFFICIENT ,coeff, 0 };
	static unsigned char dataBuf[2] = {};

	coeff_set_data[sizeof coeff_set_data - 1] = CheckSum(coeff_set_data, sizeof coeff_set_data);
	printf("SetCoefficient : ");
	SendCMD(coeff_set_data, sizeof coeff_set_data);

	if (ReadCmdResp(SET_ADJUSTMENT_COEFFICIENT, dataBuf, sizeof(dataBuf))
			== HAL_OK) {
		printf("ACK\r\n");
	} else {
		printf("NACK\r\n");
	}

}

void GetCoefficient ( uint8_t * coeff )
{
	const uint8_t coeff_get_data[] = { HPM_CMD_SEND_HEAD, 0x01, READ_ADJUSTMENT_COEFFICIENT, 0x87 };
	static unsigned char dataBuf[2] = {};
	printf("GetCoefficient : ");
	SendCMD(coeff_get_data, 4);

	if (ReadCmdResp(READ_ADJUSTMENT_COEFFICIENT, dataBuf, sizeof(dataBuf))
			== HAL_OK) {
		*coeff = dataBuf[0];
		printf("%d\r\n", dataBuf);
	} else {
		printf("NACK\r\n");
	}
}

void EnableAutoSend( void )
{
	const uint8_t auto_send_en[] = { HPM_CMD_SEND_HEAD, 0x01, ENABLE_AUTO_SEND, 0x57 };
	static unsigned char dataBuf[2] = { };
	printf("EnableAutoSend : ");
	SendCMD(auto_send_en, 4);
	if (ReadCmdResp(ENABLE_AUTO_SEND, dataBuf, sizeof(dataBuf))
			== HAL_OK) {
		printf("ACK\r\n");
	} else {
		printf("NACK\r\n");
	}
}
void DisableAutoSend( void )
{
	const uint8_t auto_send_dis[] = { HPM_CMD_SEND_HEAD, 0x01, STOP_AUTO_SEND, 0x77 };
	static unsigned char dataBuf[2] = {};
	printf("DisableAutoSend : ");
	SendCMD(auto_send_dis, 4);
	if (ReadCmdResp(STOP_AUTO_SEND, dataBuf, sizeof(dataBuf))
			== HAL_OK) {
		printf("ACK\r\n");
	} else {
		printf("NACK\r\n");
	}
}

uint16_t GetPM2_5( void )
{
	return _PM2_5;
}

uint16_t GetPM10( void )
{
	return _PM10;
}

static void SendCMD( uint8_t* data, size_t len )
{
	while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
		;
	if (HAL_UART_Transmit(huart, data, len, 500) != HAL_OK) {
		Error_Handler();
	}

}

static int ReadCmdResp (uint8_t cmdType, uint8_t * dataBuf, size_t dataBufSize)
{
	static unsigned char respBuf[HPM_MAX_RESP_SIZE];
	static unsigned int respIdx = 0;
	static unsigned int calChecksum = 0;
	//Read response
	respIdx = 0;
	calChecksum = 0;
	memset(respBuf, 0, sizeof(respBuf));

	do {
		respBuf[0] = 0;
		HAL_UART_Receive(huart, respBuf, 1, 100);
		if ((respBuf[0] == HPM_CMD_RESP_HEAD)) {
			break;
		}
		else if ((respBuf[0] == ACK) || (respBuf[0] == NACK))
		{
			HAL_UART_Receive(huart, respBuf + HPM_LEN_IDX, 1, 100); //Read the command length
			if ((respBuf[HPM_LEN_IDX] == ACK) || (respBuf[HPM_LEN_IDX] == NACK)){
				memset(dataBuf, 0, dataBufSize);
				memcpy(dataBuf, respBuf, 2);
				return HAL_OK;
			}
			return HAL_ERROR;
		}
	} while (1);

	respBuf[HPM_HEAD_IDX] = HPM_CMD_RESP_HEAD;
	HAL_UART_Receive(huart, respBuf + HPM_LEN_IDX, 1, 100); //Read the command length

	//Ensure buffers are big enough
	if (respBuf[HPM_LEN_IDX]
			&& ((respBuf[HPM_LEN_IDX] + 1) <= sizeof(respBuf) - 2)
			&& (respBuf[HPM_LEN_IDX] - 1) <= dataBufSize) {
		HAL_UART_Receive(huart, respBuf + HPM_CMD_IDX, respBuf[HPM_LEN_IDX] + 1, 500); //Read the command length
//		if (_serial.readBytes(&respBuf[HPM_CMD_IDX], respBuf[HPM_LEN_IDX] + 1)
//				== (respBuf[HPM_LEN_IDX] + 1)) { //read respBuf[HPM_LEN_IDX] num of bytes + calChecksum byte
			if (respBuf[HPM_CMD_IDX] == cmdType) { //check if CMD type matches
//
//				//Calculate and validate checksum
				for (respIdx = 0; respIdx < (2 + respBuf[HPM_LEN_IDX]);
						respIdx++) {
					calChecksum += respBuf[respIdx];
				}
				calChecksum = (65536 - calChecksum) % 256;

				if (calChecksum == respBuf[2 + respBuf[HPM_LEN_IDX]]) {
////					Serial.println("PS- Received valid data!!!");
					memset(dataBuf, 0, dataBufSize);
					memcpy(dataBuf, &respBuf[HPM_DATA_START_IDX],
							respBuf[HPM_LEN_IDX] - 1);
////					return (respBuf[HPM_LEN_IDX] - 1);
					return HAL_OK;
				}
			}
//		}
	}
	return HAL_ERROR;
}

static int CheckSum(uint8_t* payload, size_t len)
{
	uint32_t sum = 0;
	for (int idx = 0; idx < len; ++idx) {
		sum += *payload++;
	}
	return (65536 - sum) % 256;
}
