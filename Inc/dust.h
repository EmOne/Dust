/*
 * dust.h
 *
 *  Created on: Jan 9, 2020
 *      Author: anolp
 */
#ifndef __DUST_H
#define __DUST_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"

#define HPM_CMD_SEND_HEAD 0x68
#define HPM_CMD_RESP_HEAD 0x40
#define HPM_MAX_RESP_SIZE 8 // max command response size is 8 bytes
#define HPM_READ_PARTICLE_MEASURMENT_LEN 5

#define ACK		0xA5
#define NACK 	0x96

#define RESPONSE_TIME	6000u

typedef enum eDUST_STATE_T {
	INITIALIZING,
	INITIALIZED,
	SEND,
	SENT,
	RESPONSE,
	RESPOND
} eDUST_STATE;

#define READ_PARTICLE_MEASURMENT 	0x04
#define START_PARTICLE_MEASURMENT 	0x01
#define STOP_PARTICLE_MEASURMENT 	0x02
#define SET_ADJUSTMENT_COEFFICIENT 	0x08
#define READ_ADJUSTMENT_COEFFICIENT 0x10
#define STOP_AUTO_SEND 				0x20
#define ENABLE_AUTO_SEND 			0x40

typedef enum eHPM_PACKET_T {
    HPM_HEAD_IDX,
    HPM_LEN_IDX,
    HPM_CMD_IDX,
    HPM_DATA_START_IDX
} eHPM_PACKET;

void Initialize( void* hHandler );
bool ReadParticle( uint16_t* PM2_5, uint16_t* PM10 );
void StartMeasurement ( void );
void StopMeasurement ( void );
void SetCoefficient ( uint8_t coeff );
void GetCoefficient ( uint8_t* coeff );
void EnableAutoSend( void );
void DisableAutoSend( void );
uint16_t GetPM2_5( void );
uint16_t GetPM10( void );

#ifdef __cplusplus
}
#endif

#endif /* __DUST_H */
