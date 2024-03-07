/*
 * ModbusMiddleware.h
 *
 *  Created on: Mar 5, 2024
 *      Author: brandon
 */

#ifndef INC_MODBUSMIDDLEWARE_H_
#define INC_MODBUSMIDDLEWARE_H_

typedef enum
{
	eMMREG_16B = 0,
	eMMREG_32B,
	eMMREG_A64B,
	eMMREG_64B
};

typedef struct
{
	uint8_t lastAddress;
	uint8_t targetSeq:2;
	uint8_t lastSeq:2;
	uint16_t buffer[4];
} MM_Inter_t;

#endif /* INC_MODBUSMIDDLEWARE_H_ */
