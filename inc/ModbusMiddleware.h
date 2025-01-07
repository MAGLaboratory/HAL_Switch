/*
 * ModbusMiddleware.h
 *
 *  Created on: Mar 5, 2024
 *      Author: brandon
 */

#ifndef INC_MODBUSMIDDLEWARE_H_
#define INC_MODBUSMIDDLEWARE_H_
#include <stdint.h>
typedef enum
{
	eMMREG_16B = 0,
	eMMREG_32B,
	eMMREG_A64B,
	eMMREG_64B
} REGLen_t;

typedef struct
{
	uint16_t lastAddr;
	uint8_t la_inval:1;
	uint8_t targetSeq:2;
	uint16_t buffer[4];
} MM_Inter_t;

/* Extracts the indexed 2-bit data out of seq (sequence LookUp) */
/* finds the index within sequence and then shifts a mask to the target sequence */
#define SEQ_LU(i) ((seq[(i) >> 2U]&(0b11U << (((i) & 0b11U) * 2U))) >> (((i) & 0b11U) * 2U))
/* Formats four x 2-bit items into an octet */
#define TO_SEQ(a, b, c, d) ((d << 2U*3U)|(c << 2U*2U)|(b << 2U*1U)|(a))

#endif /* INC_MODBUSMIDDLEWARE_H_ */
