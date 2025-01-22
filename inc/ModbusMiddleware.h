/*
 * ModbusMiddleWare.h
 *
 *  Created on: Mar 5, 2024
 *      Author: brandon
 */

#ifndef INC_MODBUSMIDDLEWARE_H_
#define INC_MODBUSMIDDLEWARE_H_
#include <stdint.h>
#include <stdbool.h>

#define MMW_STRUCT_INTERNAL 0x1515U
#define MMW_STRUCT_EXTERNAL 0x1616U

#include "UserModbusMiddleWare.h"
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
	uint8_t targetSeq	:2;
	uint8_t la_inval	:1;
	uint16_t buffer[4];
} T_MMW_Write;


typedef struct
{
	const uint8_t *const seq;
	uint16_t (*const mb_reg[]);
} T_MMW_Data;

typedef struct
{
	uint16_t start_addr;
	uint16_t real_addr;
	uint8_t real_seq		:2;
	uint8_t cur_seq			:2;
	uint8_t buffer_invalid	:1;
	uint16_t read_buf[4];
} T_MMW_Read;

/* Extracts the indexed 2-bit data out of seq (sequence LookUp) */
/* finds the index within sequence and then shifts a mask to the target sequence */
#define SEQ_LU(s, i) ((s.seq[(i) >> 2U]&(0b11U << (((i) & 0b11U) * 2U))) >> (((i) & 0b11U) * 2U))
/* Formats four x 2-bit items into an octet */
#define TO_SEQ(a, b, c, d) ((d << 2U*3U)|(c << 2U*2U)|(b << 2U*1U)|(a))

#if defined(MMW_STRUCT_TYPE)
#if MMW_STRUCT_TYPE == MMW_STRUCT_INTERNAL
#define MMW_FD_DATA_STRUCT ()
#define MMW_FD_READ_STRUCT ()
#define MMW_FD_WRITE_STRUCT ()
#define MMW_REF_DATA_STRUCT (mb_d)
#define MMW_REF_READ_STRUCT (mb_r)
#define MMW_REF_WRITE_STRUCT (mb_w)
#define MMW_CALL_DATA_STRUCT ()
#define MMW_CALL_READ_STRUCT ()
#define MMW_CALL_WRITE_STRUCT ()
#elif MMW_STRUCT_TYPE == MMW_STRUCT_EXTERNAL
#define MMW_FD_DATA_STRUCT (T_MMW_Data *const md_st,)
#define MMW_FD_READ_STRUCT (T_MMW_Read *const mr_st,)
#define MMW_FD_WRITE_STRUCT (T_MMw_Write *const mw_st,)
#define MMW_REF_DATA_STRUCT (*md_st)
#define MMW_REF_READ_STRUCT (*mr_st)
#define MMW_REF_WRITE_STRUCT (*mw_st)
#define MMW_CALL_DATA_STRUCT (md_st,)
#define MMW_CALL_READ_STRUCT (mr_st,)
#define MMW_CALL_WRITE_STRUCT (mw_st,)
#else
#error "MMW_STRUCT_TYPE incorrectly defined"
#endif /* MMW_STRUCT_TYPE */
#else
#error "MMW_STRUCT_TYPE not defined"
#endif /* defined(MMW_STRUCT_TYPE) */

void MMW_Init(MMW_FD_DATA_STRUCT MMW_FD_READ_STRUCT MMW_FD_WRITE_STRUCT);
bool MMW_Write_Register(MMW_FD_DATA_STRUCT MMW_FD_WRITE_STRUCT uint16_t addr, uint16_t data);
bool MMW_Read_Register(MMW_FD_DATA_STRUCT MMW_FD_READ_STRUCT uint16_t addr, uint16_t *const data);

#endif /* INC_MODBUSMIDDLEWARE_H_ */
