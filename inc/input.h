/*
 * input.h
 *
 *  Created on: Feb 28, 2024
 *      Author: brandon
 */

#ifndef INC_INPUT_H_
#define INC_INPUT_H_
#include "em_assert.h"
#include <stdint.h>
#include <stdbool.h>

#define CAP_INVEC_WIDTH (4U)
#define CAP_IDX2VEC_STATUS_SHIFT(idx) (idx*CAP_INVEC_WIDTH)
#define CAP_IDX2VEC_STATUS(idx) (1U << CAP_IDX2VEC_STATUS_SHIFT(idx))
#define CAP_NUM2VEC_STATUS(num, vec) (vec & CAP_IDX2VEC_STATUS(num))
#define CAP_IDX2VEC_HOLDOFF_SHIFT(idx) (idx*CAP_INVEC_WIDTH+1U)
#define CAP_IDX2VEC_HOLDOFF(idx) (1U << CAP_IDX2VEC_HOLDOFF_SHIFT(idx))
#define CAP_NUM2VEC_HOLDOFF(num, vec) (vec & CAP_IDX2VEC_HOLDOFF(num))
#define CAP_IDX2VEC_CMD_SHIFT(idx) (idx*CAP_INVEC_WIDTH+3U)
#define CAP_IDX2VEC_CMD(idx) (1U << CAP_IDX2VEC_CMD_SHIFT(idx))
#define CAP_NUM2VEC_CMD(num, vec) (vec & CAP_IDX2VEC_CMD(num))
#define CAP_RISING_EDGE(num, vec) (CAP_NUM2VEC_STATUS(num, vec) != 0\
			&& CAP_NUM2VEC_HOLDOFF(num, vec) == 0)
#define CAP_FALLING_EDGE(num, vec) (CAP_NUM2VEC_STATUS(num, vec) == 0\
			&& CAP_NUM2VEC_HOLDOFF(num, vec) != 0)

#define COMM_INVEC_WIDTH (CAP_INVEC_WIDTH)
#define COMM_IDX2VEC_STATE_SHIFT(idx) (idx*COMM_INVEC_WIDTH)
#define COMM_IDX2VEC_STATE(idx) (1U << COMM_IDX2VEC_STATE_SHIFT(idx))
#define COMM_NUM2VEC_STATE(num, vec) (vec & COMM_IDX2VEC_STATE(num))
#define COMM_IDX2VEC_RX_SHIFT(idx) (idx*COMM_INVEC_WIDTH+1U)
#define COMM_IDX2VEC_RX(idx) (1U << COMM_IDX2VEC_RX_SHIFT(idx))
#define COMM_NUM2VEC_RX(num, vec) (vec & COMM_IDX2VEC_RX(num))
#define COMM_IDX2VEC_CSM_SHIFT(idx) (idx*COMM_INVEC_WIDTH+2U)
#define COMM_IDX2VEC_CSM(idx) (1U << COMM_IDX2VEC_CSM_SHIFT(idx))
#define COMM_NUM2VEC_CSM(num, vec) (vec & COMM_IDX2VEC_CSM(num))
#define COMM_IDX2VEC_CMD_SHIFT(idx) (idx*COMM_INVEC_WIDTH+3U)
#define COMM_IDX2VEC_CMD(idx) (1U << COMM_IDX2VEC_CMD_SHIFT(idx))
#define COMM_NUM2VEC_CMD(num, vec) (vec & COMM_IDX2VEC_CMD(num))

/* Debounce State Machine */
typedef struct
{
  uint8_t count : 7;
  uint8_t output : 1;
} DSMOutputStorageBitfieldType;

typedef union
{
  uint8_t c;
  DSMOutputStorageBitfieldType s;
} DSMOutputType;

uint8_t DebounceSM(uint8_t input, uint8_t debounceThreshold, DSMOutputType* out);

typedef struct
{
	uint8_t num;
	uint8_t vec;
	uint32_t pressCounter;
} Button_t;

typedef enum
{
	eAOSM_Off = 0,
	eAOSM_On,
	eAOSM_aOff,
	eAOSM_mOff,
} AOSM_State_t;

typedef struct
{
	uint32_t long_press;
	uint32_t on_time;
	uint32_t aoff_time;
	uint32_t moff_time;
} AOSM_CFG_t;

typedef struct
{
	AOSM_State_t state;
	AOSM_State_t lastState;
	uint32_t counter;
} AOSM_Output_t;

#define C_AOSM_LONG_PRESS (1000UL)
#define C_AOSM_ON_TIMER (1000UL * 60UL)
#define C_AOSM_OFF_TIMER (1000UL * 60UL)
#define C_AOSM_MOFF_TIMER (1000UL * 60UL)

/* Auto Off State Machine */
bool AOSM(Button_t *in, uint32_t msCounter, AOSM_CFG_t *cfg,
		AOSM_Output_t *out);

void CommSM(uint8_t num, uint8_t vec, uint32_t msCounter, uint32_t thresh, uint32_t *commCounter);

typedef enum
{
	eADSM_Off = 0,
	eADSM_nOn,
	eADSM_On,
	eADSM_nOff
} ADSM_State_t;

typedef struct
{
	ADSM_State_t state;
	uint32_t counter;
} ADSM_Output_t;

typedef struct
{
	uint32_t onThresh;
	uint32_t offThresh;
} ADSM_Cfg_t;

void ADSM(uint8_t num, uint8_t vec, uint32_t msCounter, ADSM_Cfg_t *cfg, ADSM_Output_t *out);

typedef enum
{
	eCON_AO = 0,
	eCON_MAN,
	eCON_CMD_OVR,
	eCON_CMD_MOT,
	eCON_CMD_LIT
} Control_State_t;

#endif /* INC_INPUT_H_ */
