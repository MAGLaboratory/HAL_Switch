/*
 * output.h
 *
 *  Created on: Feb 28, 2024
 *      Author: brandon
 */

#ifndef INC_OUTPUT_H_
#define INC_OUTPUT_H
#include <stdint.h>
#include <stdbool.h>
#include "HAL_Switch.h"
#include "hal-config.h"

#define REL_OUTVEC_WIDTH (2U)
#define REL_IDX2VEC_WB_SHIFT(idx) (idx*REL_OUTVEC_WIDTH)
#define REL_IDX2VEC_WB(idx) (1U << REL_IDX2VEC_WB_SHIFT(idx))
#define REL_NUM2VEC_WB(num, vec) (vec & REL_IDX2VEC_WB(num))
#define REL_IDX2VEC_OUTPUT_SHIFT(idx) (idx*REL_OUTVEC_WIDTH+1U)
#define REL_IDX2VEC_OUTPUT(idx) (1U << REL_IDX2VEC_OUTPUT_SHIFT(idx))
#define REL_NUM2VEC_OUTPUT(num, vec) (vec & REL_IDX2VEC_OUTPUT(num))
#define REL_VEC2LED_ENUM(num, vec) ((vec & (((1U << REL_OUTVEC_WIDTH) - 1U)\
		<< num*REL_OUTVEC_WIDTH)) >> num*REL_OUTVEC_WIDTH)

/* LED0 is inverted */
#define LED0R_ON()  (GPIO->P[led0r_PORT].DOUTCLR = 1u << led0r_PIN)
#define LED0G_ON()  (GPIO->P[led0g_PORT].DOUTCLR = 1u << led0g_PIN)
#define LED0B_ON()  (GPIO->P[led0b_PORT].DOUTCLR = 1u << led0b_PIN)
#define LED0R_OFF() (GPIO->P[led0r_PORT].DOUTSET = 1u << led0r_PIN)
#define LED0G_OFF() (GPIO->P[led0g_PORT].DOUTSET = 1u << led0g_PIN)
#define LED0B_OFF() (GPIO->P[led0b_PORT].DOUTSET = 1u << led0b_PIN)
#define LED1R_ON()  (GPIO->P[led1r_PORT].DOUTSET = 1u << led1r_PIN)
#define LED1G_ON()  (GPIO->P[led1g_PORT].DOUTSET = 1u << led1g_PIN)
#define LED1B_ON()  (GPIO->P[led1b_PORT].DOUTSET = 1u << led1b_PIN)
#define LED1R_OFF() (GPIO->P[led1r_PORT].DOUTCLR = 1u << led1r_PIN)
#define LED1G_OFF() (GPIO->P[led1g_PORT].DOUTCLR = 1u << led1g_PIN)
#define LED1B_OFF() (GPIO->P[led1b_PORT].DOUTCLR = 1u << led1b_PIN)

#define C_RELAY_ONF_TIME  (200u)
#define C_RELAY_ONR_TIME  (10000u-C_RELAY_ONF_TIME)
#define C_RELAY_ONFR_TIME (200u)

#define C_RELAY_CCV_FULL    (125u)
#define C_RELAY_CCV_REDUCED (125u*70u/100u)

/*****************************************************************************
 *  Type Definitions
 *****************************************************************************/
typedef enum
{
	eLED_R = 0,
	eLED_G,
	eLED_B,
	eLED_NUM_CHANNELS
}LEDchanType;

typedef enum
{
	eOUT_OFF = 0,
	eOUT_ON,
	eOUT_NUM_STATES
}OutputType;

typedef union
{
	struct
	{
		uint8_t output: 2;
		uint8_t led : 2;
		uint8_t chan: 4;
	} b;
	enum
	{
		eLED0_R_OFF  = 0b00000000,
		eLED0_R_ON	 = 0b00000001,
		eLED1_R_OFF  = 0b00000100,
		eLED1_R_ON   = 0b00000101,
		eLED0_G_OFF  = 0b00010000,
		eLED0_G_ON   = 0b00010001,
		eLED1_G_OFF  = 0b00010100,
		eLED1_G_ON   = 0b00010101,
		eLED0_B_OFF  = 0b00100000,
		eLED0_B_ON   = 0b00100001,
		eLED1_B_OFF  = 0b00100100,
		eLED1_B_ON   = 0b00100101
	} m;
} LEDMagicType;

void LED_Write(uint8_t led, LEDchanType chan, uint8_t state);

/* switch mode */
typedef enum
{
	eSW_MANUAL_CONTROL_AUTO_OFF = 0,
	eSW_MANUAL_CONTROL,
	eSW_MACHINE_CONTROL,
	eSW_DISABLE_CONTROL
} SW_State_t;

/* LED output modes */
typedef enum
{
	eLS_Off  = 0,
	eLS_aOn  = 0b01,
	eLS_aOff = 0b10,
	eLS_On   = 0b11,
	eLS_HB,
	eLS_Dummy_System,
	eLS_Dummy_System_aOff,
	eLS_Motion,
	eLS_Motion_aOff,
	eLS_Light,
	eLS_Light_aOff,
	eLS_NUM_STATES
} LED_State_t;

#define C_LED_ZERO_STATE {{0U, 0U, 0U}, 0U}
#define C_LED_BLINK_STATE {{0U, 0U, 0U}, 500U}

typedef struct
{
	uint16_t r :4;
	uint16_t g :4;
	uint16_t b :4;
} T_LED_COLOR;

typedef struct
{
	T_LED_COLOR color;
	uint16_t duration;
} T_LED_BLINK;

typedef struct
{
	uint8_t state: 1;
	uint32_t blinkCounter;
	uint16_t duration;
	const T_LED_BLINK *lastPri;
	const T_LED_BLINK *lastSec;
} T_BLINK_SEL_OUTPUT;

/* blink code */
T_LED_COLOR blink_sel(uint32_t msCounter, const T_LED_BLINK *pri,
		const T_LED_BLINK *sec, T_BLINK_SEL_OUTPUT *out);

/* soft pwm code */
void led_pwm_out(uint8_t num, uint32_t msCounter, T_LED_COLOR color);

typedef enum
{
	eRelaySMOff = 0,
	eRelaySMOnFull,
	eRelaySMOnReduced,
	eRelaySMOnFullReseat
} T_RELAY_SM_STATE;

typedef struct
{
	uint32_t cas; /*< count at start */
	T_RELAY_SM_STATE state;
} T_RELAY_SM_OUTPUT;

void RelaySM(uint8_t relayNum, uint8_t relayVec, uint32_t msCounter,
		T_RELAY_SM_OUTPUT* out);


/*
 * "Start Up / Shut Down" State Machine (SDSUSM)
 */

#define C_SDSU_DEF_HID_OFF_PERIOD (30 * 1000)
#define C_SDSU_DEF_HID_ON_PERIOD (30 * 1000)

typedef enum
{
	eSDSU_Off = 0,
	eSDSU_NotOn,
	eSDSU_On,
	eSDSU_NotOff
} SDSUSMState_t;

typedef struct
{
	uint32_t onPeriod;
	uint32_t offPeriod;
}T_SDSUSM_CFG;

typedef struct
{
	uint32_t lastCounter;
	SDSUSMState_t State;
}T_SDSUSM_OUTPUT;

bool SDSUSM(uint8_t num, uint8_t vec, uint32_t Counter, T_SDSUSM_CFG* cfg,
		T_SDSUSM_OUTPUT* Out);

#endif /* INC_OUTPUT_H_ */
