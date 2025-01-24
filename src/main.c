#include <stdint.h>
#include <stdbool.h>
#include "em_assert.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "retargetserial.h"
#include "kiricapsense.h"
#include "PetitModbusPort.h"
#include "PetitModbus.h"
#include "ModbusMiddleware.h"
#include "hal-config.h"
#include "input.h"
#include "output.h"

/*****************************************************************************
 * Defines here
 *****************************************************************************/

/* The sequence, registers, and register reference definitions */
const uint8_t seq[] =
{
	TO_SEQ(eMMREG_16B, eMMREG_16B, eMMREG_16B, eMMREG_16B),
	TO_SEQ(eMMREG_32B, eMMREG_16B, eMMREG_32B, eMMREG_16B),
	TO_SEQ(eMMREG_64B, eMMREG_A64B, eMMREG_32B, eMMREG_16B)
};

uint16_t reg_a, reg_b, reg_c, reg_d;
uint32_t reg_e, reg_f;
uint64_t reg_g;

uint16_t (*const real_mb_reg[]) =
{&reg_a, &reg_b, &reg_c, &reg_d, (uint16_t*)&reg_e, (uint16_t*)&reg_f, (uint16_t*)&reg_g};

T_MMW_Data md_st = {seq, real_mb_reg};
T_MMW_Read mr_st;
T_MMW_Write mw_st;

const uint8_t h_seq[] =
{
	TO_SEQ(eMMREG_32B, eMMREG_16B, eMMREG_32B, eMMREG_16B),
	TO_SEQ(eMMREG_16B, eMMREG_16B, eMMREG_16B, eMMREG_16B)
};

uint16_t (*const real_hd_reg[]) =
{&WS_AOSM[0].timeLeft, &WS_AOSM[1].timeLeft, &WS_AOSM[0].lastState, &WS_AOSM[1].lastState, kcs_channelBaseline[0], kcs_channelBaseline[1]};

T_MMW_Data hd_st = {h_seq, real_hd_reg};
T_MMW_Read hr_st;

const T_LED_BLINK LED_States[eLS_NUM_STATES][2] __attribute__((section(".text.consts")))=
{
	{{{0U, 0U, 0U}, 100U}, C_LED_ZERO_STATE}, // off
	{{{6U, 6U, 0U}, 500U}, C_LED_BLINK_STATE}, // WB on
	{C_LED_BLINK_STATE, {{8U, 8U, 8U}, 500U}}, // WB off
	{{{8U, 8U, 8U}, 100U}, C_LED_ZERO_STATE}, // ON
	{{{1U, 1U, 1U}, 100U}, C_LED_ZERO_STATE}, // heartbeat
	{{{8U, 0U, 0U}, 500U}, C_LED_BLINK_STATE}, // red flashing
	{{{8U, 0U, 0U}, 100U}, C_LED_ZERO_STATE}, // red
	{{{0U, 8U, 0U}, 500U}, C_LED_BLINK_STATE}, // green flashing
	{{{0U, 8U, 0U}, 100U}, C_LED_ZERO_STATE}, // green
	{{{0U, 8U, 8U}, 500U}, C_LED_BLINK_STATE}, // cyan flashing
	{{{0U, 8U, 8U}, 100U}, C_LED_ZERO_STATE}, // cyan
};


void GPIO_Init(void)
{
	/* GPIO */
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* LED 1 is Push-ON (active HIGH) */
	GPIO_PinModeSet(led1r_PORT, led1r_PIN, gpioModePushPullDrive, 0);
	GPIO_DriveModeSet(led1r_PORT, gpioDriveModeHigh); /*< GPIO A */
	GPIO_PinModeSet(led1g_PORT, led1g_PIN, gpioModePushPullDrive, 0);
	GPIO_DriveModeSet(led1g_PORT, gpioDriveModeHigh); /*< GPIO E */
	GPIO_PinModeSet(led1b_PORT, led1b_PIN, gpioModePushPullDrive, 0);
	GPIO_DriveModeSet(led1b_PORT, gpioDriveModeHigh); /*< GPIO B */
	/* LED 0 is Pull-ON (active LOW) */
	GPIO_PinModeSet(led0r_PORT, led0r_PIN, gpioModePushPullDrive, 1);
	GPIO_PinModeSet(led0g_PORT, led0g_PIN, gpioModePushPullDrive, 1);
	GPIO_DriveModeSet(led0g_PORT, gpioDriveModeHigh); /* GPIO C */
	GPIO_PinModeSet(led0b_PORT, led0b_PIN, gpioModePushPullDrive, 1);

	GPIO_PinModeSet(txen_PORT, txen_PIN, gpioModePushPullDrive, 0);

	/* Relays */
	GPIO_PinModeSet(relay0_PORT, relay0_PIN, gpioModePushPullDrive, 0);
	GPIO_PinModeSet(relay1_PORT, relay1_PIN, gpioModePushPullDrive, 0);
	GPIO_DriveModeSet(relay1_PORT, gpioDriveModeStandard); /* GPIO F */

	/* GPIO D RX / TX  */
	GPIO_PinModeSet(RETARGET_TXPORT, RETARGET_TXPIN, gpioModePushPull, 1);
	GPIO_PinModeSet(RETARGET_RXPORT, RETARGET_RXPIN, gpioModeInputPull, 1);
	GPIO_DriveModeSet(RETARGET_TXPORT, gpioDriveModeStandard); /* GPIO D */
}

void PWM_Init(void)
{
	/* Uses Timer2 to generate PWM for relay power saving */
	/* Assumes HFPERCLK is already enabled */
	CMU_ClockEnable(cmuClock_TIMER2, true);

	/* Initialize Timer2 */
	TIMER2->CTRL = TIMER_CTRL_PRESC_DIV4 | TIMER_CTRL_DEBUGRUN;
	TIMER2->TOP = 124u;
	TIMER2->CNT = 0;
	TIMER2->CC[0].CCV = 0;
	TIMER2->CC[0].CCVB = 0;
	TIMER2->CC[0].CTRL = TIMER_CC_CTRL_CMOA_CLEAR | TIMER_CC_CTRL_MODE_PWM;
	TIMER2->CC[1].CCV = 0;
	TIMER2->CC[1].CCVB = 0;
	TIMER2->CC[1].CTRL = TIMER_CC_CTRL_CMOA_CLEAR | TIMER_CC_CTRL_MODE_PWM;
	/* Enable Peripheral Routing for Relay PWM */
	TIMER2->ROUTE = TIMER_ROUTE_LOCATION_LOC3 | TIMER_ROUTE_CC0PEN
			| TIMER_ROUTE_CC1PEN;
}

void msCounter_Init(void)
{
	/* HFPER clock must be enabled before these calls */
	/* CMU for TIMER0 */
	CMU_ClockEnable(cmuClock_TIMER0, true);

	/* Initialize TIMER0 */
	/* SYSCLK is 14e6 at this point */
	/* Prescaler is div16 */
	/* 875 counts, but minus one from the fundamental counting principal */
	TIMER0->CTRL = TIMER_CTRL_PRESC_DIV16;
	TIMER0->TOP = 874u;
	TIMER0->IEN = TIMER_IEN_OF;
	TIMER0->CNT = 0;
}

void UART_Init(void)
{
	USART_TypeDef *usart = RETARGET_UART;
	USART_InitAsync_TypeDef init =
	{ usartDisable, /* Disable RX/TX when initialization is complete. */
	0, /* Use current configured reference clock for configuring baud rate. */
	38400, /* baud */
	usartOVS16, /* 16x oversampling. */
	usartDatabits8, /* 8 data bits. */
	usartNoParity, /* No parity. */
	usartStopbits1, /* 1 stop bit. */
	false, /* Do not disable majority vote. */
	false, /* Not USART PRS input mode. */
	0, /* PRS channel 0. */
	true, /* Auto CS functionality enable/disable switch */
	};
	CMU_ClockEnable(RETARGET_CLK, true);

	USART_InitAsync(usart, &init);

	usart->ROUTE = USART_ROUTE_CSPEN | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN
			| USART_ROUTE_LOCATION_LOC3;

	usart->CTRL |= USART_CTRL_CSINV;

	/* Clear previous RX interrupts */
	USART_IntClear(RETARGET_UART, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(RETARGET_IRQn);

	/* Enable RX interrupts */
	USART_IntEnable(RETARGET_UART, USART_IF_RXDATAV);
	NVIC_EnableIRQ(RETARGET_IRQn);

	/* Clear previous TX interrupts */
	USART_IntClear(RETARGET_UART, USART_IF_TXC);
	NVIC_ClearPendingIRQ(USART1_TX_IRQn);

	/* Enable TX interrupts */
	USART_IntEnable(RETARGET_UART, USART_IF_TXC);
	NVIC_EnableIRQ(USART1_TX_IRQn);

	/* Finally enable it */
	USART_Enable(usart, usartEnable);
}

/*****************************************************************************
 * Main Function and Variables
 *****************************************************************************/
/* WS probably stands for Wall Switch or something IDK. */
volatile uint32_t msCounter = 0;
uint32_t lastCounter = 0;

uint8_t capVec = 0;
uint8_t commVec = 0;
uint8_t relayVec = 0;
uint32_t pressTS[2];

T_RELAY_SM_OUTPUT WS_Relay[2];

T_SDSUSM_CFG CF_SDSU[2] =
{{C_SDSU_DEF_HID_ON_PERIOD, C_SDSU_DEF_HID_OFF_PERIOD},
{ 0, 0 }};
T_SDSUSM_OUTPUT WS_SDSU[2];

T_AOSM_CFG CF_AOSM[2] =
{{C_AOSM_LONG_PRESS, C_AOSM_ON_TIMER, C_AOSM_OFF_TIMER, C_AOSM_MOFF_TIMER},
{C_AOSM_LONG_PRESS, 0, 0, 0}};
T_AOSM_OUTPUT WS_AOSM[2];

bool simpleDisplay = false;
// no struct for communication state machine
uint32_t CSM_Counter[2];

T_ADSM_CFG CF_ADSM[eADSM_CFG_S_NUM] =
{
	{0, 0},
	{C_ADSM_ONE_MIN, C_ADSM_ONE_MIN},
	{C_ADSM_FIFTEEN_MINS, C_ADSM_FIFTEEN_MINS}
};

T_ADSM_CFG *pCF_ADSM[2] = {&CF_ADSM[0], &CF_ADSM[0]};
T_ADSM_OUTPUT WS_ADSM[2];

T_CONTROL_STATE WS_Control[2];
T_CONTROL_STATE last_WS_Control[2];

const T_LED_BLINK (*pLED_Blink_States[2][4])[2] =
{{&LED_States[eLS_Off], &LED_States[eLS_aOn], &LED_States[eLS_aOff], &LED_States[eLS_On]},
{&LED_States[eLS_Off], &LED_States[eLS_aOn], &LED_States[eLS_aOff], &LED_States[eLS_On]}};

T_BLINK_SEL_OUTPUT led_bs[2];

T_PETIT_MODBUS Petit;

void cap2cmd(uint8_t i)
{
	// set communication state machine
	// changeover state from the capacitive vector
	if (last_WS_Control[i] == eCON_AO)
	{
		BIT_CHANGE(COMM_IDX2VEC_CMD(i), commVec,
				CAP_NUM2VEC_CMD(i, capVec));
		BIT_CHANGE(COMM_IDX2VEC_STATE(i), commVec,
				CAP_NUM2VEC_CMD(i, capVec));

		if (COMM_NUM2VEC_CMD(i, capVec) != 0)
		{
			WS_ADSM[i].state = eADSM_On;
		}
		else
		{
			WS_ADSM[i].state = eADSM_Off;
		}

		// set RX bit and let Communication SM take over
		commVec |= COMM_IDX2VEC_RX(i);
	}
	else if (last_WS_Control[i] == eCON_MAN)
	{
		BIT_CHANGE(COMM_IDX2VEC_CMD(i), commVec,
				CAP_NUM2VEC_CMD(i, capVec));
		BIT_CHANGE(COMM_IDX2VEC_STATE(i), commVec,
				CAP_NUM2VEC_CMD(i, capVec));

		if (COMM_NUM2VEC_CMD(i, capVec) != 0)
		{
			WS_ADSM[i].state = eADSM_On;
		}
		else
		{
			WS_ADSM[i].state = eADSM_Off;
		}

		// set RX bit and let Communication SM take over
		commVec |= COMM_IDX2VEC_RX(i);
	}
}

uint8_t ledEnumCalc(uint8_t num, uint8_t vec, bool sdm)
{
	if (sdm)
	{
		// check the "will be" bit
		if (REL_NUM2VEC_WB(num, vec) != 0)
		{
			return eLS_On;
		}
		else
		{
			return eLS_Off;
		}
	}
	else
	{
		return REL_VEC2LED_ENUM(num, vec);
	}
}

void ledColorChange(uint8_t num)
{
	if (WS_Control[num] != last_WS_Control[num])
	{
		switch(WS_Control[num])
		{
		case eCON_AO:
			pLED_Blink_States[num][eLS_aOff] =
					&LED_States[eLS_aOff];
			pLED_Blink_States[num][eLS_On] =
					&LED_States[eLS_On];
			break;
		case eCON_MAN:
			pLED_Blink_States[num][eLS_aOff] =
					&LED_States[eLS_aOff];
			pLED_Blink_States[num][eLS_On] =
					&LED_States[eLS_On];
			break;
		case eCON_CMD_OVR:
			pLED_Blink_States[num][eLS_aOff] =
					&LED_States[eLS_Dummy_System_aOff];
			pLED_Blink_States[num][eLS_On] =
					&LED_States[eLS_Dummy_System];
			break;
		case eCON_CMD_MOT:
			pLED_Blink_States[num][eLS_aOff] =
					&LED_States[eLS_Motion_aOff];
			pLED_Blink_States[num][eLS_On] =
					&LED_States[eLS_Motion];
			break;
		case eCON_CMD_LIT:
			pLED_Blink_States[num][eLS_aOff] =
					&LED_States[eLS_Light_aOff];
			pLED_Blink_States[num][eLS_On] =
					&LED_States[eLS_Light];
			break;
		}
	}
}

void HAL_SWITCH_Process()
{
	// process
	for (uint8_t i = 0; i < KCS_NUM_CHANNELS; i++)
	{
		// transition states
		switch (WS_Control[i])
		{
		case eCON_AO:
			if (last_WS_Control[i] == eCON_MAN)
			{
				// transfer to auto off state machine based on
				// manual state.  can not determine auto off time.
				//
				// the last state in the auto off state machine
				// (aosm) is used for On state button presses.
				if (CAP_NUM2VEC_CMD(i, capVec) != 0)
				{
					WS_AOSM[i].counter = msCounter;
					WS_AOSM[i].state = eAOSM_On;
					WS_AOSM[i].lastState = eAOSM_Off;
				}
				else
				{
					WS_AOSM[i].counter = msCounter;
					WS_AOSM[i].state = eAOSM_Off;
					WS_AOSM[i].lastState = eAOSM_Off;
				}
			}
			// last control mode was commanded
			else if (last_WS_Control[i] != eCON_AO)
			{
				if (COMM_NUM2VEC_CMD(i, commVec) != 0)
				{
					WS_AOSM[i].counter = msCounter;
					WS_AOSM[i].state = eAOSM_On;
					WS_AOSM[i].lastState = eAOSM_On;
				}
				else
				{
					WS_AOSM[i].counter = msCounter;
					WS_AOSM[i].state = eAOSM_Off;
					WS_AOSM[i].lastState = eAOSM_Off;
				}
			}
			break;
		case eCON_MAN:
			if (last_WS_Control[i] == eCON_AO)
			{
				BIT_CHANGE(CAP_IDX2VEC_CMD(i), capVec,
						WS_AOSM[i].state != eAOSM_Off);
			}
			else if (last_WS_Control[i] != eCON_MAN)
			{
				BIT_CHANGE(CAP_IDX2VEC_CMD(i), capVec,
						COMM_NUM2VEC_CMD(i, commVec));
			}
			break;
		case eCON_CMD_OVR:
		case eCON_CMD_MOT:
		case eCON_CMD_LIT:
			cap2cmd(i);
			break;
		default:
			WS_Control[i] = eCON_AO;
			break;
		}
		ledColorChange(i);
		// run states
		AOSM_Input_t button;
		switch (WS_Control[i])
		{
		case eCON_AO:
			button.num = i;
			button.capVec = capVec;
			button.relayVec = relayVec;
			button.pressCounter = pressTS[i];
			BIT_CHANGE(CAP_IDX2VEC_CMD(i), capVec,
					AOSM(&button, msCounter, &CF_AOSM[i],
							&WS_AOSM[i]));
			BIT_CHANGE(REL_IDX2VEC_WB(i), relayVec,
					WS_AOSM[i].state == eAOSM_On);
			break;
		case eCON_MAN:
			if (CAP_RISING_EDGE(i, capVec))
			{
				capVec ^= CAP_IDX2VEC_CMD(i);
			}
			BIT_CHANGE(REL_IDX2VEC_WB(i), relayVec,
					CAP_NUM2VEC_CMD(i, capVec));
			break;
		case eCON_CMD_OVR:
		case eCON_CMD_MOT:
		case eCON_CMD_LIT:
			CommSM(i, commVec, msCounter, C_COMM_THRESH,
					&CSM_Counter[i]);
			ADSM(i, commVec, msCounter, pCF_ADSM[i], &WS_ADSM[i]);
			break;
		}

		// update holdoff value in the capacitance vector
		// the holdoff is more or less the "last" value
		BIT_CHANGE(CAP_IDX2VEC_HOLDOFF(i), capVec,
				CAP_NUM2VEC_STATUS(i, capVec));

		// clear RX flag
		commVec &= ~COMM_IDX2VEC_RX(i);

		last_WS_Control[i] = WS_Control[i];
	}
}

int main(void)
{
	/* Chip errata */
	CHIP_Init();

	/* Set interrupt priority to let systick preempt */
	for (uint8_t i = 0; i < 21; i++)
	{
		NVIC_SetPriority(i, 1u);
	}
	NVIC_SetPriority(SysTick_IRQn, 0U);

	CMU_ClockEnable(cmuClock_HFPER, true);

	GPIO_Init();

	/* modbus middleware init */
	MMW_Init(&md_st, &mr_st, &mw_st);
	MMW_Init(&hd_st, &hr_st, NULL);

	PETIT_MODBUS_Init(&Petit);
	Petit.Tx_Begin = PetitPortTxBegin;
	Petit.Timer_Start = PetitPortTimerStart;
	Petit.Timer_Stop = PetitPortTimerStop;

	UART_Init();

	msCounter_Init();

	KIRICAPSENSE_Init();

	/* Enable TIMER0 interrupt */
	NVIC_EnableIRQ(TIMER0_IRQn);

	PWM_Init();

	/* Start Timer0 */
	TIMER0->CMD = TIMER_CMD_START;

	/* Start Timer1 */
	/* TODO: should be in kirisaki capsense */
	TIMER1->CMD = TIMER_CMD_START;

	/* Start Timer2 */
	TIMER2->CMD = TIMER_CMD_START;

	/* Infinite loop */
	while (1)
	{
		if (msCounter - lastCounter >= 1u)
		{
			lastCounter += 1u;

			PETIT_MODBUS_Process(&Petit);
			KIRICAPSENSE_process();

			// button calculation
			for (uint8_t touchRdy = KIRICAPSENSE_pressReady(); touchRdy != 255U;
					touchRdy = KIRICAPSENSE_pressReady())
			{
				BIT_CHANGE(CAP_IDX2VEC_STATUS(touchRdy), capVec,
						KIRICAPSENSE_getPressed(touchRdy));

				if (CAP_RISING_EDGE(touchRdy, capVec))
				{
					pressTS[touchRdy] = msCounter;
				}
			}

			HAL_SWITCH_Process();


			// relay output
			for (uint8_t i = 0; i < KCS_NUM_CHANNELS; i++)
			{
				BIT_CHANGE(REL_IDX2VEC_OUTPUT(i), relayVec,
						SDSUSM(i, capVec, msCounter, &CF_SDSU[i],
								&WS_SDSU[i]));

				RelaySM(i, relayVec, msCounter, &WS_Relay[i]);
			}

			// LED output
			for (uint8_t i = 0; i < KCS_NUM_CHANNELS; i++)
			{
				const T_LED_BLINK (*currentBlink)[2] =
						pLED_Blink_States[i][ledEnumCalc(i, relayVec, simpleDisplay)];

				T_LED_COLOR normalColor = blink_sel(msCounter,
						&((*currentBlink)[0]), &((*currentBlink)[1]),
						&led_bs[i]);

				led_pwm_out(i, msCounter, normalColor);
			}
		}
		else
		{
			// wfi might actually stop the systick
		}
	}
}
