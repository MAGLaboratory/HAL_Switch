#include "em_assert.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "retargetserial.h"
#include "hal-config.h"
#include "kiricapsense.h"
#include "PetitModbusPort.h"
#include "PetitModbus.h"

/*****************************************************************************
 * Defines here
 *****************************************************************************/
#define C_HEARTBEAT_TOP (124u)
#define C_HB_SHIFT (0)
#define RELAY_INRVEC_WIDTH (3U)
#define RELAY_IDX2RVEC_HOLDOFF_SHIFT(idx) (idx*RELAY_INRVEC_WIDTH)
#define RELAY_IDX2RVEC_HOLDOFF(idx) (1u << RELAY_IDX2RVEC_HOLDOFF_SHIFT(idx))
#define RELAY_IDX2RVEC_CMD_SHIFT(idx) (idx*RELAY_INRVEC_WIDTH+1u)
#define RELAY_IDX2RVEC_CMD(idx) (1u << RELAY_IDX2RVEC_CMD_SHIFT(idx))
#define RELAY_NUM2VEC_CMD(num, vec) ((vec & RELAY_IDX2RVEC_CMD(num)))
#define RELAY_IDX2RVEC_OUTPUT_SHIFT(idx) (idx*RELAY_INRVEC_WIDTH+2U)
#define RELAY_IDX2RVEC_OUTPUT(idx) (1u << RELAY_IDX2RVEC_OUTPUT_SHIFT(idx))
#define RELAY_NUM2VEC_OUTPUT(num, vec) ((vec & RELAY_IDX2RVEC_OUTPUT(num)))
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
	eLED_B
}LEDchanType;

typedef enum
{
	eOUT_OFF = 0,
	eOUT_ON
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

typedef enum
{
	eRelaySMOff = 0,
	eRelaySMOnFull,
	eRelaySMOnReduced,
	eRelaySMOnFullReseat
} RelaySMStateType;

typedef struct
{
	uint32_t cas; /*< count at start */
	RelaySMStateType state;
} RelaySMOutputType;

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

/*****************************************************************************
 * Functions
 *****************************************************************************/
void PetitPortTxBegin(pu8_t data)
{
	PetitPortDirTx();
	USART1->TXDATA = data;
	USART_IntEnable(RETARGET_UART, USART_IF_TXC);
}

void PetitPortTimerStart(void)
{
	SysTick->LOAD = (uint32_t) (12500UL - 1UL); /* set reload register */
	SysTick->VAL = 0UL; /* Load the SysTick Counter Value */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
			SysTick_CTRL_TICKINT_Msk |
			SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
}

void PetitPortTimerStop(void)
{
	SysTick->CTRL = 0;
}

void PetitPortDirTx(void)
{
	//GPIO->P[txen_PORT].DOUTSET = 1u << txen_PIN;
}

void PetitPortDirRx(void)
{
	//GPIO->P[txen_PORT].DOUTCLR = 1u << txen_PIN;
}

void LED_Write(uint8_t led, LEDchanType chan, uint8_t state)
{
	EFM_ASSERT(led < 2u);
	EFM_ASSERT(chan < 3u);
	EFM_ASSERT(state < 2u);

	/* construct a magic number */
	LEDMagicType magic;
	magic.b.chan = chan;
	magic.b.led = led;
	magic.b.output = state;

	/* use the magic */
	switch (magic.m)
	{
	case eLED0_R_OFF:
		LED0R_OFF();
		break;
	case eLED0_R_ON:
		LED0R_ON();
		break;
	case eLED1_R_OFF:
		LED1R_OFF();
		break;
	case eLED1_R_ON:
		LED1R_ON();
		break;
	case eLED0_G_OFF:
		LED0G_OFF();
		break;
	case eLED0_G_ON:
		LED0G_ON();
		break;
	case eLED1_G_OFF:
		LED1G_OFF();
		break;
	case eLED1_G_ON:
		LED1G_ON();
		break;
	case eLED0_B_OFF:
		LED0B_OFF();
		break;
	case eLED0_B_ON:
		LED0B_ON();
		break;
	case eLED1_B_OFF:
		LED1B_OFF();
		break;
	case eLED1_B_ON:
		LED1B_ON();
		break;
	default:
		EFM_ASSERT(0);
		break;
	}
}

uint8_t DebounceSM(
	uint8_t input,
	uint8_t debounceThreshold,
	DSMOutputType* out
)
{
	EFM_ASSERT(debounceThreshold > 0u);
    if ((input != 0) != out->s.output)
    {
      out->s.count++;
    }
    else
    {
      out->s.count = 0;
    }

    if (out->s.count >= debounceThreshold)
    {
      out->s.output = !out->s.output;
      out->s.count = 0;
    }

    return out->s.output;
}

void RelaySM(
	uint8_t relayNum,
	uint8_t relayVec,
	uint32_t msCounter,
	RelaySMOutputType* out
)
{
	EFM_ASSERT(relayNum < 2u);
	/* Transitions */
	if (RELAY_NUM2VEC_OUTPUT(relayNum, relayVec))
	{
		switch (out->state)
		{
		case eRelaySMOff:
			out->state = eRelaySMOnFull;
			out->cas = msCounter;
			break;
		case eRelaySMOnFull:
			if (msCounter - out->cas >= C_RELAY_ONF_TIME)
			{
				out->state = eRelaySMOnReduced;
				out->cas = msCounter;
			}
			break;
		case eRelaySMOnReduced:
			if (msCounter - out->cas >= C_RELAY_ONR_TIME)
			{
				out->state = eRelaySMOnFullReseat;
				out->cas = msCounter;
			}
			break;
		case eRelaySMOnFullReseat:
			if (msCounter - out->cas >= C_RELAY_ONFR_TIME)
			{
				out->state = eRelaySMOnReduced;
				out->cas = msCounter;
			}
			break;
		default:
			out->state = eRelaySMOff;
			break;
		}
	}
	else
	{
		out->state = eRelaySMOff;
	}

	/* Output */
	/* Since there are only two relays and the compare channel is inverted,
	 * we can calculate the channel number by XORing the 0th bit.
	 */
	switch (out->state)
	{
	case eRelaySMOff:
		TIMER2->CC[relayNum ^ 1u].CCVB = 0;
		break;
	case eRelaySMOnFullReseat:
	case eRelaySMOnFull:
		TIMER2->CC[relayNum ^ 1u].CCVB = C_RELAY_CCV_FULL;
		break;
	case eRelaySMOnReduced:
		TIMER2->CC[relayNum ^ 1u].CCVB = C_RELAY_CCV_REDUCED;
		break;
	}
}

/*
 * "Start Up / Shut Down" State Machine (SDSUSM)
 */

#define C_SDSU_OFF_PERIOD (30 * 1000)
#define C_SDSU_ON_PERIOD (30 * 1000)

typedef enum {
	eSDSU_Off = 0,
	eSDSU_NotOn,
	eSDSU_On,
	eSDSU_NotOff
} SDSUSMState_Type;

typedef struct{
	uint32_t lastCounter;
	SDSUSMState_Type State;
}SDSUSMOutput_Type;

bool SDSUSM(uint8_t num, uint8_t vec, uint32_t Counter, SDSUSMOutput_Type* Out)
{
	bool retval = false;
	// transitions
	switch (Out->State)
	{
	case eSDSU_Off:
		if (Counter - Out->lastCounter < C_SDSU_OFF_PERIOD)
		{
			if (RELAY_NUM2VEC_CMD(num, vec))
			{
				Out->State = eSDSU_NotOn;
			}
		}
		else
		{
			Out->lastCounter = Counter - C_SDSU_OFF_PERIOD; // arithmetic overflow prevention
			if (RELAY_NUM2VEC_CMD(num, vec))
			{
				Out->State = eSDSU_On;
				Out->lastCounter = Counter;
			}
		}
		break;
	case eSDSU_NotOn:
		if (Counter - Out->lastCounter >= C_SDSU_OFF_PERIOD)
		{
			Out->lastCounter = Counter - C_SDSU_OFF_PERIOD;
			if (RELAY_NUM2VEC_CMD(num, vec))
			{
				Out->State = eSDSU_On;
				Out->lastCounter = Counter;
			}
		}

		if (RELAY_NUM2VEC_CMD(num, vec) == 0)
		{
			Out->State = eSDSU_Off;
		}
		break;
	case eSDSU_On:
		if (Counter - Out->lastCounter < C_SDSU_ON_PERIOD)
		{
			if (RELAY_NUM2VEC_CMD(num, vec) == 0)
			{
				Out->State = eSDSU_NotOff;
			}
		}
		else
		{
			Out->lastCounter = Counter - C_SDSU_ON_PERIOD;
			if (RELAY_NUM2VEC_CMD(num, vec) == 0)
			{
				Out->State = eSDSU_Off;
				Out->lastCounter = Counter;
			}
		}
		break;
	case eSDSU_NotOff:
		if (Counter - Out->lastCounter >= C_SDSU_ON_PERIOD)
		{
			Out->lastCounter = Counter - C_SDSU_ON_PERIOD;
			if (RELAY_NUM2VEC_CMD(num, vec) == 0)
			{
				Out->State = eSDSU_Off;
				Out->lastCounter = Counter;
			}
		}

		if (RELAY_NUM2VEC_CMD(num, vec) & 0b1)
		{
			Out->State = eSDSU_On;
		}
		break;
	default:
		EFM_ASSERT(0);
		Out->State = eSDSU_Off;
		Out->lastCounter = Counter;
		break;
	}

	// output
	switch (Out->State)
	{
	case eSDSU_Off:
		retval = false;
		break;
	case eSDSU_NotOn:
		retval = false;
		break;
	case eSDSU_On:
		retval = true;
		break;
	case eSDSU_NotOff:
		retval = true;
		break;
	}

	return retval;
}

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

	/* GPIO D is handled by the redirect IO at the moment */
	GPIO_PinModeSet(RETARGET_TXPORT, RETARGET_TXPIN, gpioModePushPull, 1);
	GPIO_PinModeSet(RETARGET_RXPORT, RETARGET_RXPIN, gpioModeInputPull, 1);
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
	{
		usartDisable, /* Disable RX/TX when initialization is complete. */
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
	NVIC_EnableIRQ(USART1_TX_IRQn);

	/* Finally enable it */
	USART_Enable(usart, usartEnable);
}

#define HB_DIV (9u) // approximately once a second, actually less
                    // must be at least two, but it probably won't look good
                    // at two...
#define HB_DUTYCYCLE (16u) // select zero here to bypass the dutycycle gen
#define HB_SKIP (5u) // number of heartbeat cycles to skip between blinks
#define FAST_BLINK (9u) // one second
#define SLOW_BLINK (FAST_BLINK + 2u)

uint8_t hbdCount = 0; // heartbeat dutycycle counter
uint8_t hbsCount = 0; // heartbeat skip counter

/*
 * Adapted from the IoT blinker.
 */

void blinker(uint16_t count)
{
	// one: determine heart beat
	// t1Count & (1 << HB_DIV)
	bool hb_slow = (count & (1u << HB_DIV)) != 0u;
	bool hb_fast = (count & (1u << (HB_DIV - 2u))) != 0u;
	// heart beat slow _ rising edge
	bool hbs_re = (count & ((1u << (HB_DIV + 1u)) - 1u)) == (1u << HB_DIV);
	// heart beat fast _ rising edge
	bool hbf_re = (count & ((1u << (HB_DIV - 1u)) - 1u)) == (1u << (HB_DIV - 2u));
	bool hb_internal = hb_fast && hb_slow;
	if (hbs_re == true && hbsCount < HB_SKIP)
	{
		hbsCount++;
	}
	else if (hbs_re == true)
	{
		hbsCount = 0;
	}
	// compute heartbeat skip
	hb_internal = hb_internal && (hbsCount == 0);

	// two: LED duty cycle limiter
	if (hb_internal == true && hbf_re == true)
	{
		hbdCount = 1;
		LED_Write(0, eLED_G, eOUT_ON);
		LED_Write(1, eLED_G, eOUT_ON);
	}

	// if the selected dutycycle is 0, base this part
	// off of the heatbeat generated before
	if (hbdCount < HB_DUTYCYCLE)
	{
		hbdCount++;
	}
	else if (HB_DUTYCYCLE > 0u || hb_internal == false)
	{
		LED_Write(0, eLED_G, eOUT_OFF);
		LED_Write(1, eLED_G, eOUT_OFF);
	}
}

/*****************************************************************************
 * Main Function and Variables
 *****************************************************************************/
volatile uint32_t msCounter = 0;
uint32_t lastCounter = 0;
uint8_t hbscale = 0;
uint8_t hbcounter = 0;

DSMOutputType WS_Debounce[2];

uint8_t relayVec = 0;

RelaySMOutputType WS_Relay[2];

SDSUSMOutput_Type WS_SDSU;

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

			ProcessPetitModbus();
			KIRICAPSENSE_process();

			for (uint8_t touchRdy = KIRICAPSENSE_pressReady(); touchRdy != 255;
					touchRdy = KIRICAPSENSE_pressReady())
			{
				if (DebounceSM(KIRICAPSENSE_getPressed(touchRdy), 1u,
						&WS_Debounce[touchRdy]))
				{
					if ((relayVec & RELAY_IDX2RVEC_HOLDOFF(touchRdy)) == 0u)
					{
						relayVec |= RELAY_IDX2RVEC_HOLDOFF(touchRdy); /*< Enter holdoff state on rising edge */
						relayVec ^= RELAY_IDX2RVEC_CMD(touchRdy); /*< Toggle output */
					}
				}
				else
				{
					relayVec &= ~(RELAY_IDX2RVEC_HOLDOFF(touchRdy)); /* Exit holdoff state on falling edge */
				}
			}

			blinker(lastCounter);

			LED_Write(0u, eLED_B, (relayVec & RELAY_IDX2RVEC_CMD(0u)) != 0);
			LED_Write(1u, eLED_B, (relayVec & RELAY_IDX2RVEC_CMD(1u)) != 0);

			if (SDSUSM(0U, relayVec, msCounter, &WS_SDSU))
			{
				relayVec |= RELAY_IDX2RVEC_OUTPUT(0U);
			}
			else
			{
				relayVec &= ~RELAY_IDX2RVEC_OUTPUT(0U);
			}
			relayVec = relayVec & RELAY_IDX2RVEC_CMD(1U) ? relayVec | RELAY_IDX2RVEC_OUTPUT(1U) : relayVec & ~(RELAY_IDX2RVEC_OUTPUT(1U));

			/* There are two relays and two LEDs */
			for (uint8_t i = 0; i < 2; i++)
			{
				if (relayVec & RELAY_IDX2RVEC_OUTPUT(i))
				{
					LED_Write(i, eLED_R, eOUT_ON);
				}
				else
				{
					LED_Write(i, eLED_R, eOUT_OFF);
				}

				RelaySM(i, relayVec, msCounter, &WS_Relay[i]);
			}
		}
		else
		{
			// wfi might actually stop the systick
		}
	}
}
