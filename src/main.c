#include "em_assert.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "retargetserial.h"
#include "hal-config.h"
#include "kiricapsense.h"

/*****************************************************************************
 * Defines here
 *****************************************************************************/
#define C_HEARTBEAT_TOP (124u)
#define C_HB_SHIFT (0)
#define RELAY_IDX2RVEC_HOLDOFF_SHIFT(idx) (idx*2u)
#define RELAY_IDX2RVEC_HOLDOFF(idx) (1u << RELAY_IDX2RVEC_HOLDOFF_SHIFT(idx))
#define RELAY_IDX2RVEC_OUTPUT_SHIFT(idx) (idx*2u+1u)
#define RELAY_IDX2RVEC_OUTPUT(idx) (1u << RELAY_IDX2RVEC_OUTPUT_SHIFT(idx))
#define RELAY_NUM2VEC(num, vec) (vec & 1u << ((num << 1u) + 1u))
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
	uint16_t cas; /*< count at start */
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
	uint16_t msCounter,
	RelaySMOutputType* out
)
{
	EFM_ASSERT(relayNum < 2u);
	/* Transitions */
	if (RELAY_NUM2VEC(relayNum, relayVec))
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

/*****************************************************************************
 * Variables
 *****************************************************************************/
volatile uint16_t msCounter = 0;
uint16_t lastCounter = 0;
uint8_t hbscale = 0;
uint8_t hbcounter = 0;

DSMOutputType KCS_Debounce[2];

uint8_t relayVec = 0;

RelaySMOutputType KCS_Relay[2];

int main(void)
{
	/* Chip errata */
	CHIP_Init();

	RETARGET_SerialInit(); /*< does not like GCC's PIC*/

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

	/* CMU for TIMER0 */
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);

	/* Initialize TIMER0 */
	/* SYSCLK is 14e6 at this point */
	/* Prescaler is div16 */
	/* 875 counts, but minus one from the fundamental counting principal */
	TIMER0->CTRL = TIMER_CTRL_PRESC_DIV16;
	TIMER0->TOP = 874u;
	TIMER0->IEN = TIMER_IEN_OF;
	TIMER0->CNT = 0;

	KIRICAPSENSE_Init();

	/* Enable TIMER0 interrupt */
	NVIC_EnableIRQ(TIMER0_IRQn);

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
			lastCounter++;

			if (++hbscale > C_HEARTBEAT_TOP)
			{
				hbscale = 0;
				hbcounter++;
			}

			if ((hbcounter & (1u << (C_HB_SHIFT + 2u)))
					&& (hbcounter & (1u << C_HB_SHIFT)))
			{
				LED_Write(0, eLED_G, eOUT_ON);
				LED_Write(1, eLED_G, eOUT_ON);
			}
			else
			{
				LED_Write(0, eLED_G, eOUT_OFF);
				LED_Write(1, eLED_G, eOUT_OFF);
			}

			for (uint8_t touchRdy = KIRICAPSENSE_pressReady(); touchRdy != 255;
					touchRdy = KIRICAPSENSE_pressReady())
			{
				if (DebounceSM(KIRICAPSENSE_getPressed(touchRdy), 1u,
						&KCS_Debounce[touchRdy]))
				{
					if ((relayVec & RELAY_IDX2RVEC_HOLDOFF(touchRdy)) == 0u)
					{
						relayVec |= RELAY_IDX2RVEC_HOLDOFF(touchRdy); /*< Enter holdoff state on rising edge */
						relayVec ^= RELAY_IDX2RVEC_OUTPUT(touchRdy); /*< Toggle output */
					}
					LED_Write(touchRdy, eLED_B, eOUT_ON);
				}
				else
				{
					relayVec &= ~(RELAY_IDX2RVEC_HOLDOFF(touchRdy)); /* Exit holdoff state on falling edge */
					LED_Write(touchRdy, eLED_B, eOUT_OFF);
				}
			}

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

				RelaySM(i, relayVec, msCounter, &KCS_Relay[i]);
			}
		}
	}
}
