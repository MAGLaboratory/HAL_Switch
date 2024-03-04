#include "input.h"
#include "output.h"
#include "em_device.h"

void LED_Write(uint8_t led, LEDchanType chan, uint8_t state)
{
	EFM_ASSERT(led < 2u);
	EFM_ASSERT(chan < eLED_NUM_CHANNELS);
	EFM_ASSERT(state < eOUT_NUM_STATES);

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

LED_Color_t blink_sel(uint32_t msCounter, const LED_Blink_t *pri,
		const LED_Blink_t *sec, BlinkSel_Output_t *out)
{
	if (pri != out->lastPri || sec != out->lastSec)
	{
		out->state = 0U;
		out->blinkCounter = msCounter;
		out->duration = pri->duration;
		out->lastPri = pri;
		out->lastSec = sec;
	}

	// do NOT give this function two colors with only zero duration
	while (msCounter - out->blinkCounter >= out->duration)
	{
		out->state ^= 1U;
		out->blinkCounter = msCounter;
		if (out->state == 0U)
		{
			out->duration = pri->duration;
		}
		else
		{
			out->duration = sec->duration;
		}
	}

	if (out->state == 0U)
	{
		return pri->color;
	}
	else
	{
		return sec->color;
	}
}

void led_pwm_out(uint8_t num, uint32_t msCounter, LED_Color_t color)
{
	uint8_t counter = msCounter & ((1U << 3U) - 1U);
	if (counter >= color.r)
	{
		LED_Write(num, eLED_R, eOUT_OFF);
	}
	else
	{
		LED_Write(num, eLED_R, eOUT_ON);
	}

	if (counter >= color.g)
	{
		LED_Write(num, eLED_G, eOUT_OFF);
	}
	else
	{
		LED_Write(num, eLED_G, eOUT_ON);
	}

	if (counter >= color.b)
	{
		LED_Write(num, eLED_B, eOUT_OFF);
	}
	else
	{
		LED_Write(num, eLED_B, eOUT_ON);
	}
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
	if (REL_NUM2VEC_OUTPUT(relayNum, relayVec) != 0)
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

// the SDSUSM is configured to enforce a minimum "offPeriod" and "onPeriod."
// if the command bit in the input vector violates the minimum period, the
// state machine simply keeps it off or on
// note: the output struct tick counter is not initialized by this state
// machine and should be initialized to zero.
// if the struct is a global variable, it should be zeroed by default
bool SDSUSM(uint8_t num, uint8_t vec, uint32_t Counter, SDSUSMCfg_t *cfg, SDSUSMOutput_t *Out)
{
	bool retval = false;
	// transitions
	switch (Out->State)
	{
	case eSDSU_Off:
		if (Counter - Out->lastCounter < cfg->offPeriod)
		{
			if (CAP_NUM2VEC_CMD(num, vec) != 0)
			{
				Out->State = eSDSU_NotOn;
			}
		}
		else
		{
			Out->lastCounter = Counter - cfg->offPeriod; // arithmetic overflow prevention
			if (CAP_NUM2VEC_CMD(num, vec) != 0)
			{
				Out->State = eSDSU_On;
				Out->lastCounter = Counter;
			}
		}
		break;
	case eSDSU_NotOn:
		if (Counter - Out->lastCounter >= cfg->offPeriod)
		{
			Out->lastCounter = Counter - cfg->offPeriod;
			if (CAP_NUM2VEC_CMD(num, vec) != 0)
			{
				Out->State = eSDSU_On;
				Out->lastCounter = Counter;
			}
		}

		if (CAP_NUM2VEC_CMD(num, vec) == 0)
		{
			Out->State = eSDSU_Off;
		}
		break;
	case eSDSU_On:
		if (Counter - Out->lastCounter < cfg->onPeriod)
		{
			if (CAP_NUM2VEC_CMD(num, vec) == 0)
			{
				Out->State = eSDSU_NotOff;
			}
		}
		else
		{
			Out->lastCounter = Counter - cfg->onPeriod;
			if (CAP_NUM2VEC_CMD(num, vec) == 0)
			{
				Out->State = eSDSU_Off;
				Out->lastCounter = Counter;
			}
		}
		break;
	case eSDSU_NotOff:
		if (Counter - Out->lastCounter >= cfg->onPeriod)
		{
			Out->lastCounter = Counter - cfg->onPeriod;
			if (CAP_NUM2VEC_CMD(num, vec) == 0)
			{
				Out->State = eSDSU_Off;
				Out->lastCounter = Counter;
			}
		}

		if (CAP_NUM2VEC_CMD(num, vec) != 0)
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

