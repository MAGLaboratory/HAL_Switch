/*
 * input.c
 *
 *  Created on: Feb 28, 2024
 *      Author: brandon
 */
#include "input.h"
#include <stdint.h>
#include <stdbool.h>

uint8_t DebounceSM(uint8_t input, uint8_t debounceThreshold, DSMOutputType* out)
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

bool AOSM(Button_t *in, uint32_t msCounter, AOSM_CFG_t *cfg, AOSM_Output_t *out)
{
	bool onOff = false;

	if (CAP_RISING_EDGE(in->num, in->vec))
	{
		out->lastState = out->state;
	}

	switch (out->state)
	{
	case eAOSM_Off:
		if (CAP_RISING_EDGE(in->num, in->vec))
		{
			out->state = eAOSM_On;
			out->counter = msCounter;
		}
		break;
	case eAOSM_On:
		if (cfg->on_time != 0U && msCounter - out->counter >= cfg->on_time)
		{
			out->state = eAOSM_aOff;
			out->counter = msCounter;
		}
		if (CAP_FALLING_EDGE(in->num, in->vec) && out->lastState != eAOSM_Off)
		{
			out->state = eAOSM_mOff;
			out->counter = msCounter;
		}
		if (CAP_NUM2VEC_STATUS(in->num, in->vec) != 0
				&& msCounter - in->pressCounter >= cfg->long_press)
		{
			out->state = eAOSM_Off;
		}
		break;
	case eAOSM_aOff:
		if (CAP_FALLING_EDGE(in->num, in->vec))
		{
			out->state = eAOSM_On;
			out->counter = msCounter;
		}
		if (CAP_NUM2VEC_STATUS(in->num, in->vec) != 0
				&& msCounter - in->pressCounter >= cfg->long_press)
		{
			out->state = eAOSM_Off;
		}
		if (msCounter - out->counter >= cfg->aoff_time)
		{
			out->state = eAOSM_Off;
		}
		break;
	case eAOSM_mOff:
		if (CAP_FALLING_EDGE(in->num, in->vec))
		{
			out->state = eAOSM_On;
			out->counter = msCounter;
		}
		if (msCounter - out->counter >= cfg->moff_time)
		{
			out->state = eAOSM_Off;
		}
		if (CAP_NUM2VEC_STATUS(in->num, in->vec) != 0
				&& msCounter - in->pressCounter >= cfg->long_press)
		{
			out->state = eAOSM_Off;
		}
		break;
	default:
		out->state = eAOSM_Off;
		break;
	}

	switch (out->state)
	{
	case eAOSM_Off:
		onOff = false;
		break;
	case eAOSM_On:
		onOff = true;
		break;
	case eAOSM_aOff:
		onOff = true;
		break;
	case eAOSM_mOff:
		onOff = true;
		break;
	}

	return onOff;
}
