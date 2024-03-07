/*
 * input.c
 *
 *  Created on: Feb 28, 2024
 *      Author: brandon
 */
#include "input.h"
#include "output.h"
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

bool AOSM(AOSM_Input_t *in, uint32_t msCounter,
		AOSM_CFG_t *cfg, AOSM_Output_t *out)
{
	bool onOff = false;

	if (CAP_RISING_EDGE(in->num, in->capVec))
	{
		out->lastState = out->state;
	}

	switch (out->state)
	{
	case eAOSM_Off:
		if (CAP_RISING_EDGE(in->num, in->capVec))
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
		if (CAP_FALLING_EDGE(in->num, in->capVec) && out->lastState != eAOSM_Off)
		{
			if (REL_NUM2VEC_OUTPUT(in->num, in->relayVec) != 0)
			{
				out->state = eAOSM_mOff;
				out->counter = msCounter;
			}
			else
			{
				out->state = eAOSM_Off;
			}
		}
		if (CAP_NUM2VEC_STATUS(in->num, in->capVec) != 0
				&& msCounter - in->pressCounter >= cfg->long_press)
		{
			out->state = eAOSM_Off;
		}
		break;
	case eAOSM_aOff:
		if (CAP_FALLING_EDGE(in->num, in->capVec))
		{
			out->state = eAOSM_On;
			out->counter = msCounter;
		}
		if (CAP_NUM2VEC_STATUS(in->num, in->capVec) != 0
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
		if (CAP_FALLING_EDGE(in->num, in->capVec))
		{
			out->state = eAOSM_On;
			out->counter = msCounter;
		}
		if (msCounter - out->counter >= cfg->moff_time)
		{
			out->state = eAOSM_Off;
		}
		if (CAP_NUM2VEC_STATUS(in->num, in->capVec) != 0
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

// note: the RX flag is reset outside of this function.
// communications SM is 0 when receiving
void CommSM(uint8_t num, uint8_t vec, uint32_t msCounter, uint32_t thresh, uint32_t *commCounter)
{
	switch (COMM_NUM2VEC_CSM(num, vec))
	{
			// loss of comm
		case 1:
			if (COMM_NUM2VEC_RX(num, vec) != 0)
			{
					*commCounter = msCounter;
					// CLEAR bit
					vec &= ~COMM_IDX2VEC_CSM(num);
			}
			break;
			// receiving 
		case 0:
			if (COMM_NUM2VEC_RX(num, vec) != 0)
			{
					*commCounter = msCounter;
			}
			if (msCounter - *commCounter >= thresh)
			{
					// SET bit
					vec |= COMM_IDX2VEC_CSM(num);
			}
			break;
	}

}

// the advanced debounce state machine depends on the communication state machine
// note: the CSM flag is the status bit of the communication state machine
void ADSM(uint8_t num, uint8_t vec, uint32_t msCounter, ADSM_Cfg_t *cfg, ADSM_Output_t *out)
{
	switch(out->state)
	{
	case eADSM_Off:
		// On
		if (COMM_NUM2VEC_STATE(num, vec) != 0 && COMM_NUM2VEC_CSM(num, vec) == 0)
		{
			if (cfg->onThresh != 0)
			{
				out->counter = msCounter;
				out->state = eADSM_nOn;
			}
			else
			{
				out->state = eADSM_On;
			}
		}
		break;
	case eADSM_nOn:
		// Off or communication breakdown
		if (COMM_NUM2VEC_STATE(num, vec) == 0 || COMM_NUM2VEC_CSM(num, vec) != 0)
		{
			out->state = eADSM_Off;
		}
		// if "on" is received for long enough
		else if (msCounter - out->counter >= cfg->onThresh)
		{
			out->state = eADSM_On;
		}
		break;
	case eADSM_On:
		// Communication breakdown
		if (COMM_NUM2VEC_CSM(num, vec) != 0)
		{
			out->state = eADSM_Off;
		}
		// Off
		else if (COMM_NUM2VEC_STATE(num, vec) == 0)
		{
			if (cfg->offThresh != 0)
			{
				out->counter = msCounter;
				out->state = eADSM_nOff;
			}
			else
			{
				out->state = eADSM_Off;
			}
		}
		break;
	case eADSM_nOff:
		// On
		if (COMM_NUM2VEC_STATE(num, vec) != 0)
		{
			out->state = eADSM_On;
		}
		else if (msCounter - out->counter >= cfg->offThresh)
		{
			out->state = eADSM_Off;
		}

		if (COMM_NUM2VEC_CSM(num, vec) != 0)
		{
			out->state = eADSM_Off;
		}
		break;
	default:
		out->state = eADSM_Off;
		break;
	}

	switch(out->state)
	{
	case eADSM_Off:
		// clear bit
		vec &= ~COMM_IDX2VEC_CMD(num);
		break;
	case eADSM_nOn:
		// clear bit
		vec &= ~COMM_IDX2VEC_CMD(num);
		break;
	case eADSM_On:
		// set bit
		vec |= COMM_IDX2VEC_CMD(num);
		break;
	case eADSM_nOff:
		// set bit
		vec |= COMM_IDX2VEC_CMD(num);
		break;
	}
}
