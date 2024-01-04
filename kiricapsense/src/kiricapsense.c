/*
 * Kirisaki Capsense
 *
 * This code is originally derived from the silabs capsense library but adapted so that the timing loop is external
 */

#include "em_device.h"
#include "em_acmp.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "kiricapsense.h"


/** The current channel we are sensing. */
static volatile uint8_t currentChannel;

static volatile uint32_t channelValues[KCS_NUM_CHANNELS] = { 0 };

static volatile uint32_t channelMaxValues[KCS_NUM_CHANNELS] = { 0 };


void KIRICAPSENSE_Init(void)
{
  /* Use the default STK capacitive sensing setup */
  ACMP_CapsenseInit_TypeDef capsenseInit = {
    false,            /* fullBias */
    false,            /* halfBias */
    0x7,              /* biasProg */
    acmpWarmTime512,  /* 512 cycle warmup to be safe */
    acmpHysteresisLevel1,
    acmpResistor0,
    false,            /* low power reference */
    47,             /* VDD level */
    true              /* Enable after init. */
  };

  /* Enable TIMER0, TIMER1, ACMP_CAPSENSE and PRS clock */
  CMU_ClockEnable(cmuClock_TIMER1, true);
#if defined(KCS_ACMP_CAPSENSE_CMUCLOCK)
  CMU_ClockEnable(KCS_ACMP_CAPSENSE_CMUCLOCK, true);
#else
  CMU->HFPERCLKEN0 |= KCS_ACMP_CAPSENSE_CLKEN;
#endif
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Initialize TIMER1 - Prescaler value does not matter, clock source CC1, top value 0xFFFF */
  TIMER1->CTRL = TIMER_CTRL_PRESC_DIV1024 | TIMER_CTRL_CLKSEL_CC1;
  TIMER1->TOP  = 0xFFFF;

  /*Set up TIMER1 CC1 to trigger on PRS channel 0 */
  TIMER1->CC[1].CTRL = TIMER_CC_CTRL_MODE_INPUTCAPTURE /* Input capture      */
                       | TIMER_CC_CTRL_PRSSEL_PRSCH0   /* PRS channel 0      */
                       | TIMER_CC_CTRL_INSEL_PRS       /* PRS input selected */
                       | TIMER_CC_CTRL_ICEVCTRL_RISING /* PRS on rising edge */
                       | TIMER_CC_CTRL_ICEDGE_BOTH;    /* PRS on rising edge */

  /*Set up PRS channel 0 to trigger on ACMP1 output*/
  PRS->CH[0].CTRL = PRS_CH_CTRL_EDSEL_POSEDGE      /* Posedge triggers action */
                    | PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE      /* PRS source */
                    | PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE;     /* PRS source */

  /* Set up ACMP1 in capsense mode */
  ACMP_CapsenseInit(KCS_ACMP_CAPSENSE, &capsenseInit);
  ACMP_Enable(KCS_ACMP_CAPSENSE);
  ACMP_CapsenseChannelSet(KCS_ACMP_CAPSENSE, currentChannel);
}


/* TIMER0 IRQHandler moved somewhere else. */

void KIRICAPSENSE_IT(void)
{
	uint16_t count;

	/* Stop TIMER1 */
	TIMER1->CMD = TIMER_CMD_STOP;

	/* Read out value of TIMER1 */
	count = TIMER1->CNT;
	TIMER1->CNT = 0;

	TIMER1->CMD = TIMER_CMD_START;

	/* Store value in channelValues */
	channelValues[currentChannel] = count;

	/* Update channelMaxValues */
	if (count > channelMaxValues[currentChannel]) {
		channelMaxValues[currentChannel] = count;
	}

	/* Find the next capsense channel */
	if (++currentChannel >= KCS_NUM_CHANNELS)
	{
		currentChannel = 0;
	}

	/* Set up this channel in the ACMP. */
	ACMP_CapsenseChannelSet(KCS_ACMP_CAPSENSE, currentChannel);
}

bool KIRICAPSENSE_getPressed(uint8_t channel)
{
  uint32_t treshold;
  /* Threshold is set to 12.5% below the maximum value */
  /* This calculation is performed in two steps because channelMaxValues is
   * volatile. */
  treshold  = channelMaxValues[channel];
  treshold -= channelMaxValues[channel] >> 3;

  if (channelValues[channel] < treshold) {
    return true;
  }
  return false;
}
