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
/** Flag for measurement completion. */
static volatile bool measurementComplete;

static const bool channelsInUse[ACMP_CHANNELS] = CAPSENSE_CH_IN_USE;

static void CAPSENSE_Measure(ACMP_Channel_TypeDef channel)
{
  /* Set up this channel in the ACMP. */
  ACMP_CapsenseChannelSet(KCS_ACMP_CAPSENSE, channel);

  /* Reset timers */
  TIMER0->CNT = 0;
  TIMER1->CNT = 0;

  measurementComplete = false;

  /* Start timers */
  TIMER0->CMD = TIMER_CMD_START;
  TIMER1->CMD = TIMER_CMD_START;

  /* Wait for measurement to complete */
  while ( measurementComplete == false ) {
    EMU_EnterEM1();
  }
}



void CAPSENSE_Sense(void)
{
  /* Use the default STK capacative sensing setup and enable it */
  ACMP_Enable(KCS_ACMP_CAPSENSE);

#if defined(CAPSENSE_CHANNELS)
  /* Iterate through only the channels in the channelList */
  for (currentChannel = 0; currentChannel < ACMP_CHANNELS; currentChannel++) {
    CAPSENSE_Measure(channelList[currentChannel]);
  }
#else
  /* Iterate through all channels and check which channel is in use */
  for (currentChannel = 0; currentChannel < ACMP_CHANNELS; currentChannel++) {
    /* If this channel is not in use, skip to the next one */
    if (!channelsInUse[currentChannel]) {
      continue;
    }

    CAPSENSE_Measure((ACMP_Channel_TypeDef) currentChannel);
  }
#endif

  /* Disable ACMP while not sensing to reduce power consumption */
  ACMP_Disable(KCS_ACMP_CAPSENSE);
}


void CAPSENSE_Init(void)
{
  /* Use the default STK capacitive sensing setup */
  ACMP_CapsenseInit_TypeDef capsenseInit = ACMP_CAPSENSE_INIT_DEFAULT;

  /* Enable TIMER0, TIMER1, ACMP_CAPSENSE and PRS clock */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
#if defined(KCS_ACMP_CAPSENSE_CMUCLOCK)
  CMU_ClockEnable(KCS_ACMP_CAPSENSE_CMUCLOCK, true);
#else
  CMU->HFPERCLKEN0 |= KCS_ACMP_CAPSENSE_CLKEN;
#endif
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Initialize TIMER0 - Prescaler 2^9, top value 10, interrupt on overflow */
  TIMER0->CTRL = TIMER_CTRL_PRESC_DIV512;
  TIMER0->TOP  = 10;
  TIMER0->IEN  = TIMER_IEN_OF;
  TIMER0->CNT  = 0;

  /* Initialize TIMER1 - Prescaler 2^10, clock source CC1, top value 0xFFFF */
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

  /* Enable TIMER0 interrupt */
  NVIC_EnableIRQ(TIMER0_IRQn);
}
