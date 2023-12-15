#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "retargetserial.h"
#include "hal-config.h"

#define C_HEARTBEAT_TOP (5)

uint8_t timer1Flag = 0;

void TIMER1_IRQHandler(void)
{
	/* Clear interrupt flag */
	TIMER1->IFC = TIMER_IFC_OF;

	timer1Flag = 1;
}

int main(void)
{
  /* Chip errata */
  CHIP_Init();
//
//  RETARGET_SerialInit();
//
//  CMU_ClockEnable(cmuClock_GPIO, true);
//  GPIO_PinModeSet(led1r_PORT, led1r_PIN, gpioModePushPullDrive, 1);
//  GPIO_DriveModeSet(led1r_PORT, gpioDriveModeHigh);
//
//  CMU_ClockEnable(cmuClock_HFPER, true);
//  CMU_ClockEnable(cmuClock_TIMER1, true);
//  /* Initialize TIMER1 - Prescaler 2^3, top value 174, interrupt on overflow */
//  /* The time calculations make this close to 10kHz.  14MHz / 8 / 175 */
//  TIMER1->CTRL = TIMER_CTRL_PRESC_DIV8;
//  TIMER1->TOP  = 174;
//  TIMER1->IEN  = TIMER_IEN_OF;
//  TIMER1->CNT  = 0;
//
//  NVIC_EnableIRQ(TIMER1_IRQn);
//
//  TIMER1->CMD = TIMER_CMD_START;
//
//  /* heartbeat scaler */
//  uint8_t hbscale = 0;
//  uint8_t hbcounter = 0;

  /* Infinite loop */
  while (1) {
//	  if (timer1Flag)
//	  {
//		  timer1Flag = 0;
//
//		  if (++hbscale == C_HEARTBEAT_TOP)
//		  {
//			  hbscale = 0;
//			  hbcounter++;
//		  }
//
//		  if ((hbcounter & (1 << 5)) && (hbcounter & (1 << 3)))
//		  {
//			  GPIO->P[led1r_PORT].DOUTCLR = 1 << led1r_PIN;
//		  }
//		  else
//		  {
//			  GPIO->P[led1r_PORT].DOUTSET = 1 << led1r_PIN;
//		  }
//	  }
  }
}
