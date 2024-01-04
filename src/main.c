#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "retargetserial.h"
#include "hal-config.h"
#include "kiricapsense.h"

#define C_HEARTBEAT_TOP (124)

volatile uint8_t timer0Flag = 0;
uint8_t hbscale = 0;
uint8_t hbcounter = 0;

int main(void)
{
  /* Chip errata */
  CHIP_Init();

  RETARGET_SerialInit(); /*< does not like GCC's PIC*/

  /* GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* LED 1 is Push-ON (active HIGH) */
  GPIO_PinModeSet(led1r_PORT, led1r_PIN, gpioModePushPullDrive, 1);
  GPIO_DriveModeSet(led1r_PORT, gpioDriveModeHigh); /*< GPIO A */
  GPIO_PinModeSet(led1g_PORT, led1g_PIN, gpioModePushPullDrive, 0);
  GPIO_DriveModeSet(led1g_PORT, gpioDriveModeHigh); /*< GPIO E */
  GPIO_PinModeSet(led1b_PORT, led1b_PIN, gpioModePushPullDrive, 0);
  GPIO_DriveModeSet(led1b_PORT, gpioDriveModeHigh); /*< GPIO B */
  /* LED 0 is Pull-ON (active LOW) */
  GPIO_PinModeSet(led0b_PORT, led0b_PIN, gpioModePushPullDrive, 1);
  GPIO_PinModeSet(txen_PORT, txen_PIN, gpioModePushPullDrive, 0);
  GPIO_DriveModeSet(txen_PORT, gpioDriveModeHigh); /* GPIO C */
  /* PROG(ramming) detection pin */
  /* Handle the prog jumper code later.  Jumper open while device is being programmed. */
  GPIO_PinModeSet(prog_PORT, prog_PIN, gpioModeInputPullFilter, 1);
  GPIO_DriveModeSet(prog_PORT, gpioDriveModeHigh); /* GPIO F */

  /* GPIO D is handled by the redirect IO at the moment */

  /* CMU for TIMER0 */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);

  /* Initialize TIMER0 */
  /* SYSCLK is 14e6 at this point */
  /* Prescaler is div16 */
  /* 875 counts, but minus one from the fundamental counting principal */
  TIMER0->CTRL = TIMER_CTRL_PRESC_DIV16;
  TIMER0->TOP  = 874;
  TIMER0->IEN  = TIMER_IEN_OF;
  TIMER0->CNT  = 0;

  KIRICAPSENSE_Init();

  /* Enable TIMER0 interrupt */
  NVIC_EnableIRQ(TIMER0_IRQn);

  /* Start Timer0 */
  TIMER0->CMD = TIMER_CMD_START;

  /* Start Timer1 */
  /* TODO: should be in kirisaki capsense */
  TIMER1->CMD = TIMER_CMD_START;

  /* Infinite loop */
  while (1) {
	  if (timer0Flag)
	  {
		  timer0Flag = 0;

		  if (++hbscale > C_HEARTBEAT_TOP)
		  {
			  hbscale = 0;
			  hbcounter++;
		  }

		  if ((hbcounter & (1 << 2)) && (hbcounter & (1 << 0)))
		  {
			  GPIO->P[led1r_PORT].DOUTSET = 1 << led1r_PIN;
		  }
		  else
		  {
			  GPIO->P[led1r_PORT].DOUTCLR = 1 << led1r_PIN;
		  }

		  if (KIRICAPSENSE_getPressed(1))
		  {
			  GPIO->P[led1b_PORT].DOUTSET = 1 << led1b_PIN;
		  }
		  else
		  {
			  GPIO->P[led1b_PORT].DOUTCLR = 1 << led1b_PIN;
		  }

		  /* LED 0 is inverted */
		  if (KIRICAPSENSE_getPressed(0))
		  {
			  GPIO->P[led0b_PORT].DOUTCLR = 1 << led0b_PIN;
		  }
		  else
		  {
			  GPIO->P[led0b_PORT].DOUTSET = 1 << led0b_PIN;
		  }
	  }
  }
}
