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

// Debounce State Machine
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

DSMOutputType KCS_CH0_Debounce;
DSMOutputType KCS_CH1_Debounce;

uint8_t relayVec = 0;

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
  GPIO_DriveModeSet(relay1_PORT, gpioDriveModeHigh); /* GPIO F */

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
			  GPIO->P[led1g_PORT].DOUTSET = 1 << led1g_PIN;
			  GPIO->P[led0g_PORT].DOUTCLR = 1 << led0g_PIN;
		  }
		  else
		  {
			  GPIO->P[led1g_PORT].DOUTCLR = 1 << led1g_PIN;
			  GPIO->P[led0g_PORT].DOUTSET = 1 << led0g_PIN;
		  }

		  if (DebounceSM(KIRICAPSENSE_getPressed(1), 127, &KCS_CH1_Debounce))
		  {
			  if ((relayVec & 1 << 2) == 0)
			  {
				  relayVec |= 1 << 2;
				  relayVec ^= 1 << 3;
			  }
			  GPIO->P[led1b_PORT].DOUTSET = 1 << led1b_PIN;
		  }
		  else
		  {
			  relayVec &= ~(1 << 2);
			  GPIO->P[led1b_PORT].DOUTCLR = 1 << led1b_PIN;
		  }

		  if (relayVec & 1 << 3)
		  {
			  GPIO->P[led1r_PORT].DOUTSET = 1 << led1r_PIN;
			  GPIO->P[relay1_PORT].DOUTSET = 1 << relay1_PIN;
		  }
		  else
		  {
			  GPIO->P[led1r_PORT].DOUTCLR = 1 << led1r_PIN;
			  GPIO->P[relay1_PORT].DOUTCLR = 1 << relay1_PIN;
		  }

		  /* LED 0 is inverted */
		  if (DebounceSM(KIRICAPSENSE_getPressed(0), 127, &KCS_CH0_Debounce))
		  {
			  if ((relayVec & 1 << 0) == 0)
			  {
				  relayVec |= 1 << 0;
				  relayVec ^= 1 << 1;
			  }
			  GPIO->P[led0b_PORT].DOUTCLR = 1 << led0b_PIN;
		  }
		  else
		  {
			  relayVec &= ~(1 << 0);
			  GPIO->P[led0b_PORT].DOUTSET = 1 << led0b_PIN;
		  }

		  if (relayVec & 1 << 1)
		  {
			  GPIO->P[led0r_PORT].DOUTCLR = 1 << led0r_PIN;
			  GPIO->P[relay0_PORT].DOUTSET = 1 << relay0_PIN;
		  }
		  else
		  {
			  GPIO->P[led0r_PORT].DOUTSET = 1 << led0r_PIN;
			  GPIO->P[relay0_PORT].DOUTCLR = 1 << relay0_PIN;
		  }
	  }
  }
}
