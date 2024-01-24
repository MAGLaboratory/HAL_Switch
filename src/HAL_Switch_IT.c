/*
 * HAL_Switch_IT.c
 *
 *  Created on: Jan 3, 2024
 *      Author: brandon
 */

#include "em_device.h"
#include "em_acmp.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "kiricapsense.h"
#include "hal-config.h"

extern volatile uint16_t msCounter;
volatile uint32_t msTicks = 0;

void TIMER0_IRQHandler(void)
{
  /* Clear interrupt flag */
  TIMER0->IFC = TIMER_IFC_OF;

  msCounter++;

  KIRICAPSENSE_IT();
}

void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

void Fault_Handler(void)
{
	/* Not using the systick function here because it sets interrupt priority too low */
	SysTick->LOAD = (uint32_t) (14000U - 1UL); /* set reload register */
	SysTick->VAL = 0UL; /* Load the SysTick Counter Value */
	NVIC_SetPriority(SysTick_IRQn, 0U);
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
			SysTick_CTRL_TICKINT_Msk |
			SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */

	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(led1r_PORT, led1r_PIN, gpioModePushPullDrive, 1);
	GPIO_DriveModeSet(led1r_PORT, gpioDriveModeHigh);

	uint32_t msLast = msTicks;
	uint32_t unTicked = 0;

	while (1) {
		if (msTicks - msLast > (1u << 10u) || ++unTicked > 1971323u) {
			msLast = msTicks;
			unTicked = 0;
			GPIO->P[led1r_PORT].DOUTTGL = 1 << led1r_PIN;
		}
	}
}
