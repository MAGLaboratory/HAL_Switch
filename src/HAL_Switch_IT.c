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
	SysTick_Config(14000UL);

	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(led1r_PORT, led1r_PIN, gpioModePushPullDrive, 1);
	GPIO_DriveModeSet(led1r_PORT, gpioDriveModeHigh);

// it would be nice to flash both red LEDs here, but led0 is not guaranteed

	uint32_t msLast = msTicks;

	while (1) {
		if (msLast != msTicks) {
			msLast = msTicks;
			if ((msLast >> 9) & 0x1) {
				GPIO->P[led1r_PORT].DOUTCLR = 1 << led1r_PIN;
			} else {
				GPIO->P[led1r_PORT].DOUTSET = 1 << led1r_PIN;
			}
		}
	}
}

/*
 * This is the HardFault_Handler where microcontrollers enter but nobody leaves
 */
void HardFault_Handler(void)
{
	Fault_Handler();
}

void NMI_Handler(void)
{
	Fault_Handler();
}
