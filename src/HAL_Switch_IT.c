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
#include "PetitModbusPort.h"

volatile bool systick_fault_mode = false;
extern volatile uint32_t msCounter;
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
	if (systick_fault_mode)
	{
		msTicks++;       /* increment counter necessary in Delay()*/
	}
	else
	{
		// modbus t_1.5 timer
		PetitPortTimerStop();

		// clear the modbus receiver
		PetitRxBufferReset();

		PetitPortDirRx();
	}
}

void Fault_Handler(void)
{
	systick_fault_mode = true;
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

void USART1_RX_IRQHandler(void)
{
	PetitRxBufferInsert(USART1->RXDATA);
}

void USART1_TX_IRQHandler(void)
{
	pu8_t res;
	USART_IntClear(USART1, USART_IF_TXC);
	if (PetitTxBufferPop(&res))
	{
		USART1->TXDATA = res;
	}
	else
	{
		USART_IntDisable(USART1, USART_IF_TXC);
		PetitPortDirRx();
	}
}
