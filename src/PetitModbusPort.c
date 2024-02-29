#include "em_device.h"
#include "PetitModbusPort.h"
#include "PetitModbus.h"

void PetitPortTxBegin(pu8_t data)
{
	PetitPortDirTx();
	USART1->TXDATA = data;
}

void PetitPortTimerStart(void)
{
	SysTick->LOAD = (uint32_t) (10500UL - 1UL); /* set reload register */
	SysTick->VAL = 0UL; /* Load the SysTick Counter Value */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
			SysTick_CTRL_TICKINT_Msk |
			SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
}

void PetitPortTimerStop(void)
{
	SysTick->CTRL = 0;
}

void PetitPortDirTx(void)
{
	//GPIO->P[txen_PORT].DOUTSET = 1u << txen_PIN;
}

void PetitPortDirRx(void)
{
	//GPIO->P[txen_PORT].DOUTCLR = 1u << txen_PIN;
}
