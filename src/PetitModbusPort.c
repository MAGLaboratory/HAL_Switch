#include "em_device.h"
#include "PetitModbusPort.h"
#include "PetitModbus.h"
#include "ModbusMiddleware.h"

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

bool PetitPortRegRead(uint16_t Addr, uint16_t *Data)
{
	return MMW_Read_Register(&MMW_CALL_DATA_STRUCT &MMW_CALL_READ_STRUCT Addr, Data);
}

bool PetitPortRegWrite(uint16_t Addr, uint16_t Data)
{
	return MMW_Write_Register(&MMW_CALL_DATA_STRUCT &MMW_CALL_WRITE_STRUCT Addr, Data);
}

bool PetitPortInputRegRead(uint16_t Addr, uint16_t *Data)
{
	return MMW_Read_Register(&hd_st, &hr_st, Addr, Data);	
}
