#include "em_device.h"
#include "PetitModbusPort.h"
#include "PetitModbus.h"
#include "ModbusMiddleware.h"
#include "HAL_Switch.h"

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

bool PetitPortDiscreteRead(uint16_t Addr, uint8_t *octet)
{
	if (Addr < 2)
	{
		*octet = capVec & CAP_IDX2VEC_STATUS(Addr);
	}
	else if (Addr < 4U)
	{
		*octet = commVec & COMM_IDX2VEC_STATE(Addr - 2U);
	}
	else if (Addr < 6U)
	{
		*octet = commVec & COMM_IDX2VEC_RX(Addr - 4U);
	}
	else
	{
		return false;
	}
	return true;
}

bool PetitPortCoilRead(uint16_t Addr, uint8_t *octet)
{
	if (Addr < 2U)
	{
		*octet = commVec & COMM_IDX2VEC_CMD(Addr);
	}
	else
	{
		return false;
	}
	return true;
}

bool PetitPortCoilWrite(uint16_t Addr, uint8_t octet)
{
	if (Addr < 2U)
	{
		BIT_CHANGE(commVec, COMM_IDX2VEC_CMD(Addr), octet);
		BIT_CHANGE(commVec, COMM_IDX2VEC_RX(Addr), true);
		return true;
	}
	return false;
}
