#include "ModbusMiddleware.h"
#include "PetitModbus.h"

MM_Inter_t mb =
{0, 1, 0, {0, 0, 0, 0}};

uint8_t seq[] =
{TO_SEQ( eMMREG_16B,  eMMREG_16B,  eMMREG_16B,  eMMREG_16B)};

uint16_t _mmw_read(uint16_t addr, uint16_t (*buf)[4])
{
	return 0;
}

void _mmw_write(uint16_t addr, uint16_t (*buf)[4])
{

}

void _mmw_start(uint16_t addr, uint16_t data)
{
	mb.targetSeq = SEQ_LU(addr);
	mb.lastAddr = addr;
	mb.la_inval = 0;
	mb.buffer[mb.targetSeq] = data;
}

void _mmw_inval()
{
	mb.targetSeq = 0;
	mb.la_inval = 1;
}

pu8_t write_register(uint16_t addr, uint16_t data)
{
	// if address is 0, all writes are valid and checking is not needed
	// does not account for an incorrectly programmed sequence (yet)
	if (addr == 0)
	{
		// start of sequence and previous lower address able to be assumed
		_mmw_start(addr, data);
		if (mb.targetSeq == 0U)
		{
			_mmw_write(addr,
					&(mb.buffer));
			_mmw_inval();
		}
	}
	else
	{
		if (SEQ_LU(addr - 1U) == 0U)
		{
			_mmw_start(addr, data);
		}
		// if we are in the middle of a write set
		else if (mb.la_inval == 0U && SEQ_LU(addr - 1U) == SEQ_LU(addr) + 1U
				&& mb.lastAddr == addr - 1U)
		{
			// continuing sequence
			mb.lastAddr = addr;
			mb.buffer[SEQ_LU(addr)] = data;
		}
		else
		{
			// invalid write
			// generally, the cases that end up here are trying to
			// write in the middle of a register set
			_mmw_inval(addr);
			return 0;
		}
		// last write of sequence
		if (SEQ_LU(addr) == 0U)
		{
			_mmw_write(addr,
					&(mb.buffer));
			_mmw_inval();
		}
	}
	return 1;
}

pu8_t read_register(uint16_t addr, uint16_t *data)
{
	uint16_t buf[4];
	// mmw read finds the data at the last (biggest) address that is greater than or equal to
	// the requested address
	uint16_t actual_addr = _mmw_read(addr, &buf);
	*data = buf[addr - actual_addr];
	return 1;
}
