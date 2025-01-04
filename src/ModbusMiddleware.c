#include "ModbusMiddleware.h"
#include "PetitModbus.h"

/*
 * The purpose of this middleware is to be able to access arbitrarily sized
 * registers (16-bit, 32-bit, 64-bit) using modbus.
 *
 * It works using a sequence array which contains a sequence of 2-bit
 * designators for modbus address to actual address.  The sequences always decrement unless on a boundary between an old register
 * and a new register.
 *
 * Reads retrieve the register and 
 */

MM_Inter_t mb =
{0, 1, 0, {0, 0, 0, 0}};

uint8_t seq[] =
{TO_SEQ( eMMREG_16B,  eMMREG_16B,  eMMREG_16B,  eMMREG_16B)};

uint16_t reg_a, reg_b, reg_c, reg_d;

uint16_t (*real_mb_reg[4]) =
{&reg_a, &reg_b, &reg_c, &reg_d}

uint8_t _mmw_read(uint16_t addr, uint16_t (*buf)[4])
{
	static uint16_t last_addr = 0;
	static uint16_t real_addr = 0;
	static uint8_t real_seq = SEQ_LU(0);
	static uint8_t last_seq = SEQ_LU(0);
	static uint16_t read_buf[4] = {0};
	/* Increment by one */
	/* checks the address and whether the buffer is exhausted */
	if (addr == last_addr + 1U && SEQ_LU(addr) < last_seq)
	{
		buf = &read_buf;
		last_seq = SEQ_LU(addr);
		last_addr = addr;
	}
	else if (addr == last_addr + 1U)
	{
		real_seq = SEQ_LU(addr);
		last_seq = real_seq;
		buffer_invalid = true;
	}
	else if (addr == last_addr)
	{
		// pass
		buf = &read_buf;
	}
	else
	{
		/* Traverse through fake addresses until we find our real address */
		real_addr = 0;
		last_seq = SEQ_LU(0);
		for (uint16_t i = 0; i < addr; i++)
		{
			uint8_t my_seq = SEQ_LU(i);
			/* cross over register boundary */
			if (my_seq > last_seq)
			{
				real_addr++;
				real_seq = my_seq;
			}
			last_seq = my_seq;
		}
		last_addr = addr;
		buffer_invalid = true;
	}
	if (buffer_invalid)
	{
		switch (real_seq)
		{
		case eMMREG_16B:
			buf[0] = *real_mb_reg[real_addr];
			break;
		case eMMREG_32B:
			buf[0] = (uint32_t)*real_mb_reg[real_addr] & ((1U << 16U) - 1U);
			buf[1] = (uint32_t)*real_mb_reg[real_addr] >> 16U;
			break;
		case eMMREG_64B:
			buf[0] = (uint64_t)*real_mb_reg[real_addr] & (1U << 16U) - 1U;
			buf[1] = (uint64_t)*real_mb_reg[real_addr] >> 16U & (1U << 16U) - 1U;
			buf[2] = (uint64_t)*real_mb_reg[real_addr] >> 32U & (1U << 16U) - 1U;
			buf[3] = (uint64_t)*real_mb_reg[real_addr] >> 48U & (1U << 16U) - 1U;
			break;
		default:
			// error handling here?
			break;
		}
	}
	return last_seq - real_seq;
}

void _mmw_write(uint16_t addr, uint16_t (*buf)[4])
{
	uint16_t real_addr = 0;
	uint8_t real_seq = SEQ_LU(0);
	uint8_t last_seq = SEQ_LU(0);
	for (uint16_t i = 0; i < addr; i++)
	{
		uint8_t my_seq = SEQ_LU(i);
		/* cross over register boundary */
		if (my_seq > last_seq)
		{
			real_addr++;
			real_seq = my_seq;
		}
		last_seq = my_seq;
	}
	switch (real_seq)
	{
	case eMMREG_16B:
		*real_mb_reg[real_addr] = buf[0];
		break;
	case eMMREG_32B:
		*real_mb_reg[real_addr] = buf[0] | buf[1] << 16;
		break;
	case eMMREG_64B:
		*real_mb_reg[real_addr] = buf[0] | buf[1] << 16 | buf[2] << 32 | buf[3] << 48;
		break;
	default:
		// error handling code here?
		break;
	}
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
	uint16_t buf;
	// mmw read finds the data at the last (biggest) address that is greater than or equal to
	// the requested address
	uint8_t offset = _mmw_read(addr, &buf);
	*data = buf[offset];
	return 1;
}
