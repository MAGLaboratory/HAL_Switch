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
{
	TO_SEQ(eMMREG_16B, eMMREG_16B, eMMREG_16B, eMMREG_16B),
	TO_SEQ(eMMREG_32B, eMMREG_16B, eMMREG_32B, eMMREG_16B),
	TO_SEQ(eMMREG_64B, eMMREG_A64B, eMMREG_32B, eMMREG_16B)
};

uint16_t reg_a, reg_b, reg_c, reg_d;
uint32_t reg_e, reg_f;
uint64_t reg_g;

uint16_t (*real_mb_reg[]) =
{&reg_a, &reg_b, &reg_c, &reg_d, (uint16_t*)&reg_e, (uint16_t*)&reg_f, (uint16_t*)&reg_g};

uint8_t _mmw_read(uint16_t addr, uint16_t buf[4])
{
	static uint16_t start_addr = (uint16_t)-1U;
	static uint16_t real_addr = 0;
	static uint8_t real_seq = eMMREG_16B;
	static uint8_t cur_seq = eMMREG_16B;
	static uint16_t read_buf[4] = {0};
	static bool buffer_invalid = true;
	/* Increment by one */
	/* checks the address and whether the buffer is exhausted */
	cur_seq = SEQ_LU(addr);
	if (addr >= start_addr && addr <= start_addr + real_seq && cur_seq < real_seq)
	{
		// pass
	}
	else
	{
		/* Traverse through fake addresses until we find our real address */
		real_addr = 0;
		cur_seq = SEQ_LU(0);
		real_seq = SEQ_LU(0);
		start_addr = 0;
		for (uint16_t i = 0; i <= addr; i++)
		{
			uint8_t my_seq = SEQ_LU(i);
			/* cross over register boundary */
			if (my_seq >= cur_seq)
			{
				real_addr++;
				real_seq = my_seq;
				start_addr = i;
			}
			cur_seq = my_seq;
		}
		/* For loop exit condition compensation */
		real_addr--;
		buffer_invalid = true;
	}
	if (buffer_invalid)
	{
		switch (real_seq)
		{
		case eMMREG_16B:
			read_buf[0] = *real_mb_reg[real_addr];
			break;
		case eMMREG_32B:
			read_buf[0] = *(uint32_t*)real_mb_reg[real_addr];
			read_buf[1] = *(uint32_t*)real_mb_reg[real_addr] >> 16U;
			break;
		case eMMREG_64B:
			read_buf[0] = *(uint64_t*)real_mb_reg[real_addr];
			read_buf[1] = *(uint64_t*)real_mb_reg[real_addr] >> 16U;
			read_buf[2] = *(uint64_t*)real_mb_reg[real_addr] >> 32U;
			read_buf[3] = *(uint64_t*)real_mb_reg[real_addr] >> 48U;
			break;
		default:
			// error handling here?
			break;
		}
		buffer_invalid = false;
	}
	buf = read_buf;
	return cur_seq;
}

void _mmw_write(uint16_t addr, uint16_t buf[4])
{
	uint16_t real_addr = 0;
	uint8_t real_seq = SEQ_LU(0);
	uint8_t last_seq = SEQ_LU(0);
	/* Traverse across the sequence list to find real addresses */
	/* Note that this is calculated for each address including zero */
	for (uint16_t i = 0; i <= addr; i++)
	{
		uint8_t my_seq = SEQ_LU(i);
		/* cross over register boundary */
		if (my_seq >= last_seq)
		{
			real_seq = my_seq;
			real_addr++;
		}
		last_seq = my_seq;
	}
	/* one-indexed to zero-indexed addressing compensation */
	real_addr--;
	/* Ugly but portable between endianness */
	switch (real_seq)
	{
	case eMMREG_16B:
		*(real_mb_reg[real_addr]) = buf[0U];
		break;
	case eMMREG_32B:
		*(uint32_t*)(real_mb_reg[real_addr]) = (uint32_t)buf[0U] | (uint32_t)buf[1U] << 16U;
		break;
	case eMMREG_64B:
		*(uint64_t*)(real_mb_reg[real_addr]) = (uint64_t)buf[0U] | (uint64_t)buf[1U] << 16U | (uint64_t)buf[2U] << 32U | (uint64_t)buf[3U] << 48U;
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

bool MMW_WRITE_REGISTER(uint16_t addr, uint16_t data)
{
	// if address is 0, all writes are valid and checking is not needed
	// does not account for an incorrectly programmed sequence (yet)
	if (addr == 0)
	{
		// start of sequence and previous lower address able to be assumed
		_mmw_start(addr, data);
		if (mb.targetSeq == 0U)
		{
			_mmw_write(addr, mb.buffer);
			_mmw_inval();
		}
	}
	else
	{
		// previous sequence is 0, we are starting a new sequence
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
			_mmw_write(addr, mb.buffer);
			_mmw_inval();
		}
	}
	return 1;
}

bool MMW_READ_REGISTER(uint16_t addr, uint16_t *data)
{
	uint16_t *buf = NULL;
	// mmw read finds the data at the last (biggest) address that is greater than or equal to
	// the requested address
	uint8_t offset = _mmw_read(addr, buf);
	*data = buf[offset];
	return 1;
}
