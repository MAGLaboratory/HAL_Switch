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

static MM_Inter_t mb =
{0, 1, 0, {0, 0, 0, 0}};

static const uint8_t seq[] =
{
	TO_SEQ(eMMREG_16B, eMMREG_16B, eMMREG_16B, eMMREG_16B),
	TO_SEQ(eMMREG_32B, eMMREG_16B, eMMREG_32B, eMMREG_16B),
	TO_SEQ(eMMREG_64B, eMMREG_A64B, eMMREG_32B, eMMREG_16B)
};

uint16_t reg_a, reg_b, reg_c, reg_d;
uint32_t reg_e, reg_f;
uint64_t reg_g;

uint16_t (*const real_mb_reg[]) =
{&reg_a, &reg_b, &reg_c, &reg_d, (uint16_t*)&reg_e, (uint16_t*)&reg_f, (uint16_t*)&reg_g};

static MM_Read_t mb_r;

void MMW_INIT()
{
	mb_r.start_addr = (uint16_t)-1U;
	mb_r.real_addr = 0U;
	mb_r.real_seq = SEQ_LU(0);
	mb_r.cur_seq = SEQ_LU(0);
	mb_r.buffer_invalid = true;
	mb_r.read_buf[0] = 0;
	mb_r.read_buf[1] = 0;
	mb_r.read_buf[2] = 0;
	mb_r.read_buf[3] = 0;
}

/*
 * Modbus middleware read function
 *
 * This function takes an input address and reads it to the buffer.  It has a
 * cache for the previously read address, so it is possible to take a
 * "snapshot" of the value at that address while reading out each 16-bit
 * subdivision sequentially.
 *
 * addr - input
 * buf - output
 * return: address within the output buffer to read for the specified address
 */
uint8_t _mmw_read(uint16_t addr, uint16_t (*buf[4]))
{
	/* Read structure mb_r initialized in init function */
	
	/* Increment by one */
	/* checks the address and whether the buffer is exhausted */
	mb_r.cur_seq = SEQ_LU(addr);
	if (addr >= mb_r.start_addr && 
		addr <= mb_r.start_addr + mb_r.real_seq && 
		mb_r.cur_seq < mb_r.real_seq)
	{
		// pass
	}
	else
	{
		/* Traverse through fake addresses until we find our real address */
		mb_r.real_addr = 0;
		mb_r.cur_seq = SEQ_LU(0);
		mb_r.real_seq = SEQ_LU(0);
		mb_r.start_addr = 0;
		for (uint16_t i = 0; i <= addr; i++)
		{
			uint8_t my_seq = SEQ_LU(i);
			/* cross over register boundary */
			if (my_seq >= mb_r.cur_seq)
			{
				mb_r.real_addr++;
				mb_r.real_seq = my_seq;
				mb_r.start_addr = i;
			}
			mb_r.cur_seq = my_seq;
		}
		/* For loop exit condition compensation */
		mb_r.real_addr--;
		mb_r.buffer_invalid = true;
	}
	/* 
	 * The real mb register list actually contains pointers to registers
	 * larger than uint16_t, so we use the correct cast for those registers
	 * when extracting them into the read buffer.  
	 */
	if (mb_r.buffer_invalid)
	{
		switch (mb_r.real_seq)
		{
		case eMMREG_16B:
			mb_r.read_buf[0] = *real_mb_reg[mb_r.real_addr];
			break;
		case eMMREG_32B:
			mb_r.read_buf[0] = *(uint32_t*)real_mb_reg[mb_r.real_addr];
			mb_r.read_buf[1] = *(uint32_t*)real_mb_reg[mb_r.real_addr] >> 16U;
			break;
		case eMMREG_64B:
			mb_r.read_buf[0] = *(uint64_t*)real_mb_reg[mb_r.real_addr];
			mb_r.read_buf[1] = *(uint64_t*)real_mb_reg[mb_r.real_addr] >> 16U;
			mb_r.read_buf[2] = *(uint64_t*)real_mb_reg[mb_r.real_addr] >> 32U;
			mb_r.read_buf[3] = *(uint64_t*)real_mb_reg[mb_r.real_addr] >> 48U;
			break;
		default:
			// error handling here
			break;
		}
		mb_r.buffer_invalid = false;
	}
	*buf = &mb_r.read_buf;
	return mb_r.cur_seq;
}

void _mmw_write(uint16_t addr, uint16_t buf[4])
{
	struct
	{
		uint16_t real_addr;
		uint8_t real_seq: 2;
		uint8_t last_seq: 2;
		uint8_t my_seq: 2;
	} work = {0, SEQ_LU(0), SEQ_LU(0), SEQ_LU(0)};
	/* Traverse across the sequence list to find real addresses */
	/* Note that this is calculated for each address including zero */
	for (uint16_t i = 0; i <= addr; i++)
	{
		work.my_seq = SEQ_LU(i);
		/* cross over register boundary */
		if (work.my_seq >= work.last_seq)
		{
			work.real_seq = work.my_seq;
			work.real_addr++;
		}
		work.last_seq = work.my_seq;
	}
	/* one-indexed to zero-indexed addressing compensation */
	work.real_addr--;
	/* Ugly but portable between endianness */
	switch (work.real_seq)
	{
	case eMMREG_16B:
		*(real_mb_reg[work.real_addr]) = buf[0U];
		break;
	case eMMREG_32B:
		*(uint32_t*)(real_mb_reg[work.real_addr]) = (uint32_t)buf[0U] | (uint32_t)buf[1U] << 16U;
		break;
	case eMMREG_64B:
		*(uint64_t*)(real_mb_reg[work.real_addr]) = (uint64_t)buf[0U] | (uint64_t)buf[1U] << 16U | (uint64_t)buf[2U] << 32U | (uint64_t)buf[3U] << 48U;
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
	uint8_t offset = _mmw_read(addr, &buf);
	*data = buf[offset];
	return 1;
}
