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

/* The sequence, registers, and register reference definitions */
static const uint8_t seq[] =
{
	TO_SEQ(eMMREG_16B, eMMREG_16B, eMMREG_16B, eMMREG_16B),
	TO_SEQ(eMMREG_32B, eMMREG_16B, eMMREG_32B, eMMREG_16B),
	TO_SEQ(eMMREG_64B, eMMREG_A64B, eMMREG_32B, eMMREG_16B)
};

static uint16_t reg_a, reg_b, reg_c, reg_d;
static uint32_t reg_e, reg_f;
static uint64_t reg_g;

static uint16_t (*const real_mb_reg[]) =
{&reg_a, &reg_b, &reg_c, &reg_d, (uint16_t*)&reg_e, (uint16_t*)&reg_f, (uint16_t*)&reg_g};

#if MMW_STRUCT_TYPE == MMW_STRUCT_INTERNAL
static T_MMW_Data mb_d = {seq,
		real_mb_reg};
static T_MMW_Read mb_r;
static T_MMW_Write mb_w;
#endif /* MMW_STRUCT_TYPE == MMW_STRUCT_STATIC */

void MMW_Init(MMW_FD_DATA_STRUCT MMW_FD_READ_STRUCT MMW_FD_WRITE_STRUCT)
{
	MMW_REF_READ_STRUCT.start_addr = (uint16_t)-1U;
	MMW_REF_READ_STRUCT.real_addr = 0;
	MMW_REF_READ_STRUCT.real_seq = SEQ_LU(MMW_REF_DATA_STRUCT, 0);
	MMW_REF_READ_STRUCT.cur_seq = SEQ_LU(MMW_REF_DATA_STRUCT, 0);
	MMW_REF_READ_STRUCT.buffer_invalid = true;
	MMW_REF_READ_STRUCT.read_buf[0U] = 0;
	MMW_REF_READ_STRUCT.read_buf[1U] = 0;
	MMW_REF_READ_STRUCT.read_buf[2U] = 0;
	MMW_REF_READ_STRUCT.read_buf[3U] = 0;

	MMW_REF_WRITE_STRUCT.lastAddr = (uint16_t)-1U;
	MMW_REF_WRITE_STRUCT.targetSeq = SEQ_LU(MMW_REF_DATA_STRUCT, 0);
	MMW_REF_WRITE_STRUCT.la_inval = true;
	MMW_REF_WRITE_STRUCT.buffer[0U] = 0;
	MMW_REF_WRITE_STRUCT.buffer[1U] = 0;
	MMW_REF_WRITE_STRUCT.buffer[2U] = 0;
	MMW_REF_WRITE_STRUCT.buffer[3U] = 0;
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
uint8_t _mmw_read(MMW_FD_DATA_STRUCT MMW_FD_READ_STRUCT 
				  uint16_t addr, const uint16_t **const buf)
{
	/* Read structure mb_r initialized in init function */
	/* checks the address and whether the buffer is correct */
	/* reading the first address of a real register again triggers a refresh */
	MMW_REF_READ_STRUCT.cur_seq = SEQ_LU(MMW_REF_DATA_STRUCT, addr);
	if (addr >= MMW_REF_READ_STRUCT.start_addr &&
		addr <= MMW_REF_READ_STRUCT.start_addr + MMW_REF_READ_STRUCT.real_seq && 
		MMW_REF_READ_STRUCT.cur_seq <MMW_REF_READ_STRUCT.real_seq)
	{
		/* Pass.  The buffer is up-to-date.  */
	}
	else
	{
		/* Traverse through fake addresses until we find our real address */
		MMW_REF_READ_STRUCT.real_addr = 0;
		MMW_REF_READ_STRUCT.cur_seq = SEQ_LU(MMW_REF_DATA_STRUCT, 0);
		MMW_REF_READ_STRUCT.real_seq = SEQ_LU(MMW_REF_DATA_STRUCT, 0);
		MMW_REF_READ_STRUCT.start_addr = 0;
		for (uint16_t i = 0; i <= addr; i++)
		{
			uint8_t my_seq = SEQ_LU(MMW_REF_DATA_STRUCT, i);
			/* cross over real register boundary */
			if (my_seq >= MMW_REF_READ_STRUCT.cur_seq)
			{
				MMW_REF_READ_STRUCT.real_addr++;
				MMW_REF_READ_STRUCT.real_seq = my_seq;
				MMW_REF_READ_STRUCT.start_addr = i;
			}
			MMW_REF_READ_STRUCT.cur_seq = my_seq;
		}
		/* For loop exit condition compensation */
		MMW_REF_READ_STRUCT.real_addr--;
		MMW_REF_READ_STRUCT.buffer_invalid = true;
	}
	/* 
	 * The real mb register list actually contains pointers to registers
	 * larger than uint16_t, so we use the correct cast for those registers
	 * when extracting them into the read buffer.  
	 */
	if (MMW_REF_READ_STRUCT.buffer_invalid)
	{
		switch (MMW_REF_READ_STRUCT.real_seq)
		{
		case eMMREG_16B:
			MMW_REF_READ_STRUCT.read_buf[0] = *MMW_REF_DATA_STRUCT
					.mb_reg[MMW_REF_READ_STRUCT.real_addr];
			break;
		case eMMREG_32B:
			MMW_REF_READ_STRUCT.read_buf[0] = *(uint32_t*)MMW_REF_DATA_STRUCT
					.mb_reg[MMW_REF_READ_STRUCT.real_addr];
			MMW_REF_READ_STRUCT.read_buf[1] = *(uint32_t*)MMW_REF_DATA_STRUCT
					.mb_reg[MMW_REF_READ_STRUCT.real_addr] >> 16U;
			break;
		case eMMREG_64B:
			MMW_REF_READ_STRUCT.read_buf[0] = *(uint64_t*)MMW_REF_DATA_STRUCT
					.mb_reg[MMW_REF_READ_STRUCT.real_addr];
			MMW_REF_READ_STRUCT.read_buf[1] = *(uint64_t*)MMW_REF_DATA_STRUCT
					.mb_reg[MMW_REF_READ_STRUCT.real_addr] >> 16U;
			MMW_REF_READ_STRUCT.read_buf[2] = *(uint64_t*)MMW_REF_DATA_STRUCT
					.mb_reg[MMW_REF_READ_STRUCT.real_addr] >> 32U;
			MMW_REF_READ_STRUCT.read_buf[3] = *(uint64_t*)MMW_REF_DATA_STRUCT
					.mb_reg[MMW_REF_READ_STRUCT.real_addr] >> 48U;
			break;
		default:
			// error handling here
			break;
		}
		MMW_REF_READ_STRUCT.buffer_invalid = false;
	}
	(*buf) = MMW_REF_READ_STRUCT.read_buf;
	return MMW_REF_READ_STRUCT.cur_seq;
}

void _mmw_write(MMW_FD_DATA_STRUCT MMW_FD_WRITE_STRUCT 
				uint16_t addr, uint16_t buf[4])
{
	/* This struct is here to allow a better bit-wise representation */
	struct
	{
		uint16_t real_addr;
		uint8_t real_seq: 2;
		uint8_t last_seq: 2;
		uint8_t my_seq: 2;
	} work = {0, SEQ_LU(MMW_REF_DATA_STRUCT, 0), SEQ_LU(MMW_REF_DATA_STRUCT, 0), SEQ_LU(MMW_REF_DATA_STRUCT, 0)};
	/* Traverse across the sequence list to find real addresses */
	/* Note that this is calculated for each address including zero */
	for (uint16_t i = 0; i <= addr; i++)
	{
		work.my_seq = SEQ_LU(MMW_REF_DATA_STRUCT, i);
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
		*(MMW_REF_DATA_STRUCT.mb_reg[work.real_addr]) = buf[0U];
		break;
	case eMMREG_32B:
		*(uint32_t*)(MMW_REF_DATA_STRUCT.mb_reg[work.real_addr]) = (uint32_t)buf[0U] | (uint32_t)buf[1U] << 16U;
		break;
	case eMMREG_64B:
		*(uint64_t*)(MMW_REF_DATA_STRUCT.mb_reg[work.real_addr]) = (uint64_t)buf[0U] | (uint64_t)buf[1U] << 16U | (uint64_t)buf[2U] << 32U | (uint64_t)buf[3U] << 48U;
		break;
	default:
		// error handling code here?
		break;
	}
}

void _mmw_start(MMW_FD_DATA_STRUCT MMW_FD_WRITE_STRUCT 
				uint16_t addr, uint16_t data)
{
	MMW_REF_WRITE_STRUCT.targetSeq = SEQ_LU(MMW_REF_DATA_STRUCT, addr);
	MMW_REF_WRITE_STRUCT.lastAddr = addr;
	MMW_REF_WRITE_STRUCT.la_inval = 0;
	MMW_REF_WRITE_STRUCT.buffer[MMW_REF_WRITE_STRUCT.targetSeq] = data;
}

void _mmw_inval(MMW_FD_WRITE_STRUCT)
{
	MMW_REF_WRITE_STRUCT.targetSeq = 0;
	MMW_REF_WRITE_STRUCT.la_inval = 1;
}

bool MMW_Write_Register(MMW_FD_DATA_STRUCT MMW_FD_WRITE_STRUCT 
						uint16_t addr, uint16_t data)
{
	// if address is 0, all writes are valid and checking is not needed
	// does not account for an incorrectly programmed sequence (yet)
	if (addr == 0)
	{
		// start of sequence and previous lower address able to be assumed
		_mmw_start(MMW_CALL_DATA_STRUCT MMW_CALL_WRITE_STRUCT addr, data);
		if (MMW_REF_WRITE_STRUCT.targetSeq == 0U)
		{
			_mmw_write(MMW_CALL_DATA_STRUCT MMW_CALL_WRITE_STRUCT addr,
							MMW_REF_WRITE_STRUCT.buffer);
			_mmw_inval(MMW_CALL_WRITE_STRUCT);
		}
	}
	else
	{
		// previous sequence is 0, we are starting a new sequence
		if (SEQ_LU(MMW_REF_DATA_STRUCT, addr - 1U) == 0U)
		{
			_mmw_start(MMW_CALL_DATA_STRUCT MMW_CALL_WRITE_STRUCT addr, data);
		}
		// if we are in the middle of a write set
		else if (MMW_REF_WRITE_STRUCT.la_inval == 0U &&
				SEQ_LU(MMW_REF_DATA_STRUCT, addr - 1U) == 
				  SEQ_LU(MMW_REF_DATA_STRUCT, addr) + 1U && 
				MMW_REF_WRITE_STRUCT.lastAddr == addr - 1U)
		{
			// continuing sequence
			MMW_REF_WRITE_STRUCT.lastAddr = addr;
			MMW_REF_WRITE_STRUCT.buffer[SEQ_LU(MMW_REF_DATA_STRUCT, addr)] 
					= data;
		}
		else
		{
			// invalid write
			// generally, the cases that end up here are trying to
			// write in the middle of a register set
			_mmw_inval(MMW_CALL_WRITE_STRUCT);
			return 0;
		}
		// last write of sequence
		if (SEQ_LU(MMW_REF_DATA_STRUCT, addr) == 0U)
		{
			_mmw_write(MMW_CALL_DATA_STRUCT MMW_CALL_WRITE_STRUCT addr,
					MMW_REF_WRITE_STRUCT.buffer);
			_mmw_inval(MMW_CALL_WRITE_STRUCT);
		}
	}
	return 1U;
}

bool MMW_Read_Register(MMW_FD_DATA_STRUCT MMW_FD_READ_STRUCT uint16_t addr, 
				uint16_t *const data)
{
	const uint16_t *buf = NULL;
	// mmw read finds the data at the last (biggest) address that is greater than or equal to
	// the requested address
	uint8_t offset = _mmw_read(MMW_CALL_DATA_STRUCT MMW_CALL_READ_STRUCT addr, &buf);
	*data = buf[offset];
	return 1U;
}
