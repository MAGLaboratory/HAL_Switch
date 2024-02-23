/*
 * PetitModbusUserPort.h
 *
 *  Created on: Jan 24, 2024
 *      Author: brandon
 */

#ifndef HAL_CONFIG_PETITMODBUSUSERPORT_H_
#define HAL_CONFIG_PETITMODBUSUSERPORT_H_

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_usart.h"

#define PETITMODBUS_SLAVE_ADDRESS                       ( 255 )

#define PETITMODBUS_DLY_TOP  							( 0 )

#define NUMBER_OF_PETITCOILS                            ( 1 )
#define NUMBER_OF_DISCRETE_PETITINPUTS                  ( 0 )
// Petit Modbus RTU Slave Output Register Number
// Have to put a number of registers here
// It has to be bigger than 0 (zero)!!
#define NUMBER_OF_PETITREGISTERS                 		( 3 )
#define NUMBER_OF_INPUT_PETITREGISTERS 					( 1 )

#define NUMBER_OF_REGISTERS_IN_BUFFER                   ( 5 )

#define PETITMODBUS_READ_COILS_ENABLED                  ( 1 )
#define PETITMODBUS_READ_HOLDING_REGISTERS_ENABLED      ( 1 )
#define PETITMODBUS_WRITE_SINGLE_COIL_ENABLED           ( 1 )
#define PETITMODBUS_WRITE_SINGLE_REGISTER_ENABLED       ( 1 )
#define PETITMODBUS_WRITE_MULTIPLE_COILS_ENABLED        ( 1 )
#define PETITMODBUS_WRITE_MULTIPLE_REGISTERS_ENABLED    ( 1 )
#define PETITMODBUS_READ_INPUT_REGISTERS_ENABLED        ( 1 )
// Where to process our modbus message
// 0 for processing in its own cycle
// 1 for processing in the same cycle as TX CRC calculation
#define PETITMODBUS_PROCESS_POSITION                    ( 0 )
// Cycles to delay TX once the CRC finishes calculation
// this can either be a define as stated here or an integer changed by the
// application code
// #define PETITMODBUS_DLY_TOP (0)
// Address of this device
// this can either be a define as stated here or an integer changed by the
// application code
// #define PETITMODBUS_SLAVE_ADDRESS (1)
// Allow LED functions to be specified by the user
// 0 to leave blank functions
// 1 to allow user-defined functions
#define PETIT_USER_LED PETIT_USER_LED_DEF

// how to process the CRC
// PETIT_CRC_TABULAR takes up code space but is the fastest.
//     this should probably be your default choice
// PETIT_CRC_BITWISE takes up cycles but is space efficient
// PETIT_CRC_EXTERNAL requires you to define PetitPortCRC16Calc.
//     it is possible to use hardware CRC calculation with this.
#define PETIT_CRC PETIT_CRC_TABULAR

#define PETIT_COIL PETIT_INTERNAL

#define PETIT_DISCRETE_INPUT PETIT_INTERNAL

#define PETIT_REG PETIT_INTERNAL

#define PETIT_INPUT_REG PETIT_INTERNAL
/*****************************************************************************
 */
// define this to let the CRC table reside in code memory rather than RAM
#define PETIT_CODE
#define PETIT_FLASH_ATTR __attribute__((section("CRCTBL")))
// define this for booleans
#define pb_t bool
// define this for unsigned octets
#define pu8_t uint8_t
// define this for 16-bit unsigned
#define pu16_t uint16_t
// substitute the PetitPortCRC16Calc function call for our own
#define PetitPortCRC16Calc(a, d) KirisakiCRC16Calc(a, d)
// Defines for LEDs
#define PetitLedSuc()
#define PetitLedErrFail()
#define PetitLedCrcFail()
#define PetitLedOff()

extern void PetitPortDirTx(void);
extern void PetitPortDirRx(void);

#endif /* HAL_CONFIG_PETITMODBUSUSERPORT_H_ */

// addtogroup IoT_SV_EFM8BB1LCK_Petit_Modbus_Port
/** @} */
