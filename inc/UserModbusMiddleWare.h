/*
 * UserModbusMiddleWare.h
 *
 *  Created on: 2025-01-20
 *      Author: Brandon
 *
 * This file is to be included in ModbusMiddleWare.h.
 */

#ifndef INC_USERMODBUSMIDDLEWARE_H_
#define INC_USERMODBUSMIDDLEWARE_H_

/* Define the kind of struct that the modbus middleware is supposed to use
 *
 * MMW_STRUCT_EXTERNAL lets the user define structs and use multiple structs
 * MMW_STRUCT_INTERNAL defines the structs automatically  
 */
#define MMW_STRUCT_TYPE MMW_STRUCT_EXTERNAL

#endif /* INC_USERMODBUSMIDDLEWARE_H_ */
