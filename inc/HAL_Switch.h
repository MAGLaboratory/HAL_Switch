/*
 * HAL_Switch.h
 *
 *  Created on: Feb 28, 2024
 *      Author: brandon
 */

#ifndef INC_HAL_SWITCH_H_
#define INC_HAL_SWITCH_H_
#include "ModbusMiddleware.h"

#define BIT_CHANGE(bit, vec, state) (vec = state ? vec | bit : vec & ~bit)

extern T_MMW_Data md_st;
extern T_MMW_Read mr_st;
extern T_MMW_Write mw_st;

extern T_MMW_Data hd_st;
extern T_MMW_Read hr_st;

#endif /* INC_HAL_SWITCH_H_ */
