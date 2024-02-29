/*
 * HAL_Switch.h
 *
 *  Created on: Feb 28, 2024
 *      Author: brandon
 */

#ifndef INC_HAL_SWITCH_H_
#define INC_HAL_SWITCH_H_

#define BIT_CHANGE(bit, vec, state) (vec = state ? vec | bit : vec & ~bit)

#endif /* INC_HAL_SWITCH_H_ */
