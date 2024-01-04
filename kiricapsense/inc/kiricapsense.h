/*
 * kiricapsense.h
 *
 *  Created on: Dec 15, 2023
 *      Author: brandon
 */

#ifndef KIRICAPSENSE_INC_KIRICAPSENSE_H_
#define KIRICAPSENSE_INC_KIRICAPSENSE_H_

#include <stdint.h>
#include <stdbool.h>
#include "kiricapsenseconfig.h"

void KIRICAPSENSE_Init(void);

void KIRICAPSENSE_IT (void);

bool KIRICAPSENSE_getPressed(uint8_t channel);

#endif /* KIRICAPSENSE_INC_KIRICAPSENSE_H_ */
