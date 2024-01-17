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

#define KCS_BUF0_IDX_MID        (KCS_BUF0_SZ / 2)
#define KCS_BUF1_IDX_MID        (KCS_BUF0_SZ / 2)

void KIRICAPSENSE_Init(void);

void KIRICAPSENSE_IT (void);

uint8_t KIRICAPSENSE_pressReady(void);

bool KIRICAPSENSE_getPressed(uint8_t channel);

#endif /* KIRICAPSENSE_INC_KIRICAPSENSE_H_ */
