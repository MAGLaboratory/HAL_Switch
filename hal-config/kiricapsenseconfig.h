/*
 * kirisakicapsenseconfig.h
 *
 *  Created on: Dec 15, 2023
 *      Author: brandon
 */

#ifndef HAL_CONFIG_KIRICAPSENSECONFIG_H_
#define HAL_CONFIG_KIRICAPSENSECONFIG_H_

#define KCS_ACMP_CAPSENSE                       ACMP0
#define KCS_ACMP_CAPSENSE_CLKEN                 CMU_HFPERCLKEN0_ACMP0
#define PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE     PRS_CH_CTRL_SOURCESEL_ACMP0
#define PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE     PRS_CH_CTRL_SIGSEL_ACMP0OUT

#define ACMP_CHANNELS           8             /**< Number of channels for the Analog Comparator */

#define BUTTON0_CHANNEL         0             /**< Button 0 channel */
#define BUTTON1_CHANNEL         1             /**< Button 1 channel */

#define CAPSENSE_CH_IN_USE  { true, true, false, false, false, false, false, false }

#endif /* HAL_CONFIG_KIRICAPSENSECONFIG_H_ */
