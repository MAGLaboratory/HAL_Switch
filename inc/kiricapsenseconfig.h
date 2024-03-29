/*
 * kirisakicapsenseconfig.h
 *
 *  Created on: Dec 15, 2023
 *      Author: brandon
 */

#ifndef INC_KIRICAPSENSECONFIG_H_
#define INC_KIRICAPSENSECONFIG_H_

#define KCS_ACMP_CAPSENSE                       ACMP0
#define KCS_ACMP_CAPSENSE_CLKEN                 CMU_HFPERCLKEN0_ACMP0
#define PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE     PRS_CH_CTRL_SOURCESEL_ACMP0
#define PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE     PRS_CH_CTRL_SIGSEL_ACMP0OUT

#define KCS_NUM_CHANNELS        2             /**< Number of channels used by Kirisaki Capsense */

#define BUTTON0_CHANNEL         0             /**< Button 0 channel */
#define BUTTON1_CHANNEL         1             /**< Button 1 channel */

#define KCS_CBUF_BITS			2			  /**< Number of bits in the circular buffer; must be at least 1 and no more than 8 */

#define KCS_BUF0_SZ             3u
#define KCS_BUF1_SZ             3u             /**< Also called NHD in a certain application note */
#define KCS_BUF2_SZ				3u

#define KCS_MHD                 20u
#define KCS_RECAL_THR			1000u
#define KCS_SLR_U				0u			  /**< Both slew rates operate at call speed / channel number / KCS_BUF0_SZ / KCS_BUF1_SZ / KCS_BUF2_SZ */
#define KCS_SLR_D				20u

#define  KCS_CHANNEL_IDX_2_HW {BUTTON0_CHANNEL, BUTTON1_CHANNEL}

#endif /* INC_KIRICAPSENSECONFIG_H_ */
