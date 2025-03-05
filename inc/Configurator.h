#include <stdint.h>
#include <stdbool.h>

/*
 * This header file is for the configuration state machine (configurator).
 * 
 * Also, the modbus holding registers are here too since they are critical
 * in how the configurator is used.
 */

#define C_CONF_NORMAL_MAX (1u)
#define C_CONF_UNLOCK_MAX (7u)

#define CONF_FLAG_NEW_UNLOCK_PASS
#define CONF_FLAG_NEW_PASS

/*
 * The holding registers are described here.
 *
 * The password register controls which registers are visible
 *
 * The control mode 0 and 1 allow these relay channels to be controlled by
 * different drive modes
 *
 * The KCS threshold high register controls how much higher than baseline the
 * capacitve counts have to be in order to trigger a high condition.
 *
 * The KCS threshold low register controls how much higher than baseline the
 * capacitve counts have to be in order to return to a low condition.
 *
 * The new unlock password register allows a new password to be entered for 
 * basic configuration access and control mode access.
 *
 * The new password register allows a new password to be entered for writing
 * configuration to the FLASH memory and for new passwords to be entered.
 *
 * The configuration control register signals for a configuration to be written
 * to the FLASH memory.
 */
typedef enum
{
	eHR_Pass = 0,
	eHR_CtrlMode0,
	eHR_CtrlMode1,
	eHR_KCS_threshHi,
	eHR_KCS_threshLo,
	eHR_NewUnlockPass,
	eHR_NewPass,
	eHR_Conf_Ctrl,
	eHR_NUM_REGS
} T_HoldingRegisters;

typedef enum
{
	eCFG_Idle = 0,
	eCFG_Load,
	eCFG_Unlock,
	eCFG_Cache,
	eCFG_Commit,
	eCFG_Erase,
	eCFG_Write,
	eCFG_NUM_STATES
} T_Conf_Ctrl;

