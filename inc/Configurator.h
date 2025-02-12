#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	eHR_Pass = 0,
	eHR_CtrlMode0,
	eHR_CtrlMode1,
	eHR_KCS_threshHi,
	eHR_KCS_threshLo,
	eHR_NewPass,
	eHR_Conf_Ctrl
} T_HoldingRegisters;

typedef enum
{
	eCFG_Idle = 0,
	eCFG_Load,
	eCFG_Cache,
	eCFG_Commit,
	eCFG_Erase,
	eCFG_Write
} T_Conf_Ctrl;

