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
	eHR_MB_Ctrl
} T_HoldingRegisters;

