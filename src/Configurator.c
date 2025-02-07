#include "Configurator.h"
#include "PetitModbusPort.h"

bool PetitPortRegRead(uint16_t Addr, uint16_t *Data)
{
	switch(Addr)
	{
		case eHR_Pass:
		break;
		case eHR_CtrlMode0:
		break;
		case eHR_CtrlMode1:
		break;
		case eHR_KCS_threshHi:
		break;
		case eHR_KCS_threshLo:
		break;
		case eHR_NewPass:
		break;
		case eHR_MB_Ctrl:
		break;
		default:
			// not implemented.  do nothing.
			return 0;
		break;
	}
}

bool PetitPortRegWrite(uint16_t Addr, uint16_t Data)
{
	switch(Addr)
	{
		case eHR_Pass:
		break;
		case eHR_CtrlMode0:
		break;
		case eHR_CtrlMode1:
		break;
		case eHR_KCS_threshHi:
		break;
		case eHR_KCS_threshLo:
		break;
		case eHR_NewPass:
		break;
		case eHR_MB_Ctrl:
		break;
		default:
			// not implemented.  do nothing.
			return 0;
		break;
	}
}
