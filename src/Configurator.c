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
			CFGsmPassIn(Data);
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

void CFGsmPassIn(uint16_t Pass)
{
	switch(CfgSmS)
	{
		case:
		break;
		default:
		break;
	}
}

void CFGsm(T_Conf_Ctrl CfgSmS)
{
	// transitions
	switch (CfgSmS)
	{
	case eCFG_Load:
		// transitions handled in-function
		break;
	case eCFG_Idle:
		if (pw_flag)
		{
			if (pw == cfg.pw)
			{
				CfgSmS = eCFG_Cache;
				configuring = true;
			}
			pw_flag = false;
		}
		break;
	case eCFG_Cache:
		if (pw_flag && Petit_RxTx_State == PETIT_RXTX_RX && dir_tx == false)
		{
			if (pw == C_CMD_COMMIT)
			{
				if (dcfg_check())
				{
					CfgSmS = eCFG_Commit;
					mmw_init(cfg.sid, cfg.baud);
					MB_WD_TIMEOUT = cfg.wdto;
				}
			}
			if (pw == C_CMD_CANCEL)
			{
				CfgSmS = eCFG_Idle;
				cfg = default_cfg;
				configuring = false;
			}
			pw_flag = false;
		}
		break;
	case eCFG_Commit:
		if (pw_flag && Petit_RxTx_State == PETIT_RXTX_RX && dir_tx == false)
		{
			if (pw == cfg.pw)
			{
				CfgSmS = eCFG_Erase;
			}
			if (pw == C_CMD_CANCEL)
			{
				CfgSmS = eCFG_Idle;
				configuring = false;
			}
			pw_flag = false;
		}
		break;
	case eCFG_Erase:
			CfgSmS = eCFG_Write;
		break;
	case eCFG_Write:
		// transitions handled in function
		break;
	default:
		CfgSmS = eCFG_Idle;
		break;
	}

	// outputs
	switch (CfgSmS)
	{
	case eCFG_Load:
		cfg_load();
		run_petitmodbus = false;
		break;
	case eCFG_Idle:
		run_petitmodbus = true;
		break;
	case eCFG_Unlock:
		run_petitmodbus = true;
	case eCFG_Cache:
		run_petitmodbus = true;
		// no need to turn off the switch
		break;
	case eCFG_Commit:
		run_petitmodbus = true;
		// functionality implemented in the transitions, believe it or not
		break;
	case eCFG_Erase:
		run_petitmodbus = false;
		FLASH_PageErase(C_FLASH_CONF);
		break;
	case eCFG_Write:
		run_petitmodbus = false;
		cfg_write();
		break;
	}

