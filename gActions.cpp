//------------------------------------------------------------------
// A namespace that consolidates my access to FluidNC internals
//------------------------------------------------------------------

#include "gActions.h"
#include "FluidDebug.h"

#include <SD.h>

#include <GLimits.h>                // FluidNC
#include <MotionControl.h>          // FluidNC
#include <Protocol.h>               // FluidNC
#include <Report.h>                 // FluidNC
#include <SDCard.h>                 // FluidNC
#include <Serial.h>                 // FluidNC
#include <Settings.h>               // FluidNC
#include <System.h>                 // FluidNC
#include <Machine/MachineConfig.h>  // FluidNC
#include <WebUI/Commands.h>         // FluidNC

// overrides in protocol.h

// extern volatile Percent rtFOverride;  // Feed override value in percent
// extern volatile Percent rtROverride;  // Rapid feed override value in percent
// extern volatile Percent rtSOverride;  // Spindle override value in percent
//
// rtAccessoryOverride.bit.coolantFloodOvrToggle


namespace gActions
{
    void g_reset()						{ mc_reset(); }									// MotionControl.cpp
	void g_limits_init()				{ limits_init(); }								// GLimits.cpp
	void setAlarm(uint8_t alarm) 		{ rtAlarm = static_cast<ExecAlarm>(alarm); } 	// Protocol.cpp::rtAlarm
	void setLimitMask(uint32_t mask) 	{ Machine::Axes::limitMask = mask; }
	uint32_t getNegLimitMask()			{ return Machine::Axes::negLimitMask; }
	uint32_t getPosLimitMask()			{ return Machine::Axes::posLimitMask; }
    void setNegLimitMask(uint32_t mask) { Machine::Axes::negLimitMask = mask; }
    void setPosLimitMask(uint32_t mask) { Machine::Axes::posLimitMask = mask; }
	bool getProbeSucceeded() 			{ return probe_succeeded;	}					// MotionControl.cpp
    void clearProbeSucceeded()			{ probe_succeeded = false; }					// MotionControl.cpp
    void pushGrblText(const char *text)	{ WebUI::inputBuffer.push(text); }
	void realtime_command(Cmd cmd)      { execute_realtime_command(cmd,allClients); }	// Serial.cpp

    bool do_setting(char *buf)	// Settings.cpp
	{
		Error rslt = settings_execute_line(buf,allClients,WebUI::AuthenticationLevel::LEVEL_ADMIN);
		if (rslt != Error::Ok)
		{
			g_error("Could not set parameter value");
			return false;
		}
		return true;
	}

    bool startSDJob(const char *filename)	// SDCard.cpp
	{
		SDCard *sdCard = config->_sdCard;
		if (sdCard && sdCard->begin(SDState::Idle) == SDState::Idle)
		{
			if (sdCard->openFile(SD,filename,allClients,WebUI::AuthenticationLevel::LEVEL_ADMIN))
			{
				sdCard->_readyNext = true;
				return true;
			}
			else
				g_error("Could not open file");
		}
		else
			g_error("Could not get SDCard");
		return false;
	}

};
