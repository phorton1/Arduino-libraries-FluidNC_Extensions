//-------------------------------------------------------
// An object that abstracts the state of FluidNC
//-------------------------------------------------------

#include "gStatus.h"
#include "FluidDebug.h"

#include <WiFi.h>

#include <MotionControl.h>		    // FluidNC
#include <Protocol.h>		      	// FluidNC
#include <SDCard.h>                 // FluidNC
#include <Serial.h>                 // FluidNC
#include <Machine/MachineConfig.h>  // FluidNC

#define DEBUG_WIFI  0

gStatus g_status;


// overrides in protocol.h

// extern volatile Percent rtFOverride;  // Feed override value in percent
// extern volatile Percent rtROverride;  // Rapid feed override value in percent
// extern volatile Percent rtSOverride;  // Spindle override value in percent
//
// rtAccessoryOverride.bit.coolantFloodOvrToggle




//-----------------------------
// static name methods
//-----------------------------

const char *jobStateName(JobState job_state)
{
    switch (job_state)
    {
        case JOB_NONE    : return "";
        case JOB_IDLE    : return "IDLE";
        case JOB_BUSY    : return "BUSY";
        case JOB_HOLD    : return "HOLD";
        case JOB_HOMING  : return "HOMING";
        case JOB_PROBING : return "PROBING";
        case JOB_MESHING : return "MESHING";
        case JOB_ALARM   : return "ALARM";
    }
    return "UNKNOWN_JOB_STATE";
}


const char *sysStateName(State state)
{
	switch (state)
	{
		case State::Idle       : return "Idle";
		case State::Alarm      : return "Alarm";
		case State::CheckMode  : return "CheckMode";
		case State::Homing     : return "Homing";
		case State::Cycle      : return "Cycle";
		case State::Hold       : return "Hold";
		case State::Jog        : return "Jog";
		case State::SafetyDoor : return "SafetyDoor";
		case State::Sleep      : return "Sleep";
	}
	return "UNKNOWN_STATE";
}


const char *sdStateName(SDState state)
	// BusyPrinting is same as Busy
{
	switch (state)
	{
		case SDState::Idle          : return "Idle";
		case SDState::NotPresent    : return "NotPresent";
		case SDState::Busy  		  : return "Busy";
		case SDState::BusyUploading : return "BusyUploading";
		case SDState::BusyParsing   : return "BusyParsing";
	}
	return "UNKNOWN_SD_STATE";
}



//-------------------------------
// Wifi
//-------------------------------

#if DEBUG_WIFI
	static const char *wifiEventName(WiFiEvent_t event)
	{
		switch (event)
		{
			case SYSTEM_EVENT_WIFI_READY	        : return "WIFI_READY";
			case SYSTEM_EVENT_SCAN_DONE	            : return "SCAN_DONE";
			case SYSTEM_EVENT_STA_START	            : return "STA_START";
			case SYSTEM_EVENT_STA_STOP	            : return "STA_STOP";
			case SYSTEM_EVENT_STA_CONNECTED	        : return "STA_CONNECTED";
			case SYSTEM_EVENT_STA_DISCONNECTED	    : return "STA_DISCONNECTED";
			case SYSTEM_EVENT_STA_AUTHMODE_CHANGE	: return "STA_AUTHMODE_CHANGE";
			case SYSTEM_EVENT_STA_GOT_IP	        : return "STA_GOT_IP";
			case SYSTEM_EVENT_STA_LOST_IP	        : return "STA_LOST_IP";
			case SYSTEM_EVENT_STA_WPS_ER_SUCCESS	: return "STA_WPS_ER_SUCCESS";
			case SYSTEM_EVENT_STA_WPS_ER_FAILED	    : return "STA_WPS_ER_FAILED";
			case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT	: return "STA_WPS_ER_TIMEOUT";
			case SYSTEM_EVENT_STA_WPS_ER_PIN	    : return "STA_WPS_ER_PIN";
			case SYSTEM_EVENT_AP_START	            : return "AP_START";
			case SYSTEM_EVENT_AP_STOP	            : return "AP_STOP";
			case SYSTEM_EVENT_AP_STACONNECTED	    : return "AP_STACONNECTED";
			case SYSTEM_EVENT_AP_STADISCONNECTED	: return "AP_STADISCONNECTED";
			case SYSTEM_EVENT_AP_STAIPASSIGNED	    : return "AP_STAIPASSIGNED";
			case SYSTEM_EVENT_AP_PROBEREQRECVED	    : return "AP_PROBEREQRECVED";
			case SYSTEM_EVENT_GOT_IP6	            : return "GOT_IP6";
			case SYSTEM_EVENT_ETH_START	            : return "ETH_START";
			case SYSTEM_EVENT_ETH_STOP	            : return "ETH_STOP";
			case SYSTEM_EVENT_ETH_CONNECTED         : return "ETH_CONNECTED	ESP32";
			case SYSTEM_EVENT_ETH_DISCONNECTED	    : return "ETH_DISCONNECTED";
			case SYSTEM_EVENT_ETH_GOT_IP	        : return "ETH_GOT_IP";
		}
		return "UNKNOWN_WIFI_EVENT";
	}
#endif


void gStatus::gWifiEvent(uint16_t event)
{
	switch (static_cast<WiFiEvent_t>(event))
	{
		case SYSTEM_EVENT_AP_START                 :    // ESP32 soft-AP start
		case SYSTEM_EVENT_STA_DISCONNECTED         :    // ESP32 station disconnected from AP
		case SYSTEM_EVENT_AP_STADISCONNECTED       :    // a station disconnected from ESP32 soft-AP
		case SYSTEM_EVENT_WIFI_READY               :    // ESP32 WiFi ready
			m_wifi_state = IND_STATE_ENABLED;
			break;

		// case SYSTEM_EVENT_SCAN_DONE                :    // ESP32 finish scanning AP
		case SYSTEM_EVENT_STA_START                :    // ESP32 station start
			m_wifi_state = IND_STATE_ACTIVE;
			break;

		case SYSTEM_EVENT_AP_STACONNECTED          :    // a station connected to ESP32 soft-AP
		case SYSTEM_EVENT_STA_GOT_IP               :    // ESP32 station got IP from connected AP
		case SYSTEM_EVENT_STA_CONNECTED            :    // ESP32 station connected to AP
		case SYSTEM_EVENT_STA_WPS_ER_SUCCESS       :    // ESP32 station wps succeeds in enrollee mode
			m_wifi_state = IND_STATE_READY;
			break;

		case SYSTEM_EVENT_AP_STOP                  :    // ESP32 soft-AP stop
		case SYSTEM_EVENT_STA_STOP                 :    // ESP32 station stop
		case SYSTEM_EVENT_STA_LOST_IP              :    // ESP32 station lost IP and the IP is reset to 0
		case SYSTEM_EVENT_STA_WPS_ER_FAILED        :    // ESP32 station wps fails in enrollee mode
		case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT       :    // ESP32 station wps timeout in enrollee mode
		case SYSTEM_EVENT_STA_WPS_ER_PIN           :    // ESP32 station wps pin code in enrollee mode
			m_wifi_state = IND_STATE_ERROR;
			break;

		// case SYSTEM_EVENT_STA_AUTHMODE_CHANGE      :    // the auth mode of AP connected by ESP32 station changed
		// case SYSTEM_EVENT_AP_STAIPASSIGNED         :    // ESP32 soft-AP assign an IP to a connected station
		// case SYSTEM_EVENT_AP_PROBEREQRECVED        :    // Receive probe request packet in soft-AP interface
		// case SYSTEM_EVENT_GOT_IP6                  :    // ESP32 station or ap or ethernet interface v6IP addr is preferred
		// case SYSTEM_EVENT_ETH_START                :    // ESP32 ethernet start
		// case SYSTEM_EVENT_ETH_STOP                 :    // ESP32 ethernet stop
		// case SYSTEM_EVENT_ETH_CONNECTED            :    // ESP32 ethernet phy link up
		// case SYSTEM_EVENT_ETH_DISCONNECTED         :    // ESP32 ethernet phy link down
		// case SYSTEM_EVENT_ETH_GOT_IP               :    // ESP32 ethernet got IP from connected AP
		// case SYSTEM_EVENT_MAX
	}
}


static void onWiFiEvent(WiFiEvent_t event)
	// We need to put activity indicators in FluidNC for IO to the Wifi AP/Station
	// and (for them) perhaps add telnet indicator as well.  Fairly low priority
	// for me at this point, though of interest.
{
	#if DEBUG_WIFI
		g_debug("onWifiEvent(%d) %s",event,wifiEventName(event));
	#endif
	g_status.gWifiEvent(event);
}


void gStatus::initWifiEventHandler()
	// register the wifiEvent handler
{
	WiFi.onEvent(onWiFiEvent);
}



//---------------------------------------------
// wrappers
//---------------------------------------------

SDState gStatus::getSDState(bool refresh/*=false*/)
{
	if (refresh && config->_sdCard)
		return config->_sdCard->begin(SDState::Idle);
	return m_sdcard_state;
}


float gStatus::getAxisMaxTravel(int axis)
{
	return config->_axes->_axis[axis]->_maxTravel;
}


float gStatus::getAxisPulloff(int axis)
{
	return config->_axes->_axis[axis]->_motors[0]->_pulloff;
}


bool gStatus::getProbeState()
{
	return (bool) probeState;		// in MotionControl.cpp
}


//---------------------------------------------
// updateStatus
//---------------------------------------------


void gStatus::updateStatus(bool inMeshLeveling /*=false*/)
{
	// wait until "started" (in a known state)
	// before polling FluidNC

	// SYSTEM STATE

	m_sys_state = sys.state;
	if (!m_started && m_sys_state != State::Sleep)
	{
		m_started = true;
		// g_debug("gStatus started ..");
	}
	if (!m_started)
		return;

	// SDCARD STATE

	SDCard *sdCard = config->_sdCard;
	if (sdCard)
	{
		m_sdcard_state = sdCard->get_state();
		if (m_sdcard_state == SDState::Busy)
		{
			m_active_filename = sdCard->filename();
			m_file_pct = sdCard->report_perc_complete();
		}
	}

	// JOB STATE

	JobState job_state = JOB_IDLE;

	if (inMeshLeveling)
		job_state = JOB_MESHING;
	else if (probeState == ProbeState::Active)
		job_state = JOB_PROBING;
    else if (m_sys_state == State::Homing)
        job_state = JOB_HOMING;
    else if (m_sys_state == State::Alarm)
        job_state = JOB_ALARM;
    else if (m_sys_state == State::Hold)
        job_state = JOB_HOLD;
    else if (m_sdcard_state == SDState::Busy)
        job_state = JOB_BUSY;

    // retain the previous state while cycling
	// and returning to idle

    if (job_state == JOB_IDLE &&
        m_sys_state == State::Cycle)
        job_state = m_job_state;

	m_job_state = job_state;

	// Grab the alaram state on job_state changes

	static JobState last_job_state = JOB_NONE;
	if (last_job_state != job_state)
	{
		last_job_state = job_state;
		m_last_alarm = static_cast<uint8_t>(rtAlarm);
		#if 1
			g_debug("gStatus grabbed alarm=%d",m_last_alarm);
		#endif
	}

	// POSITIONS

	float *pos = get_mpos();
	for (int i=0; i<G_NUM_AXIS; i++)
	{
		m_sys_pos[i] = motor_steps[i];
		m_machine_pos[i] = pos[i];
		m_work_pos[i] = pos[i];
			// converted below
	}
	mpos_to_wpos(m_work_pos);
		// DEPENDS on Yaml number of axes agreeing with our constant!!!!
		// or BAD THINGS will happen

}   // gStatus::updateStatus()
