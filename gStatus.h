//-------------------------------------------------------
// An object that abstracts the state of FluidNC
//-------------------------------------------------------
// Consolidates disperse FluidNC state machines into a higher
// level abstraction of a "job" and provides a single API to
// a number of FluidNC state variables, minimimizing hodgepodge
// including of FluidNC H files.

#pragma once

#include <Arduino.h>
#include <FluidTypes.h>


#define G_NUM_AXIS      3
    // this object currently only supports
    // a simple three axis machine

// indicator states for SDCard and Wifi

typedef uint8_t indicatorState_t;

#define IND_STATE_NONE         0x00
#define IND_STATE_ENABLED      0x01
#define IND_STATE_READY        0x02
#define IND_STATE_ACTIVE       0x04
#define IND_STATE_ERROR        0x08
#define IND_STATE_ALL          0x0f


// the essential job state abstraction

typedef enum JobState
{
    JOB_NONE,
    JOB_IDLE,
    JOB_BUSY,
    JOB_HOLD,
    JOB_HOMING,
    JOB_PROBING,
    JOB_MESHING,
    JOB_ALARM
};


class gStatus
{
public:

    gStatus()   {}

    void initWifiEventHandler();
    void gWifiEvent(uint16_t event);

    void updateStatus(bool inMeshLeveling=false);
        // If you are using the mesh, please pass in "inLeveling" state
        // in order to correctly set the JobState

    JobState getJobState()          { return m_job_state; }
    uint8_t getLastAlaram()         { return m_last_alarm; }
        // grabbed when job state changes so it can be displayed later

    State getSysState()             { return m_sys_state; }
    SDState getSDState(bool refresh=false);

    uint8_t getWifiState()          { return m_wifi_state; }

    const char* getActiveFilename() { return m_active_filename; }
    float filePct()                 { return m_file_pct; }

    // wrappers

    static float getFeedRate();
    static float getAxisMaxTravel(int axis);
    static float getAxisPulloff(int axis);
    static float getAxisSeekRate(int axis);     // the faster of the two
    static float getAxisFeedRate(int axis);
    static volatile float getFeedOverride();
    static float getRapidFeedOverride();
    static float getSpindleOverride();
    static bool  getProbeState();

    // public state variables

    int32_t m_sys_pos[G_NUM_AXIS];
    float m_machine_pos[G_NUM_AXIS];
    float m_work_pos[G_NUM_AXIS];

protected:

    bool m_started = false;

    // state variables

    JobState  m_job_state = JOB_NONE;
    State m_sys_state = State::Sleep;
    SDState m_sdcard_state = SDState::NotPresent;

    uint8_t m_last_alarm = 0;
    uint8_t m_wifi_state = 0;

    const char *m_active_filename;
    float m_file_pct;

};  // class gStatus


extern gStatus g_status;

extern const char *sysStateName(State state);
extern const char *sdStateName(SDState state);
extern const char *jobStateName(JobState job_state);