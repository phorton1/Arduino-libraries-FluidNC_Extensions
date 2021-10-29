//------------------------------------------------------------------
// A namespace that consolidates my access to FluidNC internals
//------------------------------------------------------------------
// besides the 'state' from gStatus, this namespace provides
// a set of consolidated entry points to "do" things in FluidNC,
// hiding the various multiple FluidNC objects from clients.


#pragma once

#include <FluidTypes.h>


namespace gActions
{
    extern void g_reset();                          // MotionControl.cpp::mc_reset()
    extern void g_limits_init();                    // GLimits.cpp::limits_init();

    extern void setAlarm(uint8_t alarm);            // Protocol.cpp::rtAlarm
    extern void setLimitMask(uint8_t mask);         // Machine::Axes::limitMask
    extern uint8_t getNegLimitMask();               // Machine::Axes::neg and posLimitMasks
    extern uint8_t getPosLimitMask();
    extern void setNegLimitMask(uint8_t mask);
    extern void setPosLimitMask(uint8_t mask);
    extern bool getProbeSucceeded();                // MotionControl.cpp::probe_succeeded
    extern void clearProbeSucceeded();              // MotionControl.cpp::probe_succeeded = false;

    extern void pushGrblText(const char *text);     // WebUI::inputBuffer.push()
    extern void realtime_command(Cmd cmd);          // Serial.cpp::execute_realtime_command()

    extern bool do_setting(char *buf);              // Settings.cpp::settings_execute_line() - should be const char*
    extern bool startSDJob(const char *filename);   // SDCard.cpp mas o menus

};
