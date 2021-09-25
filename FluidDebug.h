// old fashioned printf() style log, info, and error messages
// decoupled from calling FluidNC output routines directly
// at a small cost in code and stack space

#pragma once

extern void g_info(const char *format, ...);
extern void g_debug(const char *format, ...);
extern void g_error(const char *format, ...);
extern bool FluidNC_execute(char *buf);