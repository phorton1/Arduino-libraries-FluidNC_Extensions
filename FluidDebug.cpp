// old fashioned printf() style log, info, and error messages
// decoupled from calling FluidNC output routines directly
// at a small cost in code and stack space

#include <Arduino.h>
#include <Logging.h>		// FluidNC
#include <System.h>			// FluidNC
#include <GCode.h>          // FluidNC
#include <Protocol.h>       // FluidNC
#include <Report.h>         // FluidNC
#include <Uart.h>			// FluidNC

#define DEBUG_EXECUTE  1


void g_debug(const char *format, ...)
{
	va_list var;
	va_start(var, format);
	char display_buffer[255];
	vsprintf(display_buffer,format,var);
	log_debug(display_buffer);
	va_end(var);
}

void g_info(const char *format, ...)
{
	va_list var;
	va_start(var, format);
	char display_buffer[255];
	vsprintf(display_buffer,format,var);
	log_info(display_buffer);
	va_end(var);
}

void g_error(const char *format, ...)
{
	va_list var;
	va_start(var, format);
	char display_buffer[255];
	vsprintf(display_buffer,format,var);
	log_error(display_buffer);
	va_end(var);
}



#if 0

	bool FluidNC_execute(char *buf)
		// I think there are problems with this.
		// It kept crashing when I tried to use it
		// to do probes
	{
		#if DEBUG_EXECUTE
			g_debug("FluidNC_execute(%s)",buf);
		#endif

		Error rslt = gc_execute_line(buf, allClients);

		#if DEBUG_EXECUTE > 1
			g_debug("FluidNC_execute rslt=%d",rslt);
		#endif

		if (rslt != Error::Ok)
		{
			report_status_message(rslt, allClients);
			g_error("FluidNC_execute: gc_execute_line(%s) failed",buf);
			return false;
		}
		protocol_buffer_synchronize();
		if (sys.abort)
		{
			g_error("FluidNC_execute: gcode aborted");
			return false;           // Bail to main() program loop to reset system.
		}
		#if DEBUG_EXECUTE > 1
			g_debug("FluidNC_execute: gcode completed");
		#endif
		return true;
	}

#endif