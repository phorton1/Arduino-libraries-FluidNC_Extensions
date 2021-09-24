// old fashioned printf() style log, info, and error messages
// decoupled from calling FluidNC output routines directly
// at a small cost in code and stack space

#include <Arduino.h>
#include <Logging.h>	// FluidNC


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
