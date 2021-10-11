// YamlOverrides.h
//
// By including this H file in the main machie INO file,
// the ability to store Yaml Overrides (persistent runtime configuration)
// is added to the program via overrides of WEAK_LINKs in FluidNC

#pragma once

#include <SPIFFS.h>
#include "FluidDebug.h"
#include <Machine/MachineConfig.h>	// FluidNC
#include <Configuration/RuntimeSetting.h>	// FluidNC


#define DEBUG_YAML_OVERRIDES    		2

#define v_error g_debug
	// until I figuire out a good way


#define YAML_FILENAME   	"/yaml_tmp.txt"
#define YAML_TEMPNAME   	"/yaml_tmp.tmp"
#define MAX_YAML_LENGTH		128

static char yaml_buf[MAX_YAML_LENGTH+1] = {0};


static const char *getYamlLine(File &f)
{
	int len = 0;
	int c = f.read();
	yaml_buf[0] = 0;
	while (c >= 0 && c != '\n' && len < MAX_YAML_LENGTH)
	{
		yaml_buf[len++] = c;
		c = f.read();
	}
	if (len)
	{
		yaml_buf[len] = 0;
		char *p = yaml_buf;
		while (*p && *p != '=') p++;
		if (*p == '=')
		{
			*p++ = 0;
			#if DEBUG_YAML_OVERRIDES > 1
				g_debug("getYamlLine(%s,%s)",yaml_buf,p);
			#endif
			return p;
		}
	}
	#if DEBUG_YAML_OVERRIDES > 1
		g_debug("getYamlLine returning NULL");
	#endif
	return NULL;
}


Error saveYamlOverride(const char *path, const char *value)
	// Open the yaml.tmp file, see if the path is in it.
	// If the path was not in the file, open for write (append) and add it.
	// Otherwise, create a new temporary copy replaing the setting in the existin file and rename it.
{
	static bool in_override = false;

	if (in_override)
	{
		v_error("saveYamlOverride(%s,%s) re-entered",path,value?value:"NULL");
		return Error::AnotherInterfaceBusy;
	}
	in_override = true;

	// First pass through file to see if path exists in it

	#if DEBUG_YAML_OVERRIDES
		g_debug("saveYamlOverride(%s,%s)",path,value?value:"NULL");
	#endif

	Error err = Error::Ok;
	bool file_exists = false;
	bool file_has_path = false;
	if (SPIFFS.exists(YAML_FILENAME))
	{
		file_exists = true;
		File f = SPIFFS.open(YAML_FILENAME);
		if (f)
		{
			while (getYamlLine(f))
			{
				if (!strcmp(path,yaml_buf))
				{
					file_has_path = true;
					#if DEBUG_YAML_OVERRIDES > 1
						g_debug("saveYamlOverride() file_has_path");
					#endif
					break;
				}
			}
			f.close();
		}
		else
		{
			v_error("saveYamlOverride() could not open SPIFFS %s for reading",YAML_FILENAME);
			err = Error::FsFailedOpenFile;
		}

	}
	else
	{
		#if DEBUG_YAML_OVERRIDES > 1
			g_debug("saveYamlOverride() %s does not exist",YAML_FILENAME);
		#endif
	}

	// 2nd pass append to existing file or copy/rename

	if (err==Error::Ok && !file_exists || !file_has_path)		// just append to existing file
	{
		File f = SPIFFS.open(YAML_FILENAME,FILE_APPEND);
		if (f)
		{
			f.print(path);
			f.print("=");
			f.print(value);
			f.print('\n');
			f.close();
			#if DEBUG_YAML_OVERRIDES > 1
				g_debug("saveYamlOverride() appended %s=%s to %s",path,value,YAML_FILENAME);
			#endif
		}
		else
		{
			v_error("saveYamlOverride() could not open SPIFFS %s for writing",YAML_FILENAME);
			err = Error::FsFailedOpenFile;
		}
	}
	else if (err==Error::Ok)		// copy from old file to new file then rename
	{
		#if DEBUG_YAML_OVERRIDES > 1
			g_debug("saveYamlOverride() creating %s from %s",YAML_TEMPNAME,YAML_FILENAME);
		#endif

		File fi = SPIFFS.open(YAML_FILENAME);
		if (!fi)
		{
			v_error("saveYamlOverride() could not open SPIFFS %s for reading",YAML_FILENAME);
			err = Error::FsFailedOpenFile;
		}
		else
		{
			File fo = SPIFFS.open(YAML_TEMPNAME,FILE_WRITE);
			if (!fo)
			{
				fi.close();
				v_error("saveYamlOverride() could not open SPIFFS %s for writing",YAML_TEMPNAME);
				err = Error::FsFailedOpenFile;
			}
			else
			{
				const char *yaml_value;
				while (yaml_value = getYamlLine(fi))
				{
					if (!strcmp(path,yaml_buf))
					{
						#if DEBUG_YAML_OVERRIDES > 1
							g_debug("saveYamlOverride() replaced %s=%s",path,value);
						#endif
						if (fo.print(path) != strlen(path) ||
							fo.print("=") != 1 ||
							fo.print(value) != strlen(value))
						{
							v_error("saveYamlOverride() could not write yaml_override %s=%s",path,value);
							err = Error:: NvsSetFailed;
							break;
						}
					}
					else
					{
						if (fo.print(yaml_buf) != strlen(yaml_buf) ||
							fo.print("=") != 1 ||
							fo.print(yaml_value) != strlen(yaml_value))
						{
							v_error("saveYamlOverride() could not write existing yaml_override %s=%s",yaml_buf,yaml_value);
							err = Error:: NvsSetFailed;
							break;
						}
					}
					fo.print('\n');
				}

				fo.close();
				fi.close();

				if (err==Error::Ok)
				{
					if (!SPIFFS.remove(YAML_FILENAME))
					{
						v_error("saveYamlOverride() could not remove %s",YAML_FILENAME);
						err = Error::NvsSetFailed;
					}
					else if (!SPIFFS.rename(YAML_TEMPNAME,YAML_FILENAME))
					{
						v_error("saveYamlOverride() could not renane %s to %s",YAML_TEMPNAME,YAML_FILENAME);
						err = Error::NvsSetFailed;
					}

				}	// no error copying from one to the other
			}	// no error opening fo
		}	// no error opening fi
	}	// no error on 1st read pass

	#if DEBUG_YAML_OVERRIDES > 1
		g_debug("saveYamlOverride() returning %d",err);
	#endif

	in_override = false;
	return err;
}


void loadYamlOverrides()
{
	#if DEBUG_YAML_OVERRIDES
		g_debug("loadYamlOverrides()");
	#endif
	if (SPIFFS.exists(YAML_FILENAME))
	{
		File f = SPIFFS.open(YAML_FILENAME);
		if (f)
		{
			const char *yaml_value;
			while (yaml_value = getYamlLine(f))
			{
				Configuration::RuntimeSetting rts(yaml_buf, yaml_value, allClients);
				config->group(rts);

				// the runtime setting already set the value into the tree,
				// so validating it at this point is a bit anachrynous.
				// what about if it is a non-existent key (i.e. gone out of date)
				// or if it changed types?

				if (!rts.isHandled_)
				{
					v_error("YamlOverride(%s,%s) not handled!",yaml_buf,yaml_value);
				}

				// 	try
				// 	{
				// 		Configuration::Validator validator;
				// 		config->validate();
				// 		config->group(validator);
				// 	}
				// 	catch (std::exception& ex)
				// 	{
				// 		log_error("Validation error: " << ex.what() << " at line(" << line_num << "): " << yaml_buf << "=" << yaml_value << " in " << YAML_FILENAME);
				// 	}
			}

			f.close();
		}
		else
		{
			v_error("ERROR could not open SPIFFS %s for reading",YAML_FILENAME);
		}
	}
	else
	{
		#if DEBUG_YAML_OVERRIDES > 1
			g_debug("loadYamlOverrides() %s does not exist",YAML_FILENAME);
		#endif
	}
}


void clearYamlOverrides()
{
	#if DEBUG_YAML_OVERRIDES
		g_debug("clearYamlOverrides()");
	#endif
	SPIFFS.remove(YAML_FILENAME);
	SPIFFS.remove(YAML_TEMPNAME);
}
