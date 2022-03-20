//-------------------------------------------------
// Mesh bed levelling
//-------------------------------------------------
// 1. the mesh starts at whatever MESH_X,MESH_Y (mpos floats)
//    the machine happens to be at when the $HZ happens
// 2. uess WIDTH and HEIGHT in mm and STEPS_X and STEPS_Y integers
// 3. the MESH, and it's 6 parameters are written to a SPIFFS text file
// 4. readMesh() is from cnc3018.ino() setup after grbl_init() finishes.
//
//   If the WIDTH, HEIGHT, STEPS_X, or STEPS_Y have changed the mesh is
//   invalidated in readMesh().  The MESH_X and MESH_Y are not user params.
//
//   The other user params Z_PULLOFF, Z_MAX_TRAVEL, Z_FEED_RATE, and
//   LINE_SEG_LENGTH can change without invalidating the mesh.
//
//   thus readMesh() is also called from group() when one of the given
//   items is changing at runtime.

// my test zero is at x=62 y=9

#include "Mesh.h"
#include "FluidDebug.h"

#include <GCode.h>                              // FluidNC
#include <MotionControl.h>                      // FluidNC
#include <Protocol.h>                           // FluidNC
#include <Report.h>                             // FluidNC
#include <Serial.h>                             // FluidNC
#include <System.h>                             // FluidNC
#include <Uart.h>                               // FluidNC
#include <Configuration/RuntimeSetting.h>       // FluidNC
#include <Planner.h>
#include <Machine/MachineConfig.h>              // FluidNC

#include <gStatus.h>							// FluidNC_extensions

#include <SPIFFS.h>
#include <FS.h>


#define DEBUG_MESH          1       // upto 3
#define DEBUG_GET_Z_OFFSET  0
#define DEBUG_VREVERSE      0       // upto 2
#define DEBUG_VFORWARD      0


#define Z_AXIS       2
#define Z_AXIS_MASK  0x04
#define Y_AXIS_MASK  0x02


#define DEFAULT_MESH_HEIGHT         80        // mm
#define DEFAULT_MESH_WIDTH          125       // mm
#define DEFAULT_MESH_X_STEPS        6         // every 25mm
#define DEFAULT_MESH_Y_STEPS        4         // every 20mm
#define DEFAULT_MESH_Z_PULLOFF      2.0       // pull off after 0th point
#define DEFAULT_MESH_Z_MAX_TRAVEL   -35.0     // mm in negative direction from absolute 0,0
#define DEFAULT_MESH_Z_FEED_RATE    20.0      // mm per min
#define DEFAULT_MESH_XY_SEEK_RATE   800.00
#define DEFAULT_LINE_SEG_LENGTH     2         // mm
#define DEFAULT_NUM_PROBES          1         // count


#define MESH_DATA_FILE  "/mesh_data.txt"



Mesh::Mesh()
{
    m_is_valid = 0;
    m_in_leveling = 0;
	m_cur_step = 0;

	m_live_z = 0;
    m_last_mesh_z = 0;

	_height			    = DEFAULT_MESH_HEIGHT;
	_width			    = DEFAULT_MESH_WIDTH;
	_x_steps		    = DEFAULT_MESH_X_STEPS;
	_y_steps		    = DEFAULT_MESH_Y_STEPS;
	_z_pulloff		    = DEFAULT_MESH_Z_PULLOFF;
	_z_max_travel	    = DEFAULT_MESH_Z_MAX_TRAVEL;
	_z_feed_rate	    = DEFAULT_MESH_Z_FEED_RATE;
	_xy_seek_rate       = DEFAULT_MESH_XY_SEEK_RATE;
    _line_seg_length    = DEFAULT_LINE_SEG_LENGTH;
}


void Mesh::group(Configuration::HandlerBase& handler) // override
{
	handler.item("height", 	    _height);
	handler.item("width", 		_width);
	handler.item("x_steps",	    _x_steps);
	handler.item("y_steps",     _y_steps);
	handler.item("pulloff",     _z_pulloff);
	handler.item("max_travel",  _z_max_travel);
	handler.item("feed_rate",   _z_feed_rate);
	handler.item("xy_seek_rate",_xy_seek_rate);
    handler.item("line_seg_len",_line_seg_length);
	handler.item("num_probes",   m_num_probes);
	if (m_num_probes > 4)
		m_num_probes = 4;
	if (m_num_probes < 1)
		m_num_probes = 1;


	// possibly invalidate the mesh upon certain changes

    if (m_is_valid && handler.handlerType() == Configuration::HandlerType::Runtime)
    {
        Configuration::RuntimeSetting &rth = static_cast<Configuration::RuntimeSetting &>(handler);
        if (rth.is("height") ||
            rth.is("width") ||
            rth.is("x_steps") ||
            rth.is("y_steps"))
        {
            #if DEBUG_MESH
                g_debug("Mesh::group() calling readMesh() for validation");
            #endif
            readMesh();
        }
    }


	// MESH COMMANDS
	// a weird way to add $commands

	if (handler.handlerType() == Configuration::HandlerType::Runtime)
	{
		Configuration::RuntimeSetting &rth = static_cast<Configuration::RuntimeSetting &>(handler);
		if (rth.is("show"))
		{
			debug_mesh();
			rth.isHandled_ = true;
		}
		else if (rth.is("clear"))
		{
			invalidateMesh();
			rth.isHandled_ = true;
			//int command_result = 237;
			//handler.item("blah", command_result);
		}
		else if (rth.is("do_level"))
		{
			#if USE_MESH_TASK
				xTaskCreatePinnedToCore(
					meshTask,		// method
					"meshTask",	    // name
					4096,			// stack_size
					this,			// parameters
					1,  			// priority
					NULL,			// returned handle
					0);				// core 1=main FluidNC thread/task, 0=my UI and other tasks
			#else
				doMeshLeveling();
			#endif
			rth.isHandled_ = true;
		}
	}

}



//--------------------------------------------
// retrieval API
//--------------------------------------------


float Mesh::getZOffset(float mx, float my)
    // IN WORK COORDINATES
{
    if (!m_is_valid)
    {
        return 0.00;
    }

    #if DEBUG_GET_Z_OFFSET
        g_debug("GET_Z_OFFSET(%5.3f,%5.3f) meshxy(%5.3f,%5.3f)",mx,my,m_mesh_x,m_mesh_y);
    #endif

    // make mx and my mesh relative

    mx -= m_mesh_x;
    my -= m_mesh_y;

    #if DEBUG_GET_Z_OFFSET
        g_debug("   mesh_relxy((%5.3f,%5.3f)",mx,my);
    #endif

    // develop the integer "box" coordinates
    // into the mesh array along the percentage
    // traversed into the box

    int x_left;
    int x_right;
    float pct_x;
    if (mx < 0)
    {
        x_left = 0;
        x_right = 0;
        pct_x = 1.0;
    }
    else
    {
        x_left = mx / m_dx;
        x_right = x_left + 1;
        if (x_left > _x_steps - 1)
            x_left = _x_steps - 1;
        if (x_right > _x_steps - 1)
            x_right = _x_steps - 1;
        pct_x = (mx - x_left * m_dx) / m_dx;
        if (pct_x > 1.0) pct_x = 1.0;
    }

    int y_top;
    int y_bottom;
    float pct_y;
    if (my < 0)
    {
        y_top = 0;
        y_bottom = 0;
        pct_y = 1.0;
    }
    else
    {
        y_bottom = my / m_dy;
        y_top = y_bottom + 1;
        if (y_bottom > _y_steps - 1)
            y_bottom = _y_steps - 1;
        if (y_top > _y_steps - 1)
            y_top = _y_steps - 1;
        pct_y = (my - y_bottom * m_dy) / m_dy;
    }

    // get the four values

    float left_bottom =    m_mesh[x_left][y_bottom];
    float left_top =       m_mesh[x_left][y_top];
    float right_bottom =   m_mesh[x_right][y_bottom];
    float right_top =      m_mesh[x_right][y_top];

    // calculate the contribution
    //
    //    lt               rt
    //
    //
    //            o  pct_x,pct_y
    //
    //
    //     lb               rb

    float left = left_bottom + pct_y * (left_top - left_bottom);
    float right = right_bottom + pct_y * (right_top - right_bottom);
    float value = left + pct_x * (right - left);

    #if DEBUG_GET_Z_OFFSET
        g_debug("   zone_lrtb(%d,%d,%d,%d)  pct_x=%5.3f  pct_y=%5.3f",
            x_left,
            x_right,
            y_top,
            y_bottom,
            pct_x,
            pct_y);

        g_debug("   mesh lb(%5.3f) lt(%5.3f) rb(%5.3f) rt(%5.3f)",
            left_bottom,
            left_top,
            right_bottom,
            right_top);

        g_debug("   left(%5.3f)  right(%5.3f)  FINAL VALUE(%5.3f)",
            left,
            right,
            value);
    #endif

    return value;

}



//--------------------------------------------
// generation API
//--------------------------------------------


void Mesh::debug_mesh()
{
	g_info("MESH: position=(%0.3f,%0.3f,%0.3f)",m_mesh_x,m_mesh_y,m_zero_point);
	for (int y=_y_steps-1; y>=0; y--)
	{
		static char buf[100];
		sprintf(buf,"MESH[%d] ",y);
		for (int x=0; x<_x_steps; x++)
		{
			sprintf(&buf[strlen(buf)]," % 6.3f",m_mesh[x][y]);
		}
		g_info(buf);
	}
}


void Mesh::init_mesh()
{
    #if DEBUG_MESH > 2
        g_debug("MESH: init_mesh()");
    #endif

    m_mesh_x = 0.0;
    m_mesh_y = 0.0;

    m_zero_point = 0.0;
    m_is_valid = false;

    m_dx = _width / (_x_steps-1);
    m_dy = _height / (_y_steps-1);

    for (int x=0; x<_x_steps; x++)
    {
        for (int y=0; y<_y_steps; y++)
        {
            m_mesh[x][y] = 0.00;
        }
    }
}



static bool _mesh_execute(char *buf)
    // _ means it does not use any member variables
{
    #if DEBUG_MESH > 2
        g_debug("MESH: _mesh_execute(%s)",buf);
    #endif

    Error rslt = gc_execute_line(buf, Uart0);
    if (rslt != Error::Ok)
    {
        report_status_message(rslt, allClients);
        g_error("MESH: gc_execute_line(%s) failed",buf);
        return false;
    }
    protocol_buffer_synchronize();
    if (sys.abort)
    {
        g_error("MESH: move aborted");
        return false;           // Bail to main() program loop to reset system.
    }
    #if DEBUG_MESH > 2
        g_debug("MESH: move completed");
    #endif
    return true;
}


bool Mesh::moveTo(float x, float y)
    // move in current coordinate system
    // _ means it does not use any member variables
{
    #if DEBUG_MESH > 2
        g_debug("MESH: _moveTo(%5.3f,%5.3f)",x,y);
    #endif
    char buf[36];
    sprintf(buf,"g1 g53 x%5.3f y%5.3f f%5.3f",x,y,_xy_seek_rate);
    return _mesh_execute(buf);
}


bool Mesh::zPullOff(float from) // move z upwards relative, check that probe goes off too
{
    // float to = m_zero_point + _z_pulloff;

    float to = from + _z_pulloff;

    #if DEBUG_MESH > 2
        g_debug("MESH: zPullOff() from=%5.3f to=%5.3f",from,to);
    #endif

    char buf[30];
    sprintf(buf,"g1 g53 z%5.3f f%5.3f",to,g_status.getAxisFeedRate(Z_AXIS));
		// 	We use the slower one (feed rate is slower than seek rate) for the mesh

    bool move_ok = _mesh_execute(buf);
    if (move_ok)
    {
        if (config->_probe->tripped())
        {
            g_error("MESH: zPullOff() failed!");
            rtAlarm = ExecAlarm::HomingFailPulloff;
            return false;
        }
    }
    return true;
}




bool Mesh::probeOne(int x, int y, float *zResult)
    // do a probe and return the z value in absolute machine coordinates
{
    #if DEBUG_MESH > 2
        g_debug("MESH: probeOne()");
    #endif

    char buf[24];
    sprintf(buf,"g38.2 z%5.3f f%5.3f",
        _z_max_travel,
        _z_feed_rate);

    #if DEBUG_MESH > 2
        g_debug("MESH: probeOne() execute(%s)",buf);
    #endif

    // do upto N probes with heuristics to throw out bad values


    bool ok = true;
    int probe_num = 0;
	int i_num_probes = m_num_probes;
    float probe_values[i_num_probes];

    while (ok && probe_num < m_num_probes)
    {
		protocol_execute_realtime();
		if (sys.abort)
			return false;

        float value = 0;
        Error rslt = gc_execute_line(buf, Uart0);
        if (rslt == Error::Ok)
        {
            value = steps_to_mpos(probe_steps[Z_AXIS],Z_AXIS);
            #if DEBUG_MESH > 1
                g_debug("MESH[%d,%d] probe[%d]=%f",x,y,probe_num,value);
            #endif
            probe_values[probe_num++] = value;
    }
        else
        {
            ok = false;
            report_status_message(rslt, allClients);
            g_error("MESH: probeOne() gc_execute_line failed");
        }

		protocol_execute_realtime();
		if (sys.abort)
			return false;

        if (!zPullOff(value))
			return false;
    }

    float value = 0;
    if (ok)
    {
        for (int i=0; i<m_num_probes; i++)
            value += probe_values[i];
        value /= ((float)m_num_probes);

        #if DEBUG_MESH > 1
            g_debug("MESH[%d,%d] average=%f",x,y,value);
        #endif

		if (m_num_probes >= 3)
		{
            // throw out the extreme value
            int max_idx = -1;
            float max_delta = -1;
            for (int i=0; i<m_num_probes; i++)
            {
                float delta = abs(value-probe_values[i]);
                if (delta > max_delta)
                {
                    max_idx = i;
                    max_delta = delta;
                }
            }

            #if DEBUG_MESH > 1
                g_debug("MESH[%d,%d] throwing out %d:%f",x,y,max_idx,probe_values[max_idx]);
            #endif

            value = 0;
            for (int i=0; i<m_num_probes; i++)
            {
                if (i != max_idx)
                    value += probe_values[i];
            }
            value /= ((float)m_num_probes-1);

            #if DEBUG_MESH
                g_debug("MESH[%d,%d] FINAL AVERAGE=%f",x,y,value);
            #endif
		}
    }

    *zResult = value;
    return ok;
}


bool Mesh::doMeshLeveling()
{
    m_in_leveling = true;
	m_cur_step = 0;

    init_mesh();

    // Finish all queued commands and empty planner buffer before starting probe cycle.
    // Return if system reset has been issued.

    protocol_buffer_synchronize();
    if (sys.abort)
    {
        m_in_leveling = false;
        return false;
    }

	sys.state = State::Idle;
	config->_stepping->beginLowLatency();

    m_mesh_x = steps_to_mpos(motor_steps[X_AXIS],X_AXIS);
    m_mesh_y = steps_to_mpos(motor_steps[Y_AXIS],Y_AXIS);

    #if DEBUG_MESH
        g_debug("MESH: doMeshLeveling(%5.3f,%5.3f)",m_mesh_x,m_mesh_y);
    #endif

    for (int y=0; y<_y_steps; y++)
    {
        int start = y & 1 ? _x_steps-1 : 0;
        int end   = y & 1 ? -1 : _x_steps;
        int inc   = y & 1 ? -1 : 1;

        for (int x=start; x!=end; x+=inc)
        {
			protocol_execute_realtime();
			if (sys.abort)
				return false;

            if (moveTo(m_mesh_x + x*m_dx, m_mesh_y + y*m_dy))
            {
                float value = 0.0;
                if (probeOne(x,y,&value))
                {
                    if (x==0 && y==0)
                    {
                        m_zero_point = value;

                        #if DEBUG_MESH > 1
                            g_debug("MESH: m_zero_point <= %6.3f",m_zero_point);
                        #endif
                    }
                    else
                    {
                        m_mesh[x][y] = value - m_zero_point;

                        #if DEBUG_MESH > 1
                            g_debug("MESH: m_mesh[%d,%d] <= %6.3f",x,y,m_mesh[x][y]);
                        #endif
                    }

                }   // probeOne() succeeded
                else
                {
                    g_error("MESH: probeOne(%d,%d) failed",x,y);
                    m_in_leveling = false;
                    return false;
                }
            }   // move succeeded
            else
            {
                g_error("MESH: moveTo(%d,%d) failed",x,y);
                m_in_leveling = false;
                return false;
            }

			m_cur_step++;

        }   // for x_steps
    }   // for y_steps

	config->_stepping->endLowLatency();

	protocol_execute_realtime();
	if (sys.abort)
		return false;

	#if DEBUG_MESH
	    debug_mesh();
    #endif

	// move to the original (given) x y position and
	// z_pulloff above the determined position

    char buf[50];
    sprintf(buf,"g0 g53 x%5.3f y%5.3f z%5.3f",m_mesh_x,m_mesh_y,m_zero_point + _z_pulloff);
	bool ok = _mesh_execute(buf);

	Stepper::go_idle();         // Set steppers to the settings idle state before returning.
	sys.state = State::Idle;    // Set to IDLE when complete.

	protocol_execute_realtime();
	if (sys.abort)
		return false;

    if (ok && writeMesh())
    {
		// set the z-zero position from the mesh pulloff position

		sprintf(buf,"g10 L20 z%5.3f",_z_pulloff);
		m_is_valid = _mesh_execute(buf);
    }

    m_in_leveling = false;

	#if DEBUG_MESH
		g_debug("MESH: doMeshLeveling() %s",m_is_valid?"succeeded!":"failed");
	#endif

    return m_is_valid;
}


//--------------------------------------------
// Data File
//--------------------------------------------

void Mesh::invalidateMesh()
{
    #if DEBUG_MESH
        g_debug("--->invalidateMesh()");
    #endif
    m_is_valid = false;
    SPIFFS.remove(MESH_DATA_FILE);
}


bool readFloat(File f, float *v)
{
    #define MAX_FLOAT  12
    int len = 0;
    char buf[MAX_FLOAT+1];
    int c = f.read();
    while (c >= 0 &&
           len < MAX_FLOAT &&
           c != '\n' &&
           c != ',')
    {
        buf[len++] = c;
        c = f.read();
    }
    buf[len] = 0;
    if (c >= 0)
    {
        float val = atof(buf);
        // g_debug("read float %s=%f",buf,val);
        *v = val;
        return true;
    }
    return false;
}


void Mesh::readMesh()
{
    #if DEBUG_MESH > 1
        g_debug("readMesh()");
    #endif

    init_mesh();

    if (SPIFFS.exists(MESH_DATA_FILE))
    {
        bool ok = false;
        File f = SPIFFS.open(MESH_DATA_FILE, "r");
        if (f)
        {
            float header[7];
            if (readFloat(f,&header[0]) &&
                readFloat(f,&header[1]) &&
                readFloat(f,&header[2]) &&
                readFloat(f,&header[3]) &&
                readFloat(f,&header[4]) &&
                readFloat(f,&header[5]) &&
                readFloat(f,&header[6]))
            {
                #if DEBUG_MESH > 1
                    g_debug("got Mesh Header x,y,w,h,sx,sy,zp: %5.3f,%5.3f,%5.3f,%5.3f,%1.0f,%1.0f,%5.3f",
                        header[0],
                        header[1],
                        header[2],
                        header[3],
                        header[4],
                        header[5],
                        header[6]);
                    // g_debug("versus w,h,sx,sy %5.3f,%5.3f,%5.3f,%5.3f",
                    //     _width,
                    //     _height,
                    //     _x_steps,
                    //     _y_steps);
                #endif

                if (header[2] == _width &&
                    header[3] == _height &&
                    header[4] == _x_steps &&
                    header[5] == _y_steps)
                {
                    #if DEBUG_MESH > 1
                        g_debug("Mesh Header Valid - reading mesh");
                    #endif

                    ok = true;

                    m_mesh_x = header[0];
                    m_mesh_y = header[1];
                    m_zero_point = header[6];

                    for (int y=0; ok && (y<_y_steps); y++)
                    {
                        for (int x=0; ok && (x<_x_steps); x++)
                        {
                            if (!readFloat(f,&m_mesh[x][y]))
                            {
                                ok = false;
                                g_error("Could not read mesh value(%d,%d)",y,x);                            }
                        }
                    }

                    if (ok)
                    {
                        #if DEBUG_MESH
                            g_debug("readMesh() VALID!!");
	                        debug_mesh();
                        #endif
                        m_is_valid = true;
                    }
                }
                else
                {
                    g_debug("Mesh Header INVALID!!");
                }
            }
            else
            {
                g_error("Could not read mesh header");
            }

            f.close();

        }   // file opened

        else
        {
            g_error("Could not open %s for reading",MESH_DATA_FILE);

        }

        if (!ok)
            invalidateMesh();

    }   // file exists
}



bool writeFloat(File f, float v, bool newline=true, bool prec_comma=false)
{
    char buf[12];
    sprintf(buf,"%-5.3f",v);
    if (prec_comma)
        if (f.print(",") != 1)
            return false;
    if (f.print(buf) != strlen(buf))
        return false;
    if (newline)
        if (f.print("\n") != 1)
            return false;
    return true;
}


bool Mesh::writeMesh()
{
    g_debug("writeMesh()");
    bool ok = false;
    File f = SPIFFS.open(MESH_DATA_FILE, "w");
    if (f)
    {
        if (writeFloat(f,m_mesh_x) &&
            writeFloat(f,m_mesh_y) &&
            writeFloat(f,_width) &&
            writeFloat(f,_height) &&
            writeFloat(f,_x_steps) &&
            writeFloat(f,_y_steps) &&
            writeFloat(f,m_zero_point))
        {
            ok = true;
            for (int y=0;ok && (y<_y_steps); y++)
            {
                for (int x=0;ok && (x<_x_steps); x++)
                {
                    if (!writeFloat(f,m_mesh[x][y],false,x>0))
                        ok = false;
                }
                if (f.print("\n") != 1)
                    ok = false;
            }
        }

        f.close();
        if (!ok)
        {
            g_error("There was a problem writing to %s",MESH_DATA_FILE);
            invalidateMesh();
        }
        return ok;
    }
    g_error("WARNING: Could not open %s for writing",MESH_DATA_FILE);
    invalidateMesh();
    return false;
}



//  #if MESH_USER_DEFINED_HOMING
//  	bool Mesh::user_defined_homing(AxisMask axisMask)
//  	{
//  		// $HZ builds the mesh
//  		// $HY invalidates the mesh
//
//  		if (axisMask == Y_AXIS_MASK)
//  		{
//  			g_debug("Mesh::user_defined_homing Y calling invalidateMesh()");
//  			invalidateMesh();
//  			return true;
//  		}
//
//  		if (axisMask == Z_AXIS_MASK)
//  		{
//  			g_debug("Mesh::user_defined_homing Z calling doMeshLeveling()");
//  			doMeshLeveling();
//  			return true;
//  		}
//
//  		return false;
//  	}
//  #endif


//======================================================================
// implement "kinematics" to shoehorn the mesh into FluidNC
//======================================================================

bool Mesh::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position)
    // THE REALTIME Z IS POSITIVE TO MOVE THE HEAD UP and NEGATIVE to move the head DOWN
    // my cut depth was 0.2 and I ended up setting the realtimeZOffset to 0.15, making
    // my effective cut depth 0.05 .. and that was still deep, but you have to account
    // for the tip breaking off of the bit in the first few minutes and becoming dull
    // over an entire run..
    //
    // given a target and initial position in Machine Coordinates
    // call mc_line() to move to the new position possibly including
    // the mesh z offset
{
    float tpos[6];
    float new_pos[6];
    memset(tpos,0,6*sizeof(float));
    memset(new_pos,0,6*sizeof(float));

	#if DEBUG_VREVERSE
		g_debug("C2M from(%5.3f,%5.3f,%5.3f) to (%5.3f,%5.3f,%5.3f)",
			position[X_AXIS],
            position[Y_AXIS],
            position[Z_AXIS],
            target[X_AXIS],
            target[Y_AXIS],
            target[Z_AXIS]);
	#endif

    // if the mesh is not valid, just call a single mc_line()
    // for the entire traversal

    if (!m_is_valid ||
        sys.state == State::Homing)
    {
        memcpy(new_pos,target,3 * sizeof(float));

		// ADD the live_z to the motor position

		if (sys.state != State::Homing)
			new_pos[Z_AXIS] += m_live_z;

        if (!mc_line(new_pos, pl_data))
			return false;
        return true;
    }

    memcpy(new_pos,position,3 * sizeof(float));

    // break the x/y portion of the line up into multiple segments

	uint32_t num_segs = 1;
	float xdist = target[X_AXIS] - new_pos[X_AXIS];
	float ydist = target[Y_AXIS] - new_pos[Y_AXIS];
	float zdist = target[Z_AXIS] - new_pos[Z_AXIS];

	if (!pl_data->motion.rapidMotion && (xdist!=0 || ydist!=0))
	{
		float dist = sqrt(xdist*xdist + ydist*ydist + zdist*zdist);
		float f_num_segs = (dist + (_line_seg_length/2))/_line_seg_length;
		num_segs = f_num_segs;
		if (!num_segs) num_segs = 1;
	}

	//--------------------
	// do loop of segments
	//--------------------

	float xinc = xdist/num_segs;
	float yinc = ydist/num_segs;
    float zinc = zdist/num_segs;

	#if DEBUG_VREVERSE
		g_debug("C2M doing %d segments with incs(%5.3f,%5.3f,%5.3f)",
			num_segs,
			xinc,
			yinc,
			zinc);
	#endif

	for (uint32_t seg_num=0; seg_num<num_segs; seg_num++)
	{
		new_pos[X_AXIS] += xinc;
		new_pos[Y_AXIS] += yinc;
		new_pos[Z_AXIS] += zinc;

        // we need a working copy of the position
        // so that we don't accumulate zOffsets

        memcpy(tpos,new_pos,3 * sizeof(float));

		// ADD the live_z to the motor position

		tpos[Z_AXIS] += m_live_z;

        // get the zOffset at the work position
        // and add it to the working copy

        float zoff = getZOffset(new_pos[X_AXIS],new_pos[Y_AXIS]);
        tpos[Z_AXIS] += zoff;

        // to see values in Arduino plotter:
        // Uart0.printf("%5.3f,%5.3f,%5.3f\r\n",realtimeZOffset,position[Z_AXIS]+24.222,zoff);

		#if DEBUG_VREVERSE>1
			g_debug("C2M seg(%d) to(%5.3f,%5.3f,%5.3f) zoff=%5.3f",
				seg_num,
				tpos[X_AXIS],
				tpos[Y_AXIS],
				tpos[Z_AXIS],
				zoff);
		#endif

		delay(5);	// IMPORTANT: let other tasks run

		// hmmm ... i thought mc_line failing was the problem,
		// but this code is never executed (I think it was the
		// dealy that was needed).  And maybe $message/level=DEBUG
		// helped by emulating that

		int problem_reported = 0;
		while (!mc_line(tpos, pl_data))
		{
			if (sys.abort)
			{
				log_error("SYS_ABORT FROM MESH MC_LINE FAIL!!!");
				return false;
			}
			// report the problem once per second
			// forever with a seconds counter
			if (problem_reported++ % 100 == 0)
				log_info("MESH MC_LINE_FAILED " << problem_reported/100);
			delay(10);
		}

	}	// for each segment

    return true;

}


//--------------------------------------------------------------------------------------------------------
// motors_to_cartesian
//--------------------------------------------------------------------------------------------------------

void Mesh::motors_to_cartesian(float* cartesian, float* motors, int n_axis)
    // the input variable "motors" is "machine position"
    // the output variable is "cartesian"
{
    // the only thing we might change is the Z

    memcpy(cartesian,motors,3 * sizeof(float));

	// SUBTRACT out the live z
	// if it's negative we added it during cartesian_to_motors()

    cartesian[Z_AXIS] -= m_live_z;

    // if the mesh is not valid, just return the input

    if (!m_is_valid ||
        sys.state == State::Homing)
        return;

    // get the zOffset at the location
    // and subtract it from the z location

    float zoff = m_last_mesh_z = getZOffset(motors[X_AXIS],motors[Y_AXIS]);

    #if DEBUG_VFORWARD
        g_debug("M2C(%5.3f,%5.3f,%5.3f) subtracting z_offset(%5.3f)",
            motors[X_AXIS],
            motors[Y_AXIS],
            motors[Z_AXIS],
            zoff);
    #endif

    cartesian[Z_AXIS] -= zoff;
}


void  Mesh::setLiveZ(uint8_t cmd)
{
	float z = m_live_z;
	if (cmd == CMD_LIVE_Z_PLUS_COARSE)
		z += LIVE_Z_COARSE;
	else if (cmd == CMD_LIVE_Z_PLUS_FINE)
		z += LIVE_Z_FINE;
	else if (cmd == CMD_LIVE_Z_RESET)
		z = LIVE_Z_DEFAULT;
	else if (cmd == CMD_LIVE_Z_MINUS_FINE)
		z -= LIVE_Z_COARSE;
	else if (cmd == CMD_LIVE_Z_MINUS_COARSE)
		z -= LIVE_Z_COARSE;
	if (z > LIVE_Z_MAX)
		z = LIVE_Z_MAX;
	if (z < LIVE_Z_MIN)
		z = LIVE_Z_MIN;
	g_debug("Mesh::setLiveZ(ctrl-%c) old(%0.3f) new(%0.3f)",('A' + cmd - 1),m_live_z,z);
	m_live_z = z;
}
