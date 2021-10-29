# FluidNC_extensions

This library provides a number of extensions to the
[**FluidNC**](https://github.com/phorton1/Arduino-libraries-FluidNC) project.

- some easy to use debug/info/error **output methods**
- methods to abstract the **state** of the FluidNC machine
- methods to abstract **control** of the FluidNC machine
- methods to supplement the FluidNC **configuration** system, and
- an implementation of **mesh bed levelling**

## Output Methods

Including **FluidDebug.h** in your program will provide the following methods:


```
extern void g_info(const char *format, ...);
extern void g_debug(const char *format, ...);
extern void g_error(const char *format, ...);
```

These are old fashioned printf-like functions for ease of use.
They use static buffers of a **maximum 255 characters** in length
with no error checking, so please use common sense when calling
them.



## FluidNC State

The **gStatus** class attempts to hide some of the complexity of FluidNC
by consolidating a number of FluidNC's state variables into one object
so you don't have to include multiple FluidNC header files in your program,
and to insulate somewhat against changes to the underlying FluidNC
architecture.

```
    g_status.updateStatus();
    ...
    if (g_status.getJobState() == JOB_ALARM)
        ... your code here
```

To use it, you call **updateStatus()** on the global **g_status** object,
and then use the various accessors as needed.
Generally updateStatus() will be called from some kind of a loop.

If you are using the [**FluidNC_UI**](https://github.com/phorton1/Arduino-libraries-FluidNC_UI)
updateStatus() will automaticallly be called 30 times per second from the UI's update() task.


## FluidNC Control

Likewise, we have tried to abstract some of the most common actions
that one is likely to perform on FluidNC into a separate namespace
called **gActions** so you don't have to include multiple different
FluidNC header files and to provide some insulation against FluidNC
API changes:

```
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
```

<br>

## Mesh Levelling

Mesh Levelling is implemented by including **Mesh.h** into your FluidNC Arduino sketch.

This will declare a global extern Mesh object called **the_mesh**.  It is upto you
to provide the **definition** (instance) of the mesh object by including the
following line somewhere in your code:

```
Mesh the_mesh;
```

You call **readMesh()** from your *setup()* method after it calls FluidNC's *main_init()* method.

```
void setup()
{
    main_init();	// FluidNC setup() method
    the_mesh.readMesh();
    ...
```

In order to use the Mesh it is required that you have created your own FluidNC *Machine* which
is derived from **Machine::MachineConfig**.   Your machine provides the glue between FluidNC
and the Mesh object by implementing a number of methods, starting with the *Configurable*
**group()** method:


```
void YourMachine::group(Configuration::HandlerBase& handler) // override
{
	Machine::MachineConfig::group(handler);
	Mesh *_mesh = &the_mesh;
	handler.section("mesh",_mesh);
}
```

The actual behavior of the mesh (altering the z-position of the spindle in
realtime) is provided by your override of the two weakly bound FluidNC
methods **motorsToCartesian** and **cartesianToMotors** which call
methods on the mesh object:

```
bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position)
{
	the_mesh.cartesian_to_motors(target, pl_data, position);
}

void motors_to_cartesian(float* cartesian, float* motors, int n_axis)
{
	the_mesh.motors_to_cartesian(cartesian, motors, n_axis);
}
```


### Mesh Configuration

The mesh defines a rectangular *grid* of a certain size in millimeters,
with a certain number of *steps* in the X and Y directions where it
will *probe* the work surface and store that information for use
in the coordinate system conversions.

The mesh configuration variables can be set in your **yaml** configuration
file, or via command line parameters into the Serial Port, or via the
[**FluidNC_UI**](https://github.com/phorton1/Arduino-libraries-FluidNC_UI).

```yaml
    Mesh:
        height: 100            # height of mesh in mm
        width: 200             # width of mesh in mm
        x_steps: 10            # steps in x direction
        y_steps: 5             # steps in y direction
        pulloff: 3             # mm to "pulloff" from each sucessful prob
        max_travel: 50         # max mm travel of the Z axis during G38.2 gcode command
        feed_rate: 100         # z axis feed rate to be used when meshing (slow)
        seek_rate: 400;        # z_axis feed rate to be used when meshing (fast)
        line_seg_len: 0.5      # granularity of z rate calculations
        num_probes: 2          # upto 4 probes per point can be averaged together
```

These configuration variables can also be set via the *Serial Terminal Command Line*:

```
$mesh/x_steps=20
ok
```

but they will not be persistent between reboots unless the *YamlOverrides* feature (below)
is also included in your program.

### Mesh Commands

You can **initiate the meshing process** by issuing **$mesh/do_level** command in the terminal:

```
$mesh/do_level
ok
```

And the machine will begin probing from **whatever position it is in** at the moment you
send this command, creating a *map* of the levels in the grid you have defined.  If
that process succeeds, the *z-offsset* for all gcode commands will subsequently be
modified to account the slight differences at different points within the grid.
For points outside of the grid the z-offset will be extrapolated.

The mesh itself is stored on the *SPIFFS* in a file called **mesh_data.txt** and
can be accessed via the *WebUI* and/or modified by hand (it is a simple text file).

You can *clear* the mesh by issuing the **mesh/clear** command, and you can
see the current values by issuing the **mesh/show** command:

```
$mesh/show
[MSG:INFO: MESH: position=(105.000,15.000,-20.397)]
[MSG:INFO: MESH[3]   0.052  0.017 -0.040 -0.023 -0.123 -0.080]
[MSG:INFO: MESH[2]   0.047  0.033  0.008 -0.015 -0.014 -0.001]
[MSG:INFO: MESH[1]   0.008  0.020  0.014  0.016  0.012  0.008]
[MSG:INFO: MESH[0]   0.000  0.015  0.023 -0.005 -0.035 -0.852]
ok
```


### Live Z Offset

The **Mesh** object also provides for a **Live Z Offset** that you can modify while
a *job* is running to alter the position of the Z axis in close-to-realtime.

The *Live Z offset* can be set from the [**FluidNC_UI**](https://github.com/phorton1/Arduino-libraries-FluidNC_UI).
or from the *Serial Terminal* **if you override** the weakly bound FluidNC **user_realtime_command()** method.

```
void user_realtime_command(uint8_t command, Print &client)
{
	switch (command)
	{
        case CMD_LIVE_Z_PLUS_COARSE :
        case CMD_LIVE_Z_PLUS_FINE :
        case CMD_LIVE_Z_RESET :
        case CMD_LIVE_Z_MINUS_FINE :
        case CMD_LIVE_Z_MINUS_COARSE :
            the_mesh.setLiveZ(command);
            break;
	}
}
```

In which case the **ctrl-QWERTY** realtime commands will alter the live z-offset as follows:

- **ctrl-Q** = +0.020
- **ctrl-W** = +0.002
- **ctrl-R** = 0
- **ctrl-T** = -0.002
- **ctrl-Y** = -0.020

<br>

## YAML Overrides

If you include the file **YamlOverrides.h** in *one* C++ file in your sketch,
*most* command line settings (i.e. $mesh/x_steps=20) will be made *persistent*
including through *reboots*.

The *Yaml Overrides* are stored in a file called **yaml_temp.txt* on the *SPIFFS*,
which can be *removed* using the *WebUI* or with the **RST=\*** command line command.

Please see **YamlOverrides.h** for more information.


<br>

## Please Also See

- [**FluidNC**](https://github.com/phorton1/Arduino-libraries-FluidNC) - the next generation **ESP32 GRBL** machine
- [**FluidNC_UI**](https://github.com/phorton1/Arduino-libraries-FluidNC_UI) - a *touch screen user interface* for FluidNC
- [**esp32_cnc301832**](https://github.com/phorton1/Arduino-esp32_cnc3018) - an implementation of an inexpensive 3-axis **3018** cnc machine using this code
- the [**vMachine**](https://github.com/phorton1/Arduino-_vMachine) - a *Maslow-like* **vPlotter** cnc machine using this code


<br>

## Credits and License

This library is licensed under the
[GNU General Public License v3.0](https://github.com/phorton1/Arduino-libraries-FluidNC_extensisions/tree/master/LICENSE.TXT)

**Credits**

- To **bdring** and the **FluidNC Team**
