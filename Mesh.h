// cnc3018.h

#pragma once

#include <Configuration/Configurable.h> // FluidNC
#include <Planner.h>                    // FluidNC

// maximum mesh is big enough for 20mm grid on 3018 machine
// and occupies about 1/2K of memory

#define MAX_MESH_X_STEPS  15
#define MAX_MESH_Y_STEPS  9


class Mesh : public Configuration::Configurable
{
    public:

        Mesh();

        bool doMeshLeveling();

        bool isValid()         { return m_is_valid; }
        bool inLeveling()      { return m_in_leveling; }
            // to suppress various debug messages in other objects

        void readMesh();
            // called from setup() once to initially load the mesh from
            // the file.  Self invalidates if the parameters have changed
            // since creation. Also self invalidates via RuntimeSetting
            // changes in group()

        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position);
        void motors_to_cartesian(float* cartesian, float* motors, int n_axis);

        int getNumSteps()       { return ((int) _x_steps) * ((int)_y_steps); }
        int getCurStep()        { return m_cur_step; }
        float getZPulloff()     { return _z_pulloff; }
        float getZeroPos()      { return m_zero_point; }

        float getHeight()       { return _height; }
        float getWidth()        { return _width; }
        int   getXSteps()       { return _x_steps; }
        int   getYSteps()       { return _y_steps; }
        int   getNumProbes()    { return m_num_probes; }

    private:

        float m_mesh_x;     // position of mesh
        float m_mesh_y;

        // config variables

        float _height;
        float _width;
        float _x_steps;
        float _y_steps;

        float _z_pulloff;
        float _z_max_travel;
        float _z_feed_rate;
        float _line_seg_length;

        float  m_num_probes;

        // working variables

        int     m_cur_step;                                 // which step are we on
        bool    m_in_leveling;                              // true while in doMeshLeveling
        bool    m_is_valid;                                 // mesh levelling has completed
        float   m_zero_point;                               // the absolute machine position of z=0 at xy=0,0 (5,5)
        float   m_mesh[MAX_MESH_X_STEPS][MAX_MESH_Y_STEPS]; // the mesh
        float   m_dx;                                       // size of a step in machine coordinates
        float   m_dy;

        // FluidNC::Configurable api

        void group(Configuration::HandlerBase& handler) override;

        // implementation

        void debug_mesh();
        void init_mesh();
        void invalidateMesh();

        bool probeOne(int x, int y, float *zResult);
        bool zPullOff(float from);
        bool writeMesh();

        float getZOffset(float mx, float my);
            // After doMeshLeveling() works, m_is_valid will be true and one can
            // call getZOffset with mx,my in machine coordinates to get the
            // interpolated mesh z offset at that point.

};  // class Mesh


extern Mesh the_mesh;
    // If you include Mesh.h, you MUST declare the instance
    // in one of your sketch ino or cpp files!
