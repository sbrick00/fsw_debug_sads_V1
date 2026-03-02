#ifndef SHARES_H
#define SHARES_H

#include "taskqueue.h"
#include "taskshare.h"

// These are defined in main.cpp
extern Share<float>    torque_cmd;       // [N*m]
extern Share<float>    speed_actual;     // [RPM]
extern Share<float>    speed_cmd_last;   // [RPM]
extern Share<uint32_t> rw_fault_flags;   // bitfield

extern Queue<uint32_t> edge_time;        // [us]

#endif // SHARES_H
