# pid_controller

This module contains 2 versions of PID controllers.


1 "/pid/pid.h"
- Classic PID algorithm
- Has feature to limit the output
- Has feature to saturate the internal integrator (to avoid "WindUp")


2 "/pid_incremental/"
- Algorithm of an incremental PID, commonly used to reduce oscillations
- Has feature to limit the output
