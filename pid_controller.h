#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include <stdbool.h>
#include <stdint.h>


typedef struct PidController_s PidController_s;

bool PidController_Init(PidController_s *controller);
bool PidController_Update(PidController_s *controller, const float setpoint, const float feedback, const uint32_t cycle_time_ms, float *output);

#endif __PID_CONTROLLER_H__