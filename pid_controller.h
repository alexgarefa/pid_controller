#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include <stdbool.h>
#include <stdint.h>

struct PidController_s
{
    struct
    {
        struct
        {
            float kp;
            float ki;
            float kd;
        } gain;

        struct
        {
            bool saturate;
            float lower;
            float upper;
        } output;

    } config;

    struct
    {
        float last_error;
        float acumulated_error;
    } context;
};

typedef struct PidController_s PidController_s;

bool PidController_Init(PidController_s *controller);
bool PidController_ConfigGains(PidController_s *controller, const float kp, const float ki, const float kd);
bool PidController_Update(PidController_s *controller, const float setpoint, const float feedback, const uint32_t cycle_time_ms, float *output);
bool PidController_Flush(PidController_s *controller);

#endif __PID_CONTROLLER_H__