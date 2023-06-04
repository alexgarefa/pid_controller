#ifndef __PID_CONTROLLER_INCREMENTAL_H__
#define __PID_CONTROLLER_INCREMENTAL_H__

#include <stdbool.h>

struct PidControllerIncremental_s
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
        float last_output;
    } context;
};

typedef struct PidControllerIncremental_s PidControllerIncremental_s;

bool PidControllerIncremental_Init(PidControllerIncremental_s *controller);
bool PidControllerIncremental_ConfigGains(PidControllerIncremental_s *controller, const float kp, const float ki, const float kd);
bool PidControllerIncremental_ConfigOutput(PidControllerIncremental_s *controller, const bool saturate, const float lower, const float upper);
bool PidControllerIncremental_Update(PidControllerIncremental_s *controller, const float setpoint, const float feedback, float *output);
bool PidControllerIncremental_Flush(PidControllerIncremental_s *controller);

#endif // __PID_CONTROLLER_INCREMENTAL_H__