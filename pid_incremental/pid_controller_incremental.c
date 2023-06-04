#include "pid_controller_incremental.h"

#include <stddef.h>

bool PidControllerIncremental_Init(PidControllerIncremental_s *controller)
{
    if (controller == NULL)
        return false;

    controller->config.gain.kp = 0.0;
    controller->config.gain.ki = 0.0;
    controller->config.gain.kd = 0.0;
    controller->config.output.saturate = false;
    controller->config.output.lower = 0.0;
    controller->config.output.upper = 0.0;
    controller->context.last_error = 0.0;
    controller->context.last_output = 0.0;
    return true;
}

bool PidControllerIncremental_ConfigOutput(PidControllerIncremental_s *controller, const bool saturate, const float lower, const float upper)
{
    if (controller == NULL)
        return false;

    controller->config.output.saturate = saturate;
    controller->config.output.upper = upper;
    controller->config.output.lower = lower;
    return true;
}

bool PidControllerIncremental_ConfigGains(PidControllerIncremental_s *controller, const float kp, const float ki, const float kd)
{
    if (controller == NULL)
        return false;

    controller->config.gain.kp = kp;
    controller->config.gain.ki = ki;
    controller->config.gain.kd = kd;
    return true;
}

bool PidControllerIncremental_Update(PidControllerIncremental_s *controller, const float setpoint, const float feedback, float *p_output)
{
    if (controller == NULL)
        return false;

    const float ERROR = setpoint - feedback;
    const float ERROR_DT = ERROR - controller->context.last_error;

    const float LAST_OUTPUT = controller->context.last_output;
    const float LAST_ERROR = controller->context.last_error;

    // CALCULATE OUTPUT
    const float ACTION_P = controller->config.gain.kp * ERROR_DT;
    const float ACTION_I = controller->config.gain.ki * ERROR;
    const float ACTION_D = controller->config.gain.kd * (ERROR - 2 * LAST_ERROR + LAST_OUTPUT);
    
    float output_signal = LAST_OUTPUT + ACTION_P + ACTION_I + ACTION_D;

    // SATURATE OUTPUT
    if (controller->config.output.saturate)
    {
        if (output_signal > controller->config.output.upper)
            output_signal = controller->config.output.upper;

        if (output_signal < controller->config.output.lower)
            output_signal = controller->config.output.lower;
    }

    *p_output = output_signal;

    // STORAGE CONTEXT
    controller->context.last_error = ERROR;
    controller->context.last_output = output_signal;
    return true;
}

bool PidControllerIncremental_Flush(PidControllerIncremental_s *controller)
{
    controller->context.last_error = 0.0;
    controller->context.last_output = 0.0;
}
