#include "pid_controller.h"

#include <stddef.h>

bool PidController_Init(PidController_s *controller)
{
    if (controller == NULL)
        return false;

    controller->config.gain.kp = 0.0;
    controller->config.gain.ki = 0.0;
    controller->config.gain.kd = 0.0;
    controller->context.last_error;
    controller->context.acumulated_error;
    return true;
}

bool PidController_ConfigGains(PidController_s *controller, const float kp, const float ki, const float kd)
{
    if (controller == NULL)
        return false;

    controller->config.gain.kp = kp;
    controller->config.gain.ki = ki;
    controller->config.gain.kd = kd;
    return true;
}

bool PidController_Update(PidController_s *controller, const float setpoint, const float feedback, const uint32_t cycle_time_ms, float *p_output)
{
    if (controller == NULL || cycle_time_ms == 0U)
        return false;

    const float ERROR = setpoint - feedback;
    const float TIME_DT = cycle_time_ms * 0.001;
    const float ERROR_DT = ERROR - controller->context.last_error;
    float error_sum = controller->context.acumulated_error + (ERROR * TIME_DT);

    // CALCULATE OUTPUT
    float action_p = controller->config.gain.kp * ERROR;
    float action_i = controller->config.gain.ki * error_sum;
    float action_d = controller->config.gain.kd * ERROR_DT / TIME_DT;
    float output_signal = action_p + action_i + action_d;

    *p_output = output_signal;

    // STORAGE CONTEXT
    controller->context.last_error = ERROR;
    controller->context.acumulated_error = error_sum;

    return true;
}
