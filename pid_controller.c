#include "pid_controller.h"

#include <stddef.h>

bool PidController_Init(PidController_s *controller)
{
    if (controller == NULL)
        return false;
    return true;
}
bool PidController_Update(PidController_s *controller, const float setpoint, const float feedback, const uint32_t cycle_time_ms, float *p_output)
{
    if (controller == NULL || cycle_time_ms == 0U)
        return false;
    return true;
}
