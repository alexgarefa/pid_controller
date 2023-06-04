#include "pid_controller_incremental.h"
#include <stdint.h>

int main(void)
{
    const float kp = 1.0;
    const float ki = 0.5;
    const float kd = 0.1;
    const float out_min = 0.0;
    const float out_max = 100.0;

    // PID CONTROLLER : INIT & CONFIG
    PidControllerIncremental_s controller = {0};
    if (!PidControllerIncremental_Init(&controller)) return 0;
    if (!PidControllerIncremental_ConfigGains(&controller, kp, ki, kd)) return 0;
    if (!PidControllerIncremental_ConfigOutput(&controller, true, out_min, out_max)) return 0;

    const uint32_t TASK_CYCLE_TIME_MS = 10; // task cycle time (constant)
    uint32_t time_counter_ms = 0u;

    while (true)
    {
        // TASK LOOP
        if ((time_counter_ms % TASK_CYCLE_TIME_MS) == 0)
        {
            // READ SENSOR INPUT

            // PID CONTROLLER : UPDATE

            float feedback = 0.0;    // sensor input
            float setpoint = 1000.0; // fixed setpoint
            float output = 0.0;      // output to actuator

            if (!PidControllerIncremental_Update(&controller, setpoint, feedback, &output))
                return false;

            // SEND ACTUATOR
        }
        time_counter_ms++;
    }

    return 0;
}
