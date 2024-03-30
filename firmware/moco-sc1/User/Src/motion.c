
#include "motion.h"

#define MOTION_OPEN_LOOP
// #define MOTION_CLOSED_LOOP

#define CONTROL_VELOCITY
// #define CONTROL_ANGLE
// #define CONTROL_TORQUE

#if defined(MOTION_OPEN_LOOP) && defined(CONTROL_VELOCITY)
#elif defined(MOTION_CLOSED_LOOP) && defined(CONTROL_VELOCITY)
#elif defined(MOTION_OPEN_LOOP) && defined(CONTROL_ANGLE)
#elif defined(MOTION_CLOSED_LOOP) && defined(CONTROL_ANGLE)
#elif defined(MOTION_CLOSED_LOOP) && defined(CONTROL_TORQUE)
#else
    #error Select control type
#endif

float angle_prev = 0.0F;
float d_time = ML_TIM_MS;

void open_loop_velocity(float target_vel, float voltage, float vm, modulation_fn_t mod_fn)
{
    float angle_next = angle_prev + target_vel * d_time;
    angle_prev = angle_next;

    float ua, ub, uc;
    mod_fn(voltage, angle_next, 0, vm, &ua, &ub, &uc);
    spg4_set_pwm(ua / vm, ub / vm, uc / vm);
    //spg4_set_pwm(0, ub / vm, 0);
}
