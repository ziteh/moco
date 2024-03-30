
#include "algorithm.h"
#include "dwt.h"

float pid_cal(pid_data_t *pid)
{
    float error = pid->set_val - pid->actual_val;

    float p_term = pid->kp * error;
    float i_term = pid->ki * (pid->i_term_prev + error * pid->iteration_time);
    float d_term = pid->kd * (error - pid->err_prev) / pid->iteration_time;

    // I-term limit.
    if (i_term > pid->max)
    {
        i_term = pid->max;
    }
    else if (i_term < pid->min)
    {
        i_term = pid->min;
    }

    float output = p_term + i_term + d_term + pid->bias;

    // Final output limit.
    if (output > pid->max)
    {
        output = pid->max;
    }
    else if (output < pid->min)
    {
        output = pid->min;
    }

    pid->i_term_prev = i_term;
    pid->err_prev = error;
    return output;
}

/**
 * @brief Low-pass filter.
 *
 * @param lpf Low-pass filter config and data.
 * @param raw Raw value.
 * @return Filtered value.
 */
float lpf(lpf_data_t *lpf, float raw)
{
    uint32_t ts_now = dwt_get_cyccnt();
    float dt = dwt_get_diff_time(lpf->ts_prev, ts_now);
    lpf->ts_prev = ts_now;

    if (lpf->tf == 0.0F || dt > 0.3F) // Quick pass
    {
        lpf->val_prev = raw;
        return raw;
    }

    float a = lpf->tf / (lpf->tf + dt);
    float out = a * lpf->val_prev + (1.0F - a) * raw;
    lpf->val_prev = out;
    return out;
}

/**
 * @brief SPWM (Sinusoidal Pulse Width Modulation).
 *
 * @param[in] q
 * @param[in] theta
 * @param[in] center
 * @param[out] a
 * @param[out] b
 * @param[out] c
 */
void sin_pwm(float q, float angle_rad, float zero_angle_rad, float voltage, float *a, float *b, float *c)
{
    angle_rad = normalize_angle(angle_rad + zero_angle_rad);

    float u_alpha;
    float u_beta;
    inv_park_tf_0d(q, angle_rad, &u_alpha, &u_beta);

    float u_a;
    float u_b;
    float u_c;
    inv_clarke_tf(u_alpha, u_beta, &u_a, &u_b, &u_c);

    float vol_2 = voltage / 2.0F;
    *a = u_a + vol_2;
    *b = u_b + vol_2;
    *c = u_c + vol_2;
}

/**
 * @brief SVPWM (Space Vector Pulse Width Modulation)
 *
 * @param q
 * @param angle_rad
 * @param a
 * @param b
 * @param c
 */
void space_vector_pwm(float q, float angle_rad, float zero_angle_rad, float voltage, float *a, float *b, float *c)
{
    if (q < 0.0F)
    {
        angle_rad += _PI;
    }
    q = fabs(q);

    angle_rad = normalize_angle(angle_rad + zero_angle_rad + _PI_2);
    int8_t sector = floor(angle_rad / _PI_3) + 1;

    float k = _SQRT3 * q / voltage;
    float t1 = k * _SIN(sector * _PI_3 - angle_rad);
    float t2 = k * _SIN(angle_rad - (sector - 1.0F) * _PI_3);
    float t0_2 = (1.0F - t1 - t2) / 2.0F;

    float ta, tb, tc;
    switch (sector)
    {
    case 1:
        ta = t1 + t2 + t0_2;
        tb = t2 + t0_2;
        tc = t0_2;
        break;

    case 2:
        ta = t1 + t0_2;
        tb = t1 + t2 + t0_2;
        tc = t0_2;
        break;

    case 3:
        ta = t0_2;
        tb = t1 + t2 + t0_2;
        tc = t2 + t0_2;
        break;

    case 4:
        ta = t0_2;
        tb = t1 + t0_2;
        tc = t1 + t2 + t0_2;
        break;

    case 5:
        ta = t2 + t0_2;
        tb = t0_2;
        tc = t1 + t2 + t0_2;
        break;

    case 6:
        ta = t1 + t2 + t0_2;
        tb = t0_2;
        tc = t1 + t0_2;
        break;

    default:
        ta = 0.0F;
        tb = 0.0F;
        tc = 0.0F;
        break;
    }

    *a = ta * voltage;
    *b = tb * voltage;
    *c = tc * voltage;
}
