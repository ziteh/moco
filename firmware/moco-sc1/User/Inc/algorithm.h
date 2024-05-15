
#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include "stdint.h"
#include "math.h"
#include "cmsis_compiler.h"
#include "const.h"

#if defined(CORDIC_ENABLE)
    #error No imp
#else
    #define _COS(rad) cos(rad)
    #define _SIN(rad) sin(rad)
#endif

#define DEG_TO_RAD(v) ((float)(v) * _PI_180)
#define RAD_TO_DEG(v) ((float)(v) * _180_PI)

typedef struct
{
    float set_val;
    float actual_val;
    float kp;
    float ki;
    float kd;
    float i_term_prev;
    float err_prev;
    float iteration_time;
    float max;
    float min;
    float bias;
} pid_data_t;

typedef struct
{
    float tf;
    float val_prev;
    uint32_t ts_prev;
} lpf_data_t;

float pid_cal(pid_data_t *pid);

/**
 * @brief Clarke transform (aka alpha-beta transform).
 *
 * @param[in] a
 * @param[in] b
 * @param[out] alpha
 * @param[out] beta
 */
__STATIC_INLINE void clarke_tf(float a, float b, float *alpha, float *beta)
{
    *alpha = a;
    *beta = _1_SQRT3 * a + _2_SQRT3 * b; // beta = (2 * b + a) / sqrt(3)
}

/**
 * @brief Inverse Clarke transform
 *
 * @param[in] alpha
 * @param[in] beta
 * @param[out] a
 * @param[out] b
 * @param[out] c
 */
__STATIC_INLINE void inv_clarke_tf(float alpha, float beta, float *a, float *b, float *c)
{
    float alpha_term = -0.5F * alpha;
    float beta_term = _SQRT3_2 * beta;

    *a = alpha;
    *b = alpha_term + beta_term;
    *c = alpha_term - beta_term;
}

/**
 * @brief Park transform
 *
 * @param[in] alpha
 * @param[in] beta
 * @param[in] angle_rad
 * @param[out] d
 * @param[out] q
 */
__STATIC_INLINE void park_tf(float alpha, float beta, float theta, float *d, float *q)
{
    float cos_theta = _COS(theta);
    float sin_theta = _SIN(theta);
    *d = alpha * cos_theta + beta * sin_theta;
    *q = beta * cos_theta - alpha * sin_theta;
}

/**
 * @brief Inverse Park transform
 *
 * @param[in] d
 * @param[in] q
 * @param[in] theta
 * @param[out] alpha
 * @param[out] beta
 */
__STATIC_INLINE void inv_park_tf(float d, float q, float theta, float *alpha, float *beta)
{
    float cos_theta = _COS(theta);
    float sin_theta = _SIN(theta);
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}

/**
 * @brief Inverse Park transform, d=0.
 *
 * @param[in] q
 * @param[in] theta
 * @param[out] alpha
 * @param[out] beta
 */
__STATIC_INLINE void inv_park_tf_0d(float q, float theta_rad, float *alpha, float *beta)
{
    *alpha = -q * _SIN(theta_rad);
    *beta = q * _COS(theta_rad);
}

__STATIC_INLINE float normalize_angle(float angle_rad)
{
    float a = fmod(angle_rad, _2PI);
    return (a >= 0) ? a : (a + _2PI);
}

typedef void (*modulation_fn_t)(float q, float angle_rad, float zero_angle_rad, float voltage, float *a, float *b, float *c);

void sin_pwm(float q, float angle_rad, float zero_angle_rad, float voltage, float *a, float *b, float *c);

void space_vector_pwm(float q, float angle_rad, float zero_angle_rad, float voltage, float *a, float *b, float *c);

#endif /* ALGORITHM_H_ */
