/*
 * madgwick_filter.h
 * Madgwick 9-DoF AHRS filter — Zephyr / STM32H7A3ZI-Q
 *
 * Units expected:
 *   accel -> m/s²  |  gyro -> rad/s  |  mag -> Gauss
 */

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif
#define RAD_TO_DEG (180.0f / PI)

/* Tuning: higher = faster response but noisier.
 * Start at 0.3, raise to 0.5 if yaw drifts. */
#define BETA_DEFAULT 0.3f

/* ── Quaternion ──────────────────────────────────────────────────────── */
struct quaternion {
    float q1; /* w (scalar) */
    float q2; /* x          */
    float q3; /* y          */
    float q4; /* z          */
};

/* Current orientation estimate (updated by imu_filter / imu_filter_6dof) */
extern struct quaternion q_est;
extern float             beta;

/* ── API ─────────────────────────────────────────────────────────────── */

/* Call once at startup — resets q_est to identity [1,0,0,0] */
void madgwick_init(void);

/* 9-DoF update: accel (m/s²) + gyro (rad/s) + mag (Gauss) */
void imu_filter(float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx, float my, float mz,
                float dt);

/* 6-DoF fallback (no mag): yaw will drift */
void imu_filter_6dof(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float dt);

/* Convert q_est to Roll / Pitch / Yaw in degrees */
void eulerAngles(struct quaternion q,
                 float *roll, float *pitch, float *yaw);

/* ── Quaternion math (inline) ────────────────────────────────────────── */
struct quaternion quat_mult(struct quaternion L, struct quaternion R);

static inline void quat_scalar(struct quaternion *q, float s)
{
    q->q1 *= s; q->q2 *= s; q->q3 *= s; q->q4 *= s;
}

static inline float quat_norm(struct quaternion q)
{
    return sqrtf(q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3 + q.q4*q.q4);
}

static inline void quat_normalize(struct quaternion *q)
{
    float n = quat_norm(*q);
    if (n < 1e-10f) { q->q1=1; q->q2=0; q->q3=0; q->q4=0; return; }
    float inv = 1.0f / n;
    q->q1 *= inv; q->q2 *= inv; q->q3 *= inv; q->q4 *= inv;
}

static inline float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

#endif /* MADGWICK_FILTER_H */
