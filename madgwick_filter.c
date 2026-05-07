/*
 * madgwick_filter.c
 * Based on: S. Madgwick, "An efficient orientation filter..." 2010
 */

#include "madgwick_filter.h"

struct quaternion q_est = { 1.0f, 0.0f, 0.0f, 0.0f };
float             beta  = BETA_DEFAULT;

void madgwick_init(void)
{
    q_est.q1 = 1.0f; q_est.q2 = 0.0f;
    q_est.q3 = 0.0f; q_est.q4 = 0.0f;
    beta = BETA_DEFAULT;
}

struct quaternion quat_mult(struct quaternion L, struct quaternion R)
{
    struct quaternion p;
    p.q1 = L.q1*R.q1 - L.q2*R.q2 - L.q3*R.q3 - L.q4*R.q4;
    p.q2 = L.q1*R.q2 + L.q2*R.q1 + L.q3*R.q4 - L.q4*R.q3;
    p.q3 = L.q1*R.q3 - L.q2*R.q4 + L.q3*R.q1 + L.q4*R.q2;
    p.q4 = L.q1*R.q4 + L.q2*R.q3 - L.q3*R.q2 + L.q4*R.q1;
    return p;
}

/* ── 9-DoF ───────────────────────────────────────────────────────────── */
void imu_filter(float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx, float my, float mz,
                float dt)
{
    float q1=q_est.q1, q2=q_est.q2, q3=q_est.q3, q4=q_est.q4;

    /* Normalise accel & mag */
    float n = inv_sqrt(ax*ax + ay*ay + az*az);
    ax*=n; ay*=n; az*=n;
    n = inv_sqrt(mx*mx + my*my + mz*mz);
    mx*=n; my*=n; mz*=n;

    /* Earth magnetic field reference */
    float hx = 2.0f*mx*(0.5f-q3*q3-q4*q4) + 2.0f*my*(q2*q3-q1*q4) + 2.0f*mz*(q2*q4+q1*q3);
    float hy = 2.0f*mx*(q2*q3+q1*q4) + 2.0f*my*(0.5f-q2*q2-q4*q4) + 2.0f*mz*(q3*q4-q1*q2);
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = 2.0f*mx*(q2*q4-q1*q3) + 2.0f*my*(q3*q4+q1*q2) + 2.0f*mz*(0.5f-q2*q2-q3*q3);

    /* Objective function */
    float f1 = 2.0f*(q2*q4 - q1*q3)            - ax;
    float f2 = 2.0f*(q1*q2 + q3*q4)            - ay;
    float f3 = 2.0f*(0.5f - q2*q2 - q3*q3)     - az;
    float f4 = 2.0f*bx*(0.5f-q3*q3-q4*q4) + 2.0f*bz*(q2*q4-q1*q3) - mx;
    float f5 = 2.0f*bx*(q2*q3-q1*q4)      + 2.0f*bz*(q1*q2+q3*q4) - my;
    float f6 = 2.0f*bx*(q1*q3+q2*q4)      + 2.0f*bz*(0.5f-q2*q2-q3*q3) - mz;

    /* Gradient (J^T * f) */
    float s1 = -2.0f*q3*f1 + 2.0f*q1*f2 - 2.0f*bz*q3*f4 + (-2.0f*bx*q4+2.0f*bz*q2)*f5 + 2.0f*bx*q3*f6;
    float s2 =  2.0f*q4*f1 + 2.0f*q2*f2 - 4.0f*q2*f3 + 2.0f*bz*q4*f4 + (2.0f*bx*q3+2.0f*bz*q1)*f5 + (2.0f*bx*q4-4.0f*bz*q2)*f6;
    float s3 = -2.0f*q1*f1 + 2.0f*q3*f2 - 4.0f*q3*f3 + (-4.0f*bx*q3-2.0f*bz*q1)*f4 + (2.0f*bx*q2+2.0f*bz*q4)*f5 + (2.0f*bx*q1-4.0f*bz*q3)*f6;
    float s4 =  2.0f*q2*f1 + 2.0f*q4*f2 + (-4.0f*bx*q4+2.0f*bz*q2)*f4 + (-2.0f*bx*q1+2.0f*bz*q3)*f5 + 2.0f*bx*q2*f6;

    n = inv_sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4);
    s1*=n; s2*=n; s3*=n; s4*=n;

    /* Integrate */
    q1 += (0.5f*(-q2*gx - q3*gy - q4*gz) - beta*s1) * dt;
    q2 += (0.5f*( q1*gx + q3*gz - q4*gy) - beta*s2) * dt;
    q3 += (0.5f*( q1*gy - q2*gz + q4*gx) - beta*s3) * dt;
    q4 += (0.5f*( q1*gz + q2*gy - q3*gx) - beta*s4) * dt;

    n = inv_sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    q_est.q1=q1*n; q_est.q2=q2*n; q_est.q3=q3*n; q_est.q4=q4*n;
}

/* ── 6-DoF fallback ──────────────────────────────────────────────────── */
void imu_filter_6dof(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float dt)
{
    float q1=q_est.q1, q2=q_est.q2, q3=q_est.q3, q4=q_est.q4;

    float n = inv_sqrt(ax*ax + ay*ay + az*az);
    ax*=n; ay*=n; az*=n;

    float f1 = 2.0f*(q2*q4 - q1*q3)        - ax;
    float f2 = 2.0f*(q1*q2 + q3*q4)        - ay;
    float f3 = 2.0f*(0.5f-q2*q2-q3*q3)     - az;

    float s1 = -2.0f*q3*f1 + 2.0f*q1*f2;
    float s2 =  2.0f*q4*f1 + 2.0f*q2*f2 - 4.0f*q2*f3;
    float s3 = -2.0f*q1*f1 + 2.0f*q3*f2 - 4.0f*q3*f3;
    float s4 =  2.0f*q2*f1 + 2.0f*q4*f2;

    n = inv_sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4);
    s1*=n; s2*=n; s3*=n; s4*=n;

    q1 += (0.5f*(-q2*gx - q3*gy - q4*gz) - beta*s1) * dt;
    q2 += (0.5f*( q1*gx + q3*gz - q4*gy) - beta*s2) * dt;
    q3 += (0.5f*( q1*gy - q2*gz + q4*gx) - beta*s3) * dt;
    q4 += (0.5f*( q1*gz + q2*gy - q3*gx) - beta*s4) * dt;

    n = inv_sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    q_est.q1=q1*n; q_est.q2=q2*n; q_est.q3=q3*n; q_est.q4=q4*n;
}

/* ── Euler angles (ZYX, degrees) ─────────────────────────────────────── */
void eulerAngles(struct quaternion q, float *roll, float *pitch, float *yaw)
{
    float q1=q.q1, q2=q.q2, q3=q.q3, q4=q.q4;

    *roll  = atan2f(2.0f*(q1*q2+q3*q4), 1.0f-2.0f*(q2*q2+q3*q3)) * RAD_TO_DEG;

    float sinp = 2.0f*(q1*q3 - q4*q2);
    *pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp)
                                    : asinf(sinp) * RAD_TO_DEG;

    *yaw   = atan2f(2.0f*(q1*q4+q2*q3), 1.0f-2.0f*(q3*q3+q4*q4)) * RAD_TO_DEG;
}
