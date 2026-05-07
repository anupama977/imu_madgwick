/*
 * imu_mag_test/src/main.c
 *
 * Target : STM32H7A3ZI-Q (nucleo_h7a3zi_q)
 * Bus    : I2C4 (PF14=SCL, PF15=SDA)
 *
 * Active devices (from overlay):
 *   lsmimu  -> LSM9DS1 accel+gyro  @ 0x6B
 *   imu2    -> MPU6050 accel+gyro  @ 0x68
 *   mag1    -> LSM9DS1 mag         @ 0x1C
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include "madgwick_filter.h"

LOG_MODULE_REGISTER(imu_mag_test, LOG_LEVEL_INF);

/* Device-tree labels — must match your .overlay exactly */
#define LSM_IMU  DT_NODELABEL(lsmimu)  /* LSM9DS1 accel+gyro @ 0x6B */
#define MPU_IMU  DT_NODELABEL(imu2)    /* MPU6050 accel+gyro @ 0x68 */
#define LSM_MAG  DT_NODELABEL(mag1)    /* LSM9DS1 mag        @ 0x1C */

/* Poll rate: 100 ms = 10 Hz */
#define POLL_MS  100U
#define DT_S     (POLL_MS / 1000.0f)

/* ── sensor_value → float ────────────────────────────────────────────── */
static inline float to_float(struct sensor_value v)
{
    return (float)v.val1 + (float)v.val2 * 1e-6f;
}

/* ── Print sensor_value without FPU printf ───────────────────────────── */
static void print_sv(const char *label, struct sensor_value v)
{
    int32_t frac = v.val2 < 0 ? -v.val2 : v.val2;
    printf("%s%s%d.%06d",
           label,
           (v.val1 == 0 && v.val2 < 0) ? "-" : "",
           v.val1, frac);
}

/* ── Read MPU6050 ─────────────────────────────────────────────────────── */
static bool read_mpu(const struct device *dev,
                     float *ax, float *ay, float *az,
                     float *gx, float *gy, float *gz)
{
    struct sensor_value sax, say, saz, sgx, sgy, sgz;

    if (sensor_sample_fetch(dev) < 0) {
        LOG_ERR("MPU6050 fetch failed");
        return false;
    }
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &sax);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &say);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &saz);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_X,  &sgx);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y,  &sgy);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z,  &sgz);

    printf("[MPU6050 @ 0x68]\n");
    print_sv("  Accel X=", sax); print_sv("  Y=", say); print_sv("  Z=", saz);
    printf("  m/s^2\n");
    print_sv("  Gyro  X=", sgx); print_sv("  Y=", sgy); print_sv("  Z=", sgz);
    printf("  rad/s\n");

    *ax = to_float(sax); *ay = to_float(say); *az = to_float(saz);
    *gx = to_float(sgx); *gy = to_float(sgy); *gz = to_float(sgz);
    return true;
}

/* ── Read LSM9DS1 accel + gyro ───────────────────────────────────────── */
static bool read_lsm_imu(const struct device *dev,
                          float *ax, float *ay, float *az,
                          float *gx, float *gy, float *gz)
{
    struct sensor_value sax, say, saz, sgx, sgy, sgz;

    if (sensor_sample_fetch(dev) < 0) {
        LOG_ERR("LSM9DS1 IMU fetch failed");
        return false;
    }
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &sax);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &say);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &saz);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_X,  &sgx);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y,  &sgy);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z,  &sgz);

    printf("[LSM9DS1 IMU @ 0x6B]\n");
    print_sv("  Accel X=", sax); print_sv("  Y=", say); print_sv("  Z=", saz);
    printf("  m/s^2\n");
    print_sv("  Gyro  X=", sgx); print_sv("  Y=", sgy); print_sv("  Z=", sgz);
    printf("  rad/s\n");

    *ax = to_float(sax); *ay = to_float(say); *az = to_float(saz);
    *gx = to_float(sgx); *gy = to_float(sgy); *gz = to_float(sgz);
    return true;
}

/* ── Read LSM9DS1 magnetometer ───────────────────────────────────────── */
static bool read_lsm_mag(const struct device *dev,
                          float *mx, float *my, float *mz)
{
    struct sensor_value smx, smy, smz;

    if (sensor_sample_fetch(dev) < 0) {
        LOG_ERR("LSM9DS1 mag fetch failed");
        return false;
    }
    sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &smx);
    sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &smy);
    sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &smz);

    printf("[LSM9DS1 MAG @ 0x1C]\n");
    print_sv("  Mag   X=", smx); print_sv("  Y=", smy); print_sv("  Z=", smz);
    printf("  Gauss\n");

    *mx = to_float(smx); *my = to_float(smy); *mz = to_float(smz);
    return true;
}

/* ── Main ────────────────────────────────────────────────────────────── */
int main(void)
{
    const struct device *mpu    = DEVICE_DT_GET(MPU_IMU);
    const struct device *lsmimu = DEVICE_DT_GET(LSM_IMU);
    const struct device *lsmmag = DEVICE_DT_GET(LSM_MAG);

    /* Check devices */
    if (!device_is_ready(mpu))    { LOG_ERR("MPU6050 not ready");     return -1; }
    if (!device_is_ready(lsmimu)) { LOG_ERR("LSM9DS1 IMU not ready"); return -1; }
    if (!device_is_ready(lsmmag)) { LOG_ERR("LSM9DS1 MAG not ready"); return -1; }

    LOG_INF("All sensors ready — Madgwick filter at 10 Hz");

    /* Reset filter to identity quaternion */
    madgwick_init();

    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float roll, pitch, yaw;
    uint32_t cycle = 0;

    while (1) {
        printf("\n=== Cycle %u ===\n", cycle++);

        /* Read all sensors. LSM values overwrite MPU if both succeed. */
        read_mpu(mpu, &ax, &ay, &az, &gx, &gy, &gz);
        bool lsm_ok = read_lsm_imu(lsmimu, &ax, &ay, &az, &gx, &gy, &gz);
        bool mag_ok = read_lsm_mag(lsmmag, &mx, &my, &mz);

        /* Feed Madgwick filter */
        if (mag_ok) {
            imu_filter(ax, ay, az, gx, gy, gz, mx, my, mz, DT_S);
        } else {
            LOG_WRN("No mag — 6-DoF fallback (yaw drifts)");
            imu_filter_6dof(ax, ay, az, gx, gy, gz, DT_S);
        }

        /* Print Euler angles (integer + 2 decimal places, no FPU printf) */
        eulerAngles(q_est, &roll, &pitch, &yaw);
        int r_i = (int)roll,  r_f = (int)((roll  < 0 ? -roll  : roll)  * 100) % 100;
        int p_i = (int)pitch, p_f = (int)((pitch < 0 ? -pitch : pitch) * 100) % 100;
        int y_i = (int)yaw,   y_f = (int)((yaw   < 0 ? -yaw   : yaw)   * 100) % 100;
        printf("  Roll=%d.%02d  Pitch=%d.%02d  Yaw=%d.%02d  deg\n",
               r_i, r_f, p_i, p_f, y_i, y_f);

        (void)lsm_ok; /* suppress unused warning */
        k_sleep(K_MSEC(POLL_MS));
    }

    return 0;
}
