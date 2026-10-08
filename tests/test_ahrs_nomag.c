/** @file test_ahrs_nomag.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Tests for the magnetometer-free build of the attitude filter: ahrs.c
 * compiled with AHRS_NO_MAG and linked WITHOUT magnetic_model.c (the build
 * itself is part of the test, an unresolved magnetic_model_* symbol would
 * fail the link).
 *
 * Scenarios:
 *   1. Roll/pitch:  ARS mode levels from a wrong start and finds the gyro
 *                   bias of a rocking, noisy IMU.
 *   2. AHRS mode:   ahrs_init() rejects the heading mode.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "ahrs.h"

#ifndef AHRS_NO_MAG
#error "test_ahrs_nomag.c must be built with -DAHRS_NO_MAG"
#endif

#define GRAVITY     (9.80665f)
#define DEG2RADF(x) ((x) * (float)M_PI / 180.0f)
#define RAD2DEGF(x) ((x)*180.0f / (float)M_PI)

static int fails = 0;

#define CHECK_TRUE(cond, msg)                    \
    do {                                         \
        if (!(cond))                             \
        {                                        \
            printf("  FAIL  %-40s\n", msg);      \
            fails++;                             \
        }                                        \
        else { printf("  ok    %-40s\n", msg); } \
    } while (0)

static uint32_t g_rng = 4711u;

/* Cheap deterministic noise, sum of 12 uniforms is close to a gaussian. */
static float noise(float stddev)
{
    float s = 0.0f;
    for (int i = 0; i < 12; ++i)
    {
        g_rng = g_rng * 1664525u + 1013904223u;
        s += (float)(g_rng >> 8) * (1.0f / 16777216.0f);
    }
    return (s - 6.0f) * stddev;
}

static void scenario_nomag_roll_pitch(void)
{
    printf("\n-- scenario: roll/pitch without magnetometer code --\n");

    const float w1     = 2.0f * (float)M_PI * 0.17f;
    const float w2     = 2.0f * (float)M_PI * 0.13f;
    const float bias_x = DEG2RADF(0.5f);

    ahrs_config_t cfg          = {0};
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_rad[0]        = DEG2RADF(-15.0f); /* truth starts at +10 */
    cfg.rpy_init_rad[1]        = DEG2RADF(10.0f);  /* truth starts at -5 */
    cfg.rpy_init_stddev_rad[0] = DEG2RADF(20.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RADF(20.0f);

    ahrs_t a;
    CHECK_TRUE(ahrs_init(&a, &cfg, 0) == 0, "ARS init accepted");

    float roll_true = 0.0f, pitch_true = 0.0f;
    for (int k = 1; k <= 6000; ++k)
    {
        const float t       = (float)k * 0.01f;
        const float phi     = DEG2RADF(10.0f) + DEG2RADF(20.0f) * sinf(w1 * t);
        const float theta   = DEG2RADF(-5.0f) + DEG2RADF(10.0f) * sinf(w2 * t);
        const float phi_d   = DEG2RADF(20.0f) * w1 * cosf(w1 * t);
        const float theta_d = DEG2RADF(10.0f) * w2 * cosf(w2 * t);

        const float gyr[3] = {phi_d + bias_x + noise(0.002f), cosf(phi) * theta_d + noise(0.002f),
                              -sinf(phi) * theta_d + noise(0.002f)};
        const float acc[3] = {GRAVITY * sinf(theta) + noise(0.05f),
                              -GRAVITY * cosf(theta) * sinf(phi) + noise(0.05f),
                              -GRAVITY * cosf(theta) * cosf(phi) + noise(0.05f)};

        ahrs_update(&a, (ahrs_time_us_t)k * 10000, gyr, acc, NULL, false);
        roll_true  = phi;
        pitch_true = theta;
    }

    float roll, pitch, yaw, bias[3];
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "attitude available");
    CHECK_TRUE(ahrs_get_bias_gyr(&a, bias), "gyro bias available");
    CHECK_TRUE(fabsf(RAD2DEGF(roll - roll_true)) < 1.0f, "roll within 1 deg");
    CHECK_TRUE(fabsf(RAD2DEGF(pitch - pitch_true)) < 1.0f, "pitch within 1 deg");
    CHECK_TRUE(fabsf(RAD2DEGF(bias[0] - bias_x)) < 0.1f, "gyro bias x within 0.1 deg/s");
}

static void scenario_nomag_rejects_ahrs_mode(void)
{
    printf("\n-- scenario: AHRS mode is rejected without magnetometer code --\n");

    ahrs_config_t cfg          = {0};
    cfg.mode                   = AHRS_MODE_AHRS;
    cfg.rpy_init_stddev_rad[0] = DEG2RADF(5.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RADF(5.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RADF(5.0f);

    ahrs_t a;
    CHECK_TRUE(ahrs_init(&a, &cfg, 0) == -1, "AHRS init rejected");
}

int main(void)
{
    scenario_nomag_roll_pitch();
    scenario_nomag_rejects_ahrs_mode();
    printf("\n==== %d failures ====\n", fails);
    return fails == 0 ? 0 : 1;
}
