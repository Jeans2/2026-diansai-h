#include "bsp_line_follow.h"

#include <stddef.h>

/*
 * Non-linear symmetric positions. A single active center channel is treated
 * as almost straight, while the outer channels still provide strong turning
 * correction. The two center channels together continue to average to zero.
 */
static const int16_t gSensorPosition[GRAYSCALE_CHANNELS] = {            // -350, -250, -130, -10, 10, 130, 250, 350
    -120, -60, -40, -10, 800, 1600, 2000, 2400                          //-290, -180, -120, -50, 50, 120, 180, 290
};

static int16_t gLastVisibleError;
static int16_t gBaseSpeed = LINE_BASE_SPEED_DEFAULT;
static float gPlaceError;
static float gPlaceLastError;
static float gPlaceOutput;
static float gPlaceLastOutput;

static int16_t LineFollow_ClampTarget(int32_t target)
{
    if (target > LINE_MAX_TARGET_SPEED) {
        target = LINE_MAX_TARGET_SPEED;
    } else if (target < 0) {
        target = 0;
    }
    return (int16_t) target;
}

void LineFollow_Init(void)
{
    gLastVisibleError = 0;
    gPlaceError = 0.0f;
    gPlaceLastError = 0.0f;
    gPlaceOutput = 0.0f;
    gPlaceLastOutput = 0.0f;
}

void LineFollow_SetBaseSpeed(int16_t speed)
{
    if (speed < LINE_BASE_SPEED_MIN) {
        speed = LINE_BASE_SPEED_MIN;
    } else if (speed > LINE_BASE_SPEED_MAX) {
        speed = LINE_BASE_SPEED_MAX;
    }
    gBaseSpeed = speed;
}

int16_t LineFollow_GetBaseSpeed(void)
{
    return gBaseSpeed;
}

void LineFollow_ReadAndProcess(LineFollowState *state)
{
    int32_t positionSum = 0;
    uint8_t hitCount = 0;
    uint8_t channel;

    if (state == NULL) {
        return;
    }

    Grayscale_ReadAll(state->raw);
    state->mask = 0;

    for (channel = 0; channel < GRAYSCALE_CHANNELS; channel++) {
        if (state->raw[channel] != 0U) {
            state->mask |= (uint8_t) (1U << channel);
            positionSum += gSensorPosition[channel];
            hitCount++;
        }
    }

    state->lineVisible = (hitCount != 0U);
    state->strength = hitCount;
    state->contrast = hitCount;

    if (state->lineVisible) {
        state->error = (int16_t) (positionSum / hitCount);
#if LINE_SENSOR_REVERSED
        state->error = -state->error;
#endif
        gLastVisibleError = state->error;
    } else {
        state->error = gLastVisibleError;
    }
}

void LineFollow_ComputeTargets(LineFollowState *state)
{
    float rawOutput;
    int32_t correction;
    int32_t leftTarget;
    int32_t rightTarget;

    if (state == NULL) {
        return;
    }

    /*
     * Direction outer-loop from the user's place_pid template.
     * There is no gyro term because this car has no gyroscope.
     */
    gPlaceLastOutput = gPlaceOutput;
    gPlaceLastError = gPlaceError;
    gPlaceError = (float) state->error;

    rawOutput =
        LINE_PLACE_KP * gPlaceError +
        LINE_PLACE_KD * (gPlaceError - gPlaceLastError);
    gPlaceOutput =
        rawOutput * LINE_OUTPUT_FILTER_NEW +
        gPlaceLastOutput * LINE_OUTPUT_FILTER_OLD;

    if (gPlaceOutput > LINE_OUTPUT_LIMIT) {
        gPlaceOutput = LINE_OUTPUT_LIMIT;
    } else if (gPlaceOutput < -LINE_OUTPUT_LIMIT) {
        gPlaceOutput = -LINE_OUTPUT_LIMIT;
    }

    correction = (int32_t)
        (gPlaceOutput >= 0.0f ?
            gPlaceOutput + 0.5f : gPlaceOutput - 0.5f);

    leftTarget = (int32_t) gBaseSpeed + correction;
    rightTarget = (int32_t) gBaseSpeed - correction;

#if LINE_MOTOR_A_IS_LEFT
    state->targetA = LineFollow_ClampTarget(leftTarget);
    state->targetB = LineFollow_ClampTarget(rightTarget);
#else
    state->targetA = LineFollow_ClampTarget(rightTarget);
    state->targetB = LineFollow_ClampTarget(leftTarget);
#endif
}

void LineFollow_ClearTargets(LineFollowState *state)
{
    if (state != NULL) {
        state->targetA = 0;
        state->targetB = 0;
    }
}
