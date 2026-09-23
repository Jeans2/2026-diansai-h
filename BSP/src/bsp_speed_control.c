#include "bsp_speed_control.h"

#include "bsp_encoder.h"
#include "bsp_tb6612.h"

static SpeedControlState gSpeed;
static IncrementalPI gMotorAPi;
static IncrementalPI gMotorBPi;
static int32_t gPreviousCountA;
static int32_t gPreviousCountB;
static float gMeasuredA;
static float gMeasuredB;
static int32_t gDeltaHistoryA[SPEED_MEASURE_WINDOW];
static int32_t gDeltaHistoryB[SPEED_MEASURE_WINDOW];
static int32_t gDeltaSumA;
static int32_t gDeltaSumB;
static uint8_t gDeltaHistoryIndex;

static float SpeedControl_ClampOutput(float output)
{
    if (output > SPEED_PWM_LIMIT) {
        output = SPEED_PWM_LIMIT;
    } else if (output < 0.0f) {
        output = 0.0f;
    }
    return output;
}

static int16_t SpeedControl_LimitPwmStep(
    int16_t requested, int16_t previous)
{
    int32_t difference = (int32_t) requested - previous;

    if (difference > SPEED_PWM_MAX_RISE_STEP) {
        return previous + SPEED_PWM_MAX_RISE_STEP;
    }
    if (difference < -SPEED_PWM_MAX_FALL_STEP) {
        return previous - SPEED_PWM_MAX_FALL_STEP;
    }
    return requested;
}

static void SpeedControl_ResetPI(IncrementalPI *pi)
{
    pi->error = 0.0f;
    pi->lastError = 0.0f;
    pi->output = 0.0f;
    pi->lastOutput = 0.0f;
    pi->kp = SPEED_PI_KP;
    pi->ki = SPEED_PI_KI;
}

/*
 * Incremental PI:
 * out(k) = out(k-1)
 *        + Ki*error(k)
 *        + Kp*[error(k)-error(k-1)].
 */
static int16_t SpeedControl_IncrementalPI(
    IncrementalPI *pi, float target, float measured)
{
    float calculatedOutput;

    if (target <= 0.0f) {
        SpeedControl_ResetPI(pi);
        return 0;
    }

    pi->lastError = pi->error;
    pi->error = target - measured;
    pi->lastOutput = pi->output;

    calculatedOutput =
        pi->output +
        pi->ki * pi->error +
        pi->kp * (pi->error - pi->lastError);

    pi->output = SpeedControl_ClampOutput(calculatedOutput);

    return (int16_t) (pi->output + 0.5f);
}

static void SpeedControl_ResetMeasurement(void)
{
    uint8_t index;

    gMeasuredA = 0.0f;
    gMeasuredB = 0.0f;
    gDeltaSumA = 0;
    gDeltaSumB = 0;
    gDeltaHistoryIndex = 0;
    for (index = 0; index < SPEED_MEASURE_WINDOW; index++) {
        gDeltaHistoryA[index] = 0;
        gDeltaHistoryB[index] = 0;
    }
}

void SpeedControl_Init(void)
{
    Motor_Init();
    gPreviousCountA = Encoder_GetMotorA();
    gPreviousCountB = Encoder_GetMotorB();
    SpeedControl_ResetMeasurement();

    gSpeed.targetA = 0;
    gSpeed.targetB = 0;
    gSpeed.measuredA = 0;
    gSpeed.measuredB = 0;
    gSpeed.pwmA = 0;
    gSpeed.pwmB = 0;

    SpeedControl_ResetPI(&gMotorAPi);
    SpeedControl_ResetPI(&gMotorBPi);
    Motor_StopAll();
}

void SpeedControl_SetTargets(int16_t targetA, int16_t targetB)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    gSpeed.targetA = targetA;
    gSpeed.targetB = targetB;
    if (primask == 0U) {
        __enable_irq();
    }
}

void SpeedControl_Update(uint32_t elapsedMs)
{
    int32_t countA;
    int32_t countB;
    int32_t deltaA;
    int32_t deltaB;
    float rawSpeedA;
    float rawSpeedB;
    int16_t requestedPwmA;
    int16_t requestedPwmB;

    if (elapsedMs == 0U) {
        return;
    }

    countA = Encoder_GetMotorA();
    countB = Encoder_GetMotorB();
    deltaA = countA - gPreviousCountA;
    deltaB = countB - gPreviousCountB;
    gPreviousCountA = countA;
    gPreviousCountB = countB;

    /*
     * PI still updates every 5 ms, but speed is estimated from the most
     * recent five samples (25 ms). One encoder pulse therefore changes the
     * normalized measurement by about 2 rather than 10.
     */
    gDeltaSumA -= gDeltaHistoryA[gDeltaHistoryIndex];
    gDeltaSumB -= gDeltaHistoryB[gDeltaHistoryIndex];
    gDeltaHistoryA[gDeltaHistoryIndex] = deltaA;
    gDeltaHistoryB[gDeltaHistoryIndex] = deltaB;
    gDeltaSumA += deltaA;
    gDeltaSumB += deltaB;
    gDeltaHistoryIndex++;
    if (gDeltaHistoryIndex >= SPEED_MEASURE_WINDOW) {
        gDeltaHistoryIndex = 0;
    }

    rawSpeedA =
        (float) gDeltaSumA * (float) SPEED_SAMPLE_MS /
        ((float) SPEED_MEASURE_WINDOW * (float) elapsedMs);
    rawSpeedB =
        (float) gDeltaSumB * (float) SPEED_SAMPLE_MS /
        ((float) SPEED_MEASURE_WINDOW * (float) elapsedMs);

    gMeasuredA =
        SPEED_FILTER_NEW * rawSpeedA +
        SPEED_FILTER_OLD * gMeasuredA;
    gMeasuredB =
        SPEED_FILTER_NEW * rawSpeedB +
        SPEED_FILTER_OLD * gMeasuredB;

    gSpeed.measuredA = (int16_t)
        (gMeasuredA >= 0.0f ? gMeasuredA + 0.5f : gMeasuredA - 0.5f);
    gSpeed.measuredB = (int16_t)
        (gMeasuredB >= 0.0f ? gMeasuredB + 0.5f : gMeasuredB - 0.5f);

    requestedPwmA = SpeedControl_IncrementalPI(
        &gMotorAPi, (float) gSpeed.targetA, gMeasuredA);
    requestedPwmB = SpeedControl_IncrementalPI(
        &gMotorBPi, (float) gSpeed.targetB, gMeasuredB);

    gSpeed.pwmA = SpeedControl_LimitPwmStep(
        requestedPwmA, gSpeed.pwmA);
    gSpeed.pwmB = SpeedControl_LimitPwmStep(
        requestedPwmB, gSpeed.pwmB);

    Motor_A_Set(gSpeed.pwmA * SPEED_MOTOR_A_COMMAND_SIGN);
    Motor_B_Set(gSpeed.pwmB * SPEED_MOTOR_B_COMMAND_SIGN);
}

void SpeedControl_Stop(void)
{
    gSpeed.targetA = 0;
    gSpeed.targetB = 0;
    gSpeed.measuredA = 0;
    gSpeed.measuredB = 0;
    gSpeed.pwmA = 0;
    gSpeed.pwmB = 0;
    SpeedControl_ResetMeasurement();
    SpeedControl_ResetPI(&gMotorAPi);
    SpeedControl_ResetPI(&gMotorBPi);
    Motor_StopAll();
}

const SpeedControlState *SpeedControl_GetState(void)
{
    return &gSpeed;
}
