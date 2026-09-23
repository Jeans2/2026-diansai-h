#include "bsp_tb6612.h"

void Motor_Init(void)
{
    /*
     * Explicit push-pull motor-control outputs:
     * - Hi-Z disabled so both high and low levels are actively driven.
     * - High drive strength improves edge integrity at the AT8236 inputs.
     */
    DL_GPIO_initPeripheralOutputFunctionFeatures(
        GPIO_MOTOR_PWM_C0_IOMUX,
        GPIO_MOTOR_PWM_C0_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initPeripheralOutputFunctionFeatures(
        GPIO_MOTOR_PWM_C1_IOMUX,
        GPIO_MOTOR_PWM_C1_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_enableOutput(
        GPIO_MOTOR_PWM_C0_PORT, GPIO_MOTOR_PWM_C0_PIN);
    DL_GPIO_enableOutput(
        GPIO_MOTOR_PWM_C1_PORT, GPIO_MOTOR_PWM_C1_PIN);

    DL_GPIO_initDigitalOutputFeatures(
        MOTOR_DIR_AIN2_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(
        MOTOR_DIR_BIN2_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_clearPins(
        MOTOR_DIR_PORT, MOTOR_DIR_AIN2_PIN | MOTOR_DIR_BIN2_PIN);
    DL_GPIO_enableOutput(
        MOTOR_DIR_PORT, MOTOR_DIR_AIN2_PIN | MOTOR_DIR_BIN2_PIN);
}

static uint16_t Motor_ClampMagnitude(int16_t speed)
{
    int32_t magnitude = speed;

    if (magnitude < 0) {
        magnitude = -magnitude;
    }
    if (magnitude > MOTOR_PWM_MAX) {
        magnitude = MOTOR_PWM_MAX;
    }
    return (uint16_t) magnitude;
}

/*
 * AT8236 truth table used here:
 *   PWM input low,  DIR input low  -> coast
 *   PWM input PWM,  DIR input low  -> forward
 *   PWM input PWM,  DIR input high -> reverse during PWM-low, brake otherwise
 *   PWM input high, DIR input high -> brake
 *
 * SysConfig inverts the PWM output.  Consequently compare=1000 is constant
 * low, compare=0 is constant high, and reverse uses compare=magnitude.
 */
static void Motor_Set(
    int16_t speed, uint32_t directionPin, DL_TIMER_CC_INDEX channel)
{
    uint16_t magnitude = Motor_ClampMagnitude(speed);
    uint16_t compare;

    if (speed == 0) {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, directionPin);
        compare = MOTOR_PWM_MAX;
    } else if (speed > 0) {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, directionPin);
        compare = MOTOR_PWM_MAX - magnitude;
    } else {
        DL_GPIO_setPins(MOTOR_DIR_PORT, directionPin);
        compare = magnitude;
    }

    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST, compare, channel);
}

void Motor_A_Set(int16_t speed)
{
    Motor_Set(speed, MOTOR_DIR_AIN2_PIN, GPIO_MOTOR_PWM_C0_IDX);
}

void Motor_B_Set(int16_t speed)
{
    Motor_Set(speed, MOTOR_DIR_BIN2_PIN, GPIO_MOTOR_PWM_C1_IDX);
}

void Motor_StopAll(void)
{
    Motor_A_Set(0);
    Motor_B_Set(0);
}

void Motor_BrakeAll(void)
{
    DL_GPIO_setPins(
        MOTOR_DIR_PORT, MOTOR_DIR_AIN2_PIN | MOTOR_DIR_BIN2_PIN);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST, 0, GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST, 0, GPIO_MOTOR_PWM_C1_IDX);
}

void TB6612_Motor_Stop(void)
{
    Motor_StopAll();
}

void AO_Control(uint8_t dir, uint32_t speed)
{
    int16_t command;

    if (speed > MOTOR_PWM_MAX) {
        speed = MOTOR_PWM_MAX;
    }
    command = (int16_t) speed;
    Motor_A_Set(dir ? command : -command);
}

void BO_Control(uint8_t dir, uint32_t speed)
{
    int16_t command;

    if (speed > MOTOR_PWM_MAX) {
        speed = MOTOR_PWM_MAX;
    }
    command = (int16_t) speed;
    Motor_B_Set(dir ? command : -command);
}
