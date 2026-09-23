#include "bsp_encoder.h"

static volatile int32_t gMotorACount;
static volatile int32_t gMotorBCount;
static uint8_t gMotorAPrevious;
static uint8_t gMotorBPrevious;

static const int8_t gQuadratureDelta[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t Encoder_ReadState(uint32_t aPin, uint32_t bPin)
{
    uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, aPin | bPin);
    return (uint8_t) (((pins & aPin) ? 2U : 0U) |
                      ((pins & bPin) ? 1U : 0U));
}

void Encoder_Init(void)
{
    gMotorACount = 0;
    gMotorBCount = 0;
    gMotorAPrevious =
        Encoder_ReadState(ENCODER_MOTOR_A_A_PIN, ENCODER_MOTOR_A_B_PIN);
    gMotorBPrevious =
        Encoder_ReadState(ENCODER_MOTOR_B_A_PIN, ENCODER_MOTOR_B_B_PIN);

    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
        ENCODER_MOTOR_A_A_PIN | ENCODER_MOTOR_A_B_PIN |
        ENCODER_MOTOR_B_A_PIN | ENCODER_MOTOR_B_B_PIN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

int32_t Encoder_GetMotorA(void)
{
    int32_t value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = gMotorACount;
    if (primask == 0U) {
        __enable_irq();
    }
    return value;
}

int32_t Encoder_GetMotorB(void)
{
    int32_t value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = gMotorBCount;
    if (primask == 0U) {
        __enable_irq();
    }
    return value;
}

void Encoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    gMotorACount = 0;
    gMotorBCount = 0;
    if (primask == 0U) {
        __enable_irq();
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT,
        ENCODER_MOTOR_A_A_PIN | ENCODER_MOTOR_A_B_PIN |
        ENCODER_MOTOR_B_A_PIN | ENCODER_MOTOR_B_B_PIN);
    uint8_t current;

    if (status &
        (ENCODER_MOTOR_A_A_PIN | ENCODER_MOTOR_A_B_PIN)) {
        current =
            Encoder_ReadState(ENCODER_MOTOR_A_A_PIN, ENCODER_MOTOR_A_B_PIN);
        /*
         * Motor A's encoder connector has the opposite phase order from the
         * desired vehicle-forward convention, so invert only channel A.
         */
        gMotorACount -=
            gQuadratureDelta[(gMotorAPrevious << 2U) | current];
        gMotorAPrevious = current;
    }

    if (status &
        (ENCODER_MOTOR_B_A_PIN | ENCODER_MOTOR_B_B_PIN)) {
        current =
            Encoder_ReadState(ENCODER_MOTOR_B_A_PIN, ENCODER_MOTOR_B_B_PIN);
        gMotorBCount +=
            gQuadratureDelta[(gMotorBPrevious << 2U) | current];
        gMotorBPrevious = current;
    }

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, status);
}
