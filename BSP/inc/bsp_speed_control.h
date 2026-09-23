#ifndef BSP_SPEED_CONTROL_H
#define BSP_SPEED_CONTROL_H

#include <stdint.h>

/*
 * Speed loop runs every 5 ms. Displayed target/measured values remain
 * quadrature encoder counts normalized to 50 ms.
 */
#define SPEED_CONTROL_MS            5U
#define SPEED_SAMPLE_MS             50U
#define SPEED_MOTOR_A_COMMAND_SIGN  1
#define SPEED_MOTOR_B_COMMAND_SIGN -1

#define SPEED_PWM_LIMIT             20.0f
#define SPEED_PWM_MAX_RISE_STEP     1
#define SPEED_PWM_MAX_FALL_STEP     4

/* Five overlapping 5 ms samples form one 25 ms speed measurement window. */
#define SPEED_MEASURE_WINDOW        5U
#define SPEED_FILTER_NEW            0.50f
#define SPEED_FILTER_OLD            0.50f

/*
 * Initial gains for a 5 ms incremental speed PI. The PI output is the
 * commanded PWM itself; there is no fixed dead-zone or target feed-forward.
 */
#define SPEED_PI_KP                 6.0f    //
#define SPEED_PI_KI                 1.0f    //

typedef struct {
    float error;
    float lastError;
    float output;
    float lastOutput;
    float kp;
    float ki;
} IncrementalPI;

typedef struct {
    int16_t targetA;
    int16_t targetB;
    int16_t measuredA;
    int16_t measuredB;
    int16_t pwmA;
    int16_t pwmB;
} SpeedControlState;

void SpeedControl_Init(void);
void SpeedControl_SetTargets(int16_t targetA, int16_t targetB);
void SpeedControl_Update(uint32_t elapsedMs);
void SpeedControl_Stop(void);
const SpeedControlState *SpeedControl_GetState(void);

#endif
