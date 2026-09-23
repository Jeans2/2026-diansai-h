#ifndef BSP_LINE_FOLLOW_H
#define BSP_LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_grayscale.h"

/* Channel 0 is physical left, channel 7 is physical right. */
#define LINE_SENSOR_REVERSED       1
#define LINE_MOTOR_A_IS_LEFT       0

/* Speed unit: quadrature encoder counts per 50 ms. */
#define LINE_BASE_SPEED_DEFAULT      5
#define LINE_BASE_SPEED_MIN          5
#define LINE_BASE_SPEED_MAX        1500   //370  300
#define LINE_MAX_TARGET_SPEED      1500

/*
 * Direction outer-loop position-form PD:
 * placeOut = Kp*error + Kd*(error-lastError)
 * filteredOut = A*placeOut + (1-A)*lastOut
 */
#define LINE_PLACE_KP              1.82f   //0.42
#define LINE_PLACE_KD              3.5f   //2.2   2.4         
#define LINE_OUTPUT_FILTER_NEW     0.50f
#define LINE_OUTPUT_FILTER_OLD     0.50f
#define LINE_OUTPUT_LIMIT          10000.0f

typedef struct {
    uint16_t raw[GRAYSCALE_CHANNELS];
    uint16_t contrast;
    uint16_t strength;
    int16_t error;
    int16_t targetA;
    int16_t targetB;
    uint8_t mask;
    bool lineVisible;
} LineFollowState;

void LineFollow_Init(void);
void LineFollow_SetBaseSpeed(int16_t speed);
int16_t LineFollow_GetBaseSpeed(void);
void LineFollow_ReadAndProcess(LineFollowState *state);
void LineFollow_ComputeTargets(LineFollowState *state);
void LineFollow_ClearTargets(LineFollowState *state);

#endif
