#ifndef BSP_TB6612_H
#define BSP_TB6612_H

#include "board.h"

/*
 * Compatibility filename/API: the 2026-07-24 expansion board actually uses
 * an AT8236S. Signed speed is -1000..1000; positive is the schematic's
 * forward polarity. Swap a motor's two power wires if its physical direction
 * is opposite.
 */
#define MOTOR_PWM_MAX  1000

void Motor_Init(void);
void Motor_A_Set(int16_t speed);
void Motor_B_Set(int16_t speed);
void Motor_StopAll(void);
void Motor_BrakeAll(void);

void TB6612_Motor_Stop(void);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

#endif
