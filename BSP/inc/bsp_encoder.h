#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "board.h"

void Encoder_Init(void);
int32_t Encoder_GetMotorA(void);
int32_t Encoder_GetMotorB(void);
void Encoder_Reset(void);

#endif
