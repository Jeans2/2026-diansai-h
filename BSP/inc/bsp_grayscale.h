#ifndef BSP_GRAYSCALE_H
#define BSP_GRAYSCALE_H

#include "board.h"

#define GRAYSCALE_CHANNELS  8U

void Grayscale_SelectChannel(uint8_t channel);
uint16_t Grayscale_ReadChannel(uint8_t channel);
void Grayscale_ReadAll(uint16_t values[GRAYSCALE_CHANNELS]);

/* Compatibility helpers retained from the original single-channel driver. */
unsigned int Get_Adc_GRAYSCALE_Value(void);
unsigned int Get_Grayscale_Percentage_Value(void);

#endif
