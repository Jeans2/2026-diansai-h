#include "bsp_grayscale.h"

void Grayscale_SelectChannel(uint8_t channel)
{
    channel &= 0x07U;

    DL_GPIO_clearPins(GRAY_SELECT_PORT,
        GRAY_SELECT_AD0_PIN | GRAY_SELECT_AD1_PIN | GRAY_SELECT_AD2_PIN);

    if (channel & 0x01U) {
        DL_GPIO_setPins(GRAY_SELECT_PORT, GRAY_SELECT_AD0_PIN);
    }
    if (channel & 0x02U) {
        DL_GPIO_setPins(GRAY_SELECT_PORT, GRAY_SELECT_AD1_PIN);
    }
    if (channel & 0x04U) {
        DL_GPIO_setPins(GRAY_SELECT_PORT, GRAY_SELECT_AD2_PIN);
    }

    /* Match the reference code's mux settling time (about 100 us). */
    delay_us(100);
}

uint16_t Grayscale_ReadChannel(uint8_t channel)
{
    Grayscale_SelectChannel(channel);
    return (DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_OUT_PIN) != 0U) ?
        1U : 0U;
}

void Grayscale_ReadAll(uint16_t values[GRAYSCALE_CHANNELS])
{
    uint8_t channel;

    if (values == NULL) {
        return;
    }

    for (channel = 0; channel < GRAYSCALE_CHANNELS; channel++) {
        values[channel] = Grayscale_ReadChannel(channel);
    }
}

unsigned int Get_Adc_GRAYSCALE_Value(void)
{
    return Grayscale_ReadChannel(0);
}

unsigned int Get_Grayscale_Percentage_Value(void)
{
    return Get_Adc_GRAYSCALE_Value() ? 100U : 0U;
}
