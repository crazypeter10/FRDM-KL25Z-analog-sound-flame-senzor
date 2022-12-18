#include "MKL25Z4.h"

void ADC0_Init(void);
int ADC0_Calibrate(void);
uint16_t ADC0_Read(void);
void ADC0_IRQHandler(void);
void switch_channel(void);
void transmit_flame(uint16_t);
void transmit_sound(uint16_t);
void channel_handler(uint16_t);