#include "Adc.h"
#include "uart.h"
#include "gpio.h"
extern char read_c;
int main() {
	
	initUART0(115200);
	ADC0_Init();
	RGBLed_Init();
	for(;;) {

	}
	
}
