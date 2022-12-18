#include "Adc.h"
#include "uart.h"
#include <math.h>

#define SOUND_CHANNEL (11) //PORT C PIN 2
#define FLAME_CHANNEL (8) // PORT B PIN 0

#define RED_LED_PIN (18) // PORT B
#define GREEN_LED_PIN (19) // PORT B
#define BLUE_LED_PIN (1) // PORT D


uint8_t state;


static uint8_t switch_channel_flag = 0;

uint8_t receive;
uint8_t switch_rgb_sensor=1;
void ADC0_Init() {
	
	// Activarea semnalului de ceas pentru modulul periferic ADC
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	
	// Functia de calibrare
	ADC0_Calibrate();
	
	ADC0->CFG1 = 0x00;

	// Selectarea modului de conversie pe 16 biti single-ended --> MODE
	// Selectarea sursei de ceas pentru generarea ceasului intern --> ADICLK
	// Selectarea ratei de divizare folosit de periferic pentru generarea ceasului intern --> ADIV
	// Set ADC clock frequency fADCK less than or equal to 4 MHz (PG. 494)
	ADC0->CFG1 |= ADC_CFG1_MODE(3) |
							 ADC_CFG1_ADICLK(0) |
							 ADC_CFG1_ADIV(2);
	
	// DIFF = 0 --> Conversii single-ended (PG. 464)
	ADC0->SC1[0] = 0x00;
	ADC0->SC3 = 0x00;

	// Selectarea modului de conversii continue, 
	// pentru a-l putea folosi in tandem cu mecanismul de intreruperi
	ADC0->SC3 |= ADC_SC3_ADCO_MASK;
	
	// Activarea subsistemului de conversie prin aproximari succesive pe un anumit canal (PG.464)
	ADC0->SC1[0] |= ADC_SC1_ADCH(SOUND_CHANNEL);
	// Enables conversion complete interrupts
	ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;
	
	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_EnableIRQ(ADC0_IRQn);	
}

int ADC0_Calibrate() {
	
	// ===== For best calibration results =====
	
	ADC0_CFG1 |= ADC_CFG1_MODE(3)  |  				 // 16 bits mode
                ADC_CFG1_ADICLK(1)|  // Input Bus Clock divided by 2
                ADC_CFG1_ADIV(3);   // Clock divide by 8
	
	// The calibration will automatically begin if the SC2[ADTRG] is 0. (PG. 495)
	ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;
	
	// Set hardware averaging to maximum, that is, SC3[AVGE]=1 and SC3[AVGS]=0x11 for an average of 32 (PG. 494)
	ADC0->SC3 |= (ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3));
	
	// To initiate calibration, the user sets SC3[CAL] (PG. 495)
	ADC0->SC3 |= ADC_SC3_CAL_MASK;
	
	// At the end of a calibration sequence, SC1n[COCO] will be set (PG. 495)
	while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
	
	// At the end of the calibration routine, if SC3[CALF] is not
	// set, the automatic calibration routine is completed successfully. (PG. 495)
	if(ADC0->SC3 & ADC_SC3_CALF_MASK){
		return (1);
	}
	
	// ====== CALIBRATION FUNCTION (PG.495) =====
	
	// 1. Initialize or clear a 16-bit variable in RAM.
	uint16_t calibration_var = 0x0000;
	
	// 2. Add the plus-side calibration results CLP0, CLP1, CLP2, CLP3, CLP4, and CLPS to the variable.
	calibration_var += ADC0->CLP0;
	calibration_var += ADC0->CLP1;
	calibration_var += ADC0->CLP2;
	calibration_var += ADC0->CLP3;
	calibration_var += ADC0->CLP4;
	calibration_var += ADC0->CLPS;
	
	// 3. Divide the variable by two.
	calibration_var /= 2;
	
	// 4. Set the MSB of the variable. 
	calibration_var |= 0x8000;
	
	// 5. Store the value in the plus-side gain calibration register PG.
	ADC0->PG = ADC_PG_PG(calibration_var);
	
	// 6. Repeat the procedure for the minus-side gain calibration value.
	calibration_var = 0x0000;
	
	calibration_var += ADC0->CLM0;
	calibration_var += ADC0->CLM1;
	calibration_var += ADC0->CLM2;
	calibration_var += ADC0->CLM3;
	calibration_var += ADC0->CLM4;
	calibration_var += ADC0->CLMS;
	
	calibration_var /= 2;
	
	calibration_var |= 0x8000;
	
	ADC0->MG = ADC_MG_MG(calibration_var);
	
	// Incheierea calibrarii
	ADC0->SC3 &= ~ADC_SC3_CAL_MASK;
	
	return (0);
}

//pt mai multi senzori ii dam parametru la functie
uint16_t ADC0_Read(){
	
	// A conversion is initiated following a write to SC1A, with SC1n[ADCH] not all 1's (PG. 485)
	//ADC0->SC1[0] |= ADC_SC1_ADCH(SOUND_CHANNEL);
	
	// ADACT is set when a conversion is initiated
	// and cleared when a conversion is completed or aborted.
	while(ADC0->SC2 & ADC_SC2_ADACT_MASK);
	
	// A conversion is completed when the result of the conversion is transferred 
	// into the data result registers, Rn (PG. 486)
	
	// If the compare functions are disabled, this is indicated by setting of SC1n[COCO]
	// If hardware averaging is enabled, the respective SC1n[COCO] sets only if
	// the last of the selected number of conversions is completed (PG. 486)
	while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
	
	return (uint16_t) ADC0->R[0];
	
}

void switch_channel(){
	if(switch_channel_flag == 0){
		// s-a citit de pe Sound, trecem la flame
		ADC0->SC1[0] |= ADC_SC1_ADCH_MASK;
		ADC0->SC1[0] = 0x00;
		ADC0->SC3 = 0x00;
		
		// Selectarea modului de conversii continue, 
		// pentru a-l putea folosi in tandem cu mecanismul de intreruperi
		ADC0->SC3 |= ADC_SC3_ADCO_MASK;
		
		// Activarea subsistemului de conversie prin aproximari succesive pe un anumit canal (PG.464)
		ADC0->SC1[0] |= ADC_SC1_ADCH(FLAME_CHANNEL);
		
		
		// Enables conversion complete interrupts
		ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;

		switch_channel_flag=1;
	}else{
		// s-a citit de pe flame , trecem la sound

		ADC0->SC1[0] = 0x00;
		ADC0->SC3 = 0x00;
		
		
		// Selectarea modului de conversii continue, 
		// pentru a-l putea folosi in tandem cu mecanismul de intreruperi
		ADC0->SC3 |= ADC_SC3_ADCO_MASK;
		
		// Activarea subsistemului de conversie prin aproximari succesive pe un anumit canal (PG.464)
		ADC0->SC1[0] |= ADC_SC1_ADCH(SOUND_CHANNEL);
		
		
		// Enables conversion complete interrupts
		ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;

		switch_channel_flag=0;
	}
}

void channel_handler(uint16_t measured_voltage){

	if(switch_channel_flag == 0){
		transmit_flame(measured_voltage);
	} else{
		
		transmit_sound(measured_voltage);
	}
}


void transmit_sound(uint16_t analog_input){
	double dbValue = log10(analog_input)*(double)20;
	if(switch_rgb_sensor==0){

		if(dbValue<40){
				GPIOB_PSOR |= (1<<RED_LED_PIN);
				GPIOB_PCOR |= (1<<GREEN_LED_PIN);
				
		}else if(dbValue>=40 && dbValue<= 80){
				GPIOB_PCOR |= (1<<RED_LED_PIN);
				GPIOB_PCOR |= (1<<GREEN_LED_PIN);
		
		}else{
			GPIOB_PCOR |= (1<<RED_LED_PIN);
			GPIOB_PSOR |= (1<<GREEN_LED_PIN);
			
		}
	}
	//uint16_t analog_input = (uint16_t) (ADC0->R[0]/460)+30;
	//float dbValue = log10(analog_input)*(float)20;
	//uint16_t value= analog_input;
	uint8_t c1=(uint8_t)dbValue%10;
	uint8_t c2=(uint8_t)(dbValue/10)%10;
	uint8_t c3=(uint8_t)(dbValue/100)%10;
	uint8_t c4=(uint8_t)(dbValue/1000)%10;
	uint8_t c5=(uint8_t)(dbValue/10000)%10;
	
	UART0_Transmit('S');
	UART0_Transmit(c5+ 0x30);
	UART0_Transmit(c4+ 0x30);
	UART0_Transmit(c3+ 0x30);
	UART0_Transmit(c2+ 0x30);
	UART0_Transmit(c1+ 0x30);

	UART0_Transmit(0x0A);
	UART0_Transmit(0x0D);

}

void transmit_flame(uint16_t analog_input){
	uint16_t value= analog_input/500;
	if(switch_rgb_sensor==1){

		if(value<30){
				GPIOB_PSOR |= (1<<RED_LED_PIN);
				GPIOB_PCOR |= (1<<GREEN_LED_PIN);
	
		}else if(value>=30 && value<= 120){
				GPIOB_PCOR |= (1<<RED_LED_PIN);
				GPIOB_PCOR |= (1<<GREEN_LED_PIN);
				
		}else{
			GPIOB_PCOR |= (1<<RED_LED_PIN);
			GPIOB_PSOR |= (1<<GREEN_LED_PIN);
		
		}
	}
	//double flame= (double)analog_input;
	//uint16_t value= analog_input;
	uint8_t c1=(uint8_t)value%10;
	uint8_t c2=(uint8_t)(value/10)%10;
	uint8_t c3=(uint8_t)(value/100)%10;
	uint8_t c4=(uint8_t)(value/1000)%10;
	uint8_t c5=(uint8_t)(value/10000)%10;
	
	UART0_Transmit('F');
	UART0_Transmit(c5+ 0x30);
	UART0_Transmit(c4+ 0x30);
	UART0_Transmit(c3+ 0x30);
	UART0_Transmit(c2+ 0x30);
	UART0_Transmit(c1+ 0x30);

	UART0_Transmit(0x0A);
	UART0_Transmit(0x0D);
	

}

void ADC0_IRQHandler(){

		
	uint16_t analog_input = (uint16_t) ADC0->R[0];

	channel_handler(analog_input);
	switch_channel();
}
