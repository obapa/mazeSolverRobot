#include <drivers.h>
#include <main.h>
#include <tm4c123GH6PM.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include <PID.h>

#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"

int lookUpCom (char *com);
void analoRead(void);
void revMoto0(void);
void revMoto1(void);
void forMoto0(void);
void forMoto1(void);
void setSpeed(int speed0, int speed1);

int lookUpCom (char *com){
	int i=405;
	*com = tolower(*com);

	if (!strcmp(com,"ex"))
		i=0;
	else if (!strcmp(com,"r0"))
		i=1;
	else if (!strcmp(com,"r1"))
		i=2;
	else if (!strcmp(com,"f0"))
		i=3;
	else if (!strcmp(com,"f1"))
		i=4;
	else if (!strcmp(com,"ar"))
		i=5;
	else if (!strcmp(com,"ss"))
		i=6;
	else if (!strcmp(com,"pa"))
		i=7;
	else if (!strcmp(com,"ir"))
		i=8;

	return i;

}

void analoRead(void){
	uint32_t ADC0Value;
	    	unsigned short dist = 0;
	    	unsigned short dist1 = 0;

	    	ADCIntClear(ADC0_BASE, 1);
	    	ADCProcessorTrigger(ADC0_BASE, 1);
	    	while (!ADCIntStatus(ADC0_BASE, 1, false)) {}
	    	ADCSequenceDataGet(ADC0_BASE, 1, &ADC0Value);

	    	UARTCharPut(UART3_BASE, 'F');
	    	//SCI_OutUDec(ADC0Value);
	    	//UARTCharPut(UART0_BASE, 9);
	    	dist = ADCtoCm(ADC0Value);
	    	//SCI_OutUCm(dist);
	    	UARTCharPut(UART3_BASE, 10);
	    	UARTCharPut(UART3_BASE, 13);

	    	/*2nd sensor*/
	    	uint32_t ADC1Value;

			ADCIntClear(ADC0_BASE, 2);
			ADCProcessorTrigger(ADC0_BASE, 2);
			while (!ADCIntStatus(ADC0_BASE, 2, false)) {}
			ADCSequenceDataGet(ADC0_BASE, 2, &ADC1Value);

			UARTCharPut(UART3_BASE, 'R');
			SCI_OutUDec(ADC1Value);
			UARTCharPut(UART3_BASE, 9);

			dist1 = ADCtoCm(ADC1Value);
			//SCI_OutUCm(dist1);
			UARTCharPut(UART3_BASE, 10);
			UARTCharPut(UART3_BASE, 13);
}

void revMoto0(){
	GPIO_PORTF_DATA_R |= (1<<2);
}

void revMoto1(){
	GPIO_PORTF_DATA_R |= (1<<3);
}

void forMoto0(){
	GPIO_PORTF_DATA_R &= ~(1<<2);
}

void forMoto1(){
	GPIO_PORTF_DATA_R &= ~(1<<3);
}

void setSpeed(int speed0, int speed1){//motor0=right, motor1=left
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, speed0);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, speed1);
}
