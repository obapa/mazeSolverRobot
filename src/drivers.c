#include <tm4c123GH6PM.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"

#define PWM_FREQUENCY 55

void enBlue(void) {

	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntMasterEnable();
    IntEnable(INT_UART3);
    UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
    printString("BLUETOOTH ENABLED\n\r");
}

void enMoto(void) {
	volatile uint32_t ui32Load;
	volatile uint32_t ui32PWMClock;
	volatile uint8_t ui8Adjust;
	ui8Adjust = 83/2;

	//Enable PWM clock
	//run cpu at 40MHz

	//SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

	//enable clock to proper GPIO register
	//enable PWM1 and GPIOD
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//enable alternate functions to appropriate pins
	//Port D pin 0 configured as PWM output pin for motor 0, PWM gen 0
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	//Port D pin 1 configured as PWM output pin for motor 1, PWM gen 0
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	//configure PMCn fields in GPIOPCTL register and assign to them the PWM
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);

	//configure module 1 PWM generator 0
	ui32PWMClock = SysCtlClockGet() / 32;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

	//configure module 2 PWM generator 1
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);

	//final PWM settings and enabling for motor 0
	volatile uint32_t width=ui8Adjust * ui32Load / 50;//23294
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 10);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);

	//final PWM settings and enabling for motor 1
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 10);
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

void enAnal(void) {
	/*Board enabling ADC converter on itself, do not reference IR sensor as there is no ADC converter*/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	/*Enable PORTE for the IR Sensor, reference IRSensor to PE3, then enable interrupt*/
	SysCtlPeripheralEnable(GPIO_PORTE_BASE);
	GPIOPinTypeADC(ADC0_BASE, GPIO_PIN_3);
	GPIOPinTypeADC(ADC0_BASE, GPIO_PIN_2);

	ADCIntEnable(ADC0_BASE, 1);
	ADCSequenceEnable(ADC0_BASE, 1);
	ADCIntEnable(ADC0_BASE, 2);
	ADCSequenceEnable(ADC0_BASE, 2);

	/*IRSensor uses */
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);

	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 2);


	IntMasterEnable();

}

void UARTIntHandler(void) {     uint32_t ui32Status;
     ui32Status = UARTIntStatus(UART3_BASE, true); //get interrupt status
     UARTIntClear(UART3_BASE, ui32Status); //clear the asserted interrupts
     while(UARTCharsAvail(UART3_BASE)) //loop while there are chars
     {
         UARTCharPutNonBlocking(UART3_BASE, UARTCharGetNonBlocking(UART3_BASE));   //echo character
         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
         SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //turn off LED
     }
}

//void enInfr(void){
// Infrared Function Declarations// ------------------------------------------------------//


//Infrared Variable declarations//
int count = 0;
int set = 0;
float light = 6;

int infraRed(void) {
	//UARTCharPutNonBlocking(UART1_BASE, 'y');
	//int stop = 0;
	int inCount = 0;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
	for(set = 0; set<2; set++){
		if(set == 0) {
			GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
			//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_1, 0);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);//GPIO_PIN_3);//write port high
			//int* regA = (int*)(0x40004008) ;
			//*regA = 0xff;
			ROM_SysCtlDelay(1000000);
			//
			//GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);
			/*
			int j = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
			j = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3);
			j = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3);*/
		}else{
			int count = 0;//timerStart()
			GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);

			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) != 0){
				count ++;
			}//timerStop();
			/*
			//if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) == 0) {
				//if(inCount > light) {//6 works
					//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6);
					//Timer_stop(timer1);
	//				printString("paused\n\r");
					//ROM_SysCtlDelay(1000000);//TODO replace with timer stop
					//setSpeed(1,1);
					//
					//GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);
					//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
					//ROM_SysCtlDelay(1000000);
				//}
			//}
			/*else{
				set=0;
				inCount = 0;
				printString("resumed\n\r");
			}
			inCount++;*/

		}
	}
	return count;
}

//}

void enLED(void){
	GPIO_PORTF_DIR_R = (1<<2)|(1<<3);		//make F2/3 outputs
	GPIO_PORTF_DEN_R = (1<<2)|(1<<3);		//enable digital functions
	GPIO_PORTF_DATA_R &= !((1<<2)|(1<<3));	//Turn off LEDs
}

void enTimer(void) {
	uint32_t ui32Period1;
	uint32_t ui32Period2;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);           // enable Timer 2 periph clks
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);        // cfg Timer 2 mode - periodic

	ui32Period1 = (SysCtlClockGet() /20);                     // period = CPU clk div 20 (50ms)
	ui32Period2 = (SysCtlClockGet() /40);                     // period = CPU clk div 40 (100ms)
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period1);         // set Timer 2 period
	TimerLoadSet(TIMER2_BASE, TIMER_B, ui32Period2);

	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);        // enables Timer 2 to interrupt CPU
	TimerIntEnable(TIMER2_BASE, TIMER_TIMB_TIMEOUT);

	TimerEnable(TIMER2_BASE, TIMER_A);                      // enable Timer 2
	TimerEnable(TIMER2_BASE, TIMER_B);

}
