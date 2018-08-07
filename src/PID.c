#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Log.h>
#include <xdc/cfg/global.h>


#include <helpfulFunct.h>
#include <com.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.c"//////
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"

unsigned short ADCtoCm(unsigned short n) {//returns distance in cm
// This function converts decimal number
//   of unspecified length as an ASCII string
	//(.00048828125 for 3.3 V, .000805664 for 5 V
	float voltage  = n * 0.000805664;
	unsigned short buf, dist;
	buf = dist = 13*pow(voltage, -1.1);
	/*if(dist >= 10){
		SCI_OutUDec(dist/10);//TODO use other print function
		dist = dist%10;
	}
	SCI_OutUDec(dist);
	*/
	return buf;
}

void SCI_OutUCm(unsigned short n){//print cm
  UARTCharPut(UART3_BASE, n+'0'); /* n is between 0 and 9 */
  UARTCharPut(UART3_BASE, 99);
  UARTCharPut(UART3_BASE, 109);
}

unsigned short * IRDist(void) {

	//int i=0;
    unsigned short dist[2] = {0,0};

   // for(i = 0; i < 3; i++) {//remove
    	uint32_t ADC0Value;
    	//dist[0] = 0;
    	//dist[1] = 0;

    	//TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    	ADCIntClear(ADC0_BASE, 1);
    	ADCProcessorTrigger(ADC0_BASE, 1);
    	while (!ADCIntStatus(ADC0_BASE, 1, false)) {}
    	ADCSequenceDataGet(ADC0_BASE, 1, &ADC0Value);

//    	UARTCharPut(UART0_BASE, 'R');//Right sensor
 //   	SCI_OutUDec(ADC0Value);///////////////////////
    	//UARTCharPut(UART0_BASE, 9);
    	dist[0] = ADCtoCm(ADC0Value);
//    	SCI_OutUCm(dist);
 //   	UARTCharPut(UART0_BASE, 10);
 //   	UARTCharPut(UART0_BASE, 13);

    	//2nd sensor
    	uint32_t ADC1Value;

		ADCIntClear(ADC0_BASE, 2);
		ADCProcessorTrigger(ADC0_BASE, 2);
		while (!ADCIntStatus(ADC0_BASE, 2, false)) {}
		ADCSequenceDataGet(ADC0_BASE, 2, &ADC1Value);

	//	UARTCharPut(UART0_BASE, 'F');//Front sensor
		//SCI_OutUDec(ADC1Value);
	//	UARTCharPut(UART0_BASE, 9);

		dist[1] = ADCtoCm(ADC1Value);
		//SCI_OutUCm(dist);
	//	UARTCharPut(UART0_BASE, 10);
	//	UARTCharPut(UART0_BASE, 13);

   // }

    return dist;//right distance//TODO change to array
}

float i_Temp = 0;			//old integral value
float d_Temp = 0; 		//old derivative value
int PWM_Temp0 = 15000;		//Initial speed value which will store old value; Change 150 to
						//appropriate value

int PWM_Temp1 = 15000;		//Initial speed value which will store old value; Change 150 to
						//appropriate value TODO

void PID_Alg(void) {

	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	const float Kp = 2.40;//2.40	//500.00//Proportional value that will change with experimental results 1.00 if using only P
	const float Ki = 0.006;	//0.006//Integral value that will change with experimental results
	const float Kd = 0.0015;	//0.0015	//Derivative value that will change with experimental results
	int Set_dist = 6;			//Set distance from right wall we are aiming for; Change 5 to
								//appropriate, measured value when robot assembled and in maze
	int iMax = 100;
	int iMin = -100;
	float ErrVal;					//Calculated error value
	float P_Term;					//Calculated proportional value
	float I_Term;					//Calculated integral value
	float D_Term;					//Calculated derivative value
	int new_dist;				//new dist value
	uint32_t PWM_Val0 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_0);			//new PWM value motor0(right)
	uint32_t PWM_Val1 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_1);			//new PWM value motor1(left)
	int PWM_MAX= 22500;
	int PWM_MIN= 7500;

	unsigned short *curDist;

	curDist = IRDist();
	//Swi_post(IRSwi);

	//new_dist = IRDist();//right motor
	//ErrVal = Set_dist - curDist[0];
	ErrVal=pError(curDist[0],Set_dist);
	P_Term = Kp * ErrVal;

	i_Temp += ErrVal;
	if (i_Temp > iMax)
		i_Temp = iMax;
	else if (i_Temp < iMin)
		i_Temp = iMin;
	I_Term = Ki * i_Temp;

	D_Term = Kd * (ErrVal - d_Temp);
	d_Temp = ErrVal;

	PWM_Val0 = PWM_Temp0 - (P_Term + I_Term + D_Term)*PWM_Val0;
	PWM_Val1 = PWM_Temp1 + (P_Term + I_Term + D_Term)*PWM_Val1;
	//PWM_Val0 = PWM_Val0 - (P_Term + I_Term + D_Term)*PWM_Val0;
	//PWM_Val1 = PWM_Val1 + (P_Term + I_Term + D_Term)*PWM_Val1;

	//PWM_Val0 = PWM_Temp0 - (ErrVal)*PWM_Val0;
	//PWM_Val1 = PWM_Temp1 - (ErrVal)*PWM_Val1;

	if (PWM_Val0 > PWM_MAX){//Makes sure PWM doesn't exceed max
		PWM_Val0 = PWM_MAX;
	}else if (PWM_Val0 < PWM_MIN){
		PWM_Val0 = PWM_MIN;
	}if (PWM_Val1 > PWM_MAX){
		PWM_Val1 = PWM_MAX;
	}else if (PWM_Val1 < PWM_MIN){
		PWM_Val1 = PWM_MIN;
	}

	setSpeed(PWM_Val0,PWM_Val1);		//Change adjust_PWM with class that can change PWM
	//PWM_Temp0 = PWM_Val0;
	//PWM_Temp1 = PWM_Val1;			//TODO
	Swi_post(TurnSwi);
	//Swi_post(infraSwi);		//Find black tape and differentiate between thickness
	//Swi_post(collectSwi);		//Collect every 2nd error value in PID_Alg()
}
int temp = 0;
void collectError(void) {		//TODO add to .cfg as a Swi
	TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT);
	int i = 0;
	float ErrorBuf[20];
	ErrorBuf[temp] = ErrVal;
	temp++;
	if(temp == 19) {					//prints when 2 seconds of data collecting passes
		while (i < temp) {
			printString(ErrorBuf[i]);	//TODO change to char * instead of float
			i++;
		}
		temp = 0;
	}


}
void Turn(void) {

	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	unsigned short opening = 12;
	unsigned short wall = 6;
	unsigned short deadend = 8;

	unsigned short rightV;
	unsigned short frontV;
	unsigned short bufV;

	unsigned short *curDist;
	uint32_t PWM_Val0 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_0);			//new PWM value motor0(right)
	uint32_t PWM_Val1 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_1);			//new PWM value motor1(left)
	uint32_t PWM_buf0 = 0;		//Incremental buffer
	uint32_t PWM_buf1 = 0;		//Incremental buffer

	curDist = IRDist();
	rightV = curDist[0];
	frontV = curDist[1];

	if (rightV > opening) {		//Right Turn

		while ((frontV > opening) && (rightV >= wall)) {
			PWM_buf1 += 750;			//buffer for PWM so that PWM_Val1 will go back to
									//	original speed after turn
			PWM_Val1 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_1) + PWM_buf1;
			setSpeed(PWM_Val0, PWM_Val1);

			curDist = IRDist();		//refreshes IR sensor to go through while loop
			rightV = curDist[0];
			frontV = curDist[1];
		}
		PWM_Val1 -= PWM_buf1;
		setSpeed(PWM_Val0, PWM_Val1);
	}

	curDist = IRDist();
	frontV = curDist[1];

	if (frontV <= wall) {		//U Turn
		revMoto1();
		/*while (frontV < deadend) {

			//PWM_buf0 += 50;			//buffer for PWM so that PWM_Val1 will go back to
												//	original speed after turn
			//PWM_buf1 += 100;
			//PWM_Val0 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_0) + PWM_buf0;
			//PWM_Val1 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_1) + PWM_buf1;
			PWM_Val0 = 15000;
			PWM_Val1 = 15000;
			setSpeed(PWM_Val0, PWM_Val1);
			curDist = IRDist();
			//rightV = curDist[0];
			frontV = curDist[1];
		}*/
		//PWM_Val0 -= PWM_buf0;
		//PWM_Val1 -= PWM_buf1;

		while (frontV < opening ) {
			PWM_Val0 = 15000;
			PWM_Val1 = 15000;
			setSpeed(PWM_Val0, PWM_Val1);
			curDist = IRDist();
			frontV = curDist[1];
		}
		while (rightV <= wall) {
			curDist = IRDist();
			rightV = curDist[0];
		}
		forMoto1();
		//setSpeed(PWM_Val0, PWM_Val1);
	}

}
