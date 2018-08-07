#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Log.h>
#include <xdc/cfg/global.h>

#include <drivers.h>
#include <com.h>
#include <PID.h>
#include <helpfulFunct.h>

#include <tm4c123GH6PM.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "inc/hw_memmap.h"

#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
//#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
//#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

char readChar(void);
void printChar(char c);

int main(void) {
	enBlue();
	ROM_SysCtlDelay(1000000);
	printString("Starting Motor\n\r");
	enMoto();
	printString("Motor Started\n\r");
	ROM_SysCtlDelay(1000000);
	printString("Starting Distance Sensors\n\r");
	enAnal();
	printString("Distance Sensors Started\n\r");


	//configure LED pins
	SYSCTL_RCGCGPIO_R |= (1<<5);			//enable clock on PortF
	enLED();	//Set GPIO portF for motor direction controls
	enTimer();
	BIOS_start();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	printString("YOU ARE CONNECTED...\n\r\n");


    char c[] = "na";
    //char* c;
    int i=0;
    volatile uint32_t mSpeed=0;
    int cont = 1;//run prog
    // short * temp;
    //temp = IRDist();
//PID_Alg();
	while(cont)
	{
		printString(":");
		//TODO make the function
		for (i=0; i<2; i++){
				c[i] = readChar();
				printChar(c[i]);
			}
			printString("\n\r");

		switch(lookUpCom(c))
		{
		case 0:		//EX:	exit
			cont = 0;
			setSpeed(1,1);
			break;
		case 1:		//R0: reverse motor 0 direction(Blue LED on)
			revMoto0();	break;
		case 2:		//R1: reverse motor 1 direction(Green LED on)
			revMoto1();	break;
		case 3:		//F0: forward motor 0 direction(Blue LED off)
			forMoto0();	break;
		case 4:		//F1: forward motor 1 direction(Green LED off)
			forMoto1();	break;
		case 5:		//AR: analog read
			analoRead();	break;
		case 6:		//SS: set speed of motors
			printString("Motor speed (xxxxx): ");
			mSpeed = (uint32_t)(readInputInt(5));
			setSpeed(mSpeed,mSpeed);	break;
		case 7:		//PA: run PID algorithm
			/*while(1) {
				PID_Alg();
				ROM_SysCtlDelay(500000);

				Turn(); //Added but not tested
			}*/
			enTimer();

			BIOS_start();
			break;
		case 8://IR: take IR reading
			infraRed();
			break;
		case 405:
		default:
			//while(*)
			//printString()
			break;
		}

	}
}

char readChar(void){
	char c;
	while((UART3_FR_R & (1<<4)) != 0);//check UART receive is empty
	c = UART3_DR_R;
	return c;
}

void printChar(char c){
	while((UART3_FR_R & (1<<5)) != 0);//check UART transmit is full
	UART3_DR_R = c;
}


