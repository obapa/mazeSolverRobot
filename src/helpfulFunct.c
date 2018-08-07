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

int readInputInt(int size);
char * readInputChar(int size);
void SCI_OutUDec(unsigned short n);
char readChar(void);
void printChar(char c);

void SCI_OutUDec(unsigned short n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    SCI_OutUDec(n/10);
    n = n%10;
  }
  UARTCharPut(UART3_BASE, n+'0'); /* n is between 0 and 9 */

}

int readInputInt(int size){
	int i=0;
	char buf;
	int num=0;
	for (i=0; i<size; i++){
		buf = readChar();
	    //if(!strcmp(buf,'\n')) break;
		printChar(buf);
	    num = num*10+((int)buf-48);//TODO watch for enter
	}
	printString("\n\r");
	return num;
}

float pError(int act, int theor){
	return (float)(act-theor)/(float)theor;
}

/*
char *readInputChar(int size){
	int *c = malloc(size+1);
	int i;
	for (i=0; i<size; i++){
		c[i] = readChar();
		printChar(c[i]);
	}
	printString("\n\r");
	return c;
}*/

void printString(char *string){
	while(*string){
		printChar(*(string++));
	}
}
