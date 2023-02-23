#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "math.h"

// Prototype statements for functions found within this file.
//void InitEPwm1Example(void);
//void InitEPwm2Example(void);
//void InitEPwm3Example(void);
//interrupt void epwm1_isr(void);
//interrupt void epwm2_isr(void);
//interrupt void epwm3_isr(void);

//interrupt void adc_isr(void);
float sinne[512];
Uint32 n;
void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl();

  	for(n=0;n<512;n++)
    {  
    sinne[n]= sin(n*(6.283/512));
	//sinne[n]= 0.5;
	} 
	 n=0;

    for(;;)
   {
       asm("          NOP");
   }

} 

