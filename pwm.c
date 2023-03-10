
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "math.h"

// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
//interrupt void epwm1_isr(void);
interrupt void epwm2_isr(void);
//interrupt void epwm3_isr(void);

void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl();

   EALLOW;
   SysCtrlRegs.HISPCP.all = 0x3;//ADC_MODCLK;	// HSPCLK = SYSCLKOUT/ADC_MODCLK
   EDIS;

// Step 2. Initalize GPIO: 
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example  

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the DSP2833x_EPwm.c file
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEPwm3Gpio();   
   
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP2833x_PieCtrl.c file.
   InitPieCtrl();
   
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
   InitPieVectTable();
 //  InitAdc();
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
   EALLOW;  // This is needed to write to EALLOW protected registers
//  PieVectTable.EPWM1_INT = &epwm1_isr;
   PieVectTable.EPWM2_INT = &epwm2_isr;
//   PieVectTable.EPWM3_INT = &epwm3_isr;
//   PieVectTable.ADCINT = &adc_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

// For this example, only initialize the ePWM

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm1Example();    
   InitEPwm2Example();
   InitEPwm3Example();
   
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;
   
// Step 5. User specific code, enable interrupts:

// Enable CPU INT3 which is connected to EPWM1-3 INT:
 //  IER |= M_INT1;
   IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
 //  PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
 //  PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
 //   PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM
    for(;;)
   {
       asm("          NOP");
   }

} 


interrupt void epwm1_isr(void)
{
   // Update the CMPA and CMPB values
  // update_compare(&epwm1_info);
   EPwm1Regs.CMPA.half.CMPA = 1000; //EPWM1_MIN_CMPA;     // Set compare A value
   EPwm1Regs.CMPB = 1000; //EPWM1_MAX_CMPB;               // Set Compare B value
   
   
   // Clear INT flag for this timer
   EPwm1Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


interrupt void epwm2_isr(void)
{

   // Update the CMPA and CMPB values
  
	EPwm2Regs.CMPA.half.CMPA = 1500;//EPWM3_MIN_CMPA;    // Set compare A value
	   EPwm2Regs.CMPB = 500;
    // EDIS;
   // Clear INT flag for this timer
   EPwm2Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
   EINT;
}


interrupt void epwm3_isr(void)
{

   // Update the CMPA and CMPB values
  // update_compare(&epwm3_info);
   EPwm3Regs.CMPA.half.CMPA = 1500;//EPWM3_MIN_CMPA;    // Set compare A value
   EPwm3Regs.CMPB = 500;

   // Clear INT flag for this timer
   EPwm3Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

void InitEPwm1Example()
{

   // Setup TBCLK
   EPwm1Regs.TBPRD = 5859>>1;//EPWM1_TIMER_TBPRD;           // Set timer period 801 TBCLKs
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter
   
   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = 2700;//EPWM1_MIN_CMPA;     // Set compare A value
   EPwm1Regs.CMPB = EPwm1Regs.TBPRD>>1;//EPWM1_MAX_CMPB;               // Set Compare B value
   
   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = 0x0;//CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = 0x0;//CC_CTR_ZERO;   


   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   EPwm1Regs.AQCTLB.bit.CBU =AQ_CLEAR;             // Set PWM1B on event B, up count
   EPwm1Regs.AQCTLB.bit.CBD =  AQ_SET;           // Clear PWM1B on event B, down count

   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;//DBB_ALL
   EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;//DB_ACTV_LOC
   EPwm1Regs.DBRED = 450.0;//3us
   EPwm1Regs.DBFED = 450.0;

   // Interrupt where we will change the Compare Values
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
 //  EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;// ET_3RD;            // Generate INT on 3rd event   

     
}


void InitEPwm2Example()
{
 // Setup TBCLK
   EPwm2Regs.TBPRD = 5859>>1;//EPWM2_TIMER_TBPRD;           // Set timer period 801 TBCLKs
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter
   
   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = 1000;//EPWM2_MIN_CMPA;     // Set compare A value
   EPwm2Regs.CMPB = 1000;//EPWM2_MIN_CMPB;               // Set Compare B value
   
   // Setup counter mode
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   


   // Set actions
   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on event A, up count
   EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM2A on event B, down count

   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;         // Clear PWM2B on zero
   EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;            // Set PWM2B on period

 EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
 EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;//DBB_ALL
 EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;//DB_ACTV_LOC
 EPwm2Regs.DBRED = 450.0;//3us
 EPwm2Regs.DBFED = 450.0;
   // Interrupt where we will change the Compare Values
   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;//ET_3RD;           // Generate INT on 3rd event   

  
   
}

void InitEPwm3Example(void)
{

   
   // Setup TBCLK
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm3Regs.TBPRD =5859>>1;// EPWM3_TIMER_TBPRD;          // Set timer period
   EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm3Regs.TBCTR = 0x0000;                     // Clear counter
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;      // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadow register load on ZERO
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE =0x00;// CC_CTR_ZERO;
   EPwm3Regs.CMPCTL.bit.LOADBMODE =0x00;// CC_CTR_ZERO;   

  // Set Compare values
   EPwm3Regs.CMPA.half.CMPA =1000;//EPWM3_MIN_CMPA;    // Set compare A value
   EPwm3Regs.CMPB = 1000;//EPWM3_MAX_CMPB;              // Set Compare B value
   
   // Set Actions
   EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM3A on period
   EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM3A on event B, down count

   EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM3A on period
   EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;            // Set PWM3A on event A, up count

   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm3Regs.DBRED = 450;
   EPwm3Regs.DBFED = 450;
  
   // Interrupt where we will change the Compare Values
   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm3Regs.ETPS.bit.INTPRD = ET_1ST; //;           // Generate INT on 3rd event   
    
   
}


//===========================================================================
// No more.
//===========================================================================
