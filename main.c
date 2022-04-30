

/**
 * main.c
 */
#include <time.h>
#include <device.h>
#include <driverlib.h>
#include "board.h"
#include  <math.h>
#include "F28x_Project.h"
//added .h files
#include "Peripheral_Setup.h" //for ePWM
#include <stdbool.h>
#include <stdio.h>
// Defines ADC
// Defines
//
#define EX_ADC_RESOLUTION       12
// 12 for 12-bit conversion resolution, which supports (ADC_MODE_SINGLE_ENDED)
// Sample on single pin (VREFLO is the low reference)
// Or 16 for 16-bit conversion resolution, which supports (ADC_MODE_DIFFERENTIAL)
// Sample on pair of pins (difference between pins is converted, subject to
// common mode voltage requirements; see the device data manual)

//
// Globals
//
uint16_t adcAResult0;
uint16_t adcAResult1;
uint16_t adcDResult0;
uint16_t adcDResult1;
//
# define LOOP_COUNT 10
#define EPWM_TIMER_TBPRD 1666 //60 kHz
clock_t timeVal;
uint16_t duty_ratio = .3;
uint16_t duty_cycle = 500;
uint16_t temp = 0;////////////////////////////////////////////////////
uint32_t cycles = 1;
uint32_t delay = 100000;// .5 seconds, note: 1000000 = 1 second
//1 CPU clock cycle = 1/200E6 = 5ns. (time for each switching state to execute) //200HHz
uint32_t senseVal = 100; //value range of the sensor GPIO value, used as threshold and trigger for changing cycles, delay, and switching function
// Defines for SCI (UART)
//
// Define AUTOBAUD to use the autobaud lock feature
#define AUTOBAUD 115200
int setSpeed = 0;

//#define _LAUNCHXL_F28379D
//
// Globals
//
uint16_t loopCounter = 0;
//// global variables for ADC
float SensorV1 = 0;//Tracks input current A
float SensorV1max = 0;//Tracks input current A
float SensorV2 = 0;//input current B
float SensorV3 = 0;//input current C
float VoltageV1 = 0;
float VoltageV1max = 0;
float VoltageV2 = 0;
float VoltageV2max = 0;
float VoltageV3 = 0;
float VoltageV3max = 0;
float multiplier1 = 1;
float offset1 = 0;
float multiplier2 = 1;
float offset2 = 0;
//ADC functions
//void readADC(uint16_t sensorData1, uint16_t sensorData2, uint16_t sensorData3, uint16_t sensorData4, uint16_t sensorData5, uint16_t sensorData6);
void initADCs(void);
void initADCSOCs(void);
//
volatile int reset = 0;
Uint16 sState = 0;
void phase_switch(int cycles, long delay, uint16_t rxStatus, uint16_t sensorData1, uint16_t sensorData2, uint16_t sensorData3, uint16_t sensorData4, uint16_t sensorData5, uint16_t sensorData6);

//TRANFORM AND SECTOR DETECTING FUNCTIONS COMMENTED OUT FOR OPENLOOP CONTROL (FOR VALIDATION)
/*
// sector function routines;
void sector1(void);
void sector2(void);
void sector3(void);
void sector4(void);
void sector5(void);
void sector6(void);
//CLARKE

Uint16 V_clarkeA(Uint16 V1, Uint16 V2, Uint16 V3);
Uint16 V_clarkeB(Uint16 V1, Uint16 V2, Uint16 V3);
Uint16 i_clarkeA(Uint16 i1, Uint16 i2, Uint16 i3);
Uint16 i_clarkeB(Uint16 i1, Uint16 i2, Uint16 i3);
//park transform
Uint16 parkP(Uint16 Valpha, Uint16 Vbeta);
Uint16 parkVd(Uint16 Valpha, Uint16 Vbeta);
//Sector State Change//////////////
Uint16 fnV(Uint16 phiV);
Uint16 fnI(Uint16 phiI);
Uint16 deltaV(Uint16 phiV, Uint16 nV);
Uint16 deltaI(Uint16 phiI, Uint16 nV);
///Duty functions
Uint16 D1(Uint16 Vd, Uint16 deltaV, Uint16 deltaI);
Uint16 D2(Uint16 Vd, Uint16 deltaV, Uint16 deltaI);
Uint16 D3(Uint16 Vd, Uint16 deltaV, Uint16 deltaI);
Uint16 D4(Uint16 Vd, Uint16 deltaV, Uint16 deltaI);
Uint16 D0(Uint16 D1, Uint16 D2, Uint16 D3,Uint16 D4);
//Converting Duty Functions EVEN
Uint16 P1_even(Uint16 D1, Uint16 CMPA);
Uint16 P2_even(Uint16 D1, Uint16 D3, Uint16 CMPA);
Uint16 P3_even(Uint16 D1, Uint16 D3,Uint16 D4, Uint16 CMPA);
Uint16 P4_even(Uint16 D1, Uint16 D3,Uint16 D4, Uint16 D2, Uint16 CMPA);
Uint16 P0_even(Uint16 P1, Uint16 P2, Uint16 P3, Uint16 P4);
//Converting Duty Functions ODD
Uint16 P1_odd(Uint16 D1, Uint16 CMPA);
Uint16 P2_odd(Uint16 D1, Uint16 D3, Uint16 CMPA);
Uint16 P3_odd(Uint16 D1, Uint16 D3,Uint16 D4, Uint16 CMPA);
Uint16 P4_odd(Uint16 D1, Uint16 D3,Uint16 D4, Uint16 D2, Uint16 CMPA);
Uint16 P0_odd(Uint16 P1, Uint16 P2, Uint16 P3, Uint16 P4);

Uint16 switchState(Uint16 nV, Uint16 nI);
*/
void Sreset();
void S1(Uint16 i);
void S2(Uint16 i);
void S3(Uint16 i);
void S4(Uint16 i);
void S5(Uint16 i);
void S6(Uint16 i);
void S7(Uint16 i);
void S8(Uint16 i);
void S9(Uint16 i);
void S10(Uint16 i);
void S11(Uint16 i);
void S12(Uint16 i);
void S13(Uint16 i);
void S14(Uint16 i);
void S15(Uint16 i);
void S16(Uint16 i);
void S17(Uint16 i);
void S18(Uint16 i);
//interrupt void adcint1_isr(void);


Uint16 sub = 0;

//
// Main
//
void main(void)
{
    ////

    //SCI (UART)
    uint16_t receivedChar;//
    uint16_t receivedChar2;//
    uint16_t sensorData1 = 0;
    uint16_t sensorData2 = 0;
    uint16_t sensorData3 = 0;
    uint16_t sensorData4 = 0;
    uint16_t sensorData5 = 0;
    uint16_t sensorData6 = 0;

    unsigned char *msg; //char pointer makes an array of chars
    uint16_t rxStatus = 0U; //unsigned int (0u is the base value), 32bit signed:pp
    //
    // Initialize device clock and peripherals
    //
    Device_init();
    Board_init();

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //


//////////////////////////////////////////////////////////////////////////////////////////////////

//OUTPUTS
    Device_initGPIO();
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);

    // configure GPIO2 as trigger for LED on breadboard //SAa
        GPIO_setPadConfig(2U, GPIO_PIN_TYPE_STD);               // defines GPIO as standard input/output (programs pin to be a GPIO pin)
        GPIO_setDirectionMode(2U, GPIO_DIR_MODE_OUT);           // sets pin as an output (sets GPIO pin as output)

        // configure GPIO3 as trigger for LED on breadboard //SAb
        GPIO_setPadConfig(3U, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(3U, GPIO_DIR_MODE_OUT);

        // configure GPIO4 as trigger for LED on breadboard //SAc
        GPIO_setPadConfig(4U, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(4U, GPIO_DIR_MODE_OUT);

        // configure GPIO5 as trigger for LED on breadboard //SBa
        GPIO_setPadConfig(5U, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(5U, GPIO_DIR_MODE_OUT);

        // configure GPIO6 as trigger for LED on breadboard //SBb
        GPIO_setPadConfig(6U, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(6U, GPIO_DIR_MODE_OUT);

        // configure GPIO7 as trigger for LED on breadboard //SBc
        GPIO_setPadConfig(7U, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(7U, GPIO_DIR_MODE_OUT);

        // configure GPIO8 as trigger for LED on breadboard //SCa (orange wire)
        GPIO_setPadConfig(8U, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(8U, GPIO_DIR_MODE_OUT);

        // configure GPIO9 as trigger for LED on breadboard //SCb (red)
        GPIO_setPadConfig(9U, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(9U, GPIO_DIR_MODE_OUT);

        // configure GPIO10 as trigger for LED on breadboard //SCc (brown)
        GPIO_setPadConfig(10U, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(10U, GPIO_DIR_MODE_OUT);
//SCI (UART)
        // Configuration for the SCI Rx pin.
        //
        //DEVICE_GPIO_PIN_SCIRXDA = 43U <-- GPIO pin based on device.h if/for when F28379D is defined/included signed: pp
        GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);//extern void GPIO_setMasterCore(uint32_t pin, GPIO_CoreSelect core);
        GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);//extern void GPIO_setPinConfig(uint32_t pinConfig);
        GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);// pin = 43U , set as input
        GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);// pin43u, set to a floating input
        GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);// No synchronization

        //
        // Configuration for the SCI Tx pin.
        ////pin = 42U
        GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1);
        GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
        GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
        GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
        GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);
/////////
        //
        // Initialize interrupt controller and vector table.
    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();
//SCI (UART)
    // Initialize SCIA and its FIFO.
    // It affects the operating flags of the SCI
    //base address of the memories and peripherals.
    SCI_performSoftwareReset(SCIA_BASE); //SCIA_BASE = 0x00007200U, from hw_memmap.h

    //
    // Configure SCIA for echoback.
    //*****I CHANGED SCI_setConfig baudrate to AUTOBAUD (WAS 9600)********* signed: pp
    //SCI_CONFIG_WLEN_8 = 8 bit data, SCI_CONFIG_STOP_ONE = 1 stop bit, SCI_CONFIG_PAR_NONE = no parity
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, AUTOBAUD, (SCI_CONFIG_WLEN_8 |SCI_CONFIG_STOP_ONE |SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);
   //Send starting Message.
    msg = "\r\n\n\n\nEnter a speed (in RPM):\0";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 29);
///////////////////////////////////////////////////////////////////////////////////
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
       //
       Interrupt_initModule();

       //
       // Initialize the PIE vector table with pointers to the shell Interrupt
       // Service Routines (ISR).
       //
       Interrupt_initVectorTable();

       //
       // Set up ADCs, initializing the SOCs to be triggered by software
       //
       initADCs();
       initADCSOCs();

    EINT;
    ERTM;
    //
    // Enable global Interrupts and higher priority real-time debug events:

        //ERTM;  // Enable Global realtime interrupt DBGM
/*
        CpuTimer0Regs.TCR.bit.TSS = 1;
        CpuTimer0Regs.PRD.all = 1999999;
        CpuTimer0Regs.TCR.bit.TRB = 1;
        CpuTimer0Regs.TCR.bit.TIE = 1;
        CpuTimer0Regs.TCR.bit.TSS = 0;
*/
    for(;;)
    {
     //   sensor_control();
//SCI (UART)
        msg = "\r\nEnter a speed: \0";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 18);
        //

        // Read a character from the FIFO.
        //
        receivedChar = SCI_readCharBlockingFIFO(SCIA_BASE); //receivedChar will be the variable used to adjust speed in switching alg. Signed:PP
        receivedChar2 = SCI_readCharBlockingFIFO(SCIA_BASE);///////
        rxStatus = SCI_getRxStatus(SCIA_BASE);
        if((rxStatus & SCI_RXSTATUS_ERROR) != 0)
        {
            //
            //If Execution stops here there is some error
            //Analyze SCI_getRxStatus() API return value
            //
            ESTOP0;
        }

        //
        // Echo back the character.
        //
        msg = "  You sent: \0";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 13);
        //
        SCI_writeCharBlockingFIFO(SCIA_BASE, receivedChar);
        SCI_writeCharBlockingFIFO(SCIA_BASE, receivedChar2);//////
        setSpeed = (receivedChar-48)*10 + (receivedChar2-48);

//////
        phase_switch(cycles, delay, rxStatus, sensorData1, sensorData2,sensorData3, sensorData4, sensorData5,sensorData6);

    }
}
//SWITCHING LOGIC
  /*A = 1 1 0 0 0 0 1 1 0 0 0 0 //SAa
    B = 0 0 1 1 0 0 0 0 1 1 0 0 //SAb
    C = 0 0 0 0 1 1 0 0 0 0 1 1 //SAc
    D = 0 0 1 0 1 0 0 0 1 0 1 0 //SBa
    E = 1 0 0 0 0 1 1 0 0 0 0 1 //SBb
    F = 0 1 0 1 0 0 0 1 0 1 0 0 //SBc
    G = 0 0 0 1 0 1 0 0 0 1 0 1 //SCa
    H = 0 1 0 0 1 0 0 1 0 0 1 0 //SCb
    I = 1 0 1 0 0 0 1 0 1 0 0 0 //SCc
*/

//
/*void sensor_control()
{
   // sensor logic
}
*/

void phase_switch(int cycles, long delay, uint16_t rxStatus, uint16_t sensorData1, uint16_t sensorData2, uint16_t sensorData3, uint16_t sensorData4, uint16_t sensorData5, uint16_t sensorData6)
{
       //switchState(nV, nI);
    //Uint16 Valpha = V_clarkeA(VoltageV1, VoltageV2, VoltageV3);
    //Uint16 Vbeta = V_clarkeB(VoltageV1, VoltageV2,  VoltageV3);

    //Uint16 ialpha = i_clarkeA(SensorV1, SensorV2, SensorV3);
    //Uint16 ibeta = i_clarkeB(SensorV1, SensorV2, SensorV3);
    //park transform
    //Uint16 phiV = parkP(Valpha, Vbeta);
    //Uint16 Vd = parkVd(Valpha,Vbeta);
    //Uint16 phiI = parkP(ialpha, ibeta);
    //Uint16 Id = parkVd(ialpha,ibeta);
    //Sector State Change//////////////
    //Uint16 nV = fnV(phiV);
    //Uint16 nI = fnI(phiI);
    //**THE MATH ABOVE^ IS UtiLIZED IN FULL CLOSED LOOP MODE
    
    unsigned char *msg; //char pointer makes an array of chars
    //int i = 0;
    for(sState =1; sState<19; sState++){
         //sState = j;
        DEVICE_DELAY_US(delay/10);
         for(sub=0; sub<4;sub++){
//THE SWITCHING STATES ARE CALLED USING A LOOP ITERATING THROUGH EACH STATE (RATHER THAN USING nV & nI)
          if(sState == 1){S1(sub);}
          if(sState == 2){S2(sub);}
          if(sState == 3){S3(sub);}
          if(sState == 4){S4(sub);}
          if(sState == 5){S5(sub);}
          if(sState == 6){S6(sub);}
          if(sState == 7){S7(sub);}
          if(sState == 8){S8(sub);}
          if(sState == 9){S9(sub);}
          if(sState == 10){S10(sub);}
          if(sState == 11){S11(sub);}
          if(sState == 12){S12(sub);}
          if(sState == 13){S13(sub);}
          if(sState == 14){S14(sub);}
          if(sState == 15){S15(sub);}
          if(sState == 16){S16(sub);}
          if(sState == 17){S17(sub);}
          if(sState == 18){S18(sub);}
          //if(GpioDataRegs.GPADAT.bit.GPIO2){GPport2 = 1;}
          //else{GPport2 = 0;}
           DEVICE_DELAY_US(delay);
           Sreset();
           DEVICE_DELAY_US(delay/10);
 //THE SENSOR ADC DIGITAL SIGNALS ARE CONVERTED TO BE DISPLAYED ON THE TERMINAL
           //sensor1
           if((rxStatus & SCI_RXSTATUS_ERROR) != 0){ESTOP0;}
           msg = "\r\nsensorData1: \0 ";
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 16);
           if(sensorData1 > 999){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf(sensorData1/1000)+48);//1000's place
           }
           if(sensorData1 > 99){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData1%1000)/100)+48);//100's place
           }
           if(sensorData1 > 9){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData1%100)/10)+48);//10's place
           }
           SCI_writeCharBlockingFIFO(SCIA_BASE, (sensorData1%10)+48);//1's place
               //
           //sensor2
           if((rxStatus & SCI_RXSTATUS_ERROR) != 0){ESTOP0;}
           msg = "  sensorData2: \0 ";
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 16);
           if(sensorData2 > 999){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf(sensorData2/1000)+48);//1000's place
           }
           if(sensorData2 > 99){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData2%1000)/100)+48);//100's place
           }
           if(sensorData2 > 9){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData2%100)/10)+48);//10's place
           }
           SCI_writeCharBlockingFIFO(SCIA_BASE, (sensorData2%10)+48);//1's place
               //
           //sensor3
           if((rxStatus & SCI_RXSTATUS_ERROR) != 0){ESTOP0;}
           msg = "  sensorData3: \0";
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 16);
           if(sensorData3 > 999){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf(sensorData3/1000)+48);//1000's place
           }
           if(sensorData3 > 99){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData3%1000)/100)+48);//100's place
           }
           if(sensorData3 > 9){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData3%100)/10)+48);//10's place
           }
           SCI_writeCharBlockingFIFO(SCIA_BASE, (sensorData3%10)+48);//1's place

           //sensor4
           if((rxStatus & SCI_RXSTATUS_ERROR) != 0){ESTOP0;}
           msg = "  sensorData4: \0";
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 16);
           if(sensorData4 > 999){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf(sensorData4/1000)+48);//1000's place
           }
           if(sensorData4 > 99){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData4%1000)/100)+48);//100's place
           }
           if(sensorData4 > 9){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData4%100)/10)+48);//10's place
           }
           SCI_writeCharBlockingFIFO(SCIA_BASE, (sensorData4%10)+48);//1's place
           //sensor5
           if((rxStatus & SCI_RXSTATUS_ERROR) != 0){ESTOP0;}
           msg = "  sensorData5: \0";
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 16);
           if(sensorData5 > 999){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf(sensorData5/1000)+48);//1000's place
           }
           if(sensorData5 > 99){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData5%1000)/100)+48);//100's place
           }
           if(sensorData5 > 9){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData5%100)/10)+48);//10's place
           }
           SCI_writeCharBlockingFIFO(SCIA_BASE, (sensorData5%10)+48);//1's place
           //sensor6
           if((rxStatus & SCI_RXSTATUS_ERROR) != 0){ESTOP0;}
           msg = "  sensorData6: \0";
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 16);
           if(sensorData6 > 999){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf(sensorData6/1000)+48);//1000's place
           }
           if(sensorData6 > 99){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData6%1000)/100)+48);//100's place
           }
           if(sensorData6 > 9){
           SCI_writeCharBlockingFIFO(SCIA_BASE, floorf((sensorData6%100)/10)+48);//10's place
           }
           SCI_writeCharBlockingFIFO(SCIA_BASE, (sensorData6%10)+48);//1's place

         }
         //
         // Convert, wait for completion, and store results
         //
         ADC_forceMultipleSOC(ADCA_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5));

         //
         // Wait for ADCA to complete, then acknowledge flag
         //
         while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false)
         {
         }
         ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

         // Convert, wait for completion, and store results
         //
         ADC_forceMultipleSOC(ADCB_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5));

         //
         // Wait for ADCB to complete, then acknowledge flag
         //
         while(ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1) == false)
         {
         }
         ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
         // Convert, wait for completion, and store results
         //
         ADC_forceMultipleSOC(ADCC_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5));

         //
         // Wait for ADCC to complete, then acknowledge flag
         //
         while(ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false)
         {
         }
         ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

         // Convert, wait for completion, and store results
        //
         ADC_forceMultipleSOC(ADCD_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5| ADC_FORCE_SOC6 | ADC_FORCE_SOC7));
         //

         // Wait for ADCD to complete, then acknowledge flag
         //
         while(ADC_getInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1) == false)
         {
         }
         ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);

         //
         // Store results
         //
         //sensorData1 = ADC_readResult(ADCARESULT_BASE, ADCRESULT0);
         sensorData1 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);//AA0 **<<THE FOLLOWING LABLES REPRESEND PIN # ON THE DSP
         sensorData2 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);//AA1
         sensorData3 = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);//AB2
         sensorData5 = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3);//AB4
         sensorData4 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER4);//AC3
         sensorData6 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER5);//AC5
        // sensorData5 = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER6);//D2
        // sensorData6 = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER7);//D3
         //sensorData1 = AdcaResultRegs.ADCRESULT0; //Store ADCA0 result in SensorV1

        SensorV1 = (multiplier1*sensorData1)+ offset1;//Tracks input current A
        if (SensorV1 >= SensorV1max){ SensorV1max = SensorV1;}
        VoltageV1 = (multiplier2*sensorData2)+ offset2;
        if (VoltageV1 >= VoltageV1max){ VoltageV1max = VoltageV1;}
        VoltageV2 =(multiplier2*sensorData3)+ offset2;
        if (VoltageV2 >= VoltageV2max){ VoltageV2max = VoltageV2;}
        VoltageV3 = (multiplier2*sensorData4)+ offset2;
        if (VoltageV3 >= VoltageV3max){ VoltageV3max = VoltageV3;}

         //

         //
     }

}
/*
void InitEPwm1()
{
// Setup TBCLK

//triangle wave is used for center aligned PWM
EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;   // Count up/down (triangle wave) (1/4 the period of 60Hz = 240Hz) (switching at the peaks)
EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;      // Set timer period
EPwm1Regs.TBCTL.bit.HSPCLKDIV =0;    // TBCLK frequency to EPWMCLK frequency
EPwm1Regs.TBCTL.bit.CLKDIV = 0;
EPwm1Regs.CMPA.bit.CMPA = 0;     // Set compare A value
EPwm1Regs.CMPB.bit.CMPB = 0;     // Set compare B value
// Setup shadow register load on ZERO
EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

// Set actions
EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;     // Set PWM1A pin on Zero
EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // Clear PWM1A at CAU

EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
}
*/




// Function to configure and power up ADCs (NOT UTILIZED, INCOMPLETE).USE FOR POTENTIAL CODE OPTIMIZATION
//
/*
void readADC(uint16_t sensorData1, uint16_t sensorData2, uint16_t sensorData3, uint16_t sensorData4, uint16_t sensorData5, uint16_t sensorData6){
    //
    // Convert, wait for completion, and store results
    //
    ADC_forceMultipleSOC(ADCA_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5));

    //
    // Wait for ADCA to complete, then acknowledge flag
    //
    while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false)
    {
    }
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // Convert, wait for completion, and store results
    //
    ADC_forceMultipleSOC(ADCB_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5));

    //
    // Wait for ADCB to complete, then acknowledge flag
    //
    while(ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1) == false)
    {
    }
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
    // Convert, wait for completion, and store results
    //
    ADC_forceMultipleSOC(ADCC_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5));

    //
    // Wait for ADCC to complete, then acknowledge flag
    //
    while(ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false)
    {
    }
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    // Convert, wait for completion, and store results
   //
    ADC_forceMultipleSOC(ADCD_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5| ADC_FORCE_SOC6 | ADC_FORCE_SOC7));
    //

    // Wait for ADCD to complete, then acknowledge flag
    //
    while(ADC_getInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1) == false)
    {
    }
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);

    //
    // Store results
    //
    //sensorData1 = ADC_readResult(ADCARESULT_BASE, ADCRESULT0);
    sensorData1 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);//AA0
    sensorData2 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);//AA1
    sensorData3 = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);//AB2
    sensorData4 = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3);//AB4
    sensorData5 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER4);//AC3
    sensorData6 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER5);//AC5
   // sensorData5 = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER6);//D2
   // sensorData6 = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER7);//D3
    //sensorData1 = AdcaResultRegs.ADCRESULT0; //Store ADCA0 result in SensorV1

    //
    // Software breakpoint. At this point, conversion results are stored in
    // adcAResult0, adcAResult1, adcDResult0, and adcDResult1.
    //
    // Hit run again to get updated conversions.
    //
}

*/
void initADCs(void)
{
    //
    // Set ADCDLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCD_BASE, ADC_CLK_DIV_4_0);
    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCD_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCD_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
#endif

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCD_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);
    ADC_enableConverter(ADCD_BASE);

    DEVICE_DELAY_US(1000);
}

//
// Function to configure SOCs 0 and 1 of ADCs A and C.
//
void initADCSOCs(void)
{
    //
    // Configure SOCs of ADCA
    // - SOC0 will convert pin A0.
    // - SOC1 will convert pin A1.
    // - Both will be triggered by software only.
    // - For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    //   SYSCLK rate) will be used.  For 16-bit resolution, a sampling window
    //   of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    // - NOTE: A longer sampling window will be required if the ADC driving
    //   source is less than ideal (an ideal source would be a high bandwidth
    //   op-amp with a small series resistance). See TI application report
    //   SPRACT6 for guidance on ADC driver design.
    //

#if(EX_ADC_RESOLUTION == 12)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN0, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN1, 15);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN0, 64);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN1, 64);
#endif

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

////// Configure SOCs of ADCB
#if(EX_ADC_RESOLUTION == 12)
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN4, 15);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 64);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN4, 64);
#endif

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    ////// Configure SOCs of ADCC
    #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN3, 15);
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN5, 15);
    #elif(EX_ADC_RESOLUTION == 16)
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN3, 64);
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN5, 64);
    #endif

        //
        // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
        // sure its flag is cleared.
        //
        ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
        ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
        //
          // Configure SOCs of ADCD
          // - SOC0 will convert pin D2.
          // - SOC1 will convert pin D3.
          // - Both will be triggered by software only.
          // - For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
          //   SYSCLK rate) will be used.  For 16-bit resolution, a sampling window
          //   of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
          // - NOTE: A longer sampling window will be required if the ADC driving
          //   source is less than ideal (an ideal source would be a high bandwidth
          //   op-amp with a small series resistance). See TI application report
          //   SPRACT6 for guidance on ADC driver design.
          //

      #if(EX_ADC_RESOLUTION == 12)
          ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_SW_ONLY,
                       ADC_CH_ADCIN2, 15);
          ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_SW_ONLY,
                       ADC_CH_ADCIN3, 15);
      #elif(EX_ADC_RESOLUTION == 16)
          ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_SW_ONLY,
                       ADC_CH_ADCIN2, 64);
          ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_SW_ONLY,
                       ADC_CH_ADCIN3, 64);
      #endif

          //
          // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
          // sure its flag is cleared.
          //
          ADC_setInterruptSource(ADCD_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
          ADC_enableInterrupt(ADCD_BASE, ADC_INT_NUMBER1);
          ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
}
// THE MATH FUNCTIONS BELOW ARE COMMENTED OUT TO RUN THE OPEN LOOP CONFIGURATION (VALIDATION PURPOSES)
 /*
 //CLARKE /////////////////////////
//clarke transform

Uint16 V_clarkeA(Uint16 V1, Uint16 V2, Uint16 V3)
    {
        Uint16 Valpha = sqrt(2/3)*(V1*1 + V2*(-1/2) + V3*(-1/2));
        return Valpha;
    }
Uint16 V_clarkeB(Uint16 V1, Uint16 V2, Uint16 V3)
    {
        Uint16 Vbeta = sqrt(2/3)*(V1*0 + V2*(sqrt(3/2)) + V3*(sqrt(-3/2)));
        return Vbeta;
    }
Uint16 i_clarkeA(Uint16 i1, Uint16 i2, Uint16 i3)
    {
        Uint16 ialpha = sqrt(2/3)*(i1*1 + i2*(-1/2) + i3*(-1/2));
        return ialpha;
    }
Uint16 i_clarkeB(Uint16 i1, Uint16 i2, Uint16 i3)
    {
        Uint16 ibeta = sqrt(2/3)*(i1*0 + i2*(sqrt(3/2)) + i3*(sqrt(-3/2)));
        return ibeta;
    }

//PARK/////////////////////////
//park transform

Uint16 parkP(Uint16 Valpha, Uint16 Vbeta){
    Uint16 phi = atan2(Valpha,Vbeta);
    return phi;
}

Uint16 parkVd(Uint16 Valpha, Uint16 Vbeta){
    Uint16 Vd = (sqrt(3)/2)*sqrt(Valpha*Valpha + Vbeta*Vbeta);
    return Vd;
}

//Sector State Change//////////////
Uint16 fnV(Uint16 phiV){
    Uint16 nV = 0;
    float pi = 3.14159;
    if(phiV >= (-pi/6) && phiV < (pi/6)){nV = 1;}//Sector 1
    if(phiV >= (pi/6) && phiV < (pi/2)){nV = 2;}
    if(phiV >= (pi/2) && phiV < (5*pi/6)){nV = 3;}
    if(phiV >= (5*pi/6) && phiV < (pi)){nV = 4;}
    if(phiV >= (-pi) && phiV < (-5*pi/6)){nV = 4;}
    if(phiV >= (-5*pi) && phiV < (-pi/2)){nV = 5;}
    if(phiV >= (-pi/2) && phiV < (-pi/6)){nV = 6;}

    return nV;
}

Uint16 fnI(Uint16 phiI){
    Uint16 nI = 0;
    float pi = 3.14159;
    if(phiI >= (-pi/6) && phiI < (pi/6)){nI = 1;}
    if(phiI >= (pi/6) && phiI < (pi/2)){nI = 2;}
    if(phiI >= (pi/2) && phiI < (5*pi/6)){nI = 3;}
    if(phiI >= (5*pi/6) && phiI < (pi)){nI = 4;}
    if(phiI >= (-pi) && phiI < (-5*pi/6)){nI = 4;}
    if(phiI >= (-5*pi) && phiI < (-pi/2)){nI = 5;}
    if(phiI >= (-pi/2) && phiI < (-pi/6)){nI = 6;}

    return nI;
}
//converts the phi value found from alpha beta into the delta value for parkeri
Uint16 deltaV(Uint16 phiV, Uint16 nV){
    Uint16 deltaV = 0;
    float pi = 3.14159;
    if(nV ==1){deltaV = phiV + (pi/6);}
    if(nV ==2){deltaV = phiV - (pi/6);}
    if(nV ==3){deltaV = phiV - (pi/2);}
    if((nV ==4) && (phiV > 0)){deltaV = phiV - (5*pi/6);}
    if((nV ==4) && (phiV < 0)){deltaV = phiV + (7*pi/6);}
    if(nV ==5){deltaV = phiV + (5*pi/6);}
    if(nV ==6){deltaV = phiV + (pi/2);}

    return deltaV;
}

Uint16 deltaI(Uint16 phiI, Uint16 nI){
    Uint16 deltaI = 0;
    float pi = 3.14159;
    if(nI ==1){deltaI = phiI + (pi/6);}
    if(nI ==2){deltaI = phiI - (pi/6);}
    if(nI ==3){deltaI = phiI - (pi/2);}
    if((nI ==4) && (phiI > 0)){deltaI = phiI - (5*pi/6);}
    if((nI ==4) && (phiI < 0)){deltaI = phiI + (7*pi/6);}
    if(nI ==5){deltaI = phiI + (5*pi/6);}
    if(nI ==6){deltaI = phiI + (pi/2);}

    return deltaI;
}

///Duty functions
Uint16 D1(Uint16 Vd, Uint16 deltaV, Uint16 deltaI){
    float pi = 3.14159;
    Uint16 D1 = Vd * sin(deltaV) * sin((pi/3)-deltaI);
    return D1;
}
Uint16 D2(Uint16 Vd, Uint16 deltaV, Uint16 deltaI){
    Uint16 D2 = Vd * sin(deltaV) * sin(deltaI);
    return D2;
}
Uint16 D3(Uint16 Vd, Uint16 deltaV, Uint16 deltaI){
    float pi = 3.14159;
    Uint16 D3 = Vd * sin((pi/3)-deltaV) * sin((pi/3)-deltaI);
    return D3;
}
Uint16 D4(Uint16 Vd, Uint16 deltaV, Uint16 deltaI){
    float pi = 3.14159;
    Uint16 D4 = Vd * sin((pi/3)-deltaV) * sin(deltaI);
    return D4;
}
//Configuration zero (helps with swithcing losses), this is the unforced phase null configuration.
Uint16 D0(Uint16 D1, Uint16 D2, Uint16 D3,Uint16 D4 ){
    Uint16 D0 = 0;
    D0 = 1-D1-D2-D3-D4;
    return D0;
}

//Converting Duty Functions EVEN
Uint16 P1_even(Uint16 D1, Uint16 CMPA){
    return D1 >= CMPA;
}
Uint16 P2_even(Uint16 D1, Uint16 D3, Uint16 CMPA){
    return ((D3+D1) >= CMPA) - (D1 >= CMPA);
}
Uint16 P3_even(Uint16 D1, Uint16 D3,Uint16 D4, Uint16 CMPA){
    return (D4+(D3+D1) >= CMPA) - ((D3+D1) >= CMPA);
}
Uint16 P4_even(Uint16 D1, Uint16 D3,Uint16 D4, Uint16 D2, Uint16 CMPA){
    return (D2+D4+(D3+D1) >= CMPA) - ((D4+(D3+D1)) >= CMPA);
}
Uint16 P0_even(Uint16 P1, Uint16 P2, Uint16 P3, Uint16 P4){
    return 1-P4-P3-P2-P1;
}

//Converting Duty Functions ODD
Uint16 P1_odd(Uint16 D1, Uint16 CMPA){
    return D1 >= CMPA;
}
Uint16 P2_odd(Uint16 D1, Uint16 D3, Uint16 CMPA){
    return ((D3+D1) >= CMPA) - (D1 >= CMPA);
}
Uint16 P3_odd(Uint16 D1, Uint16 D3,Uint16 D4, Uint16 CMPA){
    return (D4+(D3+D1) >= CMPA) - ((D3+D1) >= CMPA);
}
Uint16 P4_odd(Uint16 D1, Uint16 D3,Uint16 D4, Uint16 D2, Uint16 CMPA){
    return (D2+D4+(D3+D1) >= CMPA) - ((D4+(D3+D1)) >= CMPA);
}
Uint16 P0_odd(Uint16 P1, Uint16 P2, Uint16 P3, Uint16 P4){
    return 1-P4-P3-P2-P1;
}
*/

//FINAL STATES (GOES INTO CASE VALUE)
Uint16 switchState(Uint16 nV, Uint16 nI){
    Uint16 state = 0;
    if(((nV == 1) && (nI == 1))||((nV == 4) && (nI == 4))){state = 1;}//-3
    if(((nV == 1) && (nI == 2))||((nV == 4) && (nI == 5))){state = 2;}//+2
    if(((nV == 1) && (nI == 3))||((nV == 4) && (nI == 6))){state = 3;}//-1
    if(((nV == 1) && (nI == 4))||((nV == 4) && (nI == 1))){state = 4;}//+3
    if(((nV == 1) && (nI == 5))||((nV == 4) && (nI == 2))){state = 5;}//-2
    if(((nV == 1) && (nI == 6))||((nV == 4) && (nI == 3))){state = 6;}//+1
    //
    if(((nV == 2) && (nI == 1))||((nV == 5) && (nI == 4))){state = 7;}//+9
    if(((nV == 2) && (nI == 2))||((nV == 5) && (nI == 5))){state = 8;}//-8
    if(((nV == 2) && (nI == 3))||((nV == 5) && (nI == 6))){state = 9;}//+7
    if(((nV == 2) && (nI == 4))||((nV == 5) && (nI == 1))){state = 10;}//-9
    if(((nV == 2) && (nI == 5))||((nV == 5) && (nI == 2))){state = 11;}//+8
    if(((nV == 2) && (nI == 6))||((nV == 5) && (nI == 3))){state = 12;}//-7
    //
    if(((nV == 3) && (nI == 1))||((nV == 6) && (nI == 4))){state = 13;}//-6
    if(((nV == 3) && (nI == 2))||((nV == 6) && (nI == 5))){state = 14;}//+5
    if(((nV == 3) && (nI == 3))||((nV == 6) && (nI == 6))){state = 15;}//-4
    if(((nV == 3) && (nI == 4))||((nV == 6) && (nI == 1))){state = 16;}//+6
    if(((nV == 3) && (nI == 5))||((nV == 6) && (nI == 2))){state = 17;}//-5
    if(((nV == 3) && (nI == 6))||((nV == 6) && (nI == 3))){state = 18;}//+4

    return state;
}
*/   
    
//USED TO CLEAR ALL SWITCHES TO PREVENT A SHORT
void Sreset(){
    GPIO_writePin(2U, 0); //SAa
    GPIO_writePin(3U, 0); //SAb
    GPIO_writePin(4U, 0); //SAc
    GPIO_writePin(5U, 0); //SBa
    GPIO_writePin(6U, 0); //SBb
    GPIO_writePin(7U, 0); //SBc
    GPIO_writePin(8U, 0); //SCa
    GPIO_writePin(9U, 0); //SCb
    GPIO_writePin(10U, 0); //SCc
}
//BELOW ARE ALL THE SWITCHING CONFIGURATOINS USED
void S1(Uint16 i){
    switch(i)
//-3
      {

       case 0:
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 1://+1
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://+6
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://-4
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}

void S2(Uint16 i){
    switch(i)
//+2
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 1://-3
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 2://-5
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://+6
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}

void S3(Uint16 i){
    switch(i)
//-1
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://+2
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 2://+4
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://-5
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S4(Uint16 i){
    switch(i)
//+3
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://-1
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://-6
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 3://+4
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S5(Uint16 i){
    switch(i)
//-2
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://+3
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://+5
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 3://-6
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S6(Uint16 i){
    switch(i)
//+1
      {

       case 0:
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://-2
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://-4
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://+5
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S7(Uint16 i){
    switch(i)
//+9
      {

       case 0:
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 1://-7
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://-3
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 3://+1
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S8(Uint16 i){
    switch(i)
//-8
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 1://+9
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 2://+2
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 3://-3
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S9(Uint16 i){
    switch(i)
//+7
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://-8
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 2://-1
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://+2
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S10(Uint16 i){
    switch(i)
//-9
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://+7
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://+3
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://-1
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S11(Uint16 i){
    switch(i)
//+8
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://-5
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://-2
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://+3
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S12(Uint16 i){
    switch(i)
//-7
      {

       case 0:
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://+8
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://+1
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://-2
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S13(Uint16 i){
    switch(i)
//-6
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 1://+4
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://+9
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 3://-7
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S14(Uint16 i){
    switch(i)
//+5
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 1://-6
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 2://-8
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 3://+9
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S15(Uint16 i){
    switch(i)
//-4
      {

       case 0:
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://+5
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;
       case 2://+7
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://-8
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 1); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S16(Uint16 i){
    switch(i)
//+6
      {

       case 0:
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://-4
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://-9
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://+7
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 1); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S17(Uint16 i){
    switch(i)
//-5
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://+6
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://+8
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://-9
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 1); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
void S18(Uint16 i){
    switch(i)
//+4
      {

       case 0:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 1://-5
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 1); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 2://-7
           GPIO_writePin(2U, 1); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 1); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;
       case 3://+8
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 1); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 1); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 1); //SCb
           GPIO_writePin(10U, 0); //SCc
           break;

       default:
           GPIO_writePin(2U, 0); //SAa
           GPIO_writePin(3U, 0); //SAb
           GPIO_writePin(4U, 0); //SAc
           GPIO_writePin(5U, 0); //SBa
           GPIO_writePin(6U, 0); //SBb
           GPIO_writePin(7U, 0); //SBc
           GPIO_writePin(8U, 0); //SCa
           GPIO_writePin(9U, 0); //SCb
           GPIO_writePin(10U, 0); //SCc
       }
}
//CLARKE /////////////////////////
//clarke transform

//PARK/////////////////////////
//park transform


/////interrupt will reset switching process
/*
interrupt void xint1_isr(void)
{
  phase_switch(cycles, delay);
}
*/


//
// End of File

