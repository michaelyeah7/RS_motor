/*################################################
# Hardware PWM proof of concept using
# the Tiva C Launchpad
#
# Started with example code by
# lawrence_jeff found here:
# http://forum.stellarisiti.com/topic/707-using-hardware-pwm-on-tiva-launchpad/
#
# Altered to use code found on section
# 22.3 of the TivaWare Peripheral Driver
# Library User's Guide found here:
# http://www.ti.com/lit/ug/spmu298a/spmu298a.pdf
#
#
# This example pulses three on-board LEDs
#
#################################################*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

/*include for motor driver board*/
#include "drv8323rs.h"

/*includes for uart comm*/
#include <utils/uartstdio.h>
#include <driverlib/uart.h>

/*include for adc*/
#include "driverlib/adc.h"

volatile char HALLA_data;
volatile char HALLB_data;
volatile char HALLC_data;
volatile uint16_t read_data;
volatile unsigned short isense_adc_data[3];
volatile int counter = 0;
volatile uint16_t data_dump[5000];
volatile int global_kill = 0;


void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

void InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void ThreeXModeConfig(void){

//    SysCtlPeripheralEnable(DRV8323RS_ENABLE_PERIPH);
//    GPIOPinTypeGPIOOutput(DRV8323RS_ENABLE_PORT, DRV8323RS_ENABLE_PIN);

    SPIWriteDRV8323(DRV8323RS_DRIVER_CONTROL_REG, 0b0000100000);
    read_data=SPIReadDRV8323(DRV8323RS_DRIVER_CONTROL_REG);
    UARTprintf(read_data);
}

int pwm_config(void)
{
    //Set the clock
   SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

   //Configure PWM Clock to match system
   SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

   // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(DRV8323RS_PWMA_GPIO_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_PWMA_PERIPH);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins

    SysCtlPeripheralEnable(DRV8323RS_PWMB_GPIO_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_PWMB_PERIPH);

    SysCtlPeripheralEnable(DRV8323RS_PWMC_GPIO_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_PWMC_PERIPH);

//    //Configure PF1,PF2,PF3 Pins as PWM
//    //GPIOPinConfigure(GPIO_PF1_M1PWM5);
//    GPIOPinConfigure(GPIO_PF2_M1PWM6);
//    //GPIOPinConfigure(GPIO_PF3_M1PWM7);
//    GPIOPinConfigure(GPIO_PC5_M0PWM7);
//    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //Configure PF2,PB3,PC5 Pins as PWMA,B,C
    GPIOPinConfigure(DRV8323RS_PWMA_PIN_CONFIG);
    GPIOPinTypePWM(DRV8323RS_PWMA_GPIO_PORT, DRV8323RS_PWMA_GPIO_PIN);

    GPIOPinConfigure(DRV8323RS_PWMB_PIN_CONFIG);
    GPIOPinTypeTimer(DRV8323RS_PWMB_GPIO_PORT, DRV8323RS_PWMB_GPIO_PIN);

    GPIOPinConfigure(DRV8323RS_PWMC_PIN_CONFIG);
    GPIOPinTypePWM(DRV8323RS_PWMC_GPIO_PORT, DRV8323RS_PWMC_GPIO_PIN);

    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    PWMGenConfigure(DRV8323RS_PWMA_BASE,DRV8323RS_PWMA_GEN, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    TimerConfigure(DRV8323RS_PWMB_BASE,(TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PWM));

    PWMGenConfigure(DRV8323RS_PWMC_BASE,DRV8323RS_PWMC_GEN, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(DRV8323RS_PWMA_BASE, DRV8323RS_PWMA_GEN, 400);

    TimerLoadSet(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER, 400 -1);
    TimerMatchSet(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER, 398); // PWM
    TimerEnable(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER);

    PWMGenPeriodSet(DRV8323RS_PWMC_BASE, DRV8323RS_PWMC_GEN, 400);

    //Set PWM duty-50% (Period /2)
    PWMPulseWidthSet(DRV8323RS_PWMA_BASE, DRV8323RS_PWMA_OUT,2);

    PWMPulseWidthSet(DRV8323RS_PWMC_BASE, DRV8323RS_PWMC_OUT,2);

    // Enable the PWM generator
    PWMGenEnable(DRV8323RS_PWMA_BASE, DRV8323RS_PWMA_GEN);

    PWMGenEnable(DRV8323RS_PWMC_BASE, DRV8323RS_PWMC_GEN);

    // Turn on the Output pins
    PWMOutputState(DRV8323RS_PWMA_BASE, DRV8323RS_PWMA_OUT_BIT, true);

    PWMOutputState(DRV8323RS_PWMC_BASE, DRV8323RS_PWMC_OUT_BIT, true);

}

int INLPinConfig(void){

   // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(DRV8323RS_INLA_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_INLA_PERIPH));

    SysCtlPeripheralEnable(DRV8323RS_INLB_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_INLB_PERIPH));

    SysCtlPeripheralEnable(DRV8323RS_INLC_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_INLC_PERIPH));

    GPIOPinTypeGPIOOutput(DRV8323RS_INLA_PORT,DRV8323RS_INLA_PIN);
    GPIOPinTypeGPIOOutput(DRV8323RS_INLB_PORT,DRV8323RS_INLB_PIN);
    GPIOPinTypeGPIOOutput(DRV8323RS_INLC_PORT,DRV8323RS_INLC_PIN);

}

void commutate(){
    int pwmwidthset=20;
    if((HALLA_data>0) && (HALLB_data==0) && (HALLC_data==0)){
        //100 +B -A
        GPIOPinWrite(DRV8323RS_INLA_PORT,DRV8323RS_INLA_PIN,DRV8323RS_INLA_PIN);
        GPIOPinWrite(DRV8323RS_INLB_PORT,DRV8323RS_INLB_PIN,DRV8323RS_INLB_PIN);
        GPIOPinWrite(DRV8323RS_INLC_PORT,DRV8323RS_INLC_PIN,0);

        //PWM and GND
        PWMPulseWidthSet(DRV8323RS_PWMA_BASE,DRV8323RS_PWMA_OUT, 2);
        TimerMatchSet(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER, 400-pwmwidthset);
    }
    else if((HALLA_data>0) && (HALLB_data>0) && (HALLC_data==0)){
        //110 +C -A
        //Enable calls
        GPIOPinWrite(DRV8323RS_INLA_PORT,DRV8323RS_INLA_PIN,DRV8323RS_INLA_PIN);
        GPIOPinWrite(DRV8323RS_INLB_PORT,DRV8323RS_INLB_PIN,0);
        GPIOPinWrite(DRV8323RS_INLC_PORT,DRV8323RS_INLC_PIN,DRV8323RS_INLC_PIN);

        //PWM and GND
        PWMPulseWidthSet(DRV8323RS_PWMA_BASE,DRV8323RS_PWMA_OUT, 2);
        PWMPulseWidthSet(DRV8323RS_PWMC_BASE,DRV8323RS_PWMC_OUT, pwmwidthset);
    }
    else if((HALLA_data==0) && (HALLB_data>0) && (HALLC_data==0)){
        //010 +C -B
        //Enable calls
        GPIOPinWrite(DRV8323RS_INLA_PORT,DRV8323RS_INLA_PIN,0);
        GPIOPinWrite(DRV8323RS_INLB_PORT,DRV8323RS_INLB_PIN,DRV8323RS_INLB_PIN);
        GPIOPinWrite(DRV8323RS_INLC_PORT,DRV8323RS_INLC_PIN,DRV8323RS_INLC_PIN);

        //PWM and GND
        TimerMatchSet(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER, 398);
        PWMPulseWidthSet(DRV8323RS_PWMC_BASE,DRV8323RS_PWMC_OUT, pwmwidthset);
    }
    else if((HALLA_data==0) && (HALLB_data>0) && (HALLC_data>0)){
        //011 +A -B
        //Enable calls
        GPIOPinWrite(DRV8323RS_INLA_PORT,DRV8323RS_INLA_PIN,DRV8323RS_INLA_PIN);
        GPIOPinWrite(DRV8323RS_INLB_PORT,DRV8323RS_INLB_PIN,DRV8323RS_INLB_PIN);
        GPIOPinWrite(DRV8323RS_INLC_PORT,DRV8323RS_INLC_PIN,0);

        //PWM and GND
        PWMPulseWidthSet(DRV8323RS_PWMA_BASE,DRV8323RS_PWMA_OUT, pwmwidthset);
        TimerMatchSet(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER, 398);

    }
    else if((HALLA_data==0) && (HALLB_data==0) && (HALLC_data>0)){
        //001 +A -C
        //Enable calls
        GPIOPinWrite(DRV8323RS_INLA_PORT,DRV8323RS_INLA_PIN,DRV8323RS_INLA_PIN);
        GPIOPinWrite(DRV8323RS_INLB_PORT,DRV8323RS_INLB_PIN,0);
        GPIOPinWrite(DRV8323RS_INLC_PORT,DRV8323RS_INLC_PIN,DRV8323RS_INLC_PIN);

        //PWM and GND
        PWMPulseWidthSet(DRV8323RS_PWMA_BASE,DRV8323RS_PWMA_OUT, pwmwidthset);
        PWMPulseWidthSet(DRV8323RS_PWMC_BASE,DRV8323RS_PWMC_OUT, 2);

    }
    else if((HALLA_data>0) && (HALLB_data==0) && (HALLC_data>0)){
        //101 +B -C
        //Enable calls
        GPIOPinWrite(DRV8323RS_INLA_PORT,DRV8323RS_INLA_PIN,0);
        GPIOPinWrite(DRV8323RS_INLB_PORT,DRV8323RS_INLB_PIN,DRV8323RS_INLB_PIN);
        GPIOPinWrite(DRV8323RS_INLC_PORT,DRV8323RS_INLC_PIN,DRV8323RS_INLC_PIN);

        //PWM and GND
        TimerMatchSet(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER, 400-pwmwidthset);
        PWMPulseWidthSet(DRV8323RS_PWMC_BASE,DRV8323RS_PWMC_OUT, 2);

    }
}


void HALLIntHandler(void){
    GPIOIntClear(DRV8323RS_HALLA_PORT,GPIO_PIN_2);
    GPIOIntClear(DRV8323RS_HALLB_PORT,GPIO_PIN_0);
    GPIOIntClear(DRV8323RS_HALLC_PORT,GPIO_PIN_4);
    HALLA_data = GPIOPinRead(DRV8323RS_HALLA_PORT, GPIO_PIN_2);
    HALLB_data = GPIOPinRead(DRV8323RS_HALLB_PORT, GPIO_PIN_0);
    HALLC_data = GPIOPinRead(DRV8323RS_HALLC_PORT, GPIO_PIN_4);

    commutate();
}

void HallSensorConfig(void){

    GPIOIntRegister(DRV8323RS_HALLA_PORT, HALLIntHandler);
    GPIOIntRegister(DRV8323RS_HALLB_PORT, HALLIntHandler);
    GPIOIntRegister(DRV8323RS_HALLC_PORT, HALLIntHandler);

    // Enable the GPIOA peripheral
    SysCtlPeripheralEnable(DRV8323RS_HALLA_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_HALLA_PERIPH)) ;
    GPIOPinTypeGPIOInput(DRV8323RS_HALLA_PORT, GPIO_PIN_2);

    SysCtlPeripheralEnable(DRV8323RS_HALLB_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_HALLB_PERIPH)) ;
    GPIOPinTypeGPIOInput(DRV8323RS_HALLB_PORT, GPIO_PIN_0);

    SysCtlPeripheralEnable(DRV8323RS_HALLC_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_HALLC_PERIPH));
    GPIOPinTypeGPIOInput(DRV8323RS_HALLC_PORT, GPIO_PIN_4);


    GPIOIntTypeSet(DRV8323RS_HALLA_PORT, GPIO_PIN_2, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(DRV8323RS_HALLB_PORT, GPIO_PIN_0, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(DRV8323RS_HALLC_PORT, GPIO_PIN_4, GPIO_BOTH_EDGES);

    GPIOIntEnable(DRV8323RS_HALLA_PORT, GPIO_PIN_2);
    GPIOIntEnable(DRV8323RS_HALLB_PORT, GPIO_PIN_0);
    GPIOIntEnable(DRV8323RS_HALLC_PORT, GPIO_PIN_4);

    return;
}

void config_isense(void){

    SysCtlPeripheralEnable(DRV8323RS_ISENSE_ADC_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_ISENSE_ADC_PERIPH)){}

    SysCtlPeripheralEnable(DRV8323RS_ISENSEA_GPIO_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_ISENSEB_GPIO_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_ISENSEC_GPIO_PERIPH);

    GPIOPinTypeADC(DRV8323RS_ISENSEA_GPIO_PORT, DRV8323RS_ISENSEA_GPIO_PIN);
    GPIOPinTypeADC(DRV8323RS_ISENSEB_GPIO_PORT, DRV8323RS_ISENSEB_GPIO_PIN);
    GPIOPinTypeADC(DRV8323RS_ISENSEC_GPIO_PORT, DRV8323RS_ISENSEC_GPIO_PIN); //this one

    ADCSequenceConfigure(DRV8323RS_ISENSE_ADC_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, DRV8323RS_ISENSEA_ADC_CTL_CH);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, DRV8323RS_ISENSEB_ADC_CTL_CH);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, DRV8323RS_ISENSEC_ADC_CTL_CH | ADC_CTL_IE | ADC_CTL_END );
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 1);

    return;
}

void adc_read_isense(void){
    /*figure out which sample gives us 3 seconds of 15000*/
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false))
            {
            }
    ADCIntClear(ADC0_BASE, 1);
    uint32_t adc_buffer[4];
    ADCSequenceDataGet(ADC0_BASE, 1, adc_buffer); //figure out how to map data from how it comes into array of shorts
    isense_adc_data[0] = (unsigned short) adc_buffer[0]; //A
    isense_adc_data[1] = (unsigned short) adc_buffer[1]; //B
    isense_adc_data[2] = (unsigned short) adc_buffer[2]; //C
}

void timerIntHandler(void){
    //data packing variable
    uint16_t package;

    //sprintf buff for uartprintf outputs
    char out_buff[100];

    //get current hall
    unsigned short curr_hall_state;
    curr_hall_state = HALLA_data << 2 | HALLB_data << 1 | HALLC_data;
    char curr_active_phase;
    //understand which isenseA/B/C to read based on hallstate

    switch (curr_hall_state){
        case 1:
            curr_active_phase = 0; //represents A as high
            adc_read_isense();
            break;
        case 2:
            curr_active_phase = 2; //represents C as high
            adc_read_isense();
            break;
        case 3:
            curr_active_phase = 0; //represents A as high
            adc_read_isense();
        case 4:
            curr_active_phase = 1; //represents B as high
            adc_read_isense();
            break;
        case 5:
            curr_active_phase = 1; //represents B as high
            adc_read_isense();
            break;
        case 6:
            curr_active_phase = 2; //represents C as high
            adc_read_isense();
            break;
    }

    //read new data into global short isense_adc_data[3]

    package = isense_adc_data[curr_active_phase];
    package = ((unsigned short) curr_hall_state << 12) | package;

    //put data into 5000 var array
    //data_dump[counter] = package; //THIS LINE

    sprintf(out_buff, "HS %u \t ISEN %u\n", curr_hall_state , isense_adc_data[curr_active_phase]); //alternate way of getting hallstate and sen data is masking package w (0x7000) and (0x0FFF) respectively (less cheaty)
    UARTprintf(out_buff);
    counter++;


    //end after 5000 samples
    if (counter == 5000){
        global_kill = 1;
    }

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void timer_interrupt_config(){
    //5kHz timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //16bit periodic timer
    TimerLoadSet(TIMER0_BASE, TIMER_A, 3200); //figure out how often
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, timerIntHandler);
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    return;
}


int main(void)
{
    IntMasterEnable();
    InitDRV8323RS();
    ThreeXModeConfig();
    pwm_config();
    HallSensorConfig();
    INLPinConfig();
    InitConsole();
    config_isense();
    timer_interrupt_config();

    //bool flag=false;
    //UARTprintf("hey");
    while(!global_kill){

    }
    IntMasterDisable();
    while(1){}
    /*
    for (ii = 0; ii < 5000; ++ii){
        sprintf(out_buff, "#%d has a value of %u\n", ii, data_dump[ii]);
        UARTprintf(out_buff);
    }*/
}
