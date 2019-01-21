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


#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "drv8323rs.h"

void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

void HALLIntHandler(void);

int generate_pwm(void)
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

    TimerLoadSet(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER, 1000 -1);
    TimerMatchSet(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER, 250); // PWM
    TimerEnable(DRV8323RS_PWMB_BASE, DRV8323RS_PWMB_TIMER);

    PWMGenPeriodSet(DRV8323RS_PWMC_BASE, DRV8323RS_PWMC_GEN, 400);

    //Set PWM duty-50% (Period /2)
    PWMPulseWidthSet(DRV8323RS_PWMA_BASE, DRV8323RS_PWMA_OUT,80);

    PWMPulseWidthSet(DRV8323RS_PWMC_BASE, DRV8323RS_PWMC_OUT,80);

    // Enable the PWM generator
    PWMGenEnable(DRV8323RS_PWMA_BASE, DRV8323RS_PWMA_GEN);

    PWMGenEnable(DRV8323RS_PWMC_BASE, DRV8323RS_PWMC_GEN);

    // Turn on the Output pins
    PWMOutputState(DRV8323RS_PWMA_BASE, DRV8323RS_PWMA_OUT_BIT, true);

    PWMOutputState(DRV8323RS_PWMC_BASE, DRV8323RS_PWMC_OUT_BIT, true);

};

int INLPinConfig(void){
    //Set the clock
   SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

   // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(DRV8323RS_INLA_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_INLB_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_INLC_PERIPH);

    GPIOPinTypeGPIOOutput(DRV8323RS_INLA_PORT,DRV8323RS_INLA_PIN);
    GPIOPinTypeGPIOOutput(DRV8323RS_INLB_PORT,DRV8323RS_INLB_PIN);
    GPIOPinTypeGPIOOutput(DRV8323RS_INLC_PORT,DRV8323RS_INLC_PIN);

}

int HallSensorConfig(void){
    // Enable the GPIOA peripheral
    SysCtlPeripheralEnable(DRV8323RS_HALLA_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_HALLB_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_HALLC_PERIPH);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
        {
        }

    GPIOIntRegister(DRV8323RS_HALLA_PORT, HALLIntHandler);
    GPIOIntRegister(DRV8323RS_HALLB_PORT, HALLIntHandler);
    GPIOIntRegister(DRV8323RS_HALLC_PORT, HALLIntHandler);

    GPIOPinTypeGPIOInput(DRV8323RS_HALLA_PORT, GPIO_PIN_2);
    GPIOPinTypeGPIOInput(DRV8323RS_HALLB_PORT, GPIO_PIN_0);
    GPIOPinTypeGPIOInput(DRV8323RS_HALLC_PORT, GPIO_PIN_4);

    GPIOIntTypeSet(DRV8323RS_HALLA_PORT, GPIO_PIN_2, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(DRV8323RS_HALLB_PORT, GPIO_PIN_0, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(DRV8323RS_HALLC_PORT, GPIO_PIN_4, GPIO_BOTH_EDGES);

    GPIOIntEnable(DRV8323RS_HALLA_PORT, GPIO_PIN_2);
    GPIOIntEnable(DRV8323RS_HALLB_PORT, GPIO_PIN_0);
    GPIOIntEnable(DRV8323RS_HALLC_PORT, GPIO_PIN_4);

}

void HALLIntHandler(void){
    int HALLA_data = GPIOPinRead(DRV8323RS_HALLA_PORT, GPIO_PIN_2);
    int HALLB_data = GPIOPinRead(DRV8323RS_HALLB_PORT, GPIO_PIN_0);
    int HALLC_data = GPIOPinRead(DRV8323RS_HALLC_PORT, GPIO_PIN_4);

    //commutate(HALLA_data,HALLB_data,HALLC_data);
}

void commutate(int HALLA_data,int HALLB_data,int HALLC_data){
return 0;
}
