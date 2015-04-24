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
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"


void delayMS(int ms) { SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ; }

int main(void)
{
    //Variables
    unsigned long increment = 1;	// Rate (step) change of Duty Cycle
    unsigned long DutyCycle = 15;	// The Duty Cycle sent to the PWM
    int maxDutyCycle = 320;			// This number represents the step of th Duty Cycle (when DutyCycle=320 --> Duty Cycle is 100%)
    int minDutyCycle = 1;
    uint32_t ButtonStatus = 0;		// Status of PF4

    // Set clock
   SysCtlClockSet(SYSCTL_SYSDIV_3 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ); 	// Set the system clock (160KHz)

   SysCtlPWMClockSet(SYSCTL_PWMDIV_16);			// Configure PWM Clock to match system

   // Enable the peripherals used by this program.
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	// Eneble Port F for use
   SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // The Tiva Launchpad has two modules (0 and 1). We eneble PWM Module 1
   GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);	// Configure PF1 as an Output
   GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);  	// Configure PF4 as an Input
   GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor for PF4

   // Configure PF2 to work with PWM Module 1 - Generator Block 3
   GPIOPinConfigure(GPIO_PF2_M1PWM6);
   GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

   // Configure PWM Options
   // PWM_GEN_3 Covers M1PWM6 and M1PWM7 (See page 207 4/11/13 DriverLib doc)
   PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

   // Set the Period (expressed in clock ticks)
   PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, maxDutyCycle);

   // Set where to start PWM Duty Cycle
   PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, minDutyCycle);

   // Enable the PWM generator
   PWMGenEnable(PWM1_BASE, PWM_GEN_3);

   // Turn on the Output pins
   PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

   while(1)
   {

	   delayMS(20);
       ButtonStatus = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4); // Read the status of PF4

       if (ButtonStatus <= 10)	// If button is pressed
       {	// Incriment duty cycle
    	   if (DutyCycle < maxDutyCycle){ DutyCycle += increment; }
    	   else DutyCycle = maxDutyCycle;
       }

       else		// If button is NOT pressed
       {	// decrement duty cycle
    	   if (DutyCycle > minDutyCycle){ DutyCycle -= increment; }
           else DutyCycle = minDutyCycle;
       }

       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,DutyCycle);

   }

}


