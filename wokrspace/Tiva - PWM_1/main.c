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

uint8_t ui8PinData = 2;
uint32_t ButtonStatus = 0;	//Status of Port F - Pin 4
//int wait = 0;
bool GoUpCycle = false;

void SW1IntHandler(void);
void delayMS(int ms) { SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ; }

int main(void)
{
    //Set the clock
   SysCtlClockSet(SYSCTL_SYSDIV_2 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
   //Configure PWM Clock to match system
   SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

   // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);  // Init PF4 as input
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor for PF4

/*
    //Interupt Enable
	IntEnable(INT_GPIOF);
	IntMasterEnable();
	GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);        // Disable interrupt for PF4 (in case it was enabled)
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);      // Clear pending interrupts for PF4
	GPIOIntRegister(GPIO_PORTF_BASE, SW1IntHandler);     // Register our handler function for port F
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);           // Configure PF4 for falling edge trigger

	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);     // Enable interrupt for PF4

*/
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);

    //Configure PF1,PF2,PF3 Pins as PWM
    //GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    //GPIOPinConfigure(GPIO_PF3_M1PWM7);
    //GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    //PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the Period (expressed in clock ticks)
    //PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 320);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 320);

    //Set PWM duty-50% (Period /2)
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 0);	//Red
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 15);	//Blue
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 0);	//Green

    // Enable the PWM generator
    //PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

    //Fade
    //bool fadeUp = true;
    unsigned long increment = 5;
    //unsigned long pwmNow = 100;
    //unsigned long red = 320;
    unsigned long blue = 15;
    //unsigned long green = 0;
    //bool fade_red = false;
    bool fade_blue = true;
    //bool fade_green = true;


    while(1)
    {


        delayMS(20);

        ButtonStatus = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);

        if (ButtonStatus <= 10)
        {
        	//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ui8PinData);
        	if (blue < 320)
        	{
        		blue += increment;
        	}
        	else blue = 320;
        }

        else
        {
        	//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
        	if (blue>15)
        	{
        		blue -= increment;
        	}
        	else blue = 15;
        }

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,blue);

    }

}


void SW1IntHandler(void)
{
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ui8PinData);
	GoUpCycle = true;

}
/*
void SW1IntHandlerDOWNPWM(void)
{
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
	GoUpCycle = false;
}
*/
