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
#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"

    //Variables
    unsigned long increment = 1;	// Rate (step) change of Duty Cycle
    unsigned long DutyCycle = 2;	// The Duty Cycle sent to the PWM
    int maxDutyCycle = 320;			// This number represents the step of th Duty Cycle (when DutyCycle=320 --> Duty Cycle is 100%)
    int minDutyCycle = 1;
    uint32_t ButtonStatus = 0;		// Status of PF4

    float value0, value1, value2, value3, valueavr = 0.0 ;
	uint32_t ui32ADC0Value[4];

	//Variables for PWM
	int PWM_FREQUENCY=1000;
	volatile uint32_t ui32Load;
	volatile uint32_t ui32PWMClock;
	//volatile uint8_t ui8Adjust = 83;

void delayMS(int ms) { SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ; }

void ConfigGPIO()
{
	   // Enable the peripherals used by this program.
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	// Eneble Port F for use
	   GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);	// Configure PF1 as an Output
	   GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);  	// Configure PF4 as an Input
	   GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor for PF4
}

void ConfigPWM()
{
	   SysCtlPWMClockSet(SYSCTL_PWMDIV_64);			// Configure PWM Clock to match system

	   SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // The Tiva Launchpad has two modules (0 and 1). We eneble PWM Module 1

	   // Configure PF2 to work with PWM Module 1 - Generator Block 3
	   GPIOPinConfigure(GPIO_PF2_M1PWM6);
	   GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

	   //******************************************************************************************
	   /*
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
	   */
	   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	   ui32PWMClock = SysCtlClockGet() / 64;
	   ui32Load = (ui32PWMClock / PWM_FREQUENCY)-1;
	   PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	   PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);

	   PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, minDutyCycle);
	   PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
	   PWMGenEnable(PWM1_BASE, PWM_GEN_3);

}

void ConfigADC12()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	// Enable the ADC0 periphera

	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	// Read Analog signal from PE2
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);

	ADCSequenceEnable(ADC0_BASE, 1);
}


int main(void)
{

    // Set clock
   SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);	// Set system clock


   ConfigGPIO();
   ConfigPWM();
   ConfigADC12();

   while(1)
   {

		ADCIntClear(ADC0_BASE, 1);	// Clear ADC0 interupt flag
		ADCProcessorTrigger(ADC0_BASE, 1);	// Allow to trigger the ADC conversion with software

		while(!ADCIntStatus(ADC0_BASE, 1, false))
		{
		}

		ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
		value0 = ui32ADC0Value[0];
		value1 = ui32ADC0Value[1];
		value2 = ui32ADC0Value[2];
		value3 = ui32ADC0Value[3];

		valueavr = (value0 + value1 + value2 + value3) / 4;
		maxDutyCycle = (valueavr/4096) * 625;

		if(maxDutyCycle <= 3) {maxDutyCycle = 2;}

		// Read if button is pressed or not
		ButtonStatus = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4); // Read the status of PF4

	    if (ButtonStatus <= 10)	// If button is pressed
	    {	// Incriment duty cycle
	      if (DutyCycle < maxDutyCycle){ DutyCycle += increment; }
	      else {DutyCycle -= increment;}
	     // else if (DutyCycle > maxDutyCycle){DutyCycle -= increment;}
	      //else DutyCycle = maxDutyCycle;
	    }
	    else		// If button is NOT pressed
	    {	// decrement duty cycle
	    	DutyCycle = minDutyCycle;
	    }

		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,DutyCycle);
		delayMS(20);

   }

}

