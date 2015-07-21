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

    //GPIO Variables
    uint32_t ButtonStatus = 0;		// Status of PF4

	//Variables for PWM
	volatile uint32_t ui32Load;	// 624 --
	unsigned long DutyCycle = 2;	// The Duty Cycle sent to the PWM
    int minDutyCycle = 2;			// The smallest value that the PWM will have.
    unsigned long increment = 1;	// Rate (step) change of Duty Cycle
	volatile uint32_t ui32PWMClock;	/// Used to set the PWM frequency
	int PWM_FREQUENCY=1000;	//The frequency of the PWM

	//ADC12 Variables
	float ADC_value0, ADC_value1, ADC_value2, ADC_value3, ADC_valueavr = 0.0 ;
	uint32_t ADC_ValueList[4];
	float ADC_limit0, ADC_limit1, ADC_limit2, ADC_limit3, ADC_limitavr = 0.0 ;
	uint32_t ADC_LimitList[4];
	int Poten_measurement = 2;	// The measurment from the speed potentiometer will be saved here ( range-->[2,624] )

	//Variables used in modulate_Poten_to_DC()
	float Poten_Border = 10;			// This value indicated after which point will the PWM Duty Cycle start incising. Until this value, PWM = 0
	float Duty_Cycle_Start = 30;		// What is the smallest duty cycle of the PWM after it passes Poten_Border.
	bool limits_calculated = true;
	float min_DC_Start, min_Poten_border, new_max_DC, new_max_Poten, Poten_local_value, result  = 0.0;
	double Poten_measurement_100 = 0.0;

void delayMS(int ms) { SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ; }

int modulate_Poten_to_DC()
{
	Poten_measurement_100 = ((float)Poten_measurement/(float)ui32Load) * 100.0;

	if(Poten_measurement_100 <= Poten_Border) {result = minDutyCycle;}	// If the potentiometer is very low, give a steady small duty cycle
	else
	{
		if(limits_calculated == true)
		{
			min_DC_Start = (Duty_Cycle_Start/100) * (float)ui32Load;
			min_Poten_border = (Poten_Border/100) * (float)ui32Load;

			new_max_DC = (float)ui32Load - min_DC_Start;
			new_max_Poten = (float)ui32Load - min_Poten_border;

			limits_calculated = false;
		}
		Poten_local_value = (float)Poten_measurement - min_Poten_border;

		result = ((new_max_DC * Poten_local_value) / new_max_Poten ) +  min_DC_Start;
		//((Poten_measurement_100-Poten_Border)/(100-Poten_Border))  ;
	}
	return((int)result);
}

void ConfigGPIO()
{
	// Enable the peripherals used by this program.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	// Eneble Port F for use
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);	// Configure PF1 as an Output	//**************-_-*************
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
	   PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, Poten_measurement);

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
    //** Stage 0 _ START -- Do some configurations

   SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);	// Set system clock
   ConfigGPIO();
   ConfigPWM();
   ConfigADC12();

   //** Stage 0 _ END --

   while(1)
   {
	    //** Stage 1 _ START -- Read speed potentiometer with ADC12
		ADCIntClear(ADC0_BASE, 1);	// Clear ADC0 interupt flag
		ADCProcessorTrigger(ADC0_BASE, 1);	// Allow to trigger the ADC conversion with software

		while(!ADCIntStatus(ADC0_BASE, 1, false))	//Wait for ADC to make a measurement
		{
		}

		ADCSequenceDataGet(ADC0_BASE, 1, ADC_ValueList);	//From which ADC to read from
		ADC_value0 = ADC_ValueList[0];	// 1st measurement
		ADC_value1 = ADC_ValueList[1];	// 2nd measurement
		ADC_value2 = ADC_ValueList[2];	// 3rd measurement
		ADC_value3 = ADC_ValueList[3];	// 4th measurement

		ADC_valueavr = (ADC_value0 + ADC_value1 + ADC_value2 + ADC_value3) / 4;	// calculate the mean value of the measurements
		Poten_measurement = (ADC_valueavr/4096) * (ui32Load + 1);	// Calculate the potentiometer measurement - ui32Load is 624 for 1 KHz (under the specific clock frequencies)
		//** Stage 1 _ END --

		//****** Stage 2 _ START --
		ButtonStatus = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);	// Read if LIFE-button (PF4) is pressed or not.

		Poten_measurement = modulate_Poten_to_DC();	// The Duty Cycle and Potentiometer limits.

	    if (ButtonStatus <= 10)	// If button is pressed
	    {	// Duty cycle will follow the potentiometer

	      if (DutyCycle < Poten_measurement){ DutyCycle += increment; }
	      else {DutyCycle -= increment;}
	     // else if (DutyCycle > Poten_measurement){DutyCycle -= increment;}
	      //else DutyCycle = Poten_measurement;
	    }
	    else		// If button is NOT pressed
	    {
	    	DutyCycle = minDutyCycle;	// make duty cycle almost zero
	    }

		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,DutyCycle);	// Send PWM command
		delayMS(2);	// wait


   }

}
