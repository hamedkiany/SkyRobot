/*################################################
 * PWMLib.h
 *
 *  Created on: 23 oct. 2024
 *      Author: hamed
 *
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
//#include "PWMLib.h"
int activatePWM(uint32_t VelocidadF2, uint32_t VelocidadF3)
{
    //Set the clock
   //SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

   //Configure PWM Clock to match system
   SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

   // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    //Configure PF1,PF2,PF3 Pins as PWM
    //GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE,  GPIO_PIN_2 | GPIO_PIN_3);

    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//    uint32_t ui32Load = (SysCtlClockGet() / 64 / 50) - 1;

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2,  990 * 16 );
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 990 * 16 );

    //Set PWM duty-50% (Period /2)
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,VelocidadF2 * 16 );
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,VelocidadF3 * 16  );

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE,  PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
    return 0;
}

int configPWM1(uint32_t VelocidadF2)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,VelocidadF2);
    return 0;
}

int configPWM2(uint32_t VelocidadF3)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,VelocidadF3);
    return 0;
}
int forward()
{
    configPWM1(55);
    configPWM2(95);
    return 0;
}

int rewind()
{
    configPWM1(95);
    configPWM2(55);

    return 0;
}
int right()
{
    configPWM1(75);
    configPWM2(85);

    return 0;
}
int left()
{
    configPWM1(85);
    configPWM2(70);

    return 0;
}
int stop()
{
    configPWM1(75);
    configPWM2(75);

    return 0;
}

int mover_robot(int32_t c)
{
    //D = (R * (thetaRight+thetaLeft)/2)
    return 0;
}

int girar_robot(int32_t g)
{
    //theta = R/l (tethaLeft - tethaRight) ** que l es destancia de las reudas
    return 0;
}
