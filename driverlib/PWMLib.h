/*
 * PWMLib.h
 *
 *  Created on: 23 oct. 2024
 *      Author: hamed
 */

#ifndef DRIVERLIB_PWMLIB_H_
#define DRIVERLIB_PWMLIB_H_
#include <stdint.h>

extern int activatePWM(uint32_t VelocidadF2, uint32_t VelocidadF3);

extern int configPWM1(uint32_t VelocidadF2);

extern int configPWM2(uint32_t VelocidadF3);
extern int forward();
extern int rewind();
extern int right();
extern int left();
extern int mover_robot(uint32_t c);
extern int girar_robot(uint32_t g);

#endif /* DRIVERLIB_PWMLIB_H_ */
