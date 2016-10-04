/*
 * hw_ctrl.h
 *
 *  Created on: 11 avr. 2016
 *      Author: raph
 */

#ifndef HWGPIO_HW_CTRL_H_
#define HWGPIO_HW_CTRL_H_

#define BUGGY_STOP 0
#define BUGGY_FORWARD 1
#define BUGGY_BACK 2

#define WHEEL_LEFT 0
#define WHEEL_RIGHT 1

// initialisation GPIO
int GpioSetup(void);
int setMotor(int motorName, int direction, int ratio);

#endif /* HWGPIO_HW_CTRL_H_ */
