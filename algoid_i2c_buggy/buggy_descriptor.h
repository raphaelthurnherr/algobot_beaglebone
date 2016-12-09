/*
 * buggy_descriptor.h
 *
 *  Created on: 25 nov. 2016
 *      Author: raph
 */

#ifndef ALGOIDCOM_BUGGY_DESCRIPTOR_H_
#define ALGOIDCOM_BUGGY_DESCRIPTOR_H_

typedef enum organ{
	UNKNOWN,
	MOTOR_LEFT,
	MOTOR_RIGHT,
	MOTOR_ENCODER_LEFT,
	MOTOR_ENCODER_RIGHT,
	SERVO_0,
	SERVO_1,
	SERVO_2,
	LED_0,
	LED_1,
	LED_2,
	DIN_0,
	DIN_1
}t_organ;

#define BUGGY_STOP 		0
#define BUGGY_FORWARD 	1
#define BUGGY_BACK 		2

#endif /* ALGOIDCOM_BUGGY_DESCRIPTOR_H_ */
