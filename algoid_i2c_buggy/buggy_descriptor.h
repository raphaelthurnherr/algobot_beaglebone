/*
 * buggy_descriptor.h
 *
 *  Created on: 25 nov. 2016
 *      Author: raph
 */

#ifndef ALGOIDCOM_BUGGY_DESCRIPTOR_H_
#define ALGOIDCOM_BUGGY_DESCRIPTOR_H_

typedef enum organ{
	MOTOR_LEFT,
	MOTOR_RIGHT,
	MOTOR_ENCODER_LEFT,
	MOTOR_ENCODER_RIGHT
}t_organ;

#define BUGGY_STOP 		0
#define BUGGY_FORWARD 	1
#define BUGGY_BACK 		2

#endif /* ALGOIDCOM_BUGGY_DESCRIPTOR_H_ */
