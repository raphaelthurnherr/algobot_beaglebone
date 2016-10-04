#define GPIO_MOT_A1	12
#define GPIO_MOT_A2	14
#define GPIO_MOT_B1	16
#define GPIO_MOT_B2	15

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>

#include "hw_ctrl.h"
#include "libs/BBBiolib.h"
#include "libs/BBBiolib_PWMSS.h"

float PWM_HZ =100.0f ;	/* 20KHz*/
float duty_A = 1.0f ; 	/* 1% Duty cycle for PWM 0_A output */
float duty_B = 1.0f ;	/* 1% Duty cycle for PWM 0_B output*/

// initialisation GPIO
int GpioSetup(void);


// ---------------------------------------------------------------------------
// INITIALISATION DES ENTREES/SORTIES DU SYSTEM POUR
// ---------------------------------------------------------------------------
int GpioSetup(void){
	if(iolib_init()){
		return (0);
		//printf("\n hw_ctrl.c->GpioSetup: Initililisation GPIO:: ERREUR\n");
	}
	else{
		iolib_setdir(8,GPIO_MOT_A1, BBBIO_DIR_OUT);
		iolib_setdir(8,GPIO_MOT_A2, BBBIO_DIR_OUT);
		iolib_setdir(8,GPIO_MOT_B1, BBBIO_DIR_OUT);
		iolib_setdir(8,GPIO_MOT_B2, BBBIO_DIR_OUT);
		return (1);
//		printf("\n \n hw_ctrl.c->GpioSetup: Initialisation GPIO: OK\n");
	}
}


// ---------------------------------------------------------------------------
// SETMOTOR
// ---------------------------------------------------------------------------
int setMotor(int motorName, int direction, int ratio){
	// Initialisation variable PWM
	int gpioMotorX1 = 0;
	int gpioMotorX2 = 0;

	// Vérification ratio max et min
	if(ratio > 100)
		ratio = 100;
	if (ratio<0)
		ratio = 0;

	switch(motorName){
		case WHEEL_LEFT : gpioMotorX1=GPIO_MOT_A1;
						  gpioMotorX2=GPIO_MOT_A2;
						  duty_A=ratio;
						  break;
		case WHEEL_RIGHT : gpioMotorX1=GPIO_MOT_B1;
						  gpioMotorX2=GPIO_MOT_B2;
						  duty_B=ratio;
						  break;
		default : return(0);
	}

	// Stop le mouvement des roues (volocity)
	pin_high(8,gpioMotorX1);
	pin_high(8,gpioMotorX2);

	switch(direction){
		case BUGGY_FORWARD : pin_low(8,gpioMotorX1);
							 pin_high(8,gpioMotorX2);
							 break;

		case BUGGY_BACK : 	 pin_high(8,gpioMotorX1);
							 pin_low(8,gpioMotorX2);
							 break;

		case BUGGY_STOP : 	 pin_high(8,gpioMotorX1);
							 pin_high(8,gpioMotorX2);
							 break;

		default :		     break;
	}


	if(BBBIO_PWMSS_Setting(BBBIO_PWMSS1, PWM_HZ ,duty_A , duty_B)){
		BBBIO_ehrPWM_Enable(BBBIO_PWMSS1);
		return (1);
	}
	else
		return(0);
}


// ---------------------------------------------------------------------------
// gpioClose, Termine la prise en charge GPIO
// ---------------------------------------------------------------------------

void GpioClose(void){
	BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
    iolib_free();
}
