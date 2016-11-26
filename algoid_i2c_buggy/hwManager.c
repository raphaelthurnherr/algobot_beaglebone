/*
 * hwMagager.c
 *
 *  Created on: 26 nov. 2016
 *      Author: raph
 */

#include "pthread.h"
#include <unistd.h>
#include "hwManager.h"
#include "hwControl/boardHWctrl.h"

// Thread Messager
pthread_t th_hwManager;


typedef struct tsensors{
	unsigned char din0;
	unsigned char din1;
	int usonic;
	int battery;
	float left_wheel;
	float right_wheel;
}t_sensor;


t_sensor buggySensor;
// ------------------------------------------
// Programme principale TIMER
// ------------------------------------------
void *hwTask (void * arg){

	// Reset la distance de la carte EFM8BB
	clearWheelDistance(0);
	clearWheelDistance(1);

	while(1){

		buggySensor.battery = getBatteryVoltage();							// Lecture de la tension batterie
		buggySensor.din0 = getDigitalInput(0);
		buggySensor.din1 = getDigitalInput(1);
		buggySensor.usonic = getSonarDistance()/10;
		buggySensor.left_wheel = getPulseCounter(0)*0.285;
		buggySensor.right_wheel = getPulseCounter(1)*0.285;

		printf("\n[hwManager] Battery: %dmV ultrasonic: %dcm DIN0: %d  DIN1: %d Left: %.1f  Right: %.1f \n", buggySensor.battery, buggySensor.usonic,
				buggySensor.din0, buggySensor.din1, buggySensor.left_wheel, buggySensor.right_wheel);

		usleep(1000000);
	}
	pthread_exit (0);
}

// ------------------------------------------------------------------------------------
// TIMERMANAGER: Initialisation du gestionnaire de timer
// - Démarre le thread
// ------------------------------------------------------------------------------------
int InitHwManager(void){
	// CREATION DU THREAD DE TIMER
	  if (pthread_create (&th_hwManager, NULL, hwTask, NULL)< 0) {
		return (1);
	  }else return (0);
}

// ------------------------------------------------------------------------------------
// CLOSEHWMANAGER: Fermeture du gestionnaire hardware
// - Stop le thread hardware
// ------------------------------------------------------------------------------------

int CloseHwManager(void){
	int result;
	// TERMINE LE THREAD DE MESSAGERIE
	pthread_cancel(th_hwManager);
	// Attends la terminaison du thread de messagerie
	result=pthread_join(th_hwManager, NULL);
	return (result);
}
