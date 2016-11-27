/*
 * hwMagager.c
 *
 *  Created on: 26 nov. 2016
 *      Author: raph
 */

#define  CALLBACK 0
#define  ADR	  1
#define	 CMD      2

#include "pthread.h"
#include <unistd.h>
#include "hwManager.h"
#include "hwControl/boardHWctrl.h"
#include "buggy_descriptor.h"

// Thread Messager
pthread_t th_hwManager;

struct o_motor{
	int pulseFromStartup;
	int frequency;
};

typedef struct tsensors{
	unsigned char din0;
	unsigned char din1;
	int usonic;
	int battery;
	struct o_motor left_motor;
	struct o_motor right_motor;
}t_sensor;

t_sensor buggySensor;

int i2c_command_queuing[50][3];

int timeCount_ms=0;
unsigned char motorDCactualPower[2];				// Valeur de la puissance moteur
unsigned char motorDCtargetPower[2]; 				// Valuer de consigne pour la puissance moteur
unsigned char motorDCaccelValue[2]={25,25};			// Valeur d'acceleration des moteurs
unsigned char motorDCdecelValue[2]={25,25};			// Valeur d'acceleration des moteurs


int getMotorFrequency(unsigned char motorNb);	// Retourne la fr�quence actuelle mesuree sur l'encodeur
int getMotorPulses(unsigned char motorNb);		// Retourne le nombre d'impulsion d'encodeur moteur depuis le d�marrage
char getDigitalInput(unsigned char inputNb);	// Retourne l'�tat de l'entr�e num�rique sp�cifi�e
int getSonarDistance(void);						// Retourne la distance en cm
int getBatteryVoltage(void);					// Retourne la tension battery en mV


extern int setMotorSpeed(int motorName, int ratio);
void setMotorAccelDecel(unsigned char motorNo, char accelPercent, char decelPercent);		// D�fini l'acc�leration/deceleration d'un moteur
int setMotorDirection(int motorName, int direction);
void checkDCmotorPower(void);				// Fonction temporaire pour rampe d'acceleration
unsigned char getMotorPower(unsigned char motorNr);											// Retourne la velocit� actuelle d'un moteur

void setServoPosition(unsigned char smAddr, unsigned char position);
void setLedPower(unsigned char smAddr, unsigned char power);

void execCommand(void (*ptrFunc)(char, int), char adr, int cmd);
int set_i2c_command_queue(int (*callback)(char, int),char adr, int cmd);		//
// ------------------------------------------
// Programme principale TIMER
// ------------------------------------------
void *hwTask (void * arg){
	int i;

	if(buggyBoardInit()){
		printf("# Initialisation carte HW: OK\n");
		sendMqttReport(0,"# Initialisation carte HW: OK\n");
		// Test
		setLedPower(0, 50);
	}
	else{
		printf("# Initialisation carte HW: ERREUR\n");
		sendMqttReport(0,"# Initialisation carte HW: ERREUR\n");
	}

	// Reset la distance de la carte EFM8BB
	EFM8BB_clearWheelDistance(0);
	EFM8BB_clearWheelDistance(1);

	while(1){

		// Sequencage des messages sur bus I2C � interval r�gulier
		switch(timeCount_ms){
			case 10	: buggySensor.left_motor.pulseFromStartup = EFM8BB_readPulseCounter(0); break;
			case 20	: buggySensor.right_motor.pulseFromStartup = EFM8BB_readPulseCounter(1);;break;
			case 30	: buggySensor.din0 = EFM8BB_readDigitalInput(0); break;
			case 40	: buggySensor.din1 = EFM8BB_readDigitalInput(1); break;
			case 50	: buggySensor.usonic = EFM8BB_readSonarDistance()/10; break;
			case 60	: buggySensor.battery = EFM8BB_readBatteryVoltage() ;break;
			case 70	: break;
			case 80	: break;
			case 90	: break;
			case 100 : {
//					printf("\n[hwManager] Battery: %dmV ultrasonic: %dcm DIN0: %d  DIN1: %d Left: %.1f  Right: %.1f \n", buggySensor.battery, buggySensor.usonic,
	//				buggySensor.din0, buggySensor.din1, buggySensor.left_motor.pulseFromStartup*0.285, buggySensor.right_motor.pulseFromStartup*0.285);
					break;}

			default: break;
		}

		if(i2c_command_queuing[0][CALLBACK]!=0){
			// ENVOIE DE LA COMMANDE I2C
			execCommand(i2c_command_queuing[0][CALLBACK], i2c_command_queuing[0][ADR], i2c_command_queuing[0][CMD]);
			printf("[hwManager] Commande execut� !\n");
			// DECALAGE DE LA PILE

			for(i=0;i<50;i++){
				i2c_command_queuing[i][CALLBACK] = i2c_command_queuing[i+1][CALLBACK];
				i2c_command_queuing[i][ADR] = i2c_command_queuing[i+1][ADR];
				i2c_command_queuing[i][CMD] = i2c_command_queuing[i+1][CMD];
			}
			i2c_command_queuing[i][CALLBACK]=i2c_command_queuing[i][ADR]=i2c_command_queuing[i][CMD]=0;

		}

		// Reset le compteur au bout de 100mS
		if(timeCount_ms<100)
			timeCount_ms++;
		else timeCount_ms=0;

		usleep(5000);
	}
	pthread_exit (0);
}

// ------------------------------------------------------------------------------------
// TIMERMANAGER: Initialisation du gestionnaire de timer
// - D�marre le thread
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

// ------------------------------------------------------------------------------------
// GETPULSEMOTOR: lecture de l'encodeur optique du moteur sp�cifi�
// Entr�e: Num�ro de l'encodeur
// Sortie:
// ------------------------------------------------------------------------------------
int getMotorPulses(unsigned char motorNb){
	int pulses;

	switch(motorNb){
		case 0: pulses = buggySensor.left_motor.pulseFromStartup; break;
		case 1: pulses = buggySensor.right_motor.pulseFromStartup; break;
		default: pulses= -1; break;
	}
	return pulses;
}

char getDigitalInput(unsigned char inputNb){
	char inputState;

	switch(inputNb){
		case 0: inputState = buggySensor.din0; break;
		case 1: inputState = buggySensor.din1; break;
		default: inputState=-1; break;
	}
	return inputState;
}

int getMotorFrequency(unsigned char motorNb){
	char freq;

	switch(motorNb){
		case 0: freq = buggySensor.left_motor.frequency; break;
		case 1: freq = buggySensor.right_motor.frequency; break;
		default: freq=-1; break;
	}
	return freq;
	return 0;
}


int getSonarDistance(void){
	int sonarCm=0;
	sonarCm= buggySensor.usonic;
	return sonarCm;
}

int getBatteryVoltage(void){
	int voltage=0;
	voltage = buggySensor.battery;
	return voltage;
}


// ---------------------------------------------------------------------------
// SETMOTORDIRECTION
// !!!!!!!!!!!!! FONCTION A RETRAVAILLER !!!!!!!!!!!!!!!!!!!
// ---------------------------------------------------------------------------
int setMotorDirection(int motorName, int direction){

	unsigned char motorAdress;

	// Conversion No de moteur en adresse du registre du PWM controleur
	switch(motorName){
		case MOTOR_LEFT: 	motorAdress = DCM0;	break;
		case MOTOR_RIGHT :  motorAdress = DCM1;	break;
		default : return(0);
	}

	switch(direction){
		case BUGGY_FORWARD :	set_i2c_command_queue(&MCP2308_DCmotorSetRotation, motorAdress, MCW); break;
								//MCP2308_DCmotorSetRotation(motorAdress, MCW); break;
		case BUGGY_BACK :
								set_i2c_command_queue(&MCP2308_DCmotorSetRotation, motorAdress, MCCW); break;
								//MCP2308_DCmotorSetRotation(motorAdress, MCCW); break;
		case BUGGY_STOP : 		break;
		default :		     	break;
	}


	return(1);
}

// ------------------------------------------------------------------------------------
// CHECKMOTORPOWER:
// Fonction appel�e periodiquement pour la gestion de l'acceleration
// D�cel�ration du moteur.
// Elle va augment� ou diminuer la velocite du moteur jusqu'a atteindre la consigne
// ------------------------------------------------------------------------------------
void checkDCmotorPower(void){
	unsigned char i;
	unsigned char PowerToSet;

	// Contr�le successivement la puissance sur chaque moteur et effectue une rampe d'acc�l�ration ou d�c�leration
	for(i=0;i<2;i++){
		//printf("Motor Nb: %d Adr: %2x ActualPower: %d   TargetPower: %d  \n",i, motorDCadr[i], motorDCactualPower[i], motorDCtargetPower[i]);
		if(motorDCactualPower[i] < motorDCtargetPower[i]){
			PowerToSet=motorDCactualPower[i] + ((motorDCtargetPower[i]-motorDCactualPower[i])/100)*motorDCaccelValue[i];
			//printf("Power to set: %d %",PowerToSet);

			if(motorDCactualPower[i]+motorDCaccelValue[i]<=motorDCtargetPower[i])		// Contr�le que puissance apr�s acceleration ne d�passe pas la consigne
				motorDCactualPower[i]+=motorDCaccelValue[i];						// Augmente la puissance moteur
			else motorDCactualPower[i]=motorDCtargetPower[i];						// Attribue la puissance de consigne

			set_i2c_command_queue(&PCA9685_DCmotorSetSpeed, motorDCadr[i], motorDCactualPower[i]);
			//PCA9685_DCmotorSetSpeed(motorDCadr[i], motorDCactualPower[i]);
		}

		if(motorDCactualPower[i]>motorDCtargetPower[i]){
			if(motorDCactualPower[i]-motorDCdecelValue[i]>=motorDCtargetPower[i])		// Contr�le que puissance apr�s acceleration ne d�passe pas la consigne
				motorDCactualPower[i]-=motorDCdecelValue[i];						// Diminue la puissance moteur
			else motorDCactualPower[i]=motorDCtargetPower[i];						// Attribue la puissance de consigne


			set_i2c_command_queue(&PCA9685_DCmotorSetSpeed, motorDCadr[i], motorDCactualPower[i]);
			//PCA9685_DCmotorSetSpeed(motorDCadr[i], motorDCactualPower[i]);

			// Ouvre le pont en h de commande moteur
			if(motorDCactualPower[i]==0)
				setMotorDirection(i,BUGGY_STOP);
		}
	}
}

// -------------------------------------------------------------------
// GETMOTORPOWER
// Retourne l'�tat actuelle de la puissance du moteur selectionn�
// -------------------------------------------------------------------

unsigned char getMotorPower(unsigned char motorNr){
	return motorDCactualPower[motorNr];
}


// -------------------------------------------------------------------
// setMotorAccelDecel
// D�fini les valeurs d'acceleration et decelaration du moteur
// Valeur donn�e en % de ce qu'il reste pour atteindre la consigne
// -------------------------------------------------------------------
void setMotorAccelDecel(unsigned char motorNo, char accelPercent, char decelPercent){

	// R�cup�ration de la valeur absolue de l'acceleration
	if(accelPercent<0) accelPercent*=-1;
	// D�fini un maximum de 100% d'acceleration
	if(accelPercent>100)
		accelPercent=100;

	// R�cup�ration de la valeur absolue de la deceleration
	if(decelPercent<0) decelPercent*=-1;
	// D�fini un maximum de 100% de deceleration
	if(decelPercent>100)
		decelPercent=100;

	// Ne modifie les valeurs d'acceleration et deceleration uniquement si "valable" (=>0)
	if(accelPercent>0)
		motorDCaccelValue[motorNo] = accelPercent;
	if(decelPercent>0)
		motorDCdecelValue[motorNo] = decelPercent;
}


// ---------------------------------------------------------------------------
// SETMOTORSPEED
// Applique la consigne de v�locit� pour un moteur donn�
// Cette consigne n'est pas applique directement mais sera progressivement
// atteinte par le gestionnaire d'acceleration
// ---------------------------------------------------------------------------
int setMotorSpeed(int motorName, int ratio){

	// V�rification ratio max et min comprise entre 0..100%
	if(ratio > 100)
		ratio = 100;
	if (ratio<0)
		ratio = 0;

	switch(motorName){
		case MOTOR_LEFT : motorDCtargetPower[0]=ratio; break;
		case MOTOR_RIGHT : motorDCtargetPower[1]=ratio; break;
		default : printf("\n function [setMotorSpeed] : undefine motor #%d", motorName); break;
	}

	return(1);
}


// ------------------------------------------------------------------------------------
// SET_I2C_COMMAND_QUEUE: Mise en file d'attente de l'appelle d'une fonction I2C
// ------------------------------------------------------------------------------------
int set_i2c_command_queue(int (*callback)(char, int),char adr, int cmd){
	unsigned char freeIndex, i;

	// Recherche d'un emplacement libre dans la file d'attente
	for(freeIndex=0;(freeIndex<50) && (i2c_command_queuing[freeIndex][CALLBACK]>0);freeIndex++);

	if(freeIndex>=49) printf("\n[hwManager]->File de commande pleine !\n");
	else
	{
		i2c_command_queuing[freeIndex][CALLBACK] =  callback;
		i2c_command_queuing[freeIndex][ADR] =  adr;
		i2c_command_queuing[freeIndex][CMD] =  cmd;
	}

	printf("\nPILE DE COMMANDE I2C\n");
	for(i=0;i<20;i++){
		printf("#%d  callback: %d, adr: %d cmd: %d\n",i ,i2c_command_queuing[i][CALLBACK],i2c_command_queuing[i][ADR],i2c_command_queuing[i][CMD]);
	}

	return freeIndex;
}


void setServoPosition(unsigned char smAddr, unsigned char position){
	set_i2c_command_queue(&PCA9685_setServoPos, smAddr, position);
}

void setLedPower(unsigned char smAddr, unsigned char power){
	set_i2c_command_queue(&PCA9685_setLedPower, smAddr, power);
}

// ------------------------------------------------------------------------------------
// ONTIMEOUT: Fcontion appelee en fin de timer
// appelle une fonction callback pr�d�finie par *ptrFunc
// ------------------------------------------------------------------------------------
void execCommand(void (*ptrFunc)(char, int), char adr, int cmd){
	(*ptrFunc)(adr, cmd);		// Appelle de la fonction call back pr�d�finie par *ptrFonc avec les param�tre recus
}
