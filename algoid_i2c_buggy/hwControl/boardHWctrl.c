#include "boardHWctrl.h"
#include "../timerManager.h"
#include "libs/bbb_i2c.h"


void checkDCmotorPower(void);				// Fonction temporaire pour rampe d'acceleration
unsigned char configPWMdevice(void);		// Configuration of the PCA9685 for 50Hz operation
unsigned char configGPIOdevice(void);		// Configuration IO mode of the MCP28003

void setMotorAccelDecel(unsigned char motorNo, char accelPercent, char decelPercent);		// Défini l'accéleration/deceleration d'un moteur
unsigned char getMotorPower(unsigned char motorNr);											// Retourne la velocité actuelle d'un moteur

int getSonarDistance(void);							// Get distance in mm from the EFM8BB microcontroller
char getDigitalInput(unsigned char InputNr);		// Get digital input state in mm from the EFM8BB microcontroller
int getBatteryVoltage(void);						// Get the battery voltage in mV from EFM8BB microcontroller
int getFrequency(unsigned char wheelNb);			// Get the wheel frequency
int getPulseCounter(unsigned char wheelNb);

unsigned char motorDCadr[2]={DCM0, DCM1};			// Valeur de la puissance moteur

unsigned char motorDCactualPower[2];				// Valeur de la puissance moteur
unsigned char motorDCtargetPower[2]; 				// Valuer de consigne pour la puissance moteur
unsigned char motorDCaccelValue[2]={25,25};			// Valeur d'acceleration des moteurs
unsigned char motorDCdecelValue[2]={25,25};			// Valeur d'acceleration des moteurs


//================================================================================
// BUGGYBOARDINIT
// Initialisation of the board (PWM Driver, GPIO driver, etc..)
//================================================================================

unsigned char buggyBoardInit(void){
	unsigned char err;

	err=i2cInit("/dev/i2c-2");
	err+=configPWMdevice();					// Configuration du Chip PWM pour gestion de la vélocité des DC moteur et angle servomoteur
	err+=configGPIOdevice();				// Confguration du chip d'entrées/sortie pour la gestion du sens de rotation des moteur DC
	DCmotorState(1);						// Set the HDRIVER ON
	if(err)
		return 0;							// Erreur
	else return 1;
}


//================================================================================
// DCMOTORSTATE
// Défini l'état général de tout les moteurs DC (Driver pont en H)
//
//================================================================================

void DCmotorState(unsigned char state){
	unsigned char MCP2308_GPIO_STATE;

	i2cSelectSlave(MCP2308);							// séléction du CHIP d'entrée/sortie
	MCP2308_GPIO_STATE=i2cReadByte(0x09);				// Lecture de l'état actuel des ports sur le chip d'entrée/sortie

	if(state) MCP2308_GPIO_STATE |= 0x10;				// Activation du driver pont en H
	else MCP2308_GPIO_STATE &= 0xEF;					// désactivation du driver pont en H

	i2cWriteByte(0x0A, MCP2308_GPIO_STATE);				// Envoie de la commande au chip d'entrée/sortie
}


//================================================================================
// DCMOTORSETSPEED
// Défini le duty cyle à appliquer sur les sorties du chip PWM (0..100%)
// motorAdr: Adresse de sortie du contrôleur PWM sur lequel doit être appliqué le dutyCycle
//================================================================================

void DCmotorSetSpeed(unsigned char motorAdr, unsigned char dutyCycle){
	unsigned int power;
	unsigned char PowerLow;
	unsigned char PowerHigh;

	// Défini un dutycylce de maximum 100%
	if(dutyCycle>100)
		dutyCycle=100;

	// Conversion du dutyclycle en valeur à appliquer au contrôleur PWM
	power = ((409500/100)*dutyCycle)/100;
	PowerLow = power&0x00FF;;
	PowerHigh = (power&0x0F00) >>8;

	i2cSelectSlave(PCA9680);												// Sélection du chip PWM
	i2cWriteByte(motorAdr, PowerLow);										// Envoie des valeurs correspondant au ratio
	i2cWriteByte(motorAdr+1, PowerHigh);									// sur les registres haut et bas de la sortie concernée
}


//================================================================================
// DCMOTORSETROTATION
// Défini le sense de rottion d'un moteur DC (sens horaire, antihoraire ou stop)
//================================================================================

void DCmotorSetRotation(unsigned char motorAdr, unsigned char direction){
	unsigned char MCP2308_GPIO_STATE;

	// Sélection du chip d'entrée/sortie qui pilote le pont en H
	i2cSelectSlave(MCP2308);


	MCP2308_GPIO_STATE=i2cReadByte(0x09);	// Récupération de l'état actuel des sortie sur le chip pour ne modifier que
											// le bit nénéssaire
	//	SELECTION DU MOTEUR No 0
	if(motorAdr==DCM0){
		// Désactive la commande du moteur
		// avant de changer de sens de rotation
		MCP2308_GPIO_STATE &= 0xF9;
		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);

		// Séléction du sens de rotation du moteur ou OFF
		switch(direction){
			case MCW 	 :  MCP2308_GPIO_STATE |= 0x02; break;
			case MCCW 	 : 	MCP2308_GPIO_STATE |= 0x04; break;
			case MSTOP 	 :  MCP2308_GPIO_STATE |= 0x00; break;
			default		 : ;break;
		}

		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Envoie des nouveaux état à mettre sur les sortie du chip d'entrées/sortie
	}

//	SELECTION DU MOTEUR No 1
	if(motorAdr==DCM1){

		// Désactive la commande du moteur
		// avant de changer de sens de rotation
		MCP2308_GPIO_STATE &= 0xF6;										// Force H-Bridge Off for motor 1
		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Apply the new value on the GPIO driver,

		// Séléction du sens de rotation du moteur ou OFF
		switch(direction){
			case MCW 	 :  MCP2308_GPIO_STATE |= 0x01; break;
			case MCCW 	 : 	MCP2308_GPIO_STATE |= 0x08; break;
			case MSTOP 	 :  MCP2308_GPIO_STATE |= 0x00; break;
			default		 : ;break;
		}

		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Envoie des nouveaux état à mettre sur les sortie du chip d'entrées/sortie
	}

}


//================================================================================
// SETSERVOPOS
// Défini la position a appliquer au servomoteur
// smAddr = adresse pour le port de sortie concerné sur le chip PCA9685
// position= Angle de positionnement en degré du servomoteur (de 0..100%)
//================================================================================

void setServoPos(unsigned char smAddr, unsigned char position){
	unsigned int dutyCycleValue;
	unsigned char dCLow;
	unsigned char dCHigh;

	i2cSelectSlave(PCA9680);								// Séléction du chip PWM

	// Vérifie que le positionnement défini soit entre 0 et 100%
	if(position>100)
		position=100;

	// Conversion de la position 0..100% selon le fonctionnement du servo moteur
	// Durée de pulse de minium 0.5mS et maximum 2.5mS
	dutyCycleValue = 205+(position*2.04);
	dCLow = dutyCycleValue&0x00FF;;
	dCHigh = (dutyCycleValue&0x0F00) >>8;

//	Applique les nouvelles valeures
	i2cWriteByte(smAddr, dCLow);
	i2cWriteByte(smAddr+1, dCHigh);
}

//================================================================================
// SETLEDPOWER
// Défini l'intensité d'éclairage pour led @ 50HZ selon config PCA9685
// smAddr = adresse pour le port de sortie concerné sur le chip PCA9685
// power = Intensité d'éclairage ( 0..100%)
//================================================================================

void setLedPower(unsigned char smAddr, unsigned char power){
	unsigned int dutyCycleValue;
	unsigned char dCLow;
	unsigned char dCHigh;

	i2cSelectSlave(PCA9680);								// Séléction du chip PWM

	// Vérifie que la puissance définie soit entre 0 et 100%
	if(power>100)
		power=100;

	// Conversion de la puissance 0..100% en valeur de timer pour PCA9685
	dutyCycleValue = (4096/100)*power;
	dCLow = dutyCycleValue&0x00FF;;
	dCHigh = (dutyCycleValue&0x0F00) >>8;

//	Applique les nouvelles valeures
	i2cWriteByte(smAddr, dCLow);
	i2cWriteByte(smAddr+1, dCHigh);
}



//================================================================================
// CONFIGPWMDEVICE
// Configuration initial pour le controleur PWM PCA9685
//	- Séléction horloge interne 25MHz
//	- Mode opération à 50Hz (Principalement pour la commande de servomoteurs)
//	- Sorties non inversées
//	- Ps d'auto incrementation
//================================================================================
unsigned char configPWMdevice(void){
	unsigned char err;
	err=i2cSelectSlave(PCA9680);

	// Registre MODE1, sleep before config, horloge interne à 25MHz
	i2cWriteByte(0x00, 0x10);
	// Prescaler pour opération 50Hz
	i2cWriteByte(0xFE, 0x81);

	// Registre MODE 2, sorties non inversées
	i2cWriteByte(0x01, 0x04);
	// TOUTES LED ON au clock 0
	i2cWriteByte(0xFA, 0x00);
	i2cWriteByte(0xFB, 0x00);

	// MODE 1, Système prêt,
	i2cWriteByte(0x00, 0x81);


	return err;
}


//================================================================================
// CONFIGGPIODEVICE
// Configuration initiale pour le GPIO Controleur MCP2308
//	- Non auto-incrementation
//	- Pull-up activée
//	- Pin en sortie
//================================================================================
unsigned char configGPIOdevice(void){
	unsigned char err;

	err=i2cSelectSlave(MCP2308);

	// Pas de auto-incrementation
	i2cWriteByte(0x05, 0x20);
	// Pull up activée
	i2cWriteByte(0x06, 0xFF);
	// Pin en sorties
	i2cWriteByte(0x00, 0x00);
	return err;
}




// ---------------------------------------------------------------------------
// SETMOTORDIRECTION
// !!!!!!!!!!!!! FONCTION A RETRAVAILLER !!!!!!!!!!!!!!!!!!!
// ---------------------------------------------------------------------------
int setMotorDirection(int motorName, int direction){

	unsigned char motorAdress;
	unsigned char motorAction;

	// Conversion No de moteur en adresse du registre du PWM controleur
	switch(motorName){
		case WHEEL_LEFT : 	motorAdress = DCM0;	break;
		case WHEEL_RIGHT :  motorAdress = DCM1;	break;
		default : return(0);
	}

	switch(direction){
		case BUGGY_FORWARD :	DCmotorSetRotation(motorAdress, MCW); break;
		case BUGGY_BACK : 	 	DCmotorSetRotation(motorAdress, MCCW); break;
		case BUGGY_STOP : 		break;
		default :		     	break;
	}


	return(1);
}


// ---------------------------------------------------------------------------
// SETMOTORSPEED
// Applique la consigne de vélocité pour un moteur donné
// Cette consigne n'est pas applique directement mais sera progressivement
// atteinte par le gestionnaire d'acceleration
// ---------------------------------------------------------------------------
int setMotorSpeed(int motorName, int ratio){

	// Vérification ratio max et min comprise entre 0..100%
	if(ratio > 100)
		ratio = 100;
	if (ratio<0)
		ratio = 0;

	motorDCtargetPower[motorName]=ratio;
	return(1);
}

// ------------------------------------------------------------------------------------
// CHECKMOTORPOWER:
// Fonction appelée periodiquement pour la gestion de l'acceleration
// Décelération du moteur.
// Elle va augmenté ou diminuer la velocite du moteur jusqu'a atteindre la consigne
// ------------------------------------------------------------------------------------
void checkDCmotorPower(void){
	unsigned char i;
	unsigned char PowerToSet;

	// Contrôle successivement la puissance sur chaque moteur et effectue une rampe d'accélération ou décéleration
	for(i=0;i<2;i++){
		//printf("Motor Nb: %d Adr: %2x ActualPower: %d   TargetPower: %d  \n",i, motorDCadr[i], motorDCactualPower[i], motorDCtargetPower[i]);
		if(motorDCactualPower[i] < motorDCtargetPower[i]){
			PowerToSet=motorDCactualPower[i] + ((motorDCtargetPower[i]-motorDCactualPower[i])/100)*motorDCaccelValue[i];
			//printf("Power to set: %d %",PowerToSet);

			if(motorDCactualPower[i]+motorDCaccelValue[i]<=motorDCtargetPower[i])		// Contrôle que puissance après acceleration ne dépasse pas la consigne
				motorDCactualPower[i]+=motorDCaccelValue[i];						// Augmente la puissance moteur
			else motorDCactualPower[i]=motorDCtargetPower[i];						// Attribue la puissance de consigne

			DCmotorSetSpeed(motorDCadr[i], motorDCactualPower[i]);
		}

		if(motorDCactualPower[i]>motorDCtargetPower[i]){
			if(motorDCactualPower[i]-motorDCdecelValue[i]>=motorDCtargetPower[i])		// Contrôle que puissance après acceleration ne dépasse pas la consigne
				motorDCactualPower[i]-=motorDCdecelValue[i];						// Diminue la puissance moteur
			else motorDCactualPower[i]=motorDCtargetPower[i];						// Attribue la puissance de consigne

			DCmotorSetSpeed(motorDCadr[i], motorDCactualPower[i]);

			// Ouvre le pont en h de commande moteur
			if(motorDCactualPower[i]==0)
				setMotorDirection(i,BUGGY_STOP);
		}
	}
}

// -------------------------------------------------------------------
// setMotorAccelDecel
// Défini les valeurs d'acceleration et decelaration du moteur
// Valeur donnée en % de ce qu'il reste pour atteindre la consigne
// -------------------------------------------------------------------
void setMotorAccelDecel(unsigned char motorNo, char accelPercent, char decelPercent){

	// Récupération de la valeur absolue de l'acceleration
	if(accelPercent<0) accelPercent*=-1;
	// Défini un maximum de 100% d'acceleration
	if(accelPercent>100)
		accelPercent=100;

	// Récupération de la valeur absolue de la deceleration
	if(decelPercent<0) decelPercent*=-1;
	// Défini un maximum de 100% de deceleration
	if(decelPercent>100)
		decelPercent=100;

	// Ne modifie les valeurs d'acceleration et deceleration uniquement si "valable" (=>0)
	if(accelPercent>0)
		motorDCaccelValue[motorNo] = accelPercent;
	if(decelPercent>0)
		motorDCdecelValue[motorNo] = decelPercent;
}

// -------------------------------------------------------------------
// GETSONARDISTANCE
// Lecture de la distance mesuree au sonar [mm]
// Retourn une valeures positve correspondant à la distance en mm
// ou -1 si erreur de lecture
// -------------------------------------------------------------------
int getSonarDistance(void){
	unsigned char err;
	unsigned int SonarDistance_mm;

	err=i2cSelectSlave(EFM8BB);						// Séléction du CHIP

	SonarDistance_mm=0;								// RAZ de la variable distance

	if(!err){
		SonarDistance_mm=i2cReadByte(20);
		SonarDistance_mm+=(i2cReadByte(21)<<8);
		return SonarDistance_mm;
	}else return -1;
}


// -------------------------------------------------------------------
// GETBATTERYVOLTAGE
// Lecture de la tension batterie mesuree en mV
// Retourne une valeures positve correspondant à la tension en mV
// ou -1 si erreur de lecture
// -------------------------------------------------------------------
int getBatteryVoltage(void){
	unsigned char err;
	unsigned int batteryVoltage_mV;

	err=i2cSelectSlave(EFM8BB);						// Séléction du CHIP

	batteryVoltage_mV=0;							// RAZ de la variable

	if(!err){
		batteryVoltage_mV=i2cReadByte(10);
		batteryVoltage_mV+=(i2cReadByte(11)<<8);
		return batteryVoltage_mV;
	}else return -1;
}

// -------------------------------------------------------------------
// GETFREQUENCY
// Get frequency measured on EFM8BB
// ou -1 si erreur de lecture
// -------------------------------------------------------------------
int getFrequency(unsigned char wheelNb){
	unsigned char err, regAddr;
	unsigned int freq;


	err=i2cSelectSlave(EFM8BB);						// Séléction du CHIP

	if(wheelNb==0) regAddr = 40;
	else regAddr = 41;

	freq=0;							// RAZ de la variable

	if(!err){
		freq=(i2cReadByte(regAddr));
		return freq;
	}else return -1;
}

// -------------------------------------------------------------------
// GETPULSECOUNTER
// Get pulse counter on EFM8BB
// ou -1 si erreur de lecture
// -------------------------------------------------------------------
int getPulseCounter(unsigned char wheelNb){
	unsigned char err, regAddr;
	unsigned int pulseCount;


	err=i2cSelectSlave(EFM8BB);						// Séléction du CHIP

	if(wheelNb==0) {
		regAddr = 50;
	}
	else {
		regAddr = 60;
	}

	pulseCount=0;							// RAZ de la variable

	if(!err){
		pulseCount=(i2cReadByte(regAddr));
		pulseCount=pulseCount+(i2cReadByte(regAddr+1)<<8);
		return pulseCount;
	}else return -1;
}

// -------------------------------------------------------------------
// CLEARWHEELDISTANCE
// RetourReset to 0 the pulse counter on EFM8BB
// ou -1 si erreur de lecture
// -------------------------------------------------------------------
int clearWheelDistance(unsigned char wheelNb){
	unsigned char err, regAddr;
	unsigned int pulseCount;


	err=i2cSelectSlave(EFM8BB);						// Séléction du CHIP

	if(wheelNb==0) {
		regAddr = 52;
	}
	else {
		regAddr = 62;
	}

	pulseCount=0;							// RAZ de la variable

	if(!err){
		pulseCount=(i2cReadByte(regAddr));
		return pulseCount;
	}else return -1;
}

// -------------------------------------------------------------------
// GETDIGITALINPUT
// Mesure de l'état des entrées digitale
// -------------------------------------------------------------------
char getDigitalInput(unsigned char InputNr){
	unsigned char err;
	unsigned char inputState;

	err=i2cSelectSlave(EFM8BB);

	if(!err){
		switch(InputNr){
			case 0 :	inputState=i2cReadByte(30); break;
			case 1 :	inputState=i2cReadByte(31); break;
			default:	return(-1); break;
		}

		return inputState;
	}else return -1;
}


// -------------------------------------------------------------------
// GETMOTORPOWER
// Retourne l'état actuelle de la puissance du moteur selectionné
// -------------------------------------------------------------------

unsigned char getMotorPower(unsigned char motorNr){
	return motorDCactualPower[motorNr];
}
