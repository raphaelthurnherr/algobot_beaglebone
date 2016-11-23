#include "boardHWctrl.h"
#include "../timerManager.h"
#include "libs/bbb_i2c.h"


void checkDCmotorPower(void);				// Fonction temporaire pour rampe d'acceleration
unsigned char configPWMdevice(void);		// Configuration of the PCA9685 for 50Hz operation
unsigned char configGPIOdevice(void);		// Configuration IO mode of the MCP28003

void setMotorAccelDecel(unsigned char motorNo, char accelPercent, char decelPercent);		// D�fini l'acc�leration/deceleration d'un moteur
unsigned char getMotorPower(unsigned char motorNr);											// Retourne la velocit� actuelle d'un moteur

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
	err+=configPWMdevice();					// Configuration du Chip PWM pour gestion de la v�locit� des DC moteur et angle servomoteur
	err+=configGPIOdevice();				// Confguration du chip d'entr�es/sortie pour la gestion du sens de rotation des moteur DC
	DCmotorState(1);						// Set the HDRIVER ON
	if(err)
		return 0;							// Erreur
	else return 1;
}


//================================================================================
// DCMOTORSTATE
// D�fini l'�tat g�n�ral de tout les moteurs DC (Driver pont en H)
//
//================================================================================

void DCmotorState(unsigned char state){
	unsigned char MCP2308_GPIO_STATE;

	i2cSelectSlave(MCP2308);							// s�l�ction du CHIP d'entr�e/sortie
	MCP2308_GPIO_STATE=i2cReadByte(0x09);				// Lecture de l'�tat actuel des ports sur le chip d'entr�e/sortie

	if(state) MCP2308_GPIO_STATE |= 0x10;				// Activation du driver pont en H
	else MCP2308_GPIO_STATE &= 0xEF;					// d�sactivation du driver pont en H

	i2cWriteByte(0x0A, MCP2308_GPIO_STATE);				// Envoie de la commande au chip d'entr�e/sortie
}


//================================================================================
// DCMOTORSETSPEED
// D�fini le duty cyle � appliquer sur les sorties du chip PWM (0..100%)
// motorAdr: Adresse de sortie du contr�leur PWM sur lequel doit �tre appliqu� le dutyCycle
//================================================================================

void DCmotorSetSpeed(unsigned char motorAdr, unsigned char dutyCycle){
	unsigned int power;
	unsigned char PowerLow;
	unsigned char PowerHigh;

	// D�fini un dutycylce de maximum 100%
	if(dutyCycle>100)
		dutyCycle=100;

	// Conversion du dutyclycle en valeur � appliquer au contr�leur PWM
	power = ((409500/100)*dutyCycle)/100;
	PowerLow = power&0x00FF;;
	PowerHigh = (power&0x0F00) >>8;

	i2cSelectSlave(PCA9680);												// S�lection du chip PWM
	i2cWriteByte(motorAdr, PowerLow);										// Envoie des valeurs correspondant au ratio
	i2cWriteByte(motorAdr+1, PowerHigh);									// sur les registres haut et bas de la sortie concern�e
}


//================================================================================
// DCMOTORSETROTATION
// D�fini le sense de rottion d'un moteur DC (sens horaire, antihoraire ou stop)
//================================================================================

void DCmotorSetRotation(unsigned char motorAdr, unsigned char direction){
	unsigned char MCP2308_GPIO_STATE;

	// S�lection du chip d'entr�e/sortie qui pilote le pont en H
	i2cSelectSlave(MCP2308);


	MCP2308_GPIO_STATE=i2cReadByte(0x09);	// R�cup�ration de l'�tat actuel des sortie sur le chip pour ne modifier que
											// le bit n�n�ssaire
	//	SELECTION DU MOTEUR No 0
	if(motorAdr==DCM0){
		// D�sactive la commande du moteur
		// avant de changer de sens de rotation
		MCP2308_GPIO_STATE &= 0xF9;
		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);

		// S�l�ction du sens de rotation du moteur ou OFF
		switch(direction){
			case MCW 	 :  MCP2308_GPIO_STATE |= 0x02; break;
			case MCCW 	 : 	MCP2308_GPIO_STATE |= 0x04; break;
			case MSTOP 	 :  MCP2308_GPIO_STATE |= 0x00; break;
			default		 : ;break;
		}

		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Envoie des nouveaux �tat � mettre sur les sortie du chip d'entr�es/sortie
	}

//	SELECTION DU MOTEUR No 1
	if(motorAdr==DCM1){

		// D�sactive la commande du moteur
		// avant de changer de sens de rotation
		MCP2308_GPIO_STATE &= 0xF6;										// Force H-Bridge Off for motor 1
		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Apply the new value on the GPIO driver,

		// S�l�ction du sens de rotation du moteur ou OFF
		switch(direction){
			case MCW 	 :  MCP2308_GPIO_STATE |= 0x01; break;
			case MCCW 	 : 	MCP2308_GPIO_STATE |= 0x08; break;
			case MSTOP 	 :  MCP2308_GPIO_STATE |= 0x00; break;
			default		 : ;break;
		}

		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Envoie des nouveaux �tat � mettre sur les sortie du chip d'entr�es/sortie
	}

}


//================================================================================
// SETSERVOPOS
// D�fini la position a appliquer au servomoteur
// smAddr = adresse pour le port de sortie concern� sur le chip PCA9685
// position= Angle de positionnement en degr� du servomoteur (de 0..100%)
//================================================================================

void setServoPos(unsigned char smAddr, unsigned char position){
	unsigned int dutyCycleValue;
	unsigned char dCLow;
	unsigned char dCHigh;

	i2cSelectSlave(PCA9680);								// S�l�ction du chip PWM

	// V�rifie que le positionnement d�fini soit entre 0 et 100%
	if(position>100)
		position=100;

	// Conversion de la position 0..100% selon le fonctionnement du servo moteur
	// Dur�e de pulse de minium 0.5mS et maximum 2.5mS
	dutyCycleValue = 205+(position*2.04);
	dCLow = dutyCycleValue&0x00FF;;
	dCHigh = (dutyCycleValue&0x0F00) >>8;

//	Applique les nouvelles valeures
	i2cWriteByte(smAddr, dCLow);
	i2cWriteByte(smAddr+1, dCHigh);
}

//================================================================================
// SETLEDPOWER
// D�fini l'intensit� d'�clairage pour led @ 50HZ selon config PCA9685
// smAddr = adresse pour le port de sortie concern� sur le chip PCA9685
// power = Intensit� d'�clairage ( 0..100%)
//================================================================================

void setLedPower(unsigned char smAddr, unsigned char power){
	unsigned int dutyCycleValue;
	unsigned char dCLow;
	unsigned char dCHigh;

	i2cSelectSlave(PCA9680);								// S�l�ction du chip PWM

	// V�rifie que la puissance d�finie soit entre 0 et 100%
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
//	- S�l�ction horloge interne 25MHz
//	- Mode op�ration � 50Hz (Principalement pour la commande de servomoteurs)
//	- Sorties non invers�es
//	- Ps d'auto incrementation
//================================================================================
unsigned char configPWMdevice(void){
	unsigned char err;
	err=i2cSelectSlave(PCA9680);

	// Registre MODE1, sleep before config, horloge interne � 25MHz
	i2cWriteByte(0x00, 0x10);
	// Prescaler pour op�ration 50Hz
	i2cWriteByte(0xFE, 0x81);

	// Registre MODE 2, sorties non invers�es
	i2cWriteByte(0x01, 0x04);
	// TOUTES LED ON au clock 0
	i2cWriteByte(0xFA, 0x00);
	i2cWriteByte(0xFB, 0x00);

	// MODE 1, Syst�me pr�t,
	i2cWriteByte(0x00, 0x81);


	return err;
}


//================================================================================
// CONFIGGPIODEVICE
// Configuration initiale pour le GPIO Controleur MCP2308
//	- Non auto-incrementation
//	- Pull-up activ�e
//	- Pin en sortie
//================================================================================
unsigned char configGPIOdevice(void){
	unsigned char err;

	err=i2cSelectSlave(MCP2308);

	// Pas de auto-incrementation
	i2cWriteByte(0x05, 0x20);
	// Pull up activ�e
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

	motorDCtargetPower[motorName]=ratio;
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

			DCmotorSetSpeed(motorDCadr[i], motorDCactualPower[i]);
		}

		if(motorDCactualPower[i]>motorDCtargetPower[i]){
			if(motorDCactualPower[i]-motorDCdecelValue[i]>=motorDCtargetPower[i])		// Contr�le que puissance apr�s acceleration ne d�passe pas la consigne
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

// -------------------------------------------------------------------
// GETSONARDISTANCE
// Lecture de la distance mesuree au sonar [mm]
// Retourn une valeures positve correspondant � la distance en mm
// ou -1 si erreur de lecture
// -------------------------------------------------------------------
int getSonarDistance(void){
	unsigned char err;
	unsigned int SonarDistance_mm;

	err=i2cSelectSlave(EFM8BB);						// S�l�ction du CHIP

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
// Retourne une valeures positve correspondant � la tension en mV
// ou -1 si erreur de lecture
// -------------------------------------------------------------------
int getBatteryVoltage(void){
	unsigned char err;
	unsigned int batteryVoltage_mV;

	err=i2cSelectSlave(EFM8BB);						// S�l�ction du CHIP

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


	err=i2cSelectSlave(EFM8BB);						// S�l�ction du CHIP

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


	err=i2cSelectSlave(EFM8BB);						// S�l�ction du CHIP

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


	err=i2cSelectSlave(EFM8BB);						// S�l�ction du CHIP

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
// Mesure de l'�tat des entr�es digitale
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
// Retourne l'�tat actuelle de la puissance du moteur selectionn�
// -------------------------------------------------------------------

unsigned char getMotorPower(unsigned char motorNr){
	return motorDCactualPower[motorNr];
}
