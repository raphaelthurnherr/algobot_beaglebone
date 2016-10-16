#include "boardHWctrl.h"
#include "../timerManager.h"
#include "libs/bbb_i2c.h"								// SMBUS Control

void checkDCmotorPower(void);		// Fonction temporaire pour rampe d'acceleration

unsigned char configPWMdevice(void);
unsigned char configGPIOdevice(void);

void setMotorAccelDecel(unsigned char motorNo, char accelPercent, char decelPercent);

int getSonarDistance(void);							// Get distance in mm from the EFM8BB microcontroller
char getDigitalInput(unsigned char InputNr);		// Get digital input state in mm from the EFM8BB microcontroller
int getBatteryVoltage(void);						// Get the battery voltage in mV from EFM8BB microcontroller

unsigned char motorDCadr[2]={DCM0, DCM1};	// Valeur de la puissance moteur

unsigned char motorDCactualPower[2];	// Valeur de la puissance moteur
unsigned char motorDCtargetPower[2]; // Valuer de consigne pour la puissance moteur
unsigned char motorDCaccelValue[2]={25,25};	// Valeur d'acceleration des moteurs
unsigned char motorDCdecelValue[2]={25,25};	// Valeur d'acceleration des moteurs


//================================================================================
// BUGGYBOARDINIT
// Initialisation of the board (PWM Driver, GPIO driver, etc..)
//================================================================================

unsigned char buggyBoardInit(void){
	unsigned char err;

	err=i2cInit("/dev/i2c-2");
	err+=configPWMdevice();
	err+=configGPIOdevice();
	DCmotorState(1);						// Set the HDRIVER ON
	if(err)
		return 0;
	else return 1;
}


//================================================================================
// DCMOTORSTATE
// set the DC motor driver IC to ON/OFF
//================================================================================

void DCmotorState(unsigned char state){
	unsigned char MCP2308_GPIO_STATE;

	i2cSelectSlave(MCP2308);							// Read the actual GPIOs value on the MCP23008 Port
	MCP2308_GPIO_STATE=i2cReadByte(0x09);

	if(state) MCP2308_GPIO_STATE |= 0x10;				// Driver ON
	else MCP2308_GPIO_STATE &= 0xEF;					// Driver OFF

	i2cWriteByte(0x0A, MCP2308_GPIO_STATE);				// Initiate SMBus write
}

//================================================================================
// DCMOTORSETSPEED
// set the speed to the motor (0..100%)
//================================================================================

void DCmotorSetSpeed(unsigned char motorAdr, unsigned char dutyCycle){
	unsigned int power;
	unsigned char PowerLow;
	unsigned char PowerHigh;

	// Check the maximum speed
	if(dutyCycle>100)
		dutyCycle=100;

	// Convert power % in PWM ratio
	power = ((409500/100)*dutyCycle)/100;
	PowerLow = power&0x00FF;;
	PowerHigh = (power&0x0F00) >>8;

	i2cSelectSlave(PCA9680);
	i2cWriteByte(motorAdr, PowerLow);										// Set the speed of the motor selected
	i2cWriteByte(motorAdr+1, PowerHigh);
}

//================================================================================
// DCMOTORSETROTATION
// Set the sense of rotation of the motor (Clock Wise, counter clock wise, stop)
//================================================================================

void DCmotorSetRotation(unsigned char motorAdr, unsigned char direction){
	unsigned char MCP2308_GPIO_STATE;

	// Read the actual GPIOs value on the MCP23008 Port
	i2cSelectSlave(MCP2308);

	// Read the actual GPIOs value on the MCP23008 Port
	MCP2308_GPIO_STATE=i2cReadByte(0x09);

	//	ACTION FOR MOTOR 0
	if(motorAdr==DCM0){
		MCP2308_GPIO_STATE &= 0xF9;						// Force H-Bridge Off for motor 0
		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);

		switch(direction){
			case MCW 	 :  MCP2308_GPIO_STATE |= 0x02; break;			// Select rotary sens
			case MCCW 	 : 	MCP2308_GPIO_STATE |= 0x04; break;			// Select sens reverse
			case MSTOP 	 :  MCP2308_GPIO_STATE |= 0x00; break;			// No sens selected, H-BRIDGE off
			default		 : ;break;
		}

		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Apply the new value on the GPIO driver for sens of motor
	}

//	ACTION FOR MOTOR 1
	if(motorAdr==DCM1){
		MCP2308_GPIO_STATE &= 0xF6;										// Force H-Bridge Off for motor 1

		// Read the actual GPIOs value on the MCP23008 Port
		i2cSelectSlave(MCP2308);
		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Apply the new value on the GPIO driver,

		switch(direction){
			case MCW 	 :  MCP2308_GPIO_STATE |= 0x01; break;			// Select rotary sens
			case MCCW 	 : 	MCP2308_GPIO_STATE |= 0x08; break;			// Select sens reverse
			case MSTOP 	 :  MCP2308_GPIO_STATE |= 0x00; break;			// No sens selected, H-BRIDGE off
			default		 : ;break;
		}

		i2cWriteByte(0x0A, MCP2308_GPIO_STATE);							// Apply the new value on the GPIO driver for sens of motor
	}

}


//================================================================================
// SETSERVOPOS
// Set the position of the servomotor
// smAddr = PCA9685 Output x address for servomotor
//================================================================================

void setServoPos(unsigned char smAddr, unsigned char position){
	unsigned int dutyCycleValue;
	unsigned char dCLow;
	unsigned char dCHigh;

	// Check the maximum speed
	if(position>100)
		position=100;

	// Convert position % in servomotor time (0.5mS .. 2.5mS)
	dutyCycleValue = 205+(position*2.04);
	dCLow = dutyCycleValue&0x00FF;;
	dCHigh = (dutyCycleValue&0x0F00) >>8;

//	ACTION FOR MOTOR 1
	i2cSelectSlave(PCA9680);
	i2cWriteByte(smAddr, dCLow);							// Set the speed of the motor
	i2cWriteByte(smAddr+1, dCHigh);
}
//================================================================================
// CONFIGPWMDEVICE
// Initial configuration for PWM Controller PCA9685
//	- Select internal clock 25MHz
//	- 50Hz operation (define for servomotors)
//	- Non-inverted output
//	- No auto-incrementation
//================================================================================
unsigned char configPWMdevice(void){
	unsigned char err;
	err=i2cSelectSlave(PCA9680);

	// MODE1 Register, sleep before config, internal clock 25MHz
	i2cWriteByte(0x00, 0x10);
	//SMB_Write(PCA9680, 0x00, 0x10);                     // Initiate SMBus write
	// Prescaler for 50Hz operation

	i2cWriteByte(0xFE, 0x81);
	//SMB_Write(PCA9680, 0xFE, 0x81);                     // Initiate SMBus write
	// MODE 2 register, non inverted output
	i2cWriteByte(0x01, 0x04);
	//SMB_Write(PCA9680, 0x01, 0x04);                     // Initiate SMBus write
	// ALL LED ON @ 0 clock
	i2cWriteByte(0xFA, 0x00);
	//SMB_Write(PCA9680, 0xFA, 0x00);                     // Initiate SMBus write
	// ALL LED ON @ 0 clock
	i2cWriteByte(0xFB, 0x00);
	//SMB_Write(PCA9680, 0xFB, 0x00);                     // Initiate SMBus write
	// MODE 1, System ready
	i2cWriteByte(0x00, 0x81);
	//SMB_Write(PCA9680, 0x00, 0x81);                     // Initiate SMBus write

	return err;
}


//================================================================================
// CONFIGGPIODEVICE
// Initial configuration for GPIO Controller MCP2308
//	- Non auto-incrementation
//	- Pull-up enable
//	- Pin as output
//================================================================================
unsigned char configGPIOdevice(void){
	unsigned char err;

	err=i2cSelectSlave(MCP2308);

	// No auto-incrementation
	i2cWriteByte(0x05, 0x20);
	// Pull up enable
	i2cWriteByte(0x06, 0xFF);
	// Pin as output
	i2cWriteByte(0x00, 0x00);
	return err;
}




// ---------------------------------------------------------------------------
// SETMOTORDIRECTION
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
// ---------------------------------------------------------------------------
int setMotorSpeed(int motorName, int ratio){

	// Vérification ratio max et min
	if(ratio > 100)
		ratio = 100;
	if (ratio<0)
		ratio = 0;

	motorDCtargetPower[motorName]=ratio;
	return(1);
}

// ------------------------------------------------------------------------------------
// ONTIMEOUT50ms: Fcontion appelee chaque 50mS
//
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
// Modifie les valeur d'acceleration et decelaration du moteur
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
// -------------------------------------------------------------------
	int getSonarDistance(void){
	unsigned char err;
	unsigned int SonarDistance_mm;

	SonarDistance_mm=0;
	err=i2cSelectSlave(EFM8BB);

	if(!err){
		SonarDistance_mm=i2cReadByte(20);
		SonarDistance_mm+=(i2cReadByte(21)<<8);
		return SonarDistance_mm;
	}else return -1;
}


	// -------------------------------------------------------------------
	// GETBATTERYVOLTAGE
	// Lecture de la tension batterie
	// -------------------------------------------------------------------
	int getBatteryVoltage(void){
		unsigned char err;
		unsigned int batteryVoltage_mV;

		batteryVoltage_mV=0;
		err=i2cSelectSlave(EFM8BB);

		if(!err){
			batteryVoltage_mV=i2cReadByte(10);
			batteryVoltage_mV+=(i2cReadByte(11)<<8);
			return batteryVoltage_mV;
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

