#include "boardHWctrl.h"

#include "libs/bbb_i2c.h"								// SMBUS Control


unsigned char configPWMdevice(void);
unsigned char configGPIOdevice(void);

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
// SETMOTOR
// ---------------------------------------------------------------------------
int setMotor(int motorName, int direction, int ratio){

	unsigned char motorAdress;
	unsigned char motorAction;

	// Vérification ratio max et min
	if(ratio > 100)
		ratio = 100;
	if (ratio<0)
		ratio = 0;

	switch(motorName){
		case WHEEL_LEFT : 	motorAdress = DCM0; break;
		case WHEEL_RIGHT :  motorAdress = DCM1; break;
		default : return(0);
	}
	// Set the PWM speed of the motor
	DCmotorSetSpeed(motorAdress, ratio);

	switch(direction){
		case BUGGY_FORWARD :	motorAction = MCW; break;
		case BUGGY_BACK : 	 	motorAction = MCCW; break;
		case BUGGY_STOP : 		motorAction = MSTOP; break;
		default :		     	break;
	}

	DCmotorSetRotation(motorAdress, motorAction);
	return(1);
}

