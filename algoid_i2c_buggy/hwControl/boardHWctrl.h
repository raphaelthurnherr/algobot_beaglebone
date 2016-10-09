#ifndef BOARDHWCTRL_H_
#define BOARDHWCTRL_H_

// Device addresses (7 bits, lsb is a don't care)
#define  PCA9680               	0x40 	// Device address for PWM controller
#define  MCP2308               	0x20	// Device address for GPIO controller

#define PWM_ALL_ADR				0xFC	// PCA9685 all call address

#define DCM0					0x08	// PCA9685 Output 0 address (Motor 0 speed pwm)
#define DCM1					0x0C	// PCA9685 Output 1 address (Motor 1 speed pwm)

#define LED0					0x10	// PCA9685 Output 2 address (Led 0 pwm)
#define LED1					0x14	// PCA9685 Output 3 address (Led 1 pwm)
#define LED2					0x18	// PCA9685 Output 4 address (Led 2 pwm)

#define SRM0					0x3C	// PCA9685 Output 13 address (Servomotor 0 pwm)
#define SRM1					0x40	// PCA9685 Output 14 address (Servomotor 1 pwm)
#define SRM2					0x44	// PCA9685 Output 15 address (Servomotor 2 pwm)

#define MCW 		0
#define MCCW		1
#define MSTOP 	2


#define BUGGY_STOP 0
#define BUGGY_FORWARD 1
#define BUGGY_BACK 2


#define WHEEL_LEFT 0
#define WHEEL_RIGHT 1

extern unsigned char buggyBoardInit(void);

extern void DCmotorState(unsigned char state);
extern void DCmotorSetRotation(unsigned char motorAdr, unsigned char direction);
extern void DCmotorSetSpeed(unsigned char motorAdr, unsigned char dutyCycle);
extern void setServoPos(unsigned char smAddr, unsigned char position);
extern void setDCmotorPower(unsigned char motorAdr, unsigned char power);

extern void checkDCmotorPower(void);
int setMotorDirection(int motorName, int direction);
int setMotorSpeed(int motorName, int ratio);
#endif
