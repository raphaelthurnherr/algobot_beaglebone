/*
 * bbbi2c.h
 *
 *  Created on: 1 oct. 2016
 *      Author: raph
 */

#ifndef BBB_I2C_H_
#define BBB_I2C_H_

extern unsigned char i2cInit(char *device);
extern unsigned char i2cSelectSlave(unsigned char slaveAddress);
extern int i2cReadByte(unsigned char regAddress);
extern int i2cWriteByte(unsigned char regAddress, unsigned char data);
#endif /* BBBI2C_H_ */
