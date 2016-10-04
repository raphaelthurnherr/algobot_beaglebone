#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "bbb_i2c.h"
int file;



// ---------------------------------------------------------------------------
// INITIALISATION DU BUS I2C, TENTATIVE D'OUVERTURE
// ---------------------------------------------------------------------------
unsigned char i2cInit(char *device){
    if ((file = open(device, O_RDWR)) < 0) {
             /* ERROR HANDLING: you can check errno to see what went wrong*/
            perror("Failed to open the i2c bus");
            return (0);
    }
    else
    	return (1);
}

// ---------------------------------------------------------------------------
// SELECTION D'UN ESCLAVE DESTINATAIRE SUR BUS I2C
// ---------------------------------------------------------------------------
unsigned char i2cSelectSlave(unsigned char slaveAddress){
	// Tentative de connexion à l'esclave I2C
	if (ioctl(file, I2C_SLAVE, slaveAddress) < 0) {
			 printf("Failed to acquire bus access and/or talk to slave|n");
			 /* ERROR HANDLING; you can check errno to see what wen$*/
			 return (0);
	}
    else
    	return (1);
}

// ---------------------------------------------------------------------------
// LECTURE D'UN OCTET SUR L'ESCLAVE DU BUS I2C
// ---------------------------------------------------------------------------
extern int i2cReadByte(unsigned char regAddress){
	unsigned char result;
	result = i2c_smbus_read_byte_data(file, regAddress);
     if (result < 0) {
       /* ERROR HANDLING: i2c transaction failed */
    	 return -1;
     } else {
       return result;
       /* res contains the read word */
   }
}

// ---------------------------------------------------------------------------
// ECRITURE D'UN OCTET SUR L'ESCLAVE DU BUS I2C
// ---------------------------------------------------------------------------
int i2cWriteByte(unsigned char regAddress, unsigned char data){
	i2c_smbus_write_byte_data(file, regAddress, data);
}
