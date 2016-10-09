/*
 * timerManager.h
 *
 *  Created on: 8 avr. 2016
 *      Author: raph
 */

#ifndef TIMERMANAGER_H_
#define TIMERMANAGER_H_

int InitTimerManager(void);
int CloseTimerManager(void);
int setTimerWheel(int time_ms, int (*callback)(int, int),int actionNumber, int wheelName);

extern unsigned char checkMotorPowerFlag;

#endif /* TIMERMANAGER_H_ */
