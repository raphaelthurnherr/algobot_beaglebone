/*
 * timerManager.c
 *
 *  Created on: 8 avr. 2016
 *      Author: raph
 */
#define STOPTIME 0
#define PTRFUNC  1
#define WHEEL	 2
#define ACTIONID 3

#include "pthread.h"
#include <unistd.h>

#include "timerManager.h"

// Thread Messager
pthread_t th_timers;

int myTimer[10][4];
int timeNow = 0;

void *TimerTask (void * arg){
	int i;

	while(1){

		// Controle succesivement les timers
		for(i=0;i<10;i++){
			if(myTimer[i][STOPTIME]!=0){						// Timer Actif (!=0)
				if(timeNow >= myTimer[i][STOPTIME]){
					onTimeOut(myTimer[i][PTRFUNC], myTimer[i][ACTIONID],myTimer[i][WHEEL]);	// Appelle la fonction callback
					myTimer[i][STOPTIME]=0;
				}
			}
		}
		timeNow++;
		usleep(1000);
	}
	pthread_exit (0);
}

// ------------------------------------------------------------------------------------
// TIMERMANAGER: Initialisation du gestionnaire de timer
// -
// ------------------------------------------------------------------------------------
int InitTimerManager(void){
	// CREATION DU THREAD DE TIMER
	  if (pthread_create (&th_timers, NULL, TimerTask, NULL)< 0) {
		return (1);
	  }else return (0);
}

// ------------------------------------------------------------------------------------
// CLOSETIMER: Fermeture du gestionnaire de timers
// - Stop le thread timers
// ------------------------------------------------------------------------------------

int CloseTimerManager(void){
	int result;
	// TERMINE LE THREAD DE MESSAGERIE
	pthread_cancel(th_timers);
	// Attends la terminaison du thread de messagerie
	result=pthread_join(th_timers, NULL);
	return (result);
}

// ------------------------------------------------------------------------------------
// CLOSETIMER: Fermeture du gestionnaire de timers
// - Stop le thread timers
// ------------------------------------------------------------------------------------
int setTimerWheel(int time_ms, int (*callback)(int, int),int actionNumber, int wheelName){
	//void (*ptrFunc)(int);
	//ptrFunc = callback;

	int i;
	int result;

	// Recherche un emplacement libre pour inserer les données du timer
	for(i=0;(i<10) || (result =! 0);i++){
		if(myTimer[i][STOPTIME]==0){
			myTimer[i][STOPTIME] = timeNow + time_ms;
			myTimer[i][PTRFUNC]=callback;
			myTimer[i][WHEEL]=wheelName;
			myTimer[i][ACTIONID]=actionNumber;
			result=1;
			return 1;
		}
	}

	return 0;
}

// ------------------------------------------------------------------------------------
// ONTIMEOUT: Fcontion appelee en fin de timer
// appelle une fonction callback prédéfinie
// ------------------------------------------------------------------------------------
void onTimeOut(void (*ptrFunc)(int, int),int actionNumber, int wheelName){
	(*ptrFunc)(actionNumber, wheelName);
}
