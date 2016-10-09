/*
 * timerManager.c
 *
 *  Created on: 8 avr. 2016
 *      Author: raph
 */

// Defninition des emplacement dans les variables timer avec callback
#define STOPTIME 0					// Slot, Stoptime pour callback
#define PTRFUNC  1					// Slot, Fonction callback
#define WHEEL	 2					// Slot Data
#define ACTIONID 3					// Slot action concerné

#include "pthread.h"
#include <unistd.h>
#include "timerManager.h"

// Thread Messager
pthread_t th_timers;

int myTimer[10][4];					// Données des timer callback
int timeNow = 0;					// Variable de comptage de temp actuel pour les timers avec callback
unsigned char checkMotorPowerFlag;


void *TimerTask (void * arg){
	int i;
	unsigned int cyclicTimer50ms;	// Compteur du timer cyclique

	while(1){

		// Controle successivement les timers avec fonction callback
		for(i=0;i<10;i++){
			if(myTimer[i][STOPTIME]!=0){						// Timer Actif (!=0), Ne provoque pas d'action si timer inactif
				if(timeNow >= myTimer[i][STOPTIME]){			// Fin du timer ?
					onTimeOut(myTimer[i][PTRFUNC], myTimer[i][ACTIONID],myTimer[i][WHEEL]);	// Appelle la fonction callback
					// Libère l'espace timer
					myTimer[i][STOPTIME]=myTimer[i][PTRFUNC]=myTimer[i][ACTIONID]=0;
					myTimer[i][WHEEL]=-1;
				}
			}
		}


		// Controle le time out de 50ms
		if(cyclicTimer50ms>=50){
			checkMotorPowerFlag=1;
			cyclicTimer50ms=0;				// Reset le compteur 5ms
		}

		cyclicTimer50ms++;
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
	int timerIsSet;
	int setTimerResult;


	setTimerResult=0;
	// Recherche un emplacement libre pour inserer les données du timer
	// Ecrase le timer si nouvelle consigne pour la roue
	//|| (wheelName == myTimer[i][WHEEL])

	i=0;
	while((i<10) && (!timerIsSet)){
		if(wheelName == myTimer[i][WHEEL]){
			printf("Annulation de la tâche en cours: %d pour roue: %d\n", myTimer[i][ACTIONID], wheelName);
			setTimerResult=myTimer[i][ACTIONID];			// Retourne le numéro d'action ecrassé
			myTimer[i][ACTIONID]=0;							// Libère l'emplacement car timer ecrasé
		}

		if(myTimer[i][ACTIONID]<=0){
			myTimer[i][STOPTIME] = timeNow + time_ms;					// Ajoute le temps donné au compteur actuel
			myTimer[i][PTRFUNC]=callback;								// memorisation de la fonction callback de fin de timer
			myTimer[i][WHEEL]=wheelName;								// memorisation de la donnée concernée par l'action(roue)
			myTimer[i][ACTIONID]=actionNumber;							// Memorise le no d'action
			if(!setTimerResult)setTimerResult=1;						// Retourne OK, si pas d'action écrasé
			timerIsSet=1;
			printf("Emplacement #%d ACTION ID= %d\n", i, myTimer[i][ACTIONID]);
		}

		i++;
	}

	return setTimerResult;
}

// ------------------------------------------------------------------------------------
// ONTIMEOUT: Fcontion appelee en fin de timer
// appelle une fonction callback prédéfinie par *ptrFunc
// ------------------------------------------------------------------------------------
void onTimeOut(void (*ptrFunc)(int, int),int actionNumber, int wheelName){
	(*ptrFunc)(actionNumber, wheelName);		// Appelle de la fonction call back prédéfinie par *ptrFonc avec les paramètre recus
}



