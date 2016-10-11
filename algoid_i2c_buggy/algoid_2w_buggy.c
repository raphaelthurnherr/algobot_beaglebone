
#define TASK_NUMBER 0
#define ACTION_ALGOID_ID 1
#define ACTION_COUNT 2

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "algoidCom/messagesManager.h"
#include "algoidCom/linux_json.h"
#include "algoidCom/udpPublish.h"
#include "hwControl/boardHWctrl.h"
#include "tools.h"
#include "algoid_2wd_buggy.h"
#include "timerManager.h"

int ActionTable[10][3];

int createBuggyTask(int MsgId, int actionCount);
int removeBuggyTask(int actionNumber);

// Traitement du message algoid recu
int processAlgoidMsg(void);

int make2WDaction(void);
int setWheelAction(int actionNumber, int wheelNumber, int veloc, int time);
int endWheelAction(int actionNumber, int wheelNumber);

int getWDvalue(char * wheelName);
char reportBuffer[256];
char timer10s;

int main(void) {
	if(InitMessager()) printf ("# Creation tache messagerie : ERREUR\n");
	else printf ("# Demarrage tache Messager: OK\n");

	if(InitTimerManager()) printf ("# Creation tache timer : ERREUR\n");
		else printf ("# Demarrage tache timer: OK\n");

	// Initialisation UDP pour broadcast IP Adresse
	initUDP();

	if(buggyBoardInit()){
		printf("# Initialisation carte HW: OK\n");
		sendMqttReport(0,"# Initialisation carte HW: OK\n");
	}
	else{
		printf("# Initialisation carte HW: ERREUR\n");
		sendMqttReport(0,"# Initialisation carte HW: ERREUR\n");
	}

	while(1){
		int kbInput;
//		kbInput = mygetch();

		if(kbInput=='q')
			break;

// COMMANDE ALGOID RECUE
		if(pullMsgStack(0)){
//			printf("[main] messageID:  %d  param: %d   cmd: %d\n\n",AlgoidCommand.msgID,AlgoidCommand.msgParam,AlgoidCommand.msgType );
			switch(AlgoidCommand.msgType){
				case COMMAND : processAlgoidMsg(); break;
				//case REQUEST : processMsgRequest(); break;
				default : break;
			}
		}

    	if(timer10s>=100){
    		char udpMessage[50];
    		sprintf(&udpMessage[0], "[ %s ] I'm here",ClientID);
    		sendUDPHeartBit(udpMessage);
		  // printf("\n MYMAC %s", getMACaddr());
		    timer10s=0;
    	}

    	if(checkMotorPowerFlag){
			checkDCmotorPower();
			checkMotorPowerFlag=0;
    	}

    	timer10s++;
    	usleep(100000);
	}

	int endState=CloseMessager();
	if(!endState)
		  printf( "# ARRET tache Messager - status: %d\n", endState);
	else printf( "# ARRET tache Messager erreur - status: %d\n", endState);

	return EXIT_SUCCESS;
}

// -------------------------------------------------------------------
// PROCESSCOMMAND
// -------------------------------------------------------------------
int processAlgoidMsg(void){
	switch(AlgoidCommand.msgParam){
		case LL_2WD : 	make2WDaction(); break;
		default : break;
	}
	return 0;
}

// -------------------------------------------------------------------
// make2WDaction
// -------------------------------------------------------------------
int make2WDaction(void){
	int ptrData;
	int myTaskId;
	unsigned char actionCount=0;

	// Création d'une tâche pour les toutes les actions à effectuer
	// Recois un numéro de tache en retour

	// Recherche le nombre d'action à effectuer
	if(getWDvalue("left")>=0) actionCount++;
	if(getWDvalue("right")>=0) actionCount++;

	myTaskId=createBuggyTask(AlgoidCommand.msgID, actionCount);			// 2 actions pour mouvement 2WD

	if(myTaskId>0){
		printf("Creation de tache: #%d\n", myTaskId);
		ptrData=getWDvalue("left");
		if(ptrData >=0){
			// Enregistre la donnée d'acceleration si disponible (<0)
			if(AlgoidCommand.msgValArray[ptrData].accel!=0 || AlgoidCommand.msgValArray[ptrData].decel!=0)
				setMotorAccelDecel(WHEEL_LEFT, AlgoidCommand.msgValArray[ptrData].accel, AlgoidCommand.msgValArray[ptrData].decel);
			// Effectue l'action sur la roue
			setWheelAction(myTaskId, WHEEL_LEFT, AlgoidCommand.msgValArray[ptrData].velocity, AlgoidCommand.msgValArray[ptrData].time);
		}

		ptrData=getWDvalue("right");
		if(ptrData >=0){
			// Enregistre la donnée d'acceleration si disponible (<0)
			if(AlgoidCommand.msgValArray[ptrData].accel>0 || AlgoidCommand.msgValArray[ptrData].decel>0)
				setMotorAccelDecel(WHEEL_RIGHT, AlgoidCommand.msgValArray[ptrData].accel, AlgoidCommand.msgValArray[ptrData].decel);
			setWheelAction(myTaskId, WHEEL_RIGHT, AlgoidCommand.msgValArray[ptrData].velocity, AlgoidCommand.msgValArray[ptrData].time);
		}
		if((AlgoidCommand.msgValArray[ptrData].velocity < -100) ||(AlgoidCommand.msgValArray[ptrData].velocity > 100))
			sendResponse(AlgoidCommand.msgID, "warning", "2wd", "test", 999);
		return 0;
	}
	else return 1;
}


// -------------------------------------------------------------------
// SETWHEELACTION
// Effectue l'action sur une roue spécifiée
// - Démarrage du timer avec definition de fonction call-back, et no d'action
// - Démarrage du mouvement des roues
// -------------------------------------------------------------------
int setWheelAction(int actionNumber, int wheelNumber, int veloc, int time){
	int myDirection;
	int setTimerResult;
	int endOfTask;

	if(veloc > 0)
		myDirection=BUGGY_FORWARD;
	if(veloc == 0)
		myDirection=BUGGY_STOP;
	if(veloc < 0){
		myDirection=BUGGY_BACK;
		veloc *=-1;					// Convert speed to positiva value
	}

	// Start timer and set callbackback function with arg for stop
	setTimerResult=setTimerWheel(time, &endWheelAction, actionNumber, wheelNumber);

	if(setTimerResult!=0){				// Timer pret
		if(setTimerResult>1){			// Timer ecrasé
			endOfTask=removeBuggyTask(setTimerResult);	// Supprime la tache écrasée par la nouvelle valeur
			if(endOfTask){
				sendResponse(endOfTask, "event","2wd", 0, 0);
				sprintf(reportBuffer, "FIN DES ACTIONS \"WHEEL\" pour la tache #%d\n", endOfTask);
				printf(reportBuffer);
				sendMqttReport(endOfTask, reportBuffer);
			}
		}

		if(setMotorDirection(wheelNumber,myDirection)){
			setMotorSpeed(wheelNumber, veloc);
			//setMotorDirection(wheelNumber,myDirection)
			sprintf(reportBuffer, "Start wheel %d with velocity %d for time %d\n",wheelNumber, veloc, time);
			printf(reportBuffer);
			sendMqttReport(actionNumber, reportBuffer);

		}
		else{
			sprintf(reportBuffer, "Error, impossible to start wheel %d\n",wheelNumber, veloc, time);
			printf(reportBuffer);
			sendMqttReport(actionNumber, reportBuffer);
		}

	}
	else printf("Error, Impossible to set timer wheel\n");
	return 0;
}

// -------------------------------------------------------------------
// END2WDACTION
// Fin de l'action sur une roue, (Appeler apres timeout)
// -------------------------------------------------------------------
int endWheelAction(int actionNumber, int wheelNumber){
	int endOfTask;
	//printf("Action number: %d - End of timer for wheel No: %d\n",actionNumber , wheelNumber);

	// Stop le moteur
	setMotorSpeed(wheelNumber, 0);

	// Retire l'action de la table et vérification si toute les actions sont effectuées
	// Pour la tâche en cours

	endOfTask=removeBuggyTask(actionNumber);
	if(endOfTask){
		sendResponse(endOfTask, "event","2wd", 0, 0);
		sprintf(reportBuffer, "FIN DES ACTIONS \"WHEEL\" pour la tache #%d\n", endOfTask);
		printf(reportBuffer);
		sendMqttReport(endOfTask, reportBuffer);
	}

	return 0;
}

// -------------------------------------------------------------------
// GETWDVALUE
// -------------------------------------------------------------------
int getWDvalue(char* wheelName){
	int i;
	int searchPtr = -1;
	char searchText[50];
	char * mySearch;

	// Recherche dans les donnée recues la valeur correspondante au paramètre "wheelName"

	for(i=0;i<AlgoidCommand.msgValueCnt;i++){
		memset(searchText, 0, 50);
		mySearch=AlgoidCommand.msgValArray[i].wheel;
		strncpy(searchText,mySearch, strlen(AlgoidCommand.msgValArray[i].wheel));

		if(!strcmp(searchText, wheelName))
			searchPtr=i;
	}
	return searchPtr;
}



// -------------------------------------------------------------------
// CREATBUGGYTASK Creation d'une tache avec le nombre
// d'actions à effectuer
// - Retourne le numéro d'action attribué
// - Retourne 0 si table des taches pleine (Impossible de créer)
// - Retourne -1 si Message ID existe déjà
// -------------------------------------------------------------------

int createBuggyTask(int MsgId, int actionCount){
	int i;
	int actionID;

	// défini un numéro de tache aléatoire pour l'action à executer si pas de message id saisi
	if(MsgId == 0){
		actionID = rand() & 0xFFFFFF;
		MsgId = actionID;
	}
	else actionID = MsgId;

	// Recherche un emplacement libre pour inserer les données
	for(i=0;i<10;i++){
		if(ActionTable[i][TASK_NUMBER]==0){
			ActionTable[i][TASK_NUMBER]=actionID;
			ActionTable[i][ACTION_ALGOID_ID]= MsgId;
			ActionTable[i][ACTION_COUNT]=actionCount;
			return(actionID);
		}else{
			if(ActionTable[i][TASK_NUMBER]==actionID)
			{
				sprintf(reportBuffer, "ERREUR: Tache en cours de traitement: %d\n", actionID);
				sendMqttReport(actionID, reportBuffer);
				return -1;
				}
		}
	}
	sprintf(reportBuffer, "ERREUR: Table de tâches pleine\n", actionID);
	sendMqttReport(actionID, reportBuffer);
	return(0);
}

// -------------------------------------------------------------------
// removeBuggyTask
// Mise à jour, soustrait l'action d'une tache
// - Retourne le MESSAGE ID correspondant à la tache si plus d'action à effectuer
// - Retourne 0 si actions restante
// - Retourne -1 si tache inexistante
// -------------------------------------------------------------------

int removeBuggyTask(int actionNumber){
	int i, algoidMsgId;

	// Recherche la tache correspondante dans la tâble des action
	for(i=0;i<10;i++){
		if(ActionTable[i][TASK_NUMBER]==actionNumber){
			ActionTable[i][ACTION_COUNT]--;
			//printf("UPDATE ACTION %d  reste: %d\n", actionNumber, ActionTable[i][ACTION_COUNT]);
			if((ActionTable[i][ACTION_COUNT]) <=0){
				algoidMsgId=ActionTable[i][ACTION_ALGOID_ID];
				ActionTable[i][TASK_NUMBER]=0;				// Reset/Libère l'occupation de la tâche
				ActionTable[i][ACTION_ALGOID_ID]= 0;
				ActionTable[i][ACTION_COUNT]=0;
				return(algoidMsgId);						// Retourn le numéro d'action terminé
			} else return 0;								// Action non terminées
		}
	}
	return(-1);												// Tâche inexistante
}
