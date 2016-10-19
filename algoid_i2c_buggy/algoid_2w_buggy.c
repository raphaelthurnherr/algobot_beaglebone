
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
unsigned int bufferSpeedCalcArray[2];
unsigned char ptrSpeedCalc;
int createBuggyTask(int MsgId, int actionCount);
int removeBuggyTask(int actionNumber);

void distanceEventCheck(void);
void batteryEventCheck(void);
void DINEventCheck(void);

// Data sensors
int distance[5], angle[5]={1,2,3,4,5};

int battery[5];
int DIN[5]={11, 22, 33, 44, 55};

unsigned char distWarningSended;
unsigned char DIST_EVENT_ENABLE[5];
unsigned int DIST_EVENT_LOW[5]={0,0,0,0,0};
unsigned int DIST_EVENT_HIGH[5]={65536, 65536, 65536, 65536, 65536};

unsigned char battWarningSended;
unsigned char BATT_EVENT_ENABLE[5];
unsigned int BATT_EVENT_LOW[5]={0,0,0,0,0};
unsigned int BATT_EVENT_HIGH[5]={65536, 65536, 65536, 65536, 65536};


unsigned char DIN_EVENT_ENABLE[5];
unsigned char DIN_HAS_CHANGE[5];

// Traitement du message algoid recu
int processAlgoidCommand(void);
int processAlgoidRequest(void);

int makeSensorsRequest(void);
int makeDistanceRequest(void);
int makeBatteryRequest(void);

int make2WDaction(void);
int setWheelAction(int actionNumber, int wheelNumber, int veloc, int time);
int endWheelAction(int actionNumber, int wheelNumber);

int getWDvalue(char * wheelName);
char reportBuffer[256];

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

		// COMMANDE ALGOID RECUE
		if(pullMsgStack(0)){
//			printf("[main] messageID:  %d  param: %d   cmd: %d\n\n",AlgoidCommand.msgID,AlgoidCommand.msgParam,AlgoidCommand.msgType );
			switch(AlgoidCommand.msgType){
				case COMMAND : processAlgoidCommand(); break;
				case REQUEST : processAlgoidRequest(); break;
				default : break;
			}
		}

    	if(t10secFlag){
    		char udpMessage[50];
    		sprintf(&udpMessage[0], "[ %s ] I'm here",ClientID);
    		sendUDPHeartBit(udpMessage);
		  // printf("\n MYMAC %s", getMACaddr());
    		t10secFlag=0;
    	}


    	if(checkMotorPowerFlag){
			checkDCmotorPower();
			checkMotorPowerFlag=0;
    	}


    	if(t100msFlag){
    				battery[0] = getBatteryVoltage();
		    		distance[0] = getSonarDistance();
		    		if(distance[0]<0) printf("Erreur de lecture Distance");
		    		bufferSpeedCalcArray[ptrSpeedCalc]=distance[0];
		    		if(ptrSpeedCalc++>=2)
		    			ptrSpeedCalc=0;

		    		signed char speedCmSec=(bufferSpeedCalcArray[0]-bufferSpeedCalcArray[1])*10;


		    		DINEventCheck();
		    		distanceEventCheck();
		    		batteryEventCheck();

//    		    	printf("Dist. inst mm: %d speed cm/sec: %d Battery: %d    DIN0: %d     DIN1: %d   \n",distance[0], speedCmSec, battery[0], DIN[0], DIN[1]);
    		    	t100msFlag=0;
    	}

    	usleep(1000);
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
int processAlgoidCommand(void){
	switch(AlgoidCommand.msgParam){
		case LL_2WD : 	make2WDaction(); break;
		default : break;
	}
	return 0;
}

// -------------------------------------------------------------------
// PROCESSREQUEST
// -------------------------------------------------------------------
int processAlgoidRequest(void){
	unsigned char i;

	switch(AlgoidCommand.msgParam){
		case DISTANCE : makeDistanceRequest();
						break;

		case BATTERY :  makeBatteryRequest();
						break;

		case DINPUT :	makeSensorsRequest();
						break;

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
			sendResponse(AlgoidCommand.msgID, "warning", "2wd", "test", 0);
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

// -------------------------------------------------------------------
// MAKESENSORREQUEST
// Traitement de la requete SENSORS
// -------------------------------------------------------------------
int makeSensorsRequest(void){
	unsigned char i;

	// Pas de paramètres spécifiés, retourne l'ensemble des états des DIN
	if(AlgoidCommand.msgValueCnt==0){
		AlgoidCommand.msgValueCnt=2;
		for(i=0;i<2;i++){
			AlgoidResponse[i].DINresponse.id=i;
			AlgoidResponse[i].value=DIN[i];
		}
		printf("countmake2: %d\n", AlgoidCommand.msgValueCnt);
	}else
		// Retourne les états spécifiés des DIN
		for(i=0;i<AlgoidCommand.msgValueCnt; i++){
			// Recherche de paramètres supplémentaires
			if(!strcmp(AlgoidCommand.DINsens[i].event_state, "on"))	DIN_EVENT_ENABLE[AlgoidCommand.DINsens[i].id]=1;
			else if(!strcmp(AlgoidCommand.DINsens[i].event_state, "off"))	DIN_EVENT_ENABLE[AlgoidCommand.DINsens[i].id]=0;
			AlgoidResponse[i].DINresponse.id=AlgoidCommand.DINsens[i].id;
			AlgoidResponse[i].value=DIN[AlgoidCommand.DINsens[i].id];
//			printf("DIN: %d, value: %d   modeAuto: %d\n", buffMsgValArray[i][0], buffMsgValArray[i][1], DIN_EVENT_ENABLE[AlgoidCommand.value[i]]);
		};

	sendResponse(AlgoidCommand.msgID, "response", "sensors", SENSORS_STATE, AlgoidCommand.msgValueCnt);
	return (1);
}


int makeDistanceRequest(void){
	unsigned char i;

	// Pas de paramètres spécifié, retourne l'ensemble des états des DIN
	if(AlgoidCommand.msgValueCnt==0){
		AlgoidCommand.msgValueCnt=1;
		for(i=0;i<2;i++){
			AlgoidResponse[i].DISTresponse.id=i;
			AlgoidResponse[i].value=distance[i];
		}
	}else
			for(i=0;i<AlgoidCommand.msgValueCnt; i++){
				// Recherche de paramètres supplémentaires
				// Evenement activées
				if(!strcmp(AlgoidCommand.DISTsens[i].event_state, "on")) DIST_EVENT_ENABLE[AlgoidCommand.DISTsens[i].id]=1;
				else if(!strcmp(AlgoidCommand.DISTsens[i].event_state, "off")) DIST_EVENT_ENABLE[AlgoidCommand.DISTsens[i].id]=0;
				// Evemenent haut
				if(AlgoidCommand.DISTsens[i].event_high!=0) DIST_EVENT_HIGH[AlgoidCommand.DISTsens[i].id]=AlgoidCommand.DISTsens[i].event_high*10;
				if(AlgoidCommand.DISTsens[i].event_high!=0) DIST_EVENT_LOW[AlgoidCommand.DISTsens[i].id]=AlgoidCommand.DISTsens[i].event_low*10;

				AlgoidResponse[i].DISTresponse.id=AlgoidCommand.DISTsens[i].id;
				AlgoidResponse[i].value=distance[AlgoidCommand.DISTsens[i].id];
				AlgoidResponse[i].DISTresponse.angle=angle[AlgoidCommand.DISTsens[i].id];

				if(DIST_EVENT_ENABLE[i])strcpy(AlgoidResponse[i].DISTresponse.event_state, "on");
				else strcpy(AlgoidResponse[i].DISTresponse.event_state, "off");
				AlgoidResponse[i].DISTresponse.event_high=DIST_EVENT_HIGH[AlgoidCommand.DISTsens[i].id];
				AlgoidResponse[i].DISTresponse.event_low=DIST_EVENT_LOW[AlgoidCommand.DISTsens[i].id];
			};
		sendResponse(AlgoidCommand.msgID, "response", "distance", DISTCM, AlgoidCommand.msgValueCnt);

		return 1;
}



int makeBatteryRequest(void){
	unsigned char i;

	// Pas de paramètres spécifié, retourne l'ensemble des états des batteries
	if(AlgoidCommand.msgValueCnt==0){
		AlgoidCommand.msgValueCnt=1;
		for(i=0;i<2;i++){
			AlgoidResponse[i].BATTesponse.id=i;
			AlgoidResponse[i].value=battery[i];
		}
	}else
			for(i=0;i<AlgoidCommand.msgValueCnt; i++){
				// Recherche de paramètres supplémentaires
				// Evenement activées
				if(!strcmp(AlgoidCommand.BATTsens[i].event_state, "on")) BATT_EVENT_ENABLE[AlgoidCommand.BATTsens[i].id]=1;
				else if(!strcmp(AlgoidCommand.BATTsens[i].event_state, "off")) BATT_EVENT_ENABLE[AlgoidCommand.BATTsens[i].id]=0;
				// Evemenent haut
				if(AlgoidCommand.BATTsens[i].event_high!=0) BATT_EVENT_HIGH[AlgoidCommand.BATTsens[i].id]=AlgoidCommand.BATTsens[i].event_high;
				if(AlgoidCommand.BATTsens[i].event_high!=0) BATT_EVENT_LOW[AlgoidCommand.BATTsens[i].id]=AlgoidCommand.BATTsens[i].event_low;

				AlgoidResponse[i].BATTesponse.id=AlgoidCommand.BATTsens[i].id;
				AlgoidResponse[i].value=battery[AlgoidCommand.BATTsens[i].id];
				if(BATT_EVENT_ENABLE[i])strcpy(AlgoidResponse[i].BATTesponse.event_state, "on");
				else strcpy(AlgoidResponse[i].BATTesponse.event_state, "off");
				AlgoidResponse[i].BATTesponse.event_high=BATT_EVENT_HIGH[AlgoidCommand.BATTsens[i].id];
				AlgoidResponse[i].BATTesponse.event_low=BATT_EVENT_LOW[AlgoidCommand.BATTsens[i].id];
			};
		sendResponse(AlgoidCommand.msgID, "response", "battery", BATTVOLT, AlgoidCommand.msgValueCnt);

		return 1;
}


void distanceEventCheck(void){
	unsigned char i;
	// Contrôle periodique des mesures de distances pour envoie d'evenement
	for(i=0;i<2;i++){
		if(DIST_EVENT_ENABLE[i]){
			if((distance[i]<DIST_EVENT_LOW[i]) || (distance[i]>DIST_EVENT_HIGH[i])){
				if(distWarningSended==0){
					AlgoidResponse[i].DISTresponse.id=i;
					AlgoidResponse[i].value=distance[i];
					sendResponse(AlgoidCommand.msgID, "event", "distance", DISTCM, 1);
					distWarningSended=1;
				}
			}
			else if (distWarningSended==1){
					AlgoidResponse[i].DISTresponse.id=i;
					AlgoidResponse[i].value=distance[i];
					sendResponse(AlgoidCommand.msgID, "event", "distance", DISTCM, 1);
					distWarningSended=0;
			}
		}
	}
}

void batteryEventCheck(void){
	unsigned char i;
	// Contrôle periodique des mesures de distances pour envoie d'evenement
	for(i=0;i<2;i++){
		if(BATT_EVENT_ENABLE[i]){
			if((battery[i]<BATT_EVENT_LOW[i]) || (battery[i]>BATT_EVENT_HIGH[i])){
				if(battWarningSended==0){
					AlgoidResponse[i].BATTesponse.id=i;
					AlgoidResponse[i].value=battery[i];
					sendResponse(AlgoidCommand.msgID, "event", "battery", BATTVOLT, 1);
					battWarningSended=1;
				}
			}
			// Envoie un évenement Fin de niveau bas (+50mV Hysterese)
			else if (battWarningSended==1 && battery[i]>(BATT_EVENT_LOW[i]+50)){
					AlgoidResponse[i].BATTesponse.id=i;
					AlgoidResponse[i].value=battery[i];
					sendResponse(AlgoidCommand.msgID, "event", "battery", BATTVOLT, 1);
					battWarningSended=0;
			}
		}
	}
}

void DINEventCheck(void){
	// Mise à jour de l'état des E/S
	unsigned char ptrBuff=0, DINevent=0, oldDinValue, i;

	for(i=0;i<2;i++){
		// Mise à jour de l'état des E/S
		oldDinValue=DIN[i];
		DIN[i] = getDigitalInput(i);

		// Vérifie si un changement a eu lieu sur les entrees et transmet un message
		// "event" listant les modifications
		if(DIN_EVENT_ENABLE[i] && (oldDinValue!=DIN[i])){
			AlgoidResponse[ptrBuff].DINresponse.id=i;
			AlgoidResponse[ptrBuff].value=DIN[i];
			ptrBuff++;
			printf("CHANGEMENT DIN%d, ETAT:%d\n", i, DIN[i]);
			DINevent++;
		}
	}

	if(DINevent>0)
		sendResponse(AlgoidCommand.msgID, "event", "din", SENSORS_STATE, DINevent);
}
