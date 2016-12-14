#define TASK_NUMBER 0
#define ACTION_ALGOID_ID 1
#define ACTION_COUNT 2

#define MILLISECOND   0
#define CENTIMETER	  1

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "buggy_descriptor.h"
#include "algoidCom/messagesManager.h"
#include "algoidCom/linux_json.h"
#include "algoidCom/udpPublish.h"
#include "tools.h"
#include "algoid_2wd_buggy.h"
#include "timerManager.h"
#include "hwControl/hwManager.h"

int ActionTable[10][3];

unsigned char ptrSpeedCalc;
int createBuggyTask(int MsgId, int actionCount);
int removeBuggyTask(int actionNumber);

void distanceEventCheck(void);
void batteryEventCheck(void);
void DINEventCheck(void);


////----------------------------------- A REMPLACER PAR STRUCT
// Data sensors
int distance[5], angle[5]={1,2,3,4,5};
int speed[5], dist[5];

int battery[5];
int DIN[5]={11, 22, 33, 44, 55};

float startEncoderValue[2];
float stopEncoderValue[2];

unsigned char distWarningSended;
unsigned char DIST_EVENT_ENABLE[5] = {0,0,0,0,0};
unsigned int DIST_EVENT_LOW[5]={0,0,0,0,0};
unsigned int DIST_EVENT_HIGH[5]={65536, 65536, 65536, 65536, 65536};

unsigned char battWarningSended;
unsigned char BATT_EVENT_ENABLE[5]={0,0,0,0,0};
unsigned int BATT_EVENT_LOW[5]={0,0,0,0,0};
unsigned int BATT_EVENT_HIGH[5]={65536, 65536, 65536, 65536, 65536};


unsigned char DIN_EVENT_ENABLE[5]={0,0,0,0,0};
unsigned char DIN_HAS_CHANGE[5]={0,0,0,0,0};
//// -----------------------------------------FIN REMPLACEMENT


// Traitement du message algoid recu
int processAlgoidCommand(void);
int processAlgoidRequest(void);

int makeSensorsRequest(void);
int makeDistanceRequest(void);
int makeBatteryRequest(void);

int make2WDaction(void);
int makeServoAction(void);
int setWheelAction(int actionNumber, int wheelName, int veloc, char unit, int value);
int endWheelAction(int actionNumber, int wheelNumber);
int checkMotorEncoder(int actionNumber, int encoderName);

int getWDvalue(int wheelName);
int getServoSetting(int servoName);
char reportBuffer[256];


// -------------------------------------------------------------------
// MAIN APPLICATION
// - Création de tâche de gestion de la messagerie avec ALGOID, (ALGOID->JSON->MQTT BROCKER->JSON->BUGGY)
// - Création de tâche de gestion des timers pour la commande ON/OFF des roues, de l'accélération des roues, et timer @ 50mS, 100mS, 10Sec
// - Initialisation de la carte de commande hardware pour les moteurs, capteurs, etc...
// - Initialisation d'un broadcast UDP pour publication de la pésence du buggy sur le réseau
// -------------------------------------------------------------------

int main(void) {

	system("clear");
// Création de la tâche pour la gestion de la messagerie avec ALGOID
	if(InitMessager()) printf ("# Creation tâche messagerie : ERREUR\n");
	else printf ("# Demarrage tache Messager: OK\n");

// Création de la tâche pour la gestion des différents timers utilisés
	if(InitTimerManager()) printf ("# Creation tâche timer : ERREUR\n");
		else printf ("# Demarrage tache timer: OK\n");

// Création de la tâche pour la gestion hardware
	if(InitHwManager()) printf ("# Creation tâche hardware : ERREUR\n");
		else printf ("# Demarrage tache hardware: OK\n");

// Initialisation UDP pour broadcast IP Adresse
	initUDP();
/*
// Initialisation des périphériques de la carte hardware
	if(buggyBoardInit()){
		printf("# Initialisation carte HW: OK\n");
		sendMqttReport(0,"# Initialisation carte HW: OK\n");
		// Test
		setLedPower(0, 50);
	}
	else{
		printf("# Initialisation carte HW: ERREUR\n");
		sendMqttReport(0,"# Initialisation carte HW: ERREUR\n");
	}
*/


// --------------------------------------------------------------------
// BOUCLE DU PROGRAMME PRINCIPAL
// - Messagerie avec ALGOID, attentes de messages en provenance de l'hôte -> Démarrage du traitement des commandes
// - Annonce UDP de présence du buggy sur le réseau  chaque 10Seconde
// - Gestion de l'acceleration du Buggy
// - Mesure sur les capteurs de distance, DIN et batterie
// - Gestion des évenements provoqués par les capteurs
// --------------------------------------------------------------------

	// ----------- DEBUT DE LA BOUCLE PRINCIPALE ----------

	while(1){

		// Contrôle de la messagerie, recherche d'éventuels messages ALGOID et effectue les traitements nécéssaire
		// selon le type du message [COMMAND, REQUEST, NEGOCIATION, ACK, REPONSE, ERROR, etc...]
		if(pullMsgStack(0)){
			switch(AlgoidCommand.msgType){
				case COMMAND : processAlgoidCommand(); break;						// Traitement du message de type "COMMAND"
				case REQUEST : processAlgoidRequest(); break;						// Traitement du message de type "REQUEST"
				default : printf("\[main]->Commande non prise en charge..."); break;
			}
		}


		// Gestion de la vélocité pour une acceleration proggressive
    	// modification de la vélocité environ chaque 50mS
    	if(checkMotorPowerFlag){
			checkDCmotorPower();													// Contrôle si la vélocité correspond à la consigne
			checkMotorPowerFlag=0;
    	}


		// Contrôle du TIMER 10seconde
    	if(t10secFlag){
    		// Envoie un message UDP sur le réseau, sur port 53530 (CF udpPublish.h)
    		// Avec le ID du buggy (fourni par le gestionnaire de messagerie)
    		char udpMessage[50];
    		sprintf(&udpMessage[0], "[ %s ] I'm here",ClientID);		// Formattage du message avec le Nom du client buggy
    		sendUDPHeartBit(udpMessage);								// Envoie du message
		  // printf("\n MYMAC %s", getMACaddr());
    		t10secFlag=0;
    	}


		// Contrôle du TIMER 100mS
    	// - Récupération de la tension de batterie
    	// - Récupération de la distance mesurée au sonar
    	// - Gestion des évenements batterie, digital inputs et distance
    	if(t100msFlag){

			DINEventCheck();											// Contôle de l'état des entrées numérique
																		// Génère un évenement si changement d'état détecté
			distance[0] = getSonarDistance();
			distanceEventCheck();										// Provoque un évenement de type "distance" si la distance mesurée
																		// est hors de la plage spécifiée par l'utilisateur

			battery[0] = getBatteryVoltage();
			batteryEventCheck();

			dist[0]=getMotorPulses(MOTOR_LEFT);
			dist[1]=getMotorPulses(MOTOR_RIGHT);

			speed[0]=getMotorFrequency(0);
			speed[1]=getMotorFrequency(1);
			// est hors la plage spécifiée par les paramettre utilisateur


			//printf("\n [main loop] Battery: %dmV   Sonar:%dcm",battery[0], distance[0]);

			printf("\nSpeed : G %.1f   D %.1f   ||| Dist G: %.1fcm  Dist D: %.1fcm", speed[0]*0.285, speed[1]*0.285, dist[0]*0.285, dist[1]*0.285);
			printf(" dist US: %d cm\n", distance[0]);

			t100msFlag=0;												// Quittance le flag 100mS
    	}

    	usleep(1000);													// Attente de 1ms
    }
	// ------------ FIN DE LA BOUCLE PRINCIPALE ----------------------


	// Fermetur du programme
	int endState=CloseMessager();										// Ferme la tâche de messagerie
	if(!endState)
		  printf( "# ARRET tache Messager - status: %d\n", endState);
	else printf( "# ARRET tache Messager erreur - status: %d\n", endState);

	return EXIT_SUCCESS;												// Fin du programme
}

// -------------------------------------------------------------------
// PROCESSCOMMAND
// Séléctionne et traite le paramètre de commande recue [LL2WD, BACK, FORWARD, STOP, SPIN, etc...]
// -------------------------------------------------------------------
int processAlgoidCommand(void){
	switch(AlgoidCommand.msgParam){
		case LL_2WD : 	make2WDaction(); break;			// Action avec en paramètre MOTEUR, VELOCITE, ACCELERATION, TEMPS d'action
		case SERVO  : 	makeServoAction();break;
		default : break;
	}

	return 0;
}

// -------------------------------------------------------------------
// PROCESSREQUEST
// Séléction et traite le paramètre de requete recu [DISTANCE, TENSION BATTERIE, ENTREE DIGITAL, etc...]
// -------------------------------------------------------------------
int processAlgoidRequest(void){

	switch(AlgoidCommand.msgParam){
		case DISTANCE : makeDistanceRequest();					// Requete de distance
						break;

		case BATTERY :  makeBatteryRequest();					// Requete de tension batterie
						break;

		case DINPUT :	makeSensorsRequest();					// Requet d'état des entrées digitale
						break;

		default : break;
	}
	return 0;
}


// -------------------------------------------------------------------
// make2WDaction
// Effectue une action avec les paramètre recus: MOTEUR, VELOCITE, ACCELERATION, TEMPS d'action
// -------------------------------------------------------------------
int make2WDaction(void){
	int ptrData;
	int myTaskId;
	unsigned char actionCount=0;

	// Recherche s'il y a des paramètres pour chaque roue
	// Des paramètres recu pour une roue crée une action à effectuer
	if(getWDvalue(MOTOR_LEFT)>=0) actionCount++;
	if(getWDvalue(MOTOR_RIGHT)>=0) actionCount++;

	// Ouverture d'une tâche pour les toutes les actions du message algoid à effectuer
	// Recois un numéro de tache en retour
	myTaskId=createBuggyTask(AlgoidCommand.msgID, actionCount);			// 2 actions pour mouvement 2WD

	// Démarrage des actions
	if(myTaskId>0){
		printf("Creation de tache WHEEL: #%d avec %d actions\n", myTaskId, actionCount);

		// Récupération des paramètres d'action  pour la roue "LEFT"
		ptrData=getWDvalue(MOTOR_LEFT);
		if(ptrData >=0){
			// Enregistre la donnée d'acceleration si disponible (<0)
			if(AlgoidCommand.msgValArray[ptrData].accel!=0 || AlgoidCommand.msgValArray[ptrData].decel!=0)
				setMotorAccelDecel(MOTOR_LEFT, AlgoidCommand.msgValArray[ptrData].accel, AlgoidCommand.msgValArray[ptrData].decel);
			// Effectue l'action sur la roue
			if(AlgoidCommand.msgValArray[ptrData].cm != 0)
				setWheelAction(myTaskId, MOTOR_LEFT, AlgoidCommand.msgValArray[ptrData].velocity, CENTIMETER, AlgoidCommand.msgValArray[ptrData].cm);
			else
				setWheelAction(myTaskId, MOTOR_LEFT, AlgoidCommand.msgValArray[ptrData].velocity, MILLISECOND, AlgoidCommand.msgValArray[ptrData].time);
		}

		// Récupération des paramètres d'action  pour la roue "RIGHT"
		ptrData=getWDvalue(MOTOR_RIGHT);
		if(ptrData >=0){
			// Enregistre la donnée d'acceleration si disponible (<0)
			if(AlgoidCommand.msgValArray[ptrData].accel>0 || AlgoidCommand.msgValArray[ptrData].decel>0)
				setMotorAccelDecel(MOTOR_RIGHT, AlgoidCommand.msgValArray[ptrData].accel, AlgoidCommand.msgValArray[ptrData].decel);

			if(AlgoidCommand.msgValArray[ptrData].cm != 0)
				setWheelAction(myTaskId, MOTOR_RIGHT, AlgoidCommand.msgValArray[ptrData].velocity, CENTIMETER, AlgoidCommand.msgValArray[ptrData].cm);
			else
				setWheelAction(myTaskId, MOTOR_RIGHT, AlgoidCommand.msgValArray[ptrData].velocity, MILLISECOND, AlgoidCommand.msgValArray[ptrData].time);
		}

		// Retourne un message ALGOID si velocité hors tolérences
		if((AlgoidCommand.msgValArray[ptrData].velocity < -100) ||(AlgoidCommand.msgValArray[ptrData].velocity > 100))
			sendResponse(AlgoidCommand.msgID, WARNING, LL_2WD, 0);
		return 0;
	}
	else return 1;
}


// -------------------------------------------------------------------
// makeServoAction
//
// -------------------------------------------------------------------
int makeServoAction(void){
	int ptrData;
	int myTaskId;
	int endOfTask;

	unsigned char actionCount=0;
	unsigned char action;

	// Recherche s'il y a des paramètres pour chaque roue
	// Des paramètres recu pour une roue crée une action à effectuer
	if(getServoSetting(SERVO_0)>=0) actionCount++;
	if(getServoSetting(SERVO_1)>=0) actionCount++;
	if(getServoSetting(SERVO_2)>=0) actionCount++;

	// Ouverture d'une tâche pour les toutes les actions du message algoid à effectuer
	// Recois un numéro de tache en retour
	myTaskId=createBuggyTask(AlgoidCommand.msgID, actionCount);			//

	// Démarrage des actions
	if(myTaskId>0){
		printf("Creation de tache SERVO: #%d avec %d actions\n", myTaskId, actionCount);

		for(ptrData=0; action < actionCount && ptrData<10; ptrData++){
			if(AlgoidCommand.SERVOmotor[ptrData].id>0){
				setServoPosition(AlgoidCommand.SERVOmotor[ptrData].id, AlgoidCommand.SERVOmotor[ptrData].angle);

				endOfTask=removeBuggyTask(myTaskId);
				if(endOfTask>0){
					sprintf(reportBuffer, "FIN DES ACTIONS \"SERVO\" pour la tache #%d\n", endOfTask);
					sendResponse(endOfTask, EVENT, SERVO, 0);			// Envoie un message ALGOID de fin de tâche pour l'action écrasé
					printf(reportBuffer);									// Affichage du message dans le shell
					sendMqttReport(endOfTask, reportBuffer);				// Envoie le message sur le canal MQTT "Report"
				}

				action++;
			}
		}
	}
	return 0;
}



// -------------------------------------------------------------------
// SETWHEELACTION
// Effectue l'action sur une roue spécifiée
// - Démarrage du timer avec definition de fonction call-back, et no d'action
// - Démarrage du mouvement de la roue spécifiée
// - Vélocité entre -100 et +100 qui défini le sens de rotation du moteur
// -------------------------------------------------------------------

int setWheelAction(int actionNumber, int wheelName, int veloc, char unit, int value){
	int myDirection;
	int setTimerResult;
	int endOfTask;
	unsigned char wheelNumber;

	// Conversion de la vélocité de -100...+100 en direction AVANCE ou RECULE
	if(veloc > 0)
		myDirection=BUGGY_FORWARD;
	if(veloc == 0)
		myDirection=BUGGY_STOP;
	if(veloc < 0){
		myDirection=BUGGY_BACK;
		veloc *=-1;					// Convertion en valeur positive
	}

	// Démarre de timer d'action sur la roue et spécifie la fonction call back à appeler en time-out
	// Valeur en retour >0 signifie que l'action "en retour" à été écrasée
	switch(unit){
		case  MILLISECOND:  setTimerResult=setTimerWheel(value, &endWheelAction, actionNumber, wheelName); break;
		case  CENTIMETER:   wheelNumber = getOrganNumber(wheelName);
							startEncoderValue[wheelNumber]=getMotorPulses(wheelName)*0.285;
							stopEncoderValue[wheelNumber] = startEncoderValue[wheelNumber]+ value;
							//printf("\n Encodeur #%d -> START %.2f cm  STOP %.2f cm", wheelNumber, distance, startEncoderValue[wheelNumber], stopEncoderValue[wheelNumber]);
						    setTimerResult=setTimerWheel(50, &checkMotorEncoder, actionNumber, wheelName);			// Démarre un timer pour contrôle de distance chaque 35mS
						    break;
		default: printf("\n!!! ERROR Function [setWheelAction] -> undefined unit");break;
	}

	if(setTimerResult!=0){								// Timer pret, action effectuée
		if(setTimerResult>1){							// Le timer à été écrasé par la nouvelle action en retour car sur la même roue
			endOfTask=removeBuggyTask(setTimerResult);	// Supprime l'ancienne tâche qui à été écrasée par la nouvelle action
			if(endOfTask){
				sendResponse(endOfTask, EVENT, LL_2WD, 0);			// Envoie un message ALGOID de fin de tâche pour l'action écrasé
				sprintf(reportBuffer, "FIN DES ACTIONS \"WHEEL\" pour la tache #%d\n", endOfTask);
				printf(reportBuffer);									// Affichage du message dans le shell
				sendMqttReport(endOfTask, reportBuffer);				// Envoie le message sur le canal MQTT "Report"
			}
		}

		// Défini le "nouveau" sens de rotation à applique au moteur ainsi que la consigne de vitesse
		if(setMotorDirection(wheelName, myDirection)){							// Sens de rotation
			setMotorSpeed(wheelName, veloc);									// Vitesse

			// Envoie de message ALGOID et SHELL
			sprintf(reportBuffer, "Start wheel %d with velocity %d for time %d\n",wheelNumber, veloc, value);
			printf(reportBuffer);
			sendMqttReport(actionNumber, reportBuffer);
		}
		else{
			sprintf(reportBuffer, "Error, impossible to start wheel %d\n",wheelName);
			printf(reportBuffer);
			sendMqttReport(actionNumber, reportBuffer);
		}

	}
	else printf("Error, Impossible to set timer wheel\n");
	return 0;
}

// -------------------------------------------------------------------
// END2WDACTION
// Fin de l'action sur une roue
// Fonction appelée après le timout défini par l'utilisateur, Stop le moteur spécifié
// -------------------------------------------------------------------
int endWheelAction(int actionNumber, int wheelNumber){
	int endOfTask;
	//printf("Action number: %d - End of timer for wheel No: %d\n",actionNumber , wheelNumber);

	// Stop le moteur
	setMotorSpeed(wheelNumber, 0);

	// Retire l'action de la table et vérification si toute les actions sont effectuées
	// Pour la tâche en cours donnée par le message ALGOID

	endOfTask = removeBuggyTask(actionNumber);

	// Contrôle que toutes les actions ont été effectuée pour la commande recue dans le message ALGOID
	if(endOfTask){
		sendResponse(endOfTask, RESPONSE, LL_2WD, 0);
		sprintf(reportBuffer, "FIN DES ACTIONS \"WHEEL\" pour la tache #%d\n", endOfTask);
		printf(reportBuffer);
		sendMqttReport(endOfTask, reportBuffer);
	}

	return 0;
}


// ----------------------------------------------------------------------
// CHECKMOTORENCODER
// Contrôle la distance parcourue et stop la roue si destination atteinte
// Fonction appelée après le timout défini par l'utilisateur.
// -----------------------------------------------------------------------

int checkMotorEncoder(int actionNumber, int encoderName){
	float distance;					// Variable de distance parcourue depuis le start
	unsigned char encoderNumber;

	encoderNumber = getOrganNumber(encoderName);

	distance = getMotorPulses(encoderName);

	if(distance >=0){
		distance = (distance*0.285);
    	usleep(2200);
	}else  printf("\n ERROR: I2CBUS READ\n");
	//printf("\n Encodeur #%d -> START %.2f cm  STOP %.2f cm", encoderNumber, startEncoderValue[encoderNumber], stopEncoderValue[encoderNumber]);

	if(distance >= stopEncoderValue[encoderNumber])
		endWheelAction(actionNumber, encoderName);
	else
		setTimerWheel(50, &checkMotorEncoder, actionNumber, encoderName);

	return 0;
}

// -------------------------------------------------------------------
// GETWDVALUE
// Recherche dans le message algoid, les paramètres
// [Vélocité, acceleration, sens de rotation et temps d'action] pour la roue spécifiée
// Retourne un pointeur sur le champs de paramètre correspondant à la rou spécifié
// -------------------------------------------------------------------
int getWDvalue(int wheelName){
	int i;
	int searchPtr = -1;

	// Recherche dans les donnée recues la valeur correspondante au paramètre "wheelName"
	for(i=0;i<AlgoidCommand.msgValueCnt;i++){
		if(wheelName == AlgoidCommand.msgValArray[i].wheel)
			searchPtr=i;
	}
	return searchPtr;
}

// -------------------------------------------------------------------
// GETSERVOSETTING
// Recherche dans le message algoid, les paramètres
// [velocité, angle, etat] pour une servomoteur spécifié
// Retourne un pointeur sur le champs de paramètre correspondant au servomoteur spécifié
// -------------------------------------------------------------------
int getServoSetting(int servoName){
	int i;
	int searchPtr = -1;

	// Recherche dans les donnée recues la valeur correspondante au paramètre "wheelName"
	for(i=0;i<AlgoidCommand.msgValueCnt;i++){
		if(servoName == AlgoidCommand.SERVOmotor[i].id)
		searchPtr=i;
	}
	return searchPtr;
}



// -------------------------------------------------------------------
// CREATBUGGYTASK Creation d'une tache avec le nombre
// d'actions à effectuer,
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

	// Recherche un emplacement libre dans la table d'action pour inserer les paramètre
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
	sprintf(reportBuffer, "ERREUR: Table de tâches pleine\n");
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
// Envoie une message ALGOID de type "response" avec l'état des entrées DIN
// -------------------------------------------------------------------
int makeSensorsRequest(void){
	unsigned char i;

	// Pas de paramètres spécifiés dans le message, retourne l'ensemble des états des DIN
	if(AlgoidCommand.msgValueCnt==0){
		AlgoidCommand.msgValueCnt=2;
		for(i=0;i<2;i++){
			AlgoidResponse[i].DINresponse.id=i;
			AlgoidResponse[i].value=DIN[i];
		}

	}else
		// Retourne les états spécifiés dans le message des DIN
		for(i=0;i<AlgoidCommand.msgValueCnt; i++){
			// Recherche de paramètres supplémentaires

			if(!strcmp(AlgoidCommand.DINsens[i].event_state, "on"))	DIN_EVENT_ENABLE[AlgoidCommand.DINsens[i].id]=1; 			// Activation de l'envoie de messages sur évenements
			else if(!strcmp(AlgoidCommand.DINsens[i].event_state, "off"))	DIN_EVENT_ENABLE[AlgoidCommand.DINsens[i].id]=0;    // Désactivation de l'envoie de messages sur évenements
//			printf("DIN: %d, value: %d   modeAuto: %d\n", buffMsgValArray[i][0], buffMsgValArray[i][1], DIN_EVENT_ENABLE[AlgoidCommand.value[i]]);
		};

	for(i=0;i<AlgoidCommand.msgValueCnt; i++){
		AlgoidResponse[i].DINresponse.id=AlgoidCommand.DINsens[i].id;
		AlgoidResponse[i].value=DIN[AlgoidCommand.DINsens[i].id];
		if(DIN_EVENT_ENABLE[i])strcpy(AlgoidResponse[i].DINresponse.event_state, "on");
				else strcpy(AlgoidResponse[i].DINresponse.event_state, "off");
	};

	// Envoie de la réponse MQTT
	sendResponse(AlgoidCommand.msgID, RESPONSE, DINPUT, AlgoidCommand.msgValueCnt);
	return (1);
}

// -------------------------------------------------------------------
// MAKEDISTANCEREQUEST
// Traitement de la requete de mesure de distance
// // Récupère les valeurs des paramètres "EVENT", "EVENT_HIGH", "EVENT_LOW", ANGLE
// Envoie un message ALGOID de type "response" avec la valeur distance mesurée
// -------------------------------------------------------------------
int makeDistanceRequest(void){
	unsigned char i;

	// Pas de paramètres spécifié dans le message, retourne l'ensemble des distances
	if(AlgoidCommand.msgValueCnt==0){
		AlgoidCommand.msgValueCnt=1;
		for(i=0;i<2;i++){
			AlgoidResponse[i].DISTresponse.id=i;
			AlgoidResponse[i].value=distance[i];
		}
	}else
		// Recherche et enregistrement de paramètres supplémentaires si disponible
			for(i=0;i<AlgoidCommand.msgValueCnt; i++){
				// Activation de l'envoie de messages sur évenements
				if(!strcmp(AlgoidCommand.DISTsens[i].event_state, "on")) DIST_EVENT_ENABLE[AlgoidCommand.DISTsens[i].id]=1;
				else if(!strcmp(AlgoidCommand.DISTsens[i].event_state, "off")) DIST_EVENT_ENABLE[AlgoidCommand.DISTsens[i].id]=0;
				// Evemenent haut
				if(AlgoidCommand.DISTsens[i].event_high!=0) DIST_EVENT_HIGH[AlgoidCommand.DISTsens[i].id]=AlgoidCommand.DISTsens[i].event_high;
				if(AlgoidCommand.DISTsens[i].event_high!=0) DIST_EVENT_LOW[AlgoidCommand.DISTsens[i].id]=AlgoidCommand.DISTsens[i].event_low;
			};

	for(i=0;i<AlgoidCommand.msgValueCnt; i++){
		// Récupération des paramètres actuels et chargement du buffer de reéponse
		AlgoidResponse[i].DISTresponse.id=AlgoidCommand.DISTsens[i].id;
		AlgoidResponse[i].value=distance[AlgoidCommand.DISTsens[i].id];
		AlgoidResponse[i].DISTresponse.angle=angle[AlgoidCommand.DISTsens[i].id];

		if(DIST_EVENT_ENABLE[i])strcpy(AlgoidResponse[i].DISTresponse.event_state, "on");
		else strcpy(AlgoidResponse[i].DISTresponse.event_state, "off");
		AlgoidResponse[i].DISTresponse.event_high=DIST_EVENT_HIGH[AlgoidCommand.DISTsens[i].id];
		AlgoidResponse[i].DISTresponse.event_low=DIST_EVENT_LOW[AlgoidCommand.DISTsens[i].id];
	};

	// Envoie de la réponse MQTT
	sendResponse(AlgoidCommand.msgID, RESPONSE, DISTANCE, AlgoidCommand.msgValueCnt);

		return 1;
}


// -------------------------------------------------------------------
// MAKEBATTERYREQUEST
// Traitement de la requete de mesure de tension batterie
// Récupère les valeurs des paramètres "EVENT", "EVENT_HIGH", "EVENT_LOW"
// Envoie un message ALGOID de type "response" avec la valeur des paramètres enregistrés
// -------------------------------------------------------------------

int makeBatteryRequest(void){
	unsigned char i;

	// Pas de paramètres spécifié dans le message, retourne l'ensemble des états des batteries
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
			};

	for(i=0;i<AlgoidCommand.msgValueCnt; i++){
		AlgoidResponse[i].BATTesponse.id=AlgoidCommand.BATTsens[i].id;
		AlgoidResponse[i].value=battery[AlgoidCommand.BATTsens[i].id];
		if(BATT_EVENT_ENABLE[i])strcpy(AlgoidResponse[i].BATTesponse.event_state, "on");
		else strcpy(AlgoidResponse[i].BATTesponse.event_state, "off");
		AlgoidResponse[i].BATTesponse.event_high=BATT_EVENT_HIGH[AlgoidCommand.BATTsens[i].id];
		AlgoidResponse[i].BATTesponse.event_low=BATT_EVENT_LOW[AlgoidCommand.BATTsens[i].id];
	};

	// Envoie de la réponse MQTT
	sendResponse(AlgoidCommand.msgID, RESPONSE, BATTERY, AlgoidCommand.msgValueCnt);
		return 1;
}

// -------------------------------------------------------------------
// DISTANCEEVENTCHECK
// Contrôle si la distance mesurée est hors de la plage défini par l'utilisateur
// et envoie un message de type "event" si tel est le cas.
// Un deuxième "event" est envoyé lorsque la mesure de distance entre à nouveau dans la
// plage définie.
// -------------------------------------------------------------------
void distanceEventCheck(void){
	unsigned char i;
	// Contrôle periodique des mesures de distances pour envoie d'evenement
	for(i=0;i<2;i++){
		// Vérification si envoie des EVENT activés
		if(DIST_EVENT_ENABLE[i]){
			if((distance[i]<DIST_EVENT_LOW[i]) || (distance[i]>DIST_EVENT_HIGH[i])){		// Mesure de distance hors plage
				if(distWarningSended==0){													// N'envoie l' event qu'une seule fois
					AlgoidResponse[i].DISTresponse.id=i;
					AlgoidResponse[i].value=distance[i];
					sendResponse(AlgoidCommand.msgID, EVENT, DISTANCE, 1);
					distWarningSended=1;
				}
			}
			else if (distWarningSended==1){													// Mesure de distance revenu dans la plage
					AlgoidResponse[i].DISTresponse.id=i;									// Et n'envoie qu'une seule fois le message
					AlgoidResponse[i].value=distance[i];
					sendResponse(AlgoidCommand.msgID, EVENT, DISTANCE, 1);
					distWarningSended=0;
			}
		}
	}
}


// -------------------------------------------------------------------
// BATTERYEVENTCHECK
// Contrôle si la tension mesurée est hors de la plage défini par l'utilisateur
// et envoie un message de type "event" si tel est le cas.
// Un deuxième "event" est envoyé lorsque la tension batterie entre à nouveau dans la
// plage définie.
// -------------------------------------------------------------------
// -------------------------------------------------------------------
void batteryEventCheck(void){
	unsigned char i;
	// Contrôle periodique des mesures de tension batterie pour envoie d'evenement
	for(i=0;i<2;i++){
		if(BATT_EVENT_ENABLE[i]){
			if((battery[i]<BATT_EVENT_LOW[i]) || (battery[i]>BATT_EVENT_HIGH[i])){				// Mesure tension hors plage
				if(battWarningSended==0){														// N'envoie qu'une seule fois l'EVENT
					AlgoidResponse[i].BATTesponse.id=i;
					AlgoidResponse[i].value=battery[i];
					sendResponse(AlgoidCommand.msgID, EVENT, BATTERY, 1);
					battWarningSended=1;
				}
			}
			// Envoie un évenement Fin de niveau bas (+50mV Hysterese)
			else if (battWarningSended==1 && battery[i]>(BATT_EVENT_LOW[i]+50)){				// Mesure tension dans la plage
					AlgoidResponse[i].BATTesponse.id=i;											// n'envoie qu'une seule fois après
					AlgoidResponse[i].value=battery[i];											// une hysterese de 50mV
					sendResponse(AlgoidCommand.msgID, EVENT, BATTERY, 1);
					battWarningSended=0;
			}
		}
	}
}


// -------------------------------------------------------------------
// DINEVENTCHECK
// Vérifie si une changement d'état à eu lieu sur les entrées numériques
// et envoie un event si tel est les cas.
// Seul les DIN ayant changé d'état font partie du message de réponse
// -------------------------------------------------------------------
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
		sendResponse(AlgoidCommand.msgID, EVENT, DINPUT, DINevent);
}


