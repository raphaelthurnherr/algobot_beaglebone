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

unsigned char ptrSpeedCalc;
int createBuggyTask(int MsgId, int actionCount);
int removeBuggyTask(int actionNumber);

void distanceEventCheck(void);
void batteryEventCheck(void);
void DINEventCheck(void);

void DINSafetyCheck(void);
void BatterySafetyCheck(void);
void DistanceSafetyCheck(void);


////----------------------------------- A REMPLACER PAR STRUCT
struct m_prox{
	int state;
	int event_enable;
	int safetyStop_state;
	int safetyStop_value;
	int em_stop;
};

struct m_dist{
	int value;
	int event_enable;
	int event_low;
	int event_high;
	int event_hysteresis;
	int safetyStop_state;
	int safetyStop_value;
};

struct m_voltage{
	int value;
	int event_enable;
	int event_low;
	int event_high;
	int event_hysteresis;
	int safetyStop_state;
	int safetyStop_value;
};

struct m_counter{
	float startEncoderValue;
	float stopEncoderValue;
};

typedef struct tsensor{
	struct m_prox proximity[2];
	struct m_dist distance[1];
	struct m_voltage battery[1];
	struct m_counter encoder[2];
}t_sensor;

t_sensor body;

int ActionTable[10][3];

// Data sensors
int speed[5], dist[5];



//// -----------------------------------------FIN REMPLACEMENT


// Traitement du message algoid recu
int processAlgoidCommand(void);
int processAlgoidRequest(void);

int makeSensorsRequest(void);
int makeDistanceRequest(void);
int makeBatteryRequest(void);

int make2WDaction(void);
int makeServoAction(void);
int makeLedAction(void);
int setWheelAction(int actionNumber, int wheelName, int veloc, char unit, int value);
int endWheelAction(int actionNumber, int wheelNumber);
int checkMotorEncoder(int actionNumber, int encoderName);

int getWDvalue(int wheelName);
int getServoSetting(int servoName);
char reportBuffer[256];


// -------------------------------------------------------------------
// MAIN APPLICATION
// - Cr�ation de t�che de gestion de la messagerie avec ALGOID, (ALGOID->JSON->MQTT BROCKER->JSON->BUGGY)
// - Cr�ation de t�che de gestion des timers pour la commande ON/OFF des roues, de l'acc�l�ration des roues, et timer @ 50mS, 100mS, 10Sec
// - Initialisation de la carte de commande hardware pour les moteurs, capteurs, etc...
// - Initialisation d'un broadcast UDP pour publication de la p�sence du buggy sur le r�seau
// -------------------------------------------------------------------

int main(void) {
	int i;

	system("clear");
// Cr�ation de la t�che pour la gestion de la messagerie avec ALGOID
	if(InitMessager()) printf ("# Creation t�che messagerie : ERREUR\n");
	else printf ("# Demarrage tache Messager: OK\n");

// Cr�ation de la t�che pour la gestion des diff�rents timers utilis�s
	if(InitTimerManager()) printf ("# Creation t�che timer : ERREUR\n");
		else printf ("# Demarrage tache timer: OK\n");

// Cr�ation de la t�che pour la gestion hardware
	if(InitHwManager()) printf ("# Creation t�che hardware : ERREUR\n");
		else printf ("# Demarrage tache hardware: OK\n");

// Initialisation UDP pour broadcast IP Adresse
	initUDP();
/*
// Initialisation des p�riph�riques de la carte hardware
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
// - Messagerie avec ALGOID, attentes de messages en provenance de l'h�te -> D�marrage du traitement des commandes
// - Annonce UDP de pr�sence du buggy sur le r�seau  chaque 10Seconde
// - Gestion de l'acceleration du Buggy
// - Mesure sur les capteurs de distance, DIN et batterie
// - Gestion des �venements provoqu�s par les capteurs
// --------------------------------------------------------------------

	// ----------- DEBUT DE LA BOUCLE PRINCIPALE ----------

	// Init body membre
	for(i=0;i<NBAIN;i++){
		body.battery[i].safetyStop_value=0;
		body.battery[i].event_enable=0;
		body.battery[i].event_high=65535;
		body.battery[i].event_low=0;
		body.battery[i].safetyStop_state=0;
		body.battery[i].safetyStop_value=0;
	}

	for(i=0;i<NBDIN;i++){
		body.proximity[i].em_stop=0;
		body.proximity[i].event_enable=0;
		body.proximity[i].safetyStop_state=0;
		body.proximity[i].safetyStop_value=0;
	}

	while(1){

		// Contr�le de la messagerie, recherche d'�ventuels messages ALGOID et effectue les traitements n�c�ssaire
		// selon le type du message [COMMAND, REQUEST, NEGOCIATION, ACK, REPONSE, ERROR, etc...]
		if(pullMsgStack(0)){
			switch(AlgoidCommand.msgType){
				case COMMAND : processAlgoidCommand(); break;						// Traitement du message de type "COMMAND"
				case REQUEST : processAlgoidRequest(); break;						// Traitement du message de type "REQUEST"
				default : ; break;
			}
		}


		// Gestion de la v�locit� pour une acceleration proggressive
    	// modification de la v�locit� environ chaque 50mS
    	if(checkMotorPowerFlag){
			checkDCmotorPower();													// Contr�le si la v�locit� correspond � la consigne
			checkMotorPowerFlag=0;
    	}


		// Contr�le du TIMER 10seconde
    	if(t10secFlag){
    		// Envoie un message UDP sur le r�seau, sur port 53530 (CF udpPublish.h)
    		// Avec le ID du buggy (fourni par le gestionnaire de messagerie)
    		char udpMessage[50];
    		sprintf(&udpMessage[0], "[ %s ] I'm here",ClientID);		// Formattage du message avec le Nom du client buggy
    		sendUDPHeartBit(udpMessage);								// Envoie du message
		  // printf("\n MYMAC %s", getMACaddr());
    		t10secFlag=0;
    	}


		// Contr�le du TIMER 100mS
    	// - R�cup�ration de la tension de batterie
    	// - R�cup�ration de la distance mesur�e au sonar
    	// - Gestion des �venements batterie, digital inputs et distance
    	if(t100msFlag){


			DINEventCheck();											// Cont�le de l'�tat des entr�es num�rique
																		// G�n�re un �venement si changement d'�tat d�tect�

			DINSafetyCheck();											// Cont�le de l'�tat des entr�es num�rique
			BatterySafetyCheck();
			DistanceSafetyCheck(); 										// effectue une action si safety actif


			body.distance[0].value = getSonarDistance();
			distanceEventCheck();										// Provoque un �venement de type "distance" si la distance mesur�e
																		// est hors de la plage sp�cifi�e par l'utilisateur

			body.battery[0].value = getBatteryVoltage();
			batteryEventCheck();

			dist[0]=getMotorPulses(MOTOR_LEFT);
			dist[1]=getMotorPulses(MOTOR_RIGHT);

			speed[0]=getMotorFrequency(0);
			speed[1]=getMotorFrequency(1);
			// est hors a plage sp�cifi�e par les paramettre utilisateur

			//printf("\nBattery: %d, safetyStop_state: %d safetyStop_value: %d", 0, body.battery[0].safetyStop_state, body.battery[0].safetyStop_value);
			//printf("\nSpeed : G %.1f   D %.1f   ||| Dist G: %.1fcm  Dist D: %.1fcm", speed[0]*0.285, speed[1]*0.285, dist[0]*0.285, dist[1]*0.285);
			//printf(" dist US: %d cm\n", distance[0]);

			t100msFlag=0;												// Quittance le flag 100mS
    	}

    	usleep(1000);													// Attente de 1ms
    }
	// ------------ FIN DE LA BOUCLE PRINCIPALE ----------------------


	// Fermetur du programme
	int endState=CloseMessager();										// Ferme la t�che de messagerie
	if(!endState)
		  printf( "# ARRET tache Messager - status: %d\n", endState);
	else printf( "# ARRET tache Messager erreur - status: %d\n", endState);

	return EXIT_SUCCESS;												// Fin du programme
}

// -------------------------------------------------------------------
// PROCESSCOMMAND
// S�l�ctionne et traite le param�tre de commande recue [LL2WD, BACK, FORWARD, STOP, SPIN, etc...]
// -------------------------------------------------------------------
int processAlgoidCommand(void){
	switch(AlgoidCommand.msgParam){
		case LL_2WD : 	make2WDaction(); break;			// Action avec en param�tre MOTEUR, VELOCITE, ACCELERATION, TEMPS d'action
		case SERVO  : 	makeServoAction();break;
		case pLED  : 	makeLedAction();break;
		default : break;
	}

	return 0;
}

// -------------------------------------------------------------------
// PROCESSREQUEST
// S�l�ction et traite le param�tre de requete recu [DISTANCE, TENSION BATTERIE, ENTREE DIGITAL, etc...]
// -------------------------------------------------------------------
int processAlgoidRequest(void){

	switch(AlgoidCommand.msgParam){
		case DISTANCE : makeDistanceRequest();					// Requete de distance
						break;

		case BATTERY :  makeBatteryRequest();					// Requete de tension batterie
						break;

		case DINPUT :	makeSensorsRequest();					// Requet d'�tat des entr�es digitale
						break;

		default : break;
	}
	return 0;
}


// -------------------------------------------------------------------
// make2WDaction
// Effectue une action avec les param�tre recus: MOTEUR, VELOCITE, ACCELERATION, TEMPS d'action
// -------------------------------------------------------------------
int make2WDaction(void){
	int ptrData;
	int myTaskId;
	unsigned char actionCount=0;

	// Recherche s'il y a des param�tres pour chaque roue
	// Des param�tres recu pour une roue cr�e une action � effectuer
	if(getWDvalue(MOTOR_LEFT)>=0) actionCount++;
	if(getWDvalue(MOTOR_RIGHT)>=0) actionCount++;

	// Ouverture d'une t�che pour les toutes les actions du message algoid � effectuer
	// Recois un num�ro de tache en retour
	myTaskId=createBuggyTask(AlgoidCommand.msgID, actionCount);			// 2 actions pour mouvement 2WD

	// D�marrage des actions
	if(myTaskId>0){
		printf("Creation de tache WHEEL: #%d avec %d actions\n", myTaskId, actionCount);

		// R�cup�ration des param�tres d'action  pour la roue "LEFT"
		ptrData=getWDvalue(MOTOR_LEFT);
		if(ptrData >=0){
			// Enregistre la donn�e d'acceleration si disponible (<0)
			if(AlgoidCommand.msgValArray[ptrData].accel!=0 || AlgoidCommand.msgValArray[ptrData].decel!=0)
				setMotorAccelDecel(MOTOR_LEFT, AlgoidCommand.msgValArray[ptrData].accel, AlgoidCommand.msgValArray[ptrData].decel);
			// Effectue l'action sur la roue
			if(AlgoidCommand.msgValArray[ptrData].cm != 0)
				setWheelAction(myTaskId, MOTOR_LEFT, AlgoidCommand.msgValArray[ptrData].velocity, CENTIMETER, AlgoidCommand.msgValArray[ptrData].cm);
			else
				setWheelAction(myTaskId, MOTOR_LEFT, AlgoidCommand.msgValArray[ptrData].velocity, MILLISECOND, AlgoidCommand.msgValArray[ptrData].time);
		}

		// R�cup�ration des param�tres d'action  pour la roue "RIGHT"
		ptrData=getWDvalue(MOTOR_RIGHT);
		if(ptrData >=0){
			// Enregistre la donn�e d'acceleration si disponible (<0)
			if(AlgoidCommand.msgValArray[ptrData].accel>0 || AlgoidCommand.msgValArray[ptrData].decel>0)
				setMotorAccelDecel(MOTOR_RIGHT, AlgoidCommand.msgValArray[ptrData].accel, AlgoidCommand.msgValArray[ptrData].decel);

			if(AlgoidCommand.msgValArray[ptrData].cm != 0)
				setWheelAction(myTaskId, MOTOR_RIGHT, AlgoidCommand.msgValArray[ptrData].velocity, CENTIMETER, AlgoidCommand.msgValArray[ptrData].cm);
			else
				setWheelAction(myTaskId, MOTOR_RIGHT, AlgoidCommand.msgValArray[ptrData].velocity, MILLISECOND, AlgoidCommand.msgValArray[ptrData].time);
		}

		// Retourne un message ALGOID si velocit� hors tol�rences
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

	// Recherche s'il y a des param�tres pour chaque roue
	// Des param�tres recu pour une roue cr�e une action � effectuer
	if(getServoSetting(SERVO_0)>=0) actionCount++;
	if(getServoSetting(SERVO_1)>=0) actionCount++;
	if(getServoSetting(SERVO_2)>=0) actionCount++;

	// Ouverture d'une t�che pour les toutes les actions du message algoid � effectuer
	// Recois un num�ro de tache en retour
	myTaskId=createBuggyTask(AlgoidCommand.msgID, actionCount);			//

	// D�marrage des actions
	if(myTaskId>0){
		printf("Creation de tache SERVO: #%d avec %d actions\n", myTaskId, actionCount);

		for(ptrData=0; action < actionCount && ptrData<10; ptrData++){
			if(AlgoidCommand.SERVOmotor[ptrData].id>=0){
				setServoPosition(AlgoidCommand.SERVOmotor[ptrData].id, AlgoidCommand.SERVOmotor[ptrData].angle);

				endOfTask=removeBuggyTask(myTaskId);
				if(endOfTask>0){
					sprintf(reportBuffer, "FIN DES ACTIONS \"SERVO\" pour la tache #%d\n", endOfTask);
					sendResponse(endOfTask, EVENT, SERVO, 0);				// Envoie un message ALGOID de fin de t�che pour l'action �cras�
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
// makeLedAction
//
// -------------------------------------------------------------------
int makeLedAction(void){
	int ptrData;
	int myTaskId;
	int endOfTask;

	unsigned char actionCount=0;
	unsigned char action;

	// Recherche s'il y a des param�tres pour chaque roue
	// Des param�tres recu pour une roue cr�e une action � effectuer
	if(getLedSetting(LED_0)>=0) actionCount++;
	if(getLedSetting(LED_1)>=0) actionCount++;
	if(getLedSetting(LED_2)>=0) actionCount++;

	// Ouverture d'une t�che pour les toutes les actions du message algoid � effectuer
	// Recois un num�ro de tache en retour
	myTaskId=createBuggyTask(AlgoidCommand.msgID, actionCount);			//

	// D�marrage des actions
	if(myTaskId>0){
		printf("Creation de tache LED: #%d avec %d actions\n", myTaskId, actionCount);

		for(ptrData=0; action < actionCount && ptrData<10; ptrData++){
			if(AlgoidCommand.LEDarray[ptrData].id>=0){
				setLedPower(AlgoidCommand.LEDarray[ptrData].id, AlgoidCommand.LEDarray[ptrData].powerPercent);

				endOfTask=removeBuggyTask(myTaskId);
				if(endOfTask>0){
					sprintf(reportBuffer, "FIN DES ACTIONS \"LED\" pour la tache #%d\n", endOfTask);
					sendResponse(endOfTask, EVENT, pLED, 0);			// Envoie un message ALGOID de fin de t�che pour l'action �cras�
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
// Effectue l'action sur une roue sp�cifi�e
// - D�marrage du timer avec definition de fonction call-back, et no d'action
// - D�marrage du mouvement de la roue sp�cifi�e
// - V�locit� entre -100 et +100 qui d�fini le sens de rotation du moteur
// -------------------------------------------------------------------

int setWheelAction(int actionNumber, int wheelName, int veloc, char unit, int value){
	int myDirection;
	int setTimerResult;
	int endOfTask;

	// Conversion de la v�locit� de -100...+100 en direction AVANCE ou RECULE
	if(veloc > 0)
		myDirection=BUGGY_FORWARD;
	if(veloc == 0)
		myDirection=BUGGY_STOP;
	if(veloc < 0){
		myDirection=BUGGY_BACK;
		veloc *=-1;					// Convertion en valeur positive
	}

	// D�marre de timer d'action sur la roue et sp�cifie la fonction call back � appeler en time-out
	// Valeur en retour >0 signifie que l'action "en retour" � �t� �cras�e
	switch(unit){
		case  MILLISECOND:  setTimerResult=setTimerWheel(value, &endWheelAction, actionNumber, wheelName); break;
		case  CENTIMETER:   //wheelNumber = getOrganNumber(wheelName);
							body.encoder[wheelName].startEncoderValue=getMotorPulses(wheelName)*0.285;
							body.encoder[wheelName].stopEncoderValue = body.encoder[wheelName].startEncoderValue+ value;
							//printf("\n Encodeur #%d -> START %.2f cm  STOP %.2f cm", wheelNumber, distance, startEncoderValue[wheelNumber], stopEncoderValue[wheelNumber]);
						    setTimerResult=setTimerWheel(50, &checkMotorEncoder, actionNumber, wheelName);			// D�marre un timer pour contr�le de distance chaque 35mS
						    break;
		default: printf("\n!!! ERROR Function [setWheelAction] -> undefined unit");break;
	}

	if(setTimerResult!=0){								// Timer pret, action effectu�e
		if(setTimerResult>1){							// Le timer � �t� �cras� par la nouvelle action en retour car sur la m�me roue
			endOfTask=removeBuggyTask(setTimerResult);	// Supprime l'ancienne t�che qui � �t� �cras�e par la nouvelle action
			if(endOfTask){
				sendResponse(endOfTask, EVENT, LL_2WD, 0);			// Envoie un message ALGOID de fin de t�che pour l'action �cras�
				sprintf(reportBuffer, "FIN DES ACTIONS \"WHEEL\" pour la tache #%d\n", endOfTask);
				printf(reportBuffer);									// Affichage du message dans le shell
				sendMqttReport(endOfTask, reportBuffer);				// Envoie le message sur le canal MQTT "Report"
			}
		}

		// D�fini le "nouveau" sens de rotation � applique au moteur ainsi que la consigne de vitesse
		if(setMotorDirection(wheelName, myDirection)){							// Sens de rotation
			setMotorSpeed(wheelName, veloc);									// Vitesse

			// Envoie de message ALGOID et SHELL
			sprintf(reportBuffer, "Start wheel %d with velocity %d for time %d\n", wheelName, veloc, value);
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
// Fonction appel�e apr�s le timout d�fini par l'utilisateur, Stop le moteur sp�cifi�
// -------------------------------------------------------------------
int endWheelAction(int actionNumber, int wheelNumber){
	int endOfTask;
	//printf("Action number: %d - End of timer for wheel No: %d\n",actionNumber , wheelNumber);

	// Stop le moteur
	setMotorSpeed(wheelNumber, 0);

	// Retire l'action de la table et v�rification si toute les actions sont effectu�es
	// Pour la t�che en cours donn�e par le message ALGOID

	endOfTask = removeBuggyTask(actionNumber);

	// Contr�le que toutes les actions ont �t� effectu�e pour la commande recue dans le message ALGOID
	if(endOfTask){
		sendResponse(endOfTask, EVENT, LL_2WD, 0);
		sprintf(reportBuffer, "FIN DES ACTIONS \"WHEEL\" pour la tache #%d\n", endOfTask);
		printf(reportBuffer);
		sendMqttReport(endOfTask, reportBuffer);
	}

	return 0;
}


// ----------------------------------------------------------------------
// CHECKMOTORENCODER
// Contr�le la distance parcourue et stop la roue si destination atteinte
// Fonction appel�e apr�s le timout d�fini par l'utilisateur.
// -----------------------------------------------------------------------

int checkMotorEncoder(int actionNumber, int encoderName){
	float distance;					// Variable de distance parcourue depuis le start
	//unsigned char encoderNumber;

	//encoderNumber = getOrganNumber(encoderName);

	distance = getMotorPulses(encoderName);

	if(distance >=0){
		distance = (distance*0.285);
    	usleep(2200);
	}else  printf("\n ERROR: I2CBUS READ\n");
	//printf("\n Encodeur #%d -> START %.2f cm  STOP %.2f cm", encoderNumber, startEncoderValue[encoderNumber], stopEncoderValue[encoderNumber]);

	if(distance >= body.encoder[encoderName].stopEncoderValue)
		endWheelAction(actionNumber, encoderName);
	else
		setTimerWheel(50, &checkMotorEncoder, actionNumber, encoderName);

	return 0;
}

// -------------------------------------------------------------------
// GETWDVALUE
// Recherche dans le message algoid, les param�tres
// [V�locit�, acceleration, sens de rotation et temps d'action] pour la roue sp�cifi�e
// Retourne un pointeur sur le champs de param�tre correspondant � la rou sp�cifi�
// -------------------------------------------------------------------
int getWDvalue(int wheelName){
	int i;
	int searchPtr = -1;

	// Recherche dans les donn�e recues la valeur correspondante au param�tre "wheelName"
	for(i=0;i<AlgoidCommand.msgValueCnt;i++){
		if(wheelName == AlgoidCommand.msgValArray[i].wheel)
			searchPtr=i;
	}
	return searchPtr;
}

// -------------------------------------------------------------------
// GETSERVOSETTING
// Recherche dans le message algoid, les param�tres
// pour une servomoteur sp�cifi�
// Retourne un pointeur sur le champs de param�tre correspondant au servomoteur sp�cifi�
// -------------------------------------------------------------------
int getServoSetting(int servoName){
	int i;
	int searchPtr = -1;

	// Recherche dans les donn�e recues la valeur correspondante au param�tre "wheelName"
	for(i=0;i<AlgoidCommand.msgValueCnt;i++){
		if(servoName == AlgoidCommand.SERVOmotor[i].id)
		searchPtr=i;
	}
	return searchPtr;
}


// -------------------------------------------------------------------
// GETLEDSETTING
// Recherche dans le message algoid, les param�tres
// pour une servomoteur sp�cifi�
// -------------------------------------------------------------------
int getLedSetting(int ledName){
	int i;
	int searchPtr = -1;

	// Recherche dans les donn�e recues la valeur correspondante au param�tre "wheelName"
	for(i=0;i<AlgoidCommand.msgValueCnt;i++){
		if(ledName == AlgoidCommand.LEDarray[i].id)
		searchPtr=i;
	}
	return searchPtr;
}


// -------------------------------------------------------------------
// CREATBUGGYTASK Creation d'une tache avec le nombre
// d'actions � effectuer,
// - Retourne le num�ro d'action attribu�
// - Retourne 0 si table des taches pleine (Impossible de cr�er)
// - Retourne -1 si Message ID existe d�j�
// -------------------------------------------------------------------

int createBuggyTask(int MsgId, int actionCount){
	int i;
	int actionID;


	// d�fini un num�ro de tache al�atoire pour l'action � executer si pas de message id saisi
	if(MsgId == 0){
		actionID = rand() & 0xFFFFFF;
		MsgId = actionID;
	}
	else actionID = MsgId;

	// Recherche un emplacement libre dans la table d'action pour inserer les param�tre
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
	sprintf(reportBuffer, "ERREUR: Table de t�ches pleine\n");
	sendMqttReport(actionID, reportBuffer);
	return(0);
}

// -------------------------------------------------------------------
// removeBuggyTask
// Mise � jour, soustrait l'action d'une tache
// - Retourne le MESSAGE ID correspondant � la tache si plus d'action � effectuer
// - Retourne 0 si actions restante
// - Retourne -1 si tache inexistante
// -------------------------------------------------------------------

int removeBuggyTask(int actionNumber){
	int i, algoidMsgId;

	// Recherche la tache correspondante dans la t�ble des action
	for(i=0;i<10;i++){
		if(ActionTable[i][TASK_NUMBER]==actionNumber){
			ActionTable[i][ACTION_COUNT]--;
			//printf("UPDATE ACTION %d  reste: %d\n", actionNumber, ActionTable[i][ACTION_COUNT]);
			if((ActionTable[i][ACTION_COUNT]) <=0){
				algoidMsgId=ActionTable[i][ACTION_ALGOID_ID];
				ActionTable[i][TASK_NUMBER]=0;				// Reset/Lib�re l'occupation de la t�che
				ActionTable[i][ACTION_ALGOID_ID]= 0;
				ActionTable[i][ACTION_COUNT]=0;
				return(algoidMsgId);						// Retourn le num�ro d'action termin�
			} else return 0;								// Action non termin�es
		}
	}
	return(-1);												// T�che inexistante
}

// -------------------------------------------------------------------
// MAKESENSORREQUEST
// Traitement de la requete SENSORS
// Envoie une message ALGOID de type "response" avec l'�tat des entr�es DIN
// -------------------------------------------------------------------
int makeSensorsRequest(void){
	unsigned char i;

	// Pas de param�tres sp�cifi�s dans le message, retourne l'ensemble des �tats des DIN
	if(AlgoidCommand.msgValueCnt==0){
		AlgoidCommand.msgValueCnt=NBDIN;
		for(i=0;i<NBDIN;i++){
			AlgoidResponse[i].DINresponse.id=i;
//			AlgoidResponse[i].value=body.proximity[i].state;
		}
	}else
		// ENREGISTREMENT DES NOUVEAUX PARAMETRES RECUS
		for(i=0;i<AlgoidCommand.msgValueCnt; i++){
			AlgoidResponse[i].DINresponse.id = AlgoidCommand.DINsens[i].id;

			// Recherche de param�tres suppl�mentaires et enregistrement des donn�e en "local"
			if(!strcmp(AlgoidCommand.DINsens[i].event_state, "on"))	body.proximity[AlgoidCommand.DINsens[i].id].event_enable=1; 			// Activation de l'envoie de messages sur �venements
			else if(!strcmp(AlgoidCommand.DINsens[i].event_state, "off"))	body.proximity[AlgoidCommand.DINsens[i].id].event_enable=0;    // D�sactivation de l'envoie de messages sur �venements

			if(!strcmp(AlgoidCommand.DINsens[i].safetyStop_state, "on"))	body.proximity[AlgoidCommand.DINsens[i].id].safetyStop_state=1; 			// Activation de l'envoie de messages sur �venements
			else if(!strcmp(AlgoidCommand.DINsens[i].safetyStop_state, "off"))	body.proximity[AlgoidCommand.DINsens[i].id].safetyStop_state=0;    // D�sactivation de l'envoie de messages sur �venemen

			body.proximity[AlgoidCommand.DINsens[i].id].safetyStop_value = AlgoidCommand.DINsens[i].safetyStop_value;
		};

	// RETOURNE EN REPONSE LES PARAMETRES ENREGISTRES
	for(i=0;i<AlgoidCommand.msgValueCnt; i++){
		int temp = AlgoidResponse[i].DINresponse.id;
		AlgoidResponse[i].value = body.proximity[temp].state;
		if(body.proximity[temp].event_enable) strcpy(AlgoidResponse[i].DINresponse.event_state, "on");
			else strcpy(AlgoidResponse[i].DINresponse.event_state, "off");

		if(body.proximity[temp].safetyStop_state) strcpy(AlgoidResponse[i].DINresponse.safetyStop_state, "on");
			else strcpy(AlgoidResponse[i].DINresponse.safetyStop_state, "off");
		AlgoidResponse[i].DINresponse.safetyStop_value = body.proximity[temp].safetyStop_value;
	};

	// Envoie de la r�ponse MQTT
	sendResponse(AlgoidCommand.msgID, RESPONSE, DINPUT, AlgoidCommand.msgValueCnt);
	return (1);
}

// -------------------------------------------------------------------
// MAKEDISTANCEREQUEST
// Traitement de la requete de mesure de distance
// // R�cup�re les valeurs des param�tres "EVENT", "EVENT_HIGH", "EVENT_LOW", ANGLE
// Envoie un message ALGOID de type "response" avec la valeur distance mesur�e
// -------------------------------------------------------------------
int makeDistanceRequest(void){
	unsigned char i;

	// Pas de param�tres sp�cifi� dans le message, retourne l'ensemble des distances
	if(AlgoidCommand.msgValueCnt==0){
		AlgoidCommand.msgValueCnt=NBPWM;
		for(i=0;i<NBPWM;i++){
			AlgoidResponse[i].DISTresponse.id=i;
//			AlgoidResponse[i].value=body.distance[i].value;
		}
	}else
		// ENREGISTREMENT DES NOUVEAUX PARAMETRES RECUS
			for(i=0;i<AlgoidCommand.msgValueCnt; i++){
				AlgoidResponse[i].DISTresponse.id=AlgoidCommand.DISTsens[i].id;
				// Activation de l'envoie de messages sur �venements
				if(!strcmp(AlgoidCommand.DISTsens[i].event_state, "on")) body.distance[AlgoidCommand.DISTsens[i].id].event_enable=1;
				else if(!strcmp(AlgoidCommand.DISTsens[i].event_state, "off")) body.distance[AlgoidCommand.DISTsens[i].id].event_enable=0;
				// Evemenent haut
				if(AlgoidCommand.DISTsens[i].event_high!=0) body.distance[AlgoidCommand.DISTsens[i].id].event_high=AlgoidCommand.DISTsens[i].event_high;
				if(AlgoidCommand.DISTsens[i].event_low!=0) body.distance[AlgoidCommand.DISTsens[i].id].event_low=AlgoidCommand.DISTsens[i].event_low;

				if(!strcmp(AlgoidCommand.DISTsens[i].safetyStop_state, "on")) body.distance[AlgoidCommand.DISTsens[i].id].safetyStop_state=1;
				else if(!strcmp(AlgoidCommand.DISTsens[i].safetyStop_state, "off")) body.distance[AlgoidCommand.DISTsens[i].id].safetyStop_state=0;
				body.distance[AlgoidCommand.DISTsens[i].id].safetyStop_value = AlgoidCommand.DISTsens[i].safetyStop_value;
			};

	// RETOURNE EN REPONSE LES PARAMETRES ENREGISTRES
	for(i=0;i<AlgoidCommand.msgValueCnt; i++){
		// R�cup�ration des param�tres actuels et chargement du buffer de r�ponse
		int temp = AlgoidResponse[i].DISTresponse.id;

		AlgoidResponse[i].value=body.distance[temp].value;
		//AlgoidResponse[i].DISTresponse.angle=angle[AlgoidCommand.DISTsens[i].angle];

		if(body.distance[temp].event_enable)strcpy(AlgoidResponse[i].DISTresponse.event_state, "on");
		else strcpy(AlgoidResponse[i].DISTresponse.event_state, "off");
		AlgoidResponse[i].DISTresponse.event_high=body.distance[temp].event_high;
		AlgoidResponse[i].DISTresponse.event_low=body.distance[temp].event_low;

		if(body.distance[temp].safetyStop_state)strcpy(AlgoidResponse[i].DISTresponse.safetyStop_state, "on");
		else strcpy(AlgoidResponse[i].DISTresponse.safetyStop_state, "off");
		AlgoidResponse[i].DISTresponse.safetyStop_value=body.distance[temp].safetyStop_value;
	};

	// Envoie de la r�ponse MQTT
	sendResponse(AlgoidCommand.msgID, EVENT, DISTANCE, AlgoidCommand.msgValueCnt);

		return 1;
}


// -------------------------------------------------------------------
// MAKEBATTERYREQUEST
// Traitement de la requete de mesure de tension batterie
// R�cup�re les valeurs des param�tres "EVENT", "EVENT_HIGH", "EVENT_LOW"
// Envoie un message ALGOID de type "response" avec la valeur des param�tres enregistr�s
// -------------------------------------------------------------------

int makeBatteryRequest(void){
	unsigned char i;

	// Pas de param�tres sp�cifi� dans le message, retourne l'ensemble des �tats des batteries
	if(AlgoidCommand.msgValueCnt==0){
		AlgoidCommand.msgValueCnt=1;
		for(i=0;i<2;i++){
			AlgoidResponse[i].BATTesponse.id=i;
//			AlgoidResponse[i].value=body.battery[i].value;
		}
	}else
		// ENREGISTREMENT DES NOUVEAUX PARAMETRES RECUS
			for(i=0;i<AlgoidCommand.msgValueCnt; i++){
				AlgoidResponse[i].DISTresponse.id=AlgoidCommand.BATTsens[i].id;
				// Recherche de param�tres suppl�mentaires
				// Evenement activ�es
				if(!strcmp(AlgoidCommand.BATTsens[i].event_state, "on")) body.battery[AlgoidCommand.BATTsens[i].id].event_enable=1;
				else if(!strcmp(AlgoidCommand.BATTsens[i].event_state, "off")) body.battery[AlgoidCommand.BATTsens[i].id].event_enable=0;
				// Evemenent haut
				if(AlgoidCommand.BATTsens[i].event_high!=0) body.battery[AlgoidCommand.BATTsens[i].id].event_high=AlgoidCommand.BATTsens[i].event_high;
				if(AlgoidCommand.BATTsens[i].event_high!=0) body.battery[AlgoidCommand.BATTsens[i].id].event_low=AlgoidCommand.BATTsens[i].event_low;

				if(!strcmp(AlgoidCommand.BATTsens[i].safetyStop_state, "on")) body.battery[AlgoidCommand.BATTsens[i].id].safetyStop_state=1;
				else if(!strcmp(AlgoidCommand.BATTsens[i].safetyStop_state, "off")) body.battery[AlgoidCommand.BATTsens[i].id].safetyStop_state=0;
				if(AlgoidCommand.BATTsens[i].safetyStop_value!=0) body.battery[AlgoidCommand.BATTsens[i].id].safetyStop_value=AlgoidCommand.BATTsens[i].safetyStop_value;
			};

	// RETOURNE EN REPONSE LES PARAMETRES ENREGISTRES
	for(i=0;i<AlgoidCommand.msgValueCnt; i++){
		int temp = AlgoidResponse[i].BATTesponse.id;

		AlgoidResponse[i].value=body.battery[temp].value;

		if(body.battery[temp].event_enable)strcpy(AlgoidResponse[i].BATTesponse.event_state, "on");
		else strcpy(AlgoidResponse[i].BATTesponse.event_state, "off");
		AlgoidResponse[i].BATTesponse.event_high=body.battery[temp].event_high;
		AlgoidResponse[i].BATTesponse.event_low=body.battery[temp].event_low;

		if(body.battery[temp].safetyStop_state)strcpy(AlgoidResponse[i].BATTesponse.safetyStop_state, "on");
		else strcpy(AlgoidResponse[i].BATTesponse.safetyStop_state, "off");
		AlgoidResponse[i].BATTesponse.safetyStop_value=body.battery[temp].safetyStop_value;
	};

	// Envoie de la r�ponse MQTT
	sendResponse(AlgoidCommand.msgID, EVENT, BATTERY, AlgoidCommand.msgValueCnt);
		return 1;
}

// -------------------------------------------------------------------
// DISTANCEEVENTCHECK
// Contr�le si la distance mesur�e est hors de la plage d�fini par l'utilisateur
// et envoie un message de type "event" si tel est le cas.
// Un deuxi�me "event" est envoy� lorsque la mesure de distance entre � nouveau dans la
// plage d�finie.
// -------------------------------------------------------------------
void distanceEventCheck(void){
	static unsigned char distWarningSended[1];
	unsigned char i;
	// Contr�le periodique des mesures de distances pour envoie d'evenement
	for(i=0;i<2;i++){
		// V�rification si envoie des EVENT activ�s
		if(body.distance[i].event_enable){
			if((body.distance[i].value < body.distance[i].event_low) || (body.distance[i].value > body.distance[i].event_high)){		// Mesure de distance hors plage
				if(distWarningSended[i]==0){													// N'envoie l' event qu'une seule fois
					AlgoidResponse[i].DISTresponse.id=i;
					AlgoidResponse[i].value=body.distance[i].value;
					sendResponse(AlgoidCommand.msgID, EVENT, DISTANCE, 1);
					distWarningSended[i]=1;
				}
			}
			else if (distWarningSended[i]==1){													// Mesure de distance revenu dans la plage
					AlgoidResponse[i].DISTresponse.id=i;									// Et n'envoie qu'une seule fois le message
					AlgoidResponse[i].value=body.distance[i].value;
					sendResponse(AlgoidCommand.msgID, EVENT, DISTANCE, 1);
					distWarningSended[i]=0;
			}
		}
	}
}


// -------------------------------------------------------------------
// BATTERYEVENTCHECK
// Contr�le si la tension mesur�e est hors de la plage d�fini par l'utilisateur
// et envoie un message de type "event" si tel est le cas.
// Un deuxi�me "event" est envoy� lorsque la tension batterie entre � nouveau dans la
// plage d�finie.
// -------------------------------------------------------------------
// -------------------------------------------------------------------
void batteryEventCheck(void){
	static unsigned char battWarningSended[1];
	unsigned char i;
	// Contr�le periodique des mesures de tension batterie pour envoie d'evenement
	for(i=0;i<NBAIN;i++){
		if(body.battery[i].event_enable){
			if((body.battery[i].value < body.battery[i].event_low) || (body.battery[i].value > body.battery[i].event_high)){				// Mesure tension hors plage
				if(battWarningSended[i]==0){														// N'envoie qu'une seule fois l'EVENT
					AlgoidResponse[i].BATTesponse.id=i;
					AlgoidResponse[i].value=body.battery[i].value;
					sendResponse(AlgoidCommand.msgID, EVENT, BATTERY, 1);
					battWarningSended[i]=1;
				}
			}
			// Envoie un �venement Fin de niveau bas (+50mV Hysterese)
			else if (battWarningSended[i]==1 && body.battery[i].value > (body.battery[i].event_low + body.battery[i].event_hysteresis)){				// Mesure tension dans la plage
					AlgoidResponse[i].BATTesponse.id=i;											// n'envoie qu'une seule fois apr�s
					AlgoidResponse[i].value=body.battery[i].value;											// une hysterese de 50mV
					sendResponse(AlgoidCommand.msgID, EVENT, BATTERY, 1);
					battWarningSended[i]=0;
			}
		}
	}
}


// -------------------------------------------------------------------
// DINEVENTCHECK
// V�rifie si une changement d'�tat � eu lieu sur les entr�es num�riques
// et envoie un event si tel est les cas.
// Seul les DIN ayant chang� d'�tat font partie du message de r�ponse
// -------------------------------------------------------------------
void DINEventCheck(void){
	// Mise � jour de l'�tat des E/S
	unsigned char ptrBuff=0, DINevent=0, oldDinValue[NBDIN], i;

	for(i=0;i<NBDIN;i++){
		// Mise � jour de l'�tat des E/S
		oldDinValue[i]=body.proximity[i].state;
		body.proximity[i].state = getDigitalInput(i);

		// V�rifie si un changement a eu lieu sur les entrees et transmet un message
		// "event" listant les modifications
		if(body.proximity[i].event_enable && (oldDinValue[i] != body.proximity[i].state)){
			AlgoidResponse[ptrBuff].DINresponse.id=i;
			AlgoidResponse[ptrBuff].value=body.proximity[i].state;
			ptrBuff++;
			printf("CHANGEMENT DIN%d, ETAT:%d\n", i, body.proximity[i].state);
			DINevent++;
		}
	}

	if(DINevent>0)
		sendResponse(AlgoidCommand.msgID, EVENT, DINPUT, DINevent);
}


// -------------------------------------------------------------------
// DINSAFETYCHECK
// V�rifie si une changement d'�tat � eu lieu sur les entr�es num�riques
// et effectue une action si safety actif et valeur concordante
// -------------------------------------------------------------------
void DINSafetyCheck(void){
	// Mise � jour de l'�tat des E/S
	unsigned char ptrBuff=0, DINsafety=0, i;
	static unsigned char safetyAction[NBDIN];

	for(i=0;i<NBDIN;i++){
		// Mise � jour de l'�tat des E/S
		body.proximity[i].state = getDigitalInput(i);

		// V�rifie si un changement a eu lieu sur les entrees et transmet un message
		// "event" listant les modifications
		if(body.proximity[i].safetyStop_state){
			if((body.proximity[i].safetyStop_value == body.proximity[i].state)){
				if(!safetyAction[i]){
					ptrBuff++;
					printf("SAFETY DETECTION ON DIN%d, ETAT:%d\n", i, body.proximity[i].state);
					DINsafety++;
					safetyAction[i]=1;
				}
			} else safetyAction[i]=0;
		}
	}
	if(DINsafety>0){
		// ACTION A EFFECTUER
		//sendResponse(AlgoidCommand.msgID, EVENT, DINPUT, DINsafety);
	}
}

// -------------------------------------------------------------------
// BATTERYSAFETYCHECK
// V�rifie si une changement d'�tat � eu lieu sur les entr�es num�riques
// et effectue une action si safety actif et valeur concordante
// -------------------------------------------------------------------
void BatterySafetyCheck(void){
	// Mise � jour de l'�tat des E/S
	unsigned char ptrBuff=0, AINsafety=0, i;
	static unsigned char safetyAction[NBAIN];

	for(i=0;i<NBAIN;i++){
		// Mise � jour de l'�tat des E/S
		body.battery[i].value = getBatteryVoltage();

		// V�rifie si un changement a eu lieu sur les entrees et transmet un message
		// "event" listant les modifications
		if(body.battery[i].safetyStop_state){
			if((body.battery[i].safetyStop_value > body.battery[i].value)){
				if(!safetyAction[i]){
					ptrBuff++;
					printf("SAFETY DETECTION ON BATTERY%d, VALUE:%d\n", i, body.battery[i].value);
					AINsafety++;
					safetyAction[i]=1;
				}
			} else safetyAction[i]=0;
		}
	}
	if(AINsafety>0){
		// ACTION A EFFECTUER
		//sendResponse(AlgoidCommand.msgID, EVENT, DINPUT, DINsafety);
	}
}

// -------------------------------------------------------------------
// DISTANCESAFETYCHECK
// V�rifie si une changement d'�tat � eu lieu sur les entr�es num�riques
// et effectue une action si safety actif et valeur concordante
// -------------------------------------------------------------------
void DistanceSafetyCheck(void){
	// Mise � jour de l'�tat des E/S
	unsigned char ptrBuff=0, DISTsafety=0, i;
	static unsigned char safetyAction[NBPWM];

	for(i=0;i<NBPWM;i++){
		// Mise � jour de l'�tat des E/S
		body.distance[i].value = getSonarDistance();

		// V�rifie si un changement a eu lieu sur les entrees et transmet un message
		// "event" listant les modifications
		if(body.distance[i].safetyStop_state){
			if((body.distance[i].safetyStop_value > body.distance[i].value)){
				if(!safetyAction[i]){
					ptrBuff++;
					printf("SAFETY DETECTION ON SONAR %d, VALUE:%d\n", i, body.distance[i].value);
					DISTsafety++;
					safetyAction[i]=1;
				}
			} else safetyAction[i]=0;
		}
	}
	if(DISTsafety>0){
		// ACTION A EFFECTUER
		//sendResponse(AlgoidCommand.msgID, EVENT, DINPUT, DINsafety);
	}
}

