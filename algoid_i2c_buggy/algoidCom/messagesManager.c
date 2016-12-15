
//#define ADDRESS     "192.168.5.12:1883"
#define ADDRESS     "localhost:1883"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "pthread.h"
#include "messagesManager.h"
#include "mqttProtocol.h"
#include "linux_json.h"
#include "../buggy_descriptor.h"

// Thread Messager
pthread_t th_messager;

char ClientID[50]="BUGGY_";

void sendMqttReport(int msgId, char * msg);

int  mqttMsgArrived(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void sendResponse(int msgId, unsigned char msgType, unsigned char msgParam, unsigned char valCnt);
int pushMsgStack(void);
int pullMsgStack(unsigned char ptrStack);
char clearMsgStack(unsigned char ptrStack);

// Initialisation des variables
unsigned char mqttDataReady=0;

char MqttDataBuffer[500];

// Initialisation principale du system de messagerie

void *MessagerTask (void * arg){	 													// duty cycle is 50% for ePWM0A , 25% for ePWM0B;
	int mqttStatus;
	int lastMessage;
	int i;


	// Initialisation de la pile de reception de message
	for(i=0;i<10;i++)
		clearMsgStack(i);

	// Creation d'un id unique avec l'adresse mac
	sprintf(&ClientID[6], "%s", getMACaddr());

	// Connexion au broker MQTT
	mqttStatus=mqtt_init(ADDRESS, ClientID, mqttMsgArrived);

	if(!mqttStatus){
		printf("# Connection au broker MQTT IP: %s avec ID: %s\n",ADDRESS,ClientID   );
		if(!mqttAddRXChannel("MONTEST")){
			printf("# Inscription au topic: OK\n");
		}
		else {
			printf("# Inscription au topic: ERREUR\n");
		}
	}else {
		printf("# Connexion au broker MQTT impossible !\n");
	}
// BOUCLE PRINCIPALE
		while(1)
	    {
	    	// Verification de l'arrivée d'un message MQTT
	    	if(mqttDataReady){
	    	printf("\n-----------------------------\n");
	    	printf("Nouveau message MQTT recu -> ");
	    	//printf("[DEBUG] message: %s", MqttDataBuffer);
	    	// RECEPTION DES DONNES UTILES
				if(GetAlgoidMsg(AlgoidMessageRX, MqttDataBuffer)>0){
					// Enregistrement du message dans la pile
					lastMessage=pushMsgStack();
					if(lastMessage>=0){
						sendResponse(AlgoidMessageRX.msgID, ACK, AlgoidMessageRX.msgParam, 0);
						printf("Mis en file d'attente\n");
					}
					else{
						printf("ERREUR: File d'attente pleine !\n");
					}
				}else{

					sendResponse(AlgoidMessageRX.msgID, AlgoidMessageRX.msgType, AlgoidMessageRX.msgParam, 0);
					printf("\n! MESSAGE ALGOID INCORRECT RECU !\n");
					sendMqttReport(-1, "! MESSAGE ALGOID INCORRECT RECU !");
				}
			mqttDataReady=0;
	    	}

	    	usleep(10000);
	    }
 // FIN BOUCLE PRINCIPAL

  usleep(5000);
  pthread_exit (0);
}



int pushMsgStack(void){
	int ptrMsgRXstack=0;

	// Recherche un emplacement vide dans la pile
	for(ptrMsgRXstack=0;(ptrMsgRXstack<10) && (AlgoidMsgRXStack[ptrMsgRXstack].msgID>=0);ptrMsgRXstack++);

	// CONTROLE DE L'ETAT DE LA PILE DE MESSAGE
	if(ptrMsgRXstack>=10){
		return -1;
	}else{

		// ENREGISTREMENT DU MESSAGE DANS LA PILE
		AlgoidMsgRXStack[ptrMsgRXstack]=AlgoidMessageRX;
// FOR DEBUG
		// AFFICHAGE DES DONNEES
/*
		int i;
		printf("Message to   %s   from   %s with ID %d \n",AlgoidMsgRXStack[ptrMsgRXstack].msgTo,AlgoidMsgRXStack[ptrMsgRXstack].msgFrom, AlgoidMsgRXStack[ptrMsgRXstack].msgID );
		printf("type:  %d\n",AlgoidMsgRXStack[ptrMsgRXstack].msgType);
		printf("param: %d\n",AlgoidMsgRXStack[ptrMsgRXstack].msgParam);
		printf("ValCount: %d\n",AlgoidMsgRXStack[ptrMsgRXstack].msgValueCnt);
		for(i=0;i<AlgoidMsgRXStack[ptrMsgRXstack].msgValueCnt;i++){
			if(AlgoidMsgRXStack[ptrMsgRXstack].msgParam==100)// Commande LL_WD ?
				printf("wheel: %s   velocity: %d   time: %d accel: %d\n", AlgoidMsgRXStack[ptrMsgRXstack].msgValArray[i].wheel, AlgoidMsgRXStack[ptrMsgRXstack].msgValArray[i].velocity, AlgoidMsgRXStack[ptrMsgRXstack].msgValArray[i].time, AlgoidMsgRXStack[ptrMsgRXstack].msgValArray[i].accel);
//			else
//				printf("mode: %s   value: %d\n", AlgoidMsgRXStack[ptrMsgRXstack].msgValArray[i].mode, AlgoidMsgRXStack[ptrMsgRXstack].value);
		}
*/
//END DEBUG
		ptrMsgRXstack++;
		return ptrMsgRXstack-1;
	}
}

int pullMsgStack(unsigned char ptrStack){
		int i;
		unsigned char result;

		if(AlgoidMsgRXStack[ptrStack].msgType!=-1){
			AlgoidCommand=AlgoidMsgRXStack[ptrStack];

			// Contrôle le ID, FROM, TO du message et creation générique si inexistant
			if(AlgoidCommand.msgID <= 0){
				AlgoidCommand.msgID = rand() & 0xFFFFFF;
			}

			if(strcmp(AlgoidCommand.msgFrom, "")){
				strcpy(AlgoidCommand.msgFrom,"unknown");
			}

			if(strcmp(AlgoidCommand.msgTo, "")){
				strcpy(AlgoidCommand.msgTo, "unknown");
			}

			// Déplace les elements de la pile
			for(i=ptrStack;i<9;i++){
				AlgoidMsgRXStack[ptrStack]=AlgoidMsgRXStack[ptrStack+1];
				ptrStack++;
			}

			// EFFACE LES DONNEES DE LA PILE
			strcpy(AlgoidMsgRXStack[9].msgFrom, "");
			strcpy(AlgoidMsgRXStack[9].msgTo, "");
			AlgoidMsgRXStack[9].msgID=-1;
			AlgoidMsgRXStack[9].msgParam=-1;
			AlgoidMsgRXStack[9].msgType=-1;
			AlgoidMsgRXStack[9].msgValueCnt=0;

			for(i=0;i<AlgoidMsgRXStack[9].msgValueCnt;i++){
				AlgoidMsgRXStack[9].msgValArray[i].wheel=UNKNOWN;
			}

			return 1;
		}else
			return 0;
}
// ----------------------

char clearMsgStack(unsigned char ptrStack){
		int i;

		if(ptrStack<10){
			// EFFACE LES DONNEES DE LA PILE
			strcpy(AlgoidMsgRXStack[ptrStack].msgFrom, "");
			strcpy(AlgoidMsgRXStack[ptrStack].msgTo, "");
			AlgoidMsgRXStack[ptrStack].msgID=-1;
			AlgoidMsgRXStack[ptrStack].msgParam=-1;
			AlgoidMsgRXStack[ptrStack].msgType=-1;

			for(i=0;i<AlgoidMsgRXStack[ptrStack].msgValueCnt;i++){
				strcpy(AlgoidMsgRXStack[ptrStack].msgValArray[i].wheel, "");
				AlgoidMsgRXStack[ptrStack].msgValArray[i].time=-1;
				AlgoidMsgRXStack[ptrStack].msgValArray[i].velocity=-1;
			}
			return 0;
		}
		return 1;
}

// ------------------------------------------------------------------------------------
// INITMESSAGER: Initialisation du gestionnaire de message
// - Demarrage thread messager
// -
// ------------------------------------------------------------------------------------
int InitMessager(void){
	// CREATION DU THREAD DE MESSAGERIE (Tache "MessagerTask")
	  if (pthread_create (&th_messager, NULL, MessagerTask, NULL)< 0) {
		  //Connexion au brocker MQTT

		return (1);
	  }else return (0);

}

// ------------------------------------------------------------------------------------
// CLOSEMESSAGE: Fermeture du gestionnaire de messages
// - Fermeture MQTT
// - Stop le thread manager
// ------------------------------------------------------------------------------------
int CloseMessager(void){
	int result;
	// TERMINE LE THREAD DE MESSAGERIE
	pthread_cancel(th_messager);
	// Attends la terminaison du thread de messagerie
	result=pthread_join(th_messager, NULL);
	return (result);
}


// -------------------------------------------------------------------
// Fonction Call-Back reception d'un message MQTT
// -------------------------------------------------------------------
int mqttMsgArrived(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    unsigned int i;
    char* payloadptr;
    int messageCharCount=0;

    payloadptr = message->payload;
    messageCharCount=message->payloadlen;

    for(i=0; i<messageCharCount; i++)
    {
    	MqttDataBuffer[i]=payloadptr[i];
    }

    // Termine la chaine de caractère
    MqttDataBuffer[messageCharCount]=0;

	mqttDataReady=1;

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}


// -------------------------------------------------------------------
// SENDRESPONSE
// Retourne un message MQTT
// -------------------------------------------------------------------

void sendResponse(int msgId, unsigned char msgType, unsigned char msgParam, unsigned char valCnt){
	char MQTTbuf[1024];
	char ackType[15], ackParam[15];


	// Génération du texte de reponse TYPE pour message MQTT
	switch(msgType){
		case COMMAND : strcpy(ackType, "command"); break;
		case REQUEST : strcpy(ackType, "request"); break;
		case ACK : strcpy(ackType, "ack"); break;
		case RESPONSE : strcpy(ackType, "response"); break;
		case EVENT : strcpy(ackType, "event"); break;
		case NEGOC : strcpy(ackType, "negoc"); break;
		case ERR_TYPE : strcpy(ackType, "error"); break;
		case WARNING : strcpy(ackType, "warning"); break;
		default : strcpy(ackType, "unknown"); break;
	}

// Génération du texte de reponse TYPE pour message MQTT
	switch(msgParam){
		case STOP : strcpy(ackParam, "stop"); break;
		case LL_2WD : strcpy(ackParam, "2wd"); break;
		case SERVO : strcpy(ackParam, "servo"); break;
		case MOVE : strcpy(ackParam, "move"); break;
		case DINPUT : strcpy(ackParam, "din"); break;
		case BATTERY : strcpy(ackParam, "battery"); break;
		case DISTANCE : strcpy(ackParam, "distance"); break;
		case ERR_PARAM : strcpy(ackParam, "error"); break;
		default : strcpy(ackParam, "unknown"); break;
	}

	ackToJSON(MQTTbuf, msgId, "algoid", ClientID, ackType, ackParam, msgParam, valCnt);
	mqttPutMessage("MONRET", MQTTbuf, strlen(MQTTbuf));
}

// -------------------------------------------------------------------
// SENDMQTTREPORT
// Retourne un message MQTT
// -------------------------------------------------------------------

void sendMqttReport(int msgId, char * msg){
	char MQTTbuf[1024];

	// Creation d'un id unique avec l'adresse mac
	sprintf(&MQTTbuf[0], "%s -> Message ID: %d -> %s", ClientID, msgId, msg);

	mqttPutMessage("buggyReport", MQTTbuf, strlen(MQTTbuf));
}
