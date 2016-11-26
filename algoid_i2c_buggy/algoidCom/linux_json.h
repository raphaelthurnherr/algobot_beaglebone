
// DEFINITION DES TYPES DE MESSAGE
typedef enum msgtype{
	COMMAND,
	REQUEST,
	ACK,
	RESPONSE,
	EVENT,
	NEGOC,
	ERROR,
	WARNING
} t_msgtype;

// DEFINITION DES PARAMETRES DE TYPE PARAMETRE
typedef enum msgparam{
	STOP,
	MOVE,
	LL_2WD,
	DINPUT,
	DISTANCE,
	BATTERY
}t_msgparam;


struct m2wd{
	int wheel;
	int velocity;
	int time;
	int cm;
	char accel;
	char decel;
};

struct mDin{
	int id;
	char event_state[50];
};

struct mDistance{
	int id;
	char event_state[50];
	int event_low;
	int event_high;
	int angle;
};

struct mBattery{
	int id;
	char event_state[50];
	int event_low;
	int event_high;
};

// Structure d'un message algoid recu
typedef struct JsonCommand{
	char msgTo[32];
	char msgFrom[32];
	int msgID;
	t_msgtype msgType;
	t_msgparam msgParam;
	unsigned char msgValueCnt;

	// UNION ???
	struct m2wd msgValArray[20];

	struct mDin DINsens[20];
	struct mDistance DISTsens[20];
	struct mBattery BATTsens[20];
	// UNION ???

}ALGOID;

// Structure de réponse à un message algoid
typedef struct JsonResponse{
	int value;
	// UNION ???
	struct mDin DINresponse;
	struct mBattery BATTesponse;
	struct mDistance DISTresponse;
	// UNION ???
}ALGOID_RESPONSE;

ALGOID AlgoidCommand;    // Utilisé par main.c
ALGOID AlgoidMessageRX;
ALGOID AlgoidMsgRXStack[10];

// Buffer de soirtie pour les 	msgValue[
ALGOID_RESPONSE AlgoidResponse[20];


extern char GetAlgoidMsg(ALGOID DestReceiveMessage,char *srcDataBuffer);
void ackToJSON(char * buffer, int msgId, char* to, char * from, char * msgType,char * msgParam, unsigned char value, unsigned char count);
