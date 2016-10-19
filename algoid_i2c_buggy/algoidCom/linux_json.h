
// DEFINITION DES TYPES DE MESSAGE

#define COMMAND 0
#define REQUEST 1
#define ACK     2
#define RESPONSE 3
#define EVENT    4
#define NEGOC    5
#define ERROR    6
#define WARNING  7

	// DEFINITION DES PARAMETRES DE TYPE COMMANDE
	#define STOP     0
	#define FORWARD  1
	#define BACK     2
	#define LEFT     3
	#define RIGHT    4
	#define ROTATE_LEFT 5
	#define ROTATE_RIGHT 6
	#define FW_SPIN_LEFT 7
	#define FW_SPIN_RIGHT 8
	#define BACK_SPIN_LEFT 9
	#define BACK_SPIN_RIGHT 10
	#define MOTOR_STATE		 11
	#define LL_2WD			100				// LOW LEVEL WHEEL DIRECTIV

	// DEFINITION DES PARAMETRES DE TYPE REQUEST
	#define DISTANCE   	   20
	#define DISTANCE_MAP   21
	#define DINPUT	       22
	#define BATTERY		   30

	// DEFINITION DES MODES POUR TYPE DE COMMANDE
	#define DISTCM		0
	#define BATTVOLT    1
	#define SENSORS_STATE    2


struct m2wd{
	char wheel[50];
	int velocity;
	int time;
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
	int msgType;
	int msgParam;
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
