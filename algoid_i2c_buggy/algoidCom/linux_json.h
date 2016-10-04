
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
	#define DISTANCE   	   0
	#define DISTANCE_MAP   1
	#define SENSORS	       2

	// DEFINITION DES MODES POUR TYPE DE COMMANDE
	#define CM "distCm"
	#define STEP "step"
	#define INFINIT "infinit"
	#define SECONDS "timeSec"

struct mValue{
	char mode[100];
	int value;

	char wheel[50];
	int velocity;
	int time;
};

typedef struct JsonCommand{
	char msgTo[32];
	char msgFrom[32];
	int msgID;
	int msgType;
	int msgParam;
	unsigned char msgValueCnt;
	struct mValue msgValArray[20];
}ALGOID;

ALGOID AlgoidCommand;    // Utilisé par main.c
ALGOID AlgoidMessageRX;
ALGOID AlgoidMsgRXStack[10];

extern char GetAlgoidMsg(ALGOID DestReceiveMessage,char *srcDataBuffer);
void ackToJSON(char * buffer, int msgId, char* to, char * from, char * msgType,char * msgParam, char * value, int data);
