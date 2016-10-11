/*
 ============================================================================
 Name        : linux_JSON.c
 Author      : Raph
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */
#define KEY_TO "{'MsgTo'"
#define KEY_FROM "{'MsgFrom'"
#define KEY_MSGID "{'MsgID'"
#define KEY_MESSAGE ""
#define KEY_MESSAGE_TYPE "{'MsgData'{'MsgType'"
#define KEY_MESSAGE_PARAM "{'MsgData'{'MsgParam'"
#define KEY_MESSAGE_VALUE "{'MsgData'{'MsgValue'"

#define KEY_MESSAGE_VALUE_MODE "{'MsgData'{'MsgValue'[*{'mode'"
#define KEY_MESSAGE_VALUE_VALUE "{'MsgData'{'MsgValue'[*{'value'"

#define KEY_MESSAGE_VALUE_WHEEL "{'MsgData'{'MsgValue'[*{'wheel'"
#define KEY_MESSAGE_VALUE_VELOCITY "{'MsgData'{'MsgValue'[*{'velocity'"
#define KEY_MESSAGE_VALUE_TIME "{'MsgData'{'MsgValue'[*{'time'"
#define KEY_MESSAGE_VALUE_ACCEL "{'MsgData'{'MsgValue'[*{'accel'"
#define KEY_MESSAGE_VALUE_DECEL "{'MsgData'{'MsgValue'[*{'decel'"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "linux_json.h"
#include "libs/lib_json/jRead.h"
#include "libs/lib_json/jWrite.h"

void ackToJSON(char * buffer, int msgId, char* to, char * from, char * msgType,char * msgParam, char * value, int data);
char GetAlgoidMsg(ALGOID destMessage, char *srcBuffer);

ALGOID myReplyMessage;

// -----------------------------------------------------------------------------
// MAIN, APPLICATION PRINCIPALE-------------------------------------------------
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// GetAlgoidMsg
// Get message from buffer and set in the message structure
// -----------------------------------------------------------------------------

char GetAlgoidMsg(ALGOID destMessage, char *srcBuffer){
	struct jReadElement element;
	int i;
	int result;

	// ENTETE DE MESSAGE
		jRead_string((char *)srcBuffer, KEY_TO, AlgoidMessageRX.msgTo, 15, NULL );
		jRead_string((char *)srcBuffer, KEY_FROM, AlgoidMessageRX.msgFrom, 15, NULL );
		AlgoidMessageRX.msgID= jRead_int((char *)srcBuffer,  KEY_MSGID, NULL);

	// MESSAGE TYPE
				char myDataString[20];
				// Clear string
				for(i=0;i<20;i++) myDataString[i]=0;

				jRead_string((char *)srcBuffer,  KEY_MESSAGE_TYPE,myDataString,15, NULL);

				AlgoidMessageRX.msgType= -1;
				if(!strcmp(myDataString, "command")) AlgoidMessageRX.msgType = COMMAND;
				if(!strcmp(myDataString, "request")) AlgoidMessageRX.msgType = REQUEST;
				if(!strcmp(myDataString, "ack")) AlgoidMessageRX.msgType = ACK;
				if(!strcmp(myDataString, "response")) AlgoidMessageRX.msgType = RESPONSE;
				if(!strcmp(myDataString, "event")) AlgoidMessageRX.msgType = EVENT;
				if(!strcmp(myDataString, "negoc")) AlgoidMessageRX.msgType = NEGOC;
				if(!strcmp(myDataString, "error")) AlgoidMessageRX.msgType = ERROR;

	// MESSAGE PARAM
				// Clear string
				for(i=0;i<20;i++) myDataString[i]=0;
				jRead_string((char *)srcBuffer,  KEY_MESSAGE_PARAM,myDataString,15, NULL);

				AlgoidMessageRX.msgParam=-1;
					if(!strcmp(myDataString, "forward")) AlgoidMessageRX.msgParam = FORWARD;
					if(!strcmp(myDataString, "back")) AlgoidMessageRX.msgParam = BACK;
					if(!strcmp(myDataString, "left")) AlgoidMessageRX.msgParam = LEFT;
					if(!strcmp(myDataString, "right")) AlgoidMessageRX.msgParam = RIGHT;
					if(!strcmp(myDataString, "rotateLeft")) AlgoidMessageRX.msgParam = ROTATE_LEFT;
					if(!strcmp(myDataString, "rotateRight")) AlgoidMessageRX.msgParam = ROTATE_RIGHT;
					if(!strcmp(myDataString, "stop")) AlgoidMessageRX.msgParam = STOP;
					if(!strcmp(myDataString, "2wd")) AlgoidMessageRX.msgParam = LL_2WD;

	// DATA ARRAY
				  jRead((char *)srcBuffer, KEY_MESSAGE_VALUE, &element );
				  if(element.dataType == JREAD_ARRAY )
				  {
					  AlgoidMessageRX.msgValueCnt=element.elements;
				      for(i=0; i<element.elements; i++ )    // loop for no. of elements in JSON
				      {
				    	  if(AlgoidMessageRX.msgParam == LL_2WD){
				    		  result = jRead_string((char *)srcBuffer, KEY_MESSAGE_VALUE_WHEEL, AlgoidMessageRX.msgValArray[i].wheel, 15, &i );
					    	  AlgoidMessageRX.msgValArray[i].velocity= jRead_long((char *)srcBuffer, KEY_MESSAGE_VALUE_VELOCITY, &i);
					    	  AlgoidMessageRX.msgValArray[i].time= jRead_long((char *)srcBuffer, KEY_MESSAGE_VALUE_TIME, &i);
					    	  AlgoidMessageRX.msgValArray[i].accel= jRead_long((char *)srcBuffer, KEY_MESSAGE_VALUE_ACCEL, &i);
					    	  AlgoidMessageRX.msgValArray[i].decel= jRead_long((char *)srcBuffer, KEY_MESSAGE_VALUE_DECEL, &i);
				    	  }
				    	  else{
				    		  result = jRead_string((char *)srcBuffer, KEY_MESSAGE_VALUE_MODE, AlgoidMessageRX.msgValArray[i].mode, 15, &i );
					    	  AlgoidMessageRX.msgValArray[i].value= jRead_long((char *)srcBuffer, KEY_MESSAGE_VALUE_VALUE, &i);
				    	  }
				    	  if(result == 0)
				    		  return 0;
				    }
				  }

				  //
				  if(AlgoidMessageRX.msgParam < 0 || AlgoidMessageRX.msgType < 0){
					  return 0;
				  }else return 1;
}


// -----------------------------------------------------------------------------
// replyToHost
// convert the structure in JSON format & Send to host
// -----------------------------------------------------------------------------
void ackToJSON(char * buffer, int msgId, char* to, char * from, char * msgType,char * msgParam, char * value, int data ){
	unsigned int buflen= 1024;

// Formatage de la réponse en JSON
	jwOpen( buffer, buflen, JW_OBJECT, JW_PRETTY );		// start root object
		jwObj_string( "MsgTo", to );				// add object key:value pairs
		jwObj_string( "MsgFrom", from );				// add object key:value pairs
		jwObj_int( "MsgID", msgId );
		jwObj_object( "MsgData" );
			jwObj_string( "MsgType", msgType );				// add object key:value pairs
			if(msgParam!=0) jwObj_string( "MsgParam", msgParam );				// add object key:value pairs
			/*
			if(value!=""){
				jwObj_array("MsgValue");
				jwObj_int("test", 9 );
			}
			*/

		jwEnd();
		jwClose();
}
