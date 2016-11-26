/*
 * messagesManager.h
 *
 *  Created on: 15 mars 2016
 *      Author: raph
 */

#ifndef MESSAGESMANAGER_H_
#define MESSAGESMANAGER_H_

extern char ClientID[50];

// Initialisation de la messagerie system (JSON<->MQTT)
int InitMessager(void);
int pullMsgStack(unsigned char ptrStack);
int CloseMessager(void);
void sendMqttReport(int msgId, char * msg);
void sendResponse(int msgId, int msgType, int msgParam, unsigned char valCnt);
#endif /* MESSAGESMANAGER_H_ */
