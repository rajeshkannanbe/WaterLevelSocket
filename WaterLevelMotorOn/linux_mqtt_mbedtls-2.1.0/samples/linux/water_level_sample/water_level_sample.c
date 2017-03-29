/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

/**
 * @file shadow_sample.c
 * @brief A simple connected window example demonstrating the use of Thing Shadow
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <limits.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include<unistd.h>    //write


#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_shadow_interface.h"

/*!
 * The goal of this sample application is to demonstrate the capabilities of shadow.
 * This device(say Connected Window) will open the window of a room based on temperature
 * It can report to the Shadow the following parameters:
 *  1. temperature of the room (double)
 *  2. status of the window (open or close)
 * It can act on commands from the cloud. In this case it will open or close the window based on the json object "windowOpen" data[open/close]
 *
 * The two variables from a device's perspective are double temperature and bool windowOpen
 * The device needs to act on only on windowOpen variable, so we will create a primitiveJson_t object with callback
 The Json Document in the cloud will be
 {
 "reported": {
 "temperature": 0,
 "windowOpen": false
 },
 "desired": {
 "windowOpen": false
 }
 }
 */

#define ROOMTEMPERATURE_UPPERLIMIT 32.0f
#define ROOMTEMPERATURE_LOWERLIMIT 25.0f
#define STARTING_ROOMTEMPERATURE ROOMTEMPERATURE_LOWERLIMIT

#define MAX_LENGTH_OF_UPDATE_JSON_BUFFER 200

static char certDirectory[PATH_MAX + 1] = "../../../certs";
static char HostAddress[255] = AWS_IOT_MQTT_HOST;
static uint32_t port = AWS_IOT_MQTT_PORT;
static uint8_t numPubs = 5;

//serial global variable 
pthread_t dsp_status_thread;
unsigned char deviceStatus = 0;
bool threadStatus = 1;
/**
* @brief Thread variables
*/
pthread_t NodeMicoStatusThread;


static void * MicoSocketCntlThread(void *);

typedef struct waterLevData{
        int waterLevel;
        bool motorStatus;
}waterLevelData;

static waterLevelData updateData;
unsigned char socketDataStatus = 0;
void updateWaterLevelData(char * data);

static int motorstate = 0;

static void simulateRoomTemperature(float *pRoomTemperature) {
	static float deltaChange;

	if(*pRoomTemperature >= ROOMTEMPERATURE_UPPERLIMIT) {
		deltaChange = -0.5f;
	} else if(*pRoomTemperature <= ROOMTEMPERATURE_LOWERLIMIT) {
		deltaChange = 0.5f;
	}

	*pRoomTemperature += deltaChange;
}

void ShadowUpdateStatusCallback(const char *pThingName, ShadowActions_t action, Shadow_Ack_Status_t status,
								const char *pReceivedJsonDocument, void *pContextData) {
	IOT_UNUSED(pThingName);
	IOT_UNUSED(action);
	IOT_UNUSED(pReceivedJsonDocument);
	IOT_UNUSED(pContextData);

	if(SHADOW_ACK_TIMEOUT == status) {
		IOT_INFO("Update Timeout--");
	} else if(SHADOW_ACK_REJECTED == status) {
		IOT_INFO("Update RejectedXX");
	} else if(SHADOW_ACK_ACCEPTED == status) {
		IOT_INFO("Update Accepted !!");
	}
}

void windowActuate_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
	IOT_UNUSED(pJsonString);
	IOT_UNUSED(JsonStringDataLen);
	bool status = *(bool *) (pContext->pData);
		
	if(pContext != NULL) {
		IOT_INFO("Delta - MOTOR state changed to %d", *(bool *) (pContext->pData));
		if(status != updateData.motorStatus)
			motorstate = 1;
	}
}

void parseInputArgsForConnectParams(int argc, char **argv) {
	int opt;

	while(-1 != (opt = getopt(argc, argv, "h:p:c:n:"))) {
		switch(opt) {
			case 'h':
				strcpy(HostAddress, optarg);
				IOT_DEBUG("Host %s", optarg);
				break;
			case 'p':
				port = atoi(optarg);
				IOT_DEBUG("arg %s", optarg);
				break;
			case 'c':
				strcpy(certDirectory, optarg);
				IOT_DEBUG("cert root directory %s", optarg);
				break;
			case 'n':
				numPubs = atoi(optarg);
				IOT_DEBUG("num pubs %s", optarg);
				break;
			case '?':
				if(optopt == 'c') {
					IOT_ERROR("Option -%c requires an argument.", optopt);
				} else if(isprint(optopt)) {
					IOT_WARN("Unknown option `-%c'.", optopt);
				} else {
					IOT_WARN("Unknown option character `\\x%x'.", optopt);
				}
				break;
			default:
				IOT_ERROR("ERROR in command line argument parsing");
				break;
		}
	}

}

int main(int argc, char **argv) {
	IoT_Error_t rc = FAILURE;
	int32_t i = 0;
	int ret = 0;
	char JsonDocumentBuffer[MAX_LENGTH_OF_UPDATE_JSON_BUFFER];
	size_t sizeOfJsonDocumentBuffer = sizeof(JsonDocumentBuffer) / sizeof(JsonDocumentBuffer[0]);
	char *pJsonStringToUpdate;
	int waterLevelval;
	waterLevelData getData;

	bool motorOn = false;
	jsonStruct_t motorstatus;
	motorstatus.cb = windowActuate_Callback;
	motorstatus.pData = &motorOn;
	motorstatus.pKey = "Motoron";
	motorstatus.type = SHADOW_JSON_BOOL;

	jsonStruct_t waterLevelHandler;
	waterLevelHandler.cb = NULL;
	waterLevelHandler.pKey = "waterlevel";
	waterLevelHandler.pData =&waterLevelval;
	waterLevelHandler.type = SHADOW_JSON_INT32;

	char rootCA[PATH_MAX + 1];
	char clientCRT[PATH_MAX + 1];
	char clientKey[PATH_MAX + 1];
	char CurrentWD[PATH_MAX + 1];

	IOT_INFO("\nAWS IoT SDK Version %d.%d.%d-%s\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

	getcwd(CurrentWD, sizeof(CurrentWD));
	snprintf(rootCA, PATH_MAX + 1, "%s/%s/%s", CurrentWD, certDirectory, AWS_IOT_ROOT_CA_FILENAME);
	snprintf(clientCRT, PATH_MAX + 1, "%s/%s/%s", CurrentWD, certDirectory, AWS_IOT_CERTIFICATE_FILENAME);
	snprintf(clientKey, PATH_MAX + 1, "%s/%s/%s", CurrentWD, certDirectory, AWS_IOT_PRIVATE_KEY_FILENAME);

	IOT_DEBUG("rootCA %s", rootCA);
	IOT_DEBUG("clientCRT %s", clientCRT);
	IOT_DEBUG("clientKey %s", clientKey);

	parseInputArgsForConnectParams(argc, argv);

	// initialize the mqtt client
	AWS_IoT_Client mqttClient;

	ShadowInitParameters_t sp = ShadowInitParametersDefault;
	sp.pHost = AWS_IOT_MQTT_HOST;
	sp.port = AWS_IOT_MQTT_PORT;
	sp.pClientCRT = clientCRT;
	sp.pClientKey = clientKey;
	sp.pRootCA = rootCA;
	sp.enableAutoReconnect = false;
	sp.disconnectHandler = NULL;

	//serial initilize and communicate arduion via serial communication in thread
	ret = pthread_create(&NodeMicoStatusThread ,NULL, MicoSocketCntlThread,(void *) 0);
        if ( ret != 0 ){
                printf("pthread_create error: %s (%d)\n", strerror(errno), errno);
                return -1;
        }

	IOT_INFO("Shadow Init");
#if 1	
	rc = aws_iot_shadow_init(&mqttClient, &sp);
	if(SUCCESS != rc) {
		IOT_ERROR("Shadow Connection Error");
		return rc;
	}

	ShadowConnectParameters_t scp = ShadowConnectParametersDefault;
	scp.pMyThingName = AWS_IOT_MY_THING_NAME;
	scp.pMqttClientId = AWS_IOT_MQTT_CLIENT_ID;
	scp.mqttClientIdLen = (uint16_t) strlen(AWS_IOT_MQTT_CLIENT_ID);

	IOT_INFO("Shadow Connect");
	rc = aws_iot_shadow_connect(&mqttClient, &scp);
	if(SUCCESS != rc) {
		IOT_ERROR("Shadow Connection Error");
		return rc;
	}

	/*
	 * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
	 *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
	 *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
	 */
	rc = aws_iot_shadow_set_autoreconnect_status(&mqttClient, true);
	if(SUCCESS != rc) {
		IOT_ERROR("Unable to set Auto Reconnect to true - %d", rc);
		return rc;
	}

	rc = aws_iot_shadow_register_delta(&mqttClient, &motorstatus);

	if(SUCCESS != rc) {
		IOT_ERROR("Shadow Register Delta Error");
	}
	//temperature = STARTING_ROOMTEMPERATURE;
	//strcpy(waterLevelStr,"HIGH");
	// loop and publish a change in temperature
	while(NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc) {
		rc = aws_iot_shadow_yield(&mqttClient, 200);
		if(NETWORK_ATTEMPTING_RECONNECT == rc) {
			sleep(5);
			// If the client is attempting to reconnect we will skip the rest of the loop.
			continue;
		}
		//simulateRoomTemperature(&temperature);
		if(GetUpdateSocketData(&getData))
		{
			waterLevelval = getData.waterLevel;
			motorOn = getData.motorStatus;
			socketDataStatus = 1; 	
		}
		else{
			sleep(5);
			continue;
		}
		if(motorstate)
		{
			sleep(1);
		}
		 IOT_INFO("\n=======================================================================================\n");
                IOT_INFO("On Device: motor state %s\n", motorOn ? "true" : "false");
		IOT_INFO("Water Level %s\n",(waterLevelval == 3) ? "HIGH":(waterLevelval == 2)?"Mediem":"LOW");
		rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
		if(SUCCESS == rc) {
			rc = aws_iot_shadow_add_reported(JsonDocumentBuffer, sizeOfJsonDocumentBuffer, 2, &waterLevelHandler,
											 &motorstatus);
			if(SUCCESS == rc) {
				rc = aws_iot_finalize_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
				if(SUCCESS == rc) {
					IOT_INFO("Update Shadow: %s", JsonDocumentBuffer);
					rc = aws_iot_shadow_update(&mqttClient, AWS_IOT_MY_THING_NAME, JsonDocumentBuffer,
											   ShadowUpdateStatusCallback, NULL, 4, true);
				}
			}
		}
		IOT_INFO("*****************************************************************************************\n");
		sleep(5);
	}

	if(SUCCESS != rc) {
		IOT_ERROR("An error occurred in the loop %d", rc);
	}

	IOT_INFO("Disconnecting");
	rc = aws_iot_shadow_disconnect(&mqttClient);

	if(SUCCESS != rc) {
		IOT_ERROR("Disconnect error %d", rc);
	}
#endif
	while(1){
		sleep(2);
	}
	return rc;
}





static void * MicoSocketCntlThread(void * args)
{
	int socket_desc , client_sock , c , read_size;
	struct sockaddr_in server , client;
	char client_message[200];
	char server_message[200];

	while(threadStatus){
		//Create socket
		socket_desc = socket(AF_INET , SOCK_STREAM , 0);
		if (socket_desc == -1)
		{
			printf("Could not create socket");
		}
		puts("Socket created");

		//Prepare the sockaddr_in structure
		server.sin_family = AF_INET;
		server.sin_addr.s_addr = INADDR_ANY;
		server.sin_port = htons( 8888 );

		//Bind
		if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
		{
        		//print the error message
        		perror("bind failed. Error");
			continue;
        		//return 1;
    		}
    		puts("bind done");

    		//Listen
    		listen(socket_desc , 3);

    		//Accept and incoming connection
    		puts("Waiting for incoming connections...");
    		c = sizeof(struct sockaddr_in);

    		//accept connection from an incoming client
    		client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
    		if (client_sock < 0)
    		{
        		perror("accept failed");
        		continue;
    		}

		puts("Connection accepted");

    		//Receive a message from client
    		while( (read_size = recv(client_sock , client_message , 200 , 0)) > 0 )
    		{
			  //Send the message back to client
		        //printf("1 size %d msg %s \r\n",read_size,client_message);
        		if(read_size == 4){
                		printf("2 size %d msg %s \r\n",read_size,client_message);
                		if(!motorstate){
					strcpy(server_message,"telemetry");
                			write(client_sock ,server_message, strlen(server_message));
				}
				else{
					strcpy(server_message,"cmd");
                                        write(client_sock ,server_message, strlen(server_message));
					motorstate = 0;

				}
        		}
        		else{
				printf("Return ..");
		                strcpy(server_message,"error");
                		write(client_sock , server_message , strlen(server_message));
                		sleep(2);
                		memset(server_message,0,200);
                		continue;

        		}
			memset(client_message,0,200);
        		read_size = recv(client_sock , client_message , 200 ,0);
        		printf("len %d data %s \r\n",read_size,client_message);
        		//write(client_sock , client_message , strlen(client_message));
			updateWaterLevelData(client_message);
			memset(client_message,0,200);
        		memset(server_message,0,200);
    		}

    		if(read_size == 0)
    		{
	       		puts("Client disconnected");
			sleep(2);
        		fflush(stdout);
    		}
    		else if(read_size == -1)
    		{
			sleep(2);
        		perror("recv failed");
    		}
	}
	return NULL;
}


int GetUpdateSocketData(waterLevelData *data)
{
    if(socketDataStatus == 0)
        return socketDataStatus;
    else{
        data->waterLevel = updateData.waterLevel;
        data->motorStatus = updateData.motorStatus;
        //socketDataStatus = 0;
    }

    return socketDataStatus;
}

void updateWaterLevelData(char * data)
{
    int count=0,num = 0;
    char buf[4];
    while(data[count] !='\0' )
    {
        if(data[count] == '{')
            count++;
        else if( data[count]==',')
        {

            buf[num]='\0';
            updateData.waterLevel = atoi(buf);
            num = 0;
            count++;

        }
        else if(data[count] == '}')
        {
            buf[num]='\0';
            updateData.motorStatus = atoi(buf);
            num = 0;
            count++;

        }
        else{
            buf[num++] = data[count++];
            }
    }
    socketDataStatus = 1;
}
			 	
