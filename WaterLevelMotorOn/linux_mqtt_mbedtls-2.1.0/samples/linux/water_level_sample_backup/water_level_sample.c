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
//pthread_t dsp_status_thread;
unsigned char deviceStatus = 0;
struct termios SerialPortSettings;
int serialThr = 1;
unsigned char serialDataStatus = 0 ;

typedef struct waterLevData{
		int waterLevel;
		int motorStatus;
}waterLevelData;

static waterLevelData updateData;

/**
* @brief Thread variables
*/
pthread_t serialStatusThread;


static int GetUpdateSerialData(waterLevelData *data);
static void * SerialCntlThread(void *);




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

	if(pContext != NULL) {
		IOT_INFO("Delta - Window state changed to %d", *(bool *) (pContext->pData));
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
	float temperature = 0.0;
	int waterLevelTemp,motorStatusTemp;
	bool windowOpen = false;
	char deviceStatusStr[50];
#if 0
	jsonStruct_t windowActuator;
	windowActuator.cb = windowActuate_Callback;
	windowActuator.pData = &windowOpen;
	windowActuator.pKey = "windowOpen";
	windowActuator.type = SHADOW_JSON_BOOL;

	jsonStruct_t temperatureHandler;
	temperatureHandler.cb = NULL;
	temperatureHandler.pKey = "temperature";
	temperatureHandler.pData = &temperature;
	temperatureHandler.type = SHADOW_JSON_FLOAT;
#endif
		
	jsonStruct_t deviceStatusControl;
	deviceStatusControl.cb = NULL;
	deviceStatusControl.pKey = "DeviceStatus";
	deviceStatusControl.pData = deviceStatusStr;
	deviceStatusControl.type = SHADOW_JSON_STRING;
	 
	jsonStruct_t waterLevelControl;
	waterLevelControl.cb = NULL;
	waterLevelControl.pKey = "WaterLevel";
	waterLevelControl.pData = &waterLevelTemp;
	waterLevelControl.type = SHADOW_JSON_INT32;

	jsonStruct_t motorStatusControl;
	motorStatusControl.cb = NULL;
	motorStatusControl.pKey = "MotorStatus";
	motorStatusControl.pData = &motorStatusTemp;
	motorStatusControl.type = SHADOW_JSON_INT32;
	
	char rootCA[PATH_MAX + 1];
	char clientCRT[PATH_MAX + 1];
	char clientKey[PATH_MAX + 1];
	char CurrentWD[PATH_MAX + 1];
	
	waterLevelData getSerialData;
	
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
	ret = pthread_create(&serialStatusThread ,NULL, SerialCntlThread,(void *) 0);
        if ( ret != 0 ){
                printf("pthread_create error: %s (%d)\n", strerror(errno), errno);
                return -1;
        }

	IOT_INFO("Shadow Init");
	/*while(1){
		if(GetUpdateSerialData(&getSerialData))
		{
			printf("waterlevel %d motorstatus %d \r\n",getSerialData.waterLevel,getSerialData.motorStatus);
			
		}
		else
			printf("DEVICE is OFFLINE \r\n");		
		sleep(1);
	}*/
	rc = aws_iot_shadow_init(&mqttClient, &sp);
	if(SUCCESS != rc) {
		IOT_ERROR("Shadow Connection Error");
		serialThr = 0;
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
		serialThr = 0;
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
		serialThr = 0;
		return rc;
	}

	//rc = aws_iot_shadow_register_delta(&mqttClient, &windowActuator);

	//if(SUCCESS != rc) {
	//	IOT_ERROR("Shadow Register Delta Error");
	//}
	temperature = STARTING_ROOMTEMPERATURE;

	// loop and publish a change in temperature
	while(NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc) {
		

		rc = aws_iot_shadow_yield(&mqttClient, 200);
		if(NETWORK_ATTEMPTING_RECONNECT == rc) {
			sleep(1);
			// If the client is attempting to reconnect we will skip the rest of the loop.
			continue;
		}
		//IOT_INFO("\n=======================================================================================\n");
		//IOT_INFO("On Device: window state %s", windowOpen ? "true" : "false");
		//simulateRoomTemperature(&temperature);
		if(GetUpdateSerialData(&getSerialData))
		{
			printf("waterlevel %d motorstatus %d \r\n",getSerialData.waterLevel,getSerialData.motorStatus);
			waterLevelTemp = getSerialData.waterLevel;
			motorStatusTemp = getSerialData.motorStatus;
			strcpy(deviceStatusStr,"ONLINE");	
		}
		else
		{
			printf("DEVICE is OFFLINE \r\n");
			strcpy(deviceStatusStr,"OFFLINE");
			sleep(1);
		}		
		rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
		if(SUCCESS == rc) {
			rc = aws_iot_shadow_add_reported(JsonDocumentBuffer, sizeOfJsonDocumentBuffer, 3,&deviceStatusControl,&waterLevelControl,
											 &motorStatusControl);
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
		sleep(1);
	}

	if(SUCCESS != rc) {
		IOT_ERROR("An error occurred in the loop %d", rc);
		serialThr = 0;
		usleep(100*1000);
	}

	IOT_INFO("Disconnecting");
	rc = aws_iot_shadow_disconnect(&mqttClient);

	if(SUCCESS != rc) {
		IOT_ERROR("Disconnect error %d", rc);
	}

	return rc;
}

int GetUpdateSerialData(waterLevelData *data)
{
	if(serialDataStatus == 0)
		return serialDataStatus;
	else{
		data->waterLevel = updateData.waterLevel;
		data->motorStatus = updateData.motorStatus;
		serialDataStatus = 0;
	}

	return 1;
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
	serialDataStatus = 1;
		
}

static void * SerialCntlThread(void * args)
{
	char line[100];
        int chkin,count = 0;
        char getch;

        int file= open("/dev/ttyACM0", O_RDWR | O_NOCTTY|O_NONBLOCK);

        
               // printf("USB Serial success \r\n");
        
	

	while(serialThr)
	{
		
        	if (file == 0 || file == -1 )
        	{
                	printf("open_port: Unable to open ttyUSB0 \n");
			deviceStatus = 0;
        		file= open("/dev/ttyACM0", O_RDWR | O_NOCTTY|O_NONBLOCK);
			sleep(1);
		}
		else{
			if(!deviceStatus)
			{
				printf("Serial device connected \r\n");
				deviceStatus = 1;
				tcgetattr(file, &SerialPortSettings);
			        cfsetispeed(&SerialPortSettings,B9600);
        			cfsetospeed(&SerialPortSettings,B9600);
        			//SerialPortSettings.c_cflag &= ~PARENB;   // No Parity

        			//SerialPortSettings.c_cflag &= ~CSTOPB; //Stop bits = 1

        			SerialPortSettings.c_cflag &= ~CSIZE; /* Clears the Mask       */
        			SerialPortSettings.c_cflag |=  CS8;   /* Set the data bits = 8 */

        			//SerialPortSettings.c_cflag |= CREAD | CLOCAL;

        			//SerialPortSettings.c_lflag = 0;
        			//SerialPortSettings.c_cc[VTIME] = 10;
        			SerialPortSettings.c_cflag     &=  ~CRTSCTS;           // no flow control
        			SerialPortSettings.c_cc[VMIN]   =  1;                  // read doesn't block
        			SerialPortSettings.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
        			SerialPortSettings.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines


        			tcsetattr(file,TCSANOW,&SerialPortSettings);
        			fcntl(file, F_SETFL,0);

			}

	                while ((chkin=read(file,&getch,1))>=0 && serialThr )
        	        {
                	        if (chkin<0)
                        	{
                                	printf("cannot read from port\n");
                        	}
                        	else
                        	{
                                	line[count] = getch;
					//printf("getch %d  chr %c \r\n",getch,getch);
                                	count++;
                                	if((getch == '\n') || (getch == '\0')|| (getch == '\r'))
                                	{
						line[count-1]='\0';
                                        	count = 0;
						
                                        	if(strlen(line) > 2){
                                                	//printf("data=%s \r\n",line);
                                        		updateWaterLevelData(line);
							memset(line,'\0',sizeof(line));
							usleep(300*1000);			
						}
                                	}
                        	}
                        	
                	}
			//sleep(1);
			//usleep(400);
			printf("chkin %d\r\n",chkin);

        	}

	}
	if( file !=0 || file != -1)  
		close(file);
	return NULL;
}
			 	
