/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */

#include <ESP8266WiFi.h>

#define WATER_LEVEL_HIGH 14
#define WATER_LEVEL_MEDIUM  12
#define WATER_LEVEL_LOW 13

#define LED_HIGH_PIN 15
#define LED_MEDIUM_PIN 3
#define LED_LOW_PIN 1

byte sensorPin[] = {WATER_LEVEL_LOW, WATER_LEVEL_MEDIUM, WATER_LEVEL_HIGH};
byte ledPin[] = {LED_LOW_PIN, LED_MEDIUM_PIN, LED_HIGH_PIN}; // number of leds = numbers of sensors

const byte sensors = 3;
int level = 0;
int count = 0;
int storelevel = 0; 
int storemotor = 0;

const char* ssid     = "vstar";
const char* password = "12345678";

const char* host = "192.168.0.100";
//const char* streamId   = "....................";
//const char* privateKey = "....................";
int motor = 5;

char buffer[20];
char waterlevelbuffer[8];
char motorstatus[5];
int motoron(bool on);
int waterlevel(void);
void setup() {
  Serial.begin(115200);
  delay(10);

     // initialize serial communication at 9600 bits per second:

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //waterlevel();
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("connecting tested to ");

   for(int i = 0; i < sensors; i++) {
     pinMode(sensorPin[i], INPUT); 
     digitalWrite(sensorPin[i],LOW);
     //pinMode(ledPin[i], OUTPUT);
  }
  pinMode(3,OUTPUT);
  pinMode(15,OUTPUT);
  pinMode(motor, OUTPUT);
  digitalWrite(motor, LOW); 

}

int value = 0;
int ret;
void loop() {
  delay(5000);
  ++value;

  Serial.print("connecting to ");
  Serial.println(host);
  
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 8888;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  
  // We now create a URI for the request
  //String url = "/input/";
  //url += streamId;
  //url += "?private_key=";
  //url += privateKey;
  //url += "&value=";
  //url += value;
  while(1){
  Serial.print("Requesting \r\n");
  //Serial.println(url);
  
  // This will send the request to the server
  client.print("WTD?");
  unsigned long timeout = millis();
  
  while (client.available() == 0) {
    if (millis() - timeout > 60000) {
      ret = client.available();
      Serial.print(ret);
      Serial.println(">>> Client Timeout !  ");
      client.stop();
      client.print("error");
      return;
    }
  }
  
  // Read all the lines of the reply from server and print them to Serial
    while(client.available()){
      String line = client.readStringUntil('\r');
      Serial.print(line);
      if(line == "telemetry"){
        //client.print("receive telemetry \r\n");
        Serial.print("receive telemetry \r\n"); 
        waterlevel();
        client.print(buffer);
      }
      else if(line == "cmd"){
      //motor on
        Serial.print("cmd \r\n");
        if(digitalRead(motor))
          motoron(0);
        else
          motoron(1);
        waterlevel();
        client.print(buffer);
        
      }
      else{
        //error
      client.print("error");
    }
    }
  }
  
  Serial.println();
  Serial.println("closing connection");
}

int waterlevel()
{

  level = 0;
  for(int i = 0; i < sensors; i++) {
     if(digitalRead(sensorPin[i]) == HIGH) {
        //digitalWrite(ledPin[i], HIGH);
        //Serial.println(sensorPin[i]);
        level |= (1<< i);
        
     } else {
       //digitalWrite(ledPin[i], LOW);       
     }    
  }
  if(digitalRead(motor))
    strcpy(motorstatus,"1");
  else
    strcpy(motorstatus,"0");
  //Serial.println(level);
  switch(level) {
     case 1:
       //LOW
       strcpy(waterlevelbuffer,"1");
       
      break;
     case 3:
       //Serial.print("MED\r\n");
       //digitalWrite(motor, LOW);
       strcpy(waterlevelbuffer,"2");
       break;
     case 7:
       strcpy(waterlevelbuffer,"3");
       if(digitalRead(motor)){
        //digitalWrite(motor, LOW);
        //strcpy(motorstatus,"0");
       }
       else{
        //strcpy(motorstatus,"0");
       }
       break;
     case 0:
        strcpy(waterlevelbuffer,"0");
        if(digitalRead(motor)){
        
        //strcpy(motorstatus,"1");
       }
       else{
        //digitalWrite(motor, HIGH);
        //strcpy(motorstatus,"1");
       } 
      break;
     default:
      strcpy(waterlevelbuffer,"-1");
      if(digitalRead(motor)){
        digitalWrite(motor, LOW);
        strcpy(motorstatus,"0");
       }
       else{
        strcpy(motorstatus,"0");
       }

     break;
  }
  //if(( storelevel != level)|| (storemotor != digitalRead(motor))  ||(count == 100) )
  //{
     //count = 0;
      // print string for total acceleration, separated by line for ease of reading
     memset(buffer,'\0',sizeof(buffer));
     strcat(buffer,"{");
     strcat(buffer,waterlevelbuffer);
     strcat(buffer,",");
     strcat(buffer,motorstatus);
     strcat(buffer,"}");
     //Serial.println(buffer);
  
  
    
  //}

  storelevel = level;
  storemotor = digitalRead(motor);
  return 1;
}

int motoron(bool on)
{
  
level = 0;
  if(!on)
  {
    digitalWrite(motor, LOW);
    return 1;
  }
  
  for(int i = 0; i < sensors; i++) {
     if(digitalRead(sensorPin[i]) == HIGH) {
        //digitalWrite(ledPin[i], HIGH);
        //Serial.println(sensorPin[i]);
        level |= (1<< i);
        
     } else {
       //digitalWrite(ledPin[i], LOW);       
     }    
  }
  if(level == 7)
  {
    Serial.print("motor on  \r\n");
    digitalWrite(motor, HIGH);
    digitalWrite(LED_HIGH_PIN, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(1, HIGH);
    
    return 0;
  }
  else{
      digitalWrite(motor, HIGH);
  }

  return 1;


}

