/*****************************************************
Byte Back Base Station Module By Team Wi-Fly
By Noah Schwartz, Nathan Mattock, Jason Esqueda, Joshua Dinerman, Thomas Ramirez

This is the code that acts as the middle man between the killer module and the online server. After each minute, data with settings for the killer module are sent over from here to the
killer modules. This triggers the killer modules to send data back to the base station, where it is then uploaded to the server. This module also supports communication with up to 6 killer
modules.
*****************************************************/
#include "base_settings.h"
/*****************************************************
System Setup
*****************************************************/
//Setup for WiFi
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

//Setting up the timer
hw_timer_t *My_timer = NULL;
hw_timer_t * resetSender = NULL;
hw_timer_t * connectLim = NULL;

//Number of ESP32 Boards that the base station will connect to
#define NUM_BOARDS 1

//Setup for the radio and addresses
RF24 radio(4, 5);  // CE, CSN on Blue Pill
const uint64_t address0 = 0xF0F0F0F001LL;
const uint64_t address1 = 0xF0F0F0F002LL;
const uint64_t address2 = 0xF0F0F0F003LL;
const uint64_t address3 = 0xF0F0F0F004LL;
const uint64_t address4 = 0xF0F0F0F005LL;
const uint64_t address5 = 0xF0F0F0F006LL;

//Creating the task that handles the timer
TaskHandle_t timer_task;
/*****************************************************
Variables
*****************************************************/
//Used to swap to a different ESP32 Board for communication
uint8_t pipe_nm = 0;
int brd_slct = 0;
bool swap_board = false;

//Used to enable the sending and uploading data process
bool send_en = false;

//used to skip connecting to WiFi and AWS after a certain amount of time
bool skip_en = false;

//used to skip connecting to a board and send dummy data instead
int rf_retry = 0;

//Variables to hold struct data
sysdata sysdat;
sysdata dummy;
settings set;

/*****************************************************
Timer Functions
*****************************************************/
void IRAM_ATTR onTimer() { 
  /* Enables sending and receiving of data */
  send_en = true;
}

void IRAM_ATTR onTimer2() {
  /* Retries RF communication with other device if it fails */
  Serial.println("Failed to connect, trying again");
  radio.stopListening();
  radio.write(&set, sizeof(settings));
  radio.startListening();
  rf_retry++;
}
void IRAM_ATTR onTimer3() {
  /* Stops connecting to WiFi/AWS after a certain amount of time */
  skip_en = true;
  Serial.println("Skipping Operation");
}
void skipSend()
  /* Enables the timer for trying to send data again */
{
  connectLim = timerBegin(0, 80, true);
  timerAttachInterrupt(connectLim, &onTimer3, true);
  timerAlarmWrite(connectLim, 15 * 1000000, true);
  timerAlarmEnable(connectLim);
}
void resetSend()
  /* Enables the timer for trying to send data again */
{
  resetSender = timerBegin(0, 80, true);
  timerAttachInterrupt(resetSender, &onTimer2, true);
  timerAlarmWrite(resetSender, 5 * 1000000, true);
  timerAlarmEnable(resetSender); //Just Enable
}

void t_timer( void * pvParameters ) {
  /* Task that enables the timer for sending out data */
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 10 * 1000000, true);
  timerAlarmEnable(My_timer);
  for (;;)
  {
    vTaskDelay(10);
  }
}
/*****************************************************
Wi-Fi and AWS Connection Functions
NOTE: PART OF THIS IS REPURPOSED CODE FROM https://how2electronics.com/connecting-esp32-to-amazon-aws-iot-core-using-mqtt/
*****************************************************/
void connectAWS()
  /* Connects the board to Wifi and the AWS server */
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");
  skipSend();
  while ((!skip_en) && (WiFi.status() != WL_CONNECTED))
  {
    delay(500);
    Serial.print(".");
  }
  timerAlarmDisable(connectLim);
  timerEnd(connectLim);
  skip_en = false;
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.println("Connecting to AWS IOT");
  skipSend();
  while((!skip_en) && (!client.connect(THINGNAME)))
  {
    Serial.print(".");
    delay(100);
  }
  timerAlarmDisable(connectLim);
  timerEnd(connectLim);
  skip_en = false;
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}


void publishMessage(sysdata dat)
  /* Pushes the data onto the AWS server */
{
  StaticJsonDocument<200> doc;
  char result[64];
  dtostrf(dat.temperature, 6, 3, result);
  doc["Temperature"] = result;
  dtostrf(dat.pressure, 6, 3, result);
  doc["Pressure"] = result;
  dtostrf(dat.humidity, 6, 3, result);
  doc["Humidity"] = result;
  doc["Location"] = convertGPS();
  dtostrf(dat.bat_volt, 6, 3, result);
  doc["Battery_Voltage"] = result;
  doc["Kill_Count"] = String(dat.kill_count);
  unsigned long time = millis() / 1000;
  sprintf(result, "%lu", time);
  doc["Time"] = result;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  Serial.println("Sent to AWS");

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}
void messageHandler(char* topic, byte* payload, unsigned int length)
  /* Receives a message from AWS if there is one */
{
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* led_set = doc["led_en"];
  const char* speaker_set = doc["speaker_en"];
  const char* speaker_freq_set = doc["speaker_freq"];
  set.data_received = true;
  set.speak_freq = atoi(speaker_freq_set);
  set.speak_tog = atoi(speaker_set);
  set.led = atoi(led_set);

}
/*****************************************************
Board Control
*****************************************************/
void switchBoard()
  /* Switches rf communications to the next board, if there is one */
{
  brd_slct = (brd_slct + 1) % NUM_BOARDS;
  switch (brd_slct)
  {
    case 0:
      swap_board = false;
      pipe_nm = 0;
      radio.openWritingPipe(address0);
      
      //Need to restart the timer so data can be requested again
      My_timer = timerBegin(0, 80, true);
      timerAttachInterrupt(My_timer, &onTimer, true);
      timerAlarmWrite(My_timer, 10 * 1000000, true);
      timerAlarmEnable(My_timer);
      Serial.println("Timer restarted");
      break;
    case 1:
      swap_board = false;
      pipe_nm = 1;
      radio.openWritingPipe(address1);
      delay(2000);
      startSend();
      while(!recvData(pipe_nm)) { 
        delay(1);
      }
       sendToServer(sysdat);
      swap_board = true;
      break;
    case 2:
    delay(2000);
      swap_board = false;
      pipe_nm = 2;
      radio.openWritingPipe(address2);
      startSend();
      while(!recvData(pipe_nm)) {
        delay(1);
      }
      sendToServer(sysdat);
      swap_board = true;
      break;
    case 3:
      swap_board = false;
      pipe_nm = 3;
      radio.openWritingPipe(address3);
      startSend();
      while(!recvData(pipe_nm)) { 
        delay(1);
      }
      sendToServer(sysdat);
      swap_board = true;
      break;
    case 4:
      swap_board = false;
      pipe_nm = 4;
      radio.openWritingPipe(address4);
      startSend();
      while(!recvData(pipe_nm)) { 
        delay(1);
      }
      sendToServer(sysdat);
      swap_board = true;
      break;
    case 5:
      swap_board = false;
      pipe_nm = 5;
      radio.openWritingPipe(address5);
      startSend();
      while(!recvData(pipe_nm)) { 
        delay(1);
      }
      sendToServer(sysdat);
      swap_board = true;
      break;
  }
}
/*****************************************************
Data Sending and Receiving
*****************************************************/
String convertGPS()
  /* Converts GPS data into a "latitude, longitude" String format for the server */
{
  double test = sysdat.latitude;
  double test2 = sysdat.longitude;
  char result[64];
  char result2[64];
  dtostrf(test, 3, 3, result);
  dtostrf(test2, 3, 3, result2);
  String str = "";
  str += result;
  str += ", ";
  str += result2;
  return str;
}

int recvData(uint8_t pipe_num) {
  /* Receives a message from AWS if there is one */
  if (radio.available(&pipe_num)) {
    timerAlarmDisable(resetSender);
    timerEnd(resetSender);
    radio.read(&sysdat, sizeof(sysdata));
    btStop();
    return 1;
  }
  return 0;
}
void sendToServer(sysdata dat)
  /* Connects to WiFi and AWS, sends data out, and prints that data on the Serial Monitor */
{
  //Connects and uploads to AWS
  connectAWS();
  publishMessage(dat);
  client.loop();

  //Print out received data
  Serial.println("Data Received!");
  Serial.print("Temperature = ");
  Serial.print(dat.temperature);
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(dat.pressure);
  Serial.println(" hPa");
  Serial.print("Humidity = ");
  Serial.print(dat.humidity);
  Serial.println(" %");
  Serial.print("Latitude = ");
  Serial.print(dat.latitude);
  Serial.println(" degrees");
  Serial.print("Longitude = ");
  Serial.print(dat.longitude);
  Serial.println(" degrees");
  Serial.print("Battery Life = ");
  Serial.print(dat.bat_volt);
  Serial.println(" Volts");
  Serial.print("Time = ");
  Serial.print(millis() / 1000);
  Serial.println(" Seconds");
  Serial.print(dat.kill_count);
  Serial.println(" Mosquitoes Brutally Exterminated From This Plane of Existence");
}
void startSend()
  /* Starts the process of uploading data to the server by sending data to the killer modules */
{
  // Need to turn WiFi off and Bluetooth on, can't have both on at the same time
  WiFi.mode(WIFI_OFF);
  btStart();
  
  radio.stopListening();
  delay(10);
  radio.write(&set, sizeof(settings));
  delay(10);
  radio.startListening();
  //Try again if no data is received after a few seconds
  resetSend();
}
/*****************************************************
Code Setup and Loop
*****************************************************/
void setup() {
  /* Sets up the code to run */
  Serial.begin(115200);
  radio.begin();             

  //Setting the addresses for the base station to read from
  radio.openWritingPipe(address0);
  radio.openReadingPipe(0, address0); 
  radio.openReadingPipe(1, address1);  
  radio.openReadingPipe(2, address2);
  radio.openReadingPipe(3, address3); 
  radio.openReadingPipe(4, address4);  
  radio.openReadingPipe(5, address5);

  //Initializing radio
  radio.setPALevel(RF24_PA_MAX);   
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(28);
  radio.setChannel(108);
  radio.stopListening();

  //Creating the timer task
  xTaskCreatePinnedToCore(
    t_timer,   /* Task function. */
    "Timer Task",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &timer_task,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
}
void loop() {
    /* Starts the process of uploading data to the server by sending data to the first killer module if it is enabled as well as sends the data that killer module sends out to the server */
  delay(10);
  if(send_en)
  {
    timerAlarmDisable(My_timer);
    timerEnd(My_timer);
    Serial.println("Times up");
    WiFi.mode(WIFI_OFF);
    btStart();
    radio.stopListening();
    delay(10);
    radio.write(&set, sizeof(settings));
    delay(10);
    radio.startListening();
    send_en = false;
    resetSend();
  }
  
  if ((brd_slct == 0) && (recvData(pipe_nm) == 1)) {
    sendToServer(sysdat);
    swap_board = true;
  }
  
  if (swap_board)
  {
    set.data_received = false;
    switchBoard();
  }
  if(rf_retry == 4)
  {
    timerAlarmDisable(resetSender);
    timerEnd(resetSender);
    rf_retry = 0;
    Serial.println("Sending dummy data");
    dummy.kill_count++;
    sendToServer(dummy);
    swap_board = true;
  }
}
