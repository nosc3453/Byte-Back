/*****************************************************
Byte Back Killer Module By Team Wi-Fly
By Noah Schwartz, Nathan Mattock, Jason Esqueda, Joshua Dinerman, Thomas Ramirez

This is the main code that controls the mosquito killing module of Byte Back. This code does two main things: recieves instructions from the base station and sends data back to it,
as well as reading from an IR Sensor, which triggers the fan and killing mechanism. If a kill is confirmed, the fan and grid will turn off and the LEDs and Speakers will turn on briely. If
no kill can be confirmed after a set amount of seconds, then the IR sensors will be enabled to trigger again.
*****************************************************/
#include "killer_module_defines.h"
/*****************************************************
System Setup
*****************************************************/
//Timer Setup
TaskHandle_t mqo_fnd;
TaskHandle_t data_s;
hw_timer_t *My_timer = NULL;

//GPS Setup
TinyGPSPlus gps;
static const int RXPin = 16, TXPin = 17;
SoftwareSerial ss(RXPin, TXPin);
static const uint32_t GPSBaud = 9600;

//BME Setup
Adafruit_BME680 bme;

//Radio Setup
RF24 radio(4, 5);
const uint64_t address = 0xF0F0F0F001;
/*****************************************************
Variables
*****************************************************/
//Bool for enabling checking for grid voltage after the IR Sensor has been enabled
bool v_thres_c = false;

//Sets the radio to either receive or send data, or nothing at all
int send_data = 0; //-1 for sending, 0 for none, 1 for receiving

//Enables reading from the IR sensor
bool ir_en = true;

//Enables checking for how long the speakers and leds have been on
bool sl = false;

bool led_en = false;
bool speaker_en = false;

//Double to store the Grid, Battery, and IR Sensor voltage
double battery_v = 0;
double grid_v = 0;
double ir_v = 0;

//Set up for the Speaker
const int TONE_PWM_CHANNEL = 0;
const int MIN_FREQ_HZ = 500;  // min frequency in hz

//Variables to store the Time and Delay for when the fan is activated
unsigned long db_time = 0;
unsigned long db_delay = 2500; //EDIT THIS TO CHANGE HOW LONG THE DELAY IS IN BETWEEN TURNING ON THE FAN AND IT RECOGNIZING THE MOSQUITO IS KILLED

//Variables to store the Time and Delay for when the led and speakers are activated and deactivated
unsigned long sl_delay = 2000;
unsigned long sl_time = 0;


//Variables to hold struct data in
sysdata sysdat;
settings set;

/*****************************************************
System Setup
*****************************************************/
void setup() {
  /* System Setup */
  Serial.begin(115200);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(GRID_PIN, INPUT);
  pinMode(LED_PIN , OUTPUT);

  //Start the GPS and BME
  ss.begin(GPSBaud);
  //if (!bme.begin()) 
  //{
  //  Serial.println("Could not find a valid BME280 sensor, check wiring!");
  //  while(1);
  //}
  //Initializing the Speaker 
  ledcWriteTone(TONE_PWM_CHANNEL, MIN_FREQ_HZ);

  //Creating two tasks on two different cores (will run simultaneously)
  xTaskCreatePinnedToCore(
                    t_mqo_fnd,   /* Task function. */
                    "Mosquito Found Task",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &mqo_fnd,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
  xTaskCreatePinnedToCore(
                    t_data_s,   /* Task function. */
                    "Data Send Task",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &data_s,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */
}
/*****************************************************
Data Gathering
*****************************************************/
void BMEdata() 
  /* Reads BME Data from the BME Sensor */
{
  sysdat.temperature = bme.readTemperature();
  sysdat.pressure = bme.readPressure() / 100.0F;
  sysdat.humidity = bme.readHumidity();
  sysdat.bat_volt = battery_v * 2;
}
static void smartDelay(unsigned long ms)
  /* Delay to read from the GPS */
  /* Repurposed code from example GPS code */
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
void GPSdata() 
  /* Reads GPS Data from the GPS Sensor */
{
  smartDelay(100);
  sysdat.latitude = gps.location.lat();
  sysdat.longitude = gps.location.lng();
}
/*****************************************************
Timer Callback Function
*****************************************************/
void IRAM_ATTR onTimer(){
  /* Turns off the killing mechanism and enables a reading from the IR Sensor if no mosquito is killed within a certain amount of time */
  timerRestart(My_timer);
  timerAlarmDisable(My_timer);
  Serial.println("No Mosquito Found, turning off the fan and enabling the IR Sensor)");
  v_thres_c = false;
  digitalWrite(FAN_PIN, LOW);
  ir_en = true;
}
/*****************************************************
Receiving data
*****************************************************/
int recvData() 
{
  /* Used to check if data has been send from the base station to the killer module */
  if (radio.available()) 
  {
    radio.read(&set, sizeof(settings));
    return 1;
  }
  return 0;
}
/*****************************************************
Tasks
*****************************************************/
void t_mqo_fnd( void * pvParameters ){
  /* Creates the timer that counts down how long the kill mechanism is activated for as well as checks to see if a kill has been confirmed and turns on the speakers and LEDs if so */
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 8 * 1000000, true); //CHANGE THIS NUMBER (x * 1000000 where x is an integer that can be changed) TO CHANGE THE NUMBER OF SECONDS THE FAN WILL STAY ON FOR
  for(;;){
    vTaskDelay(1);
    //If statement to allow the grid to charge up
    if((millis() - db_time) > db_delay) //SWAPPED THESE IF STATEMENTS 4/18/23(v_thres_c) && (grid_v > 0) && (grid_v < THRESHOLD)
    {
      //Checks if the grid voltage has been dropped, signifying a kill
      if((v_thres_c) && (grid_v >= 0) && (grid_v < 0.04))
        {         
          timerRestart(My_timer);
          timerAlarmDisable(My_timer);
          Serial.println("Turning off the Fan!");
          digitalWrite(FAN_PIN, LOW);
          v_thres_c = false;
          sysdat.kill_count++;
          if(!speaker_en)
          {
            ledcAttachPin(SPEAKER_PIN, TONE_PWM_CHANNEL);
          }
          if(!led_en)
          {
            digitalWrite(LED_PIN, HIGH);
          }
          sl_time = millis();
          db_time = millis();
          if((!led_en) && (!speaker_en))
          {
            sl = true;
          }
          else
          {
            sl = false;
          }
          ir_en = true;
       }
    }
    //Checks if the LEDs and Speaker are on, if so, turn it off after a delay of sl_delay
    if((sl) && ((millis() - sl_time) >= sl_delay))
      {
        ledcDetachPin(SPEAKER_PIN);
        digitalWrite(LED_PIN, LOW);
        sl = false;
      }
  } 
}

void t_data_s( void * pvParameters ){
    /* Initializes the radio as well as controls sending and receiving data to and from the base station */
    send_data = 1;
    //Radio Initialization
    radio.begin();
    radio.openWritingPipe(address);
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setPayloadSize(28);
    radio.setChannel(108);
    radio.startListening();
  for(;;){
    vTaskDelay(1);
    //Sends data if it is enabled
    if(send_data == -1)
    {
      delay(100);
      Serial.println("Sending Data!");
      BMEdata();
      GPSdata();
      radio.write(&sysdat, sizeof(sysdata));
      send_data = 1;
      Serial.println("Data Sent!");
      Serial.println("Data Received!");
      Serial.print("Temperature = ");
      Serial.print(sysdat.temperature);
      Serial.println(" *C");
      Serial.print("Pressure = ");
      Serial.print(sysdat.pressure);
      Serial.println(" hPa");
      Serial.print("Humidity = ");
      Serial.print(sysdat.humidity);
      Serial.println(" %");
      Serial.print("Latitude = ");
      Serial.print(sysdat.latitude);
      Serial.println(" degrees");
      Serial.print("Longitude = ");
      Serial.print(sysdat.longitude);
      Serial.println(" degrees");
      Serial.print("Battery Life = ");
      Serial.print(sysdat.bat_volt);
      Serial.println(" Percent");
      Serial.print("Time = ");
      Serial.print(millis() / 1000);
      Serial.println(" Seconds");
      Serial.print(sysdat.kill_count);
      Serial.println(" Mosquitoes Brutally Exterminated From This Plane of Existence");
      radio.startListening();
    }
    //Receives data and changes system settings if it is specified
    if (recvData() and (send_data == 1)) {
      if(set.data_received)
      {
        set.data_received = false;
        ledcWriteTone(TONE_PWM_CHANNEL, set.speak_freq);
        if(set.speak_tog)
        {
          ledcAttachPin(SPEAKER_PIN, TONE_PWM_CHANNEL);
          speaker_en = true;
        }
        else
        {
          speaker_en = false;
          ledcDetachPin(SPEAKER_PIN);
        }
        if(set.led)
        {
          digitalWrite(LED_PIN, HIGH);
          led_en = true;
        }
        else
        {
          led_en = false;
          digitalWrite(LED_PIN, LOW);
        }
      }
      Serial.println("Sending data initiated!");
      send_data = -1;
      radio.stopListening();
    }
  } 
}
/*****************************************************
System Loop
*****************************************************/
void loop() {
  /* System Loop */
  delay(50); //delay here to prevent the system breaking down, can be increased or decreased
  
  //Reading voltage from the grid, battery, and IR sensor(s)
  grid_v = analogRead(GRID_PIN);
  grid_v = (grid_v * 3.3) / 4095;
  battery_v = analogRead(BATTERY_VOLTAGE_PIN);
  battery_v = (battery_v * 3.3) / 4095;
  ir_v = analogRead(IR_SENSOR_PIN);
  ir_v = (ir_v * 3.3) / 4095;
  Serial.println(grid_v);
  //Checks if the IR Sensor(s) has/have been enabled. If so, start the killing mechanism
  if((ir_en) && (ir_v <= 1))
  {
    grid_v=4; //Avoids reading issues with reading the grid voltage
    digitalWrite(FAN_PIN, HIGH);
    delay(2000);
    ir_en = false;
    Serial.println("A mosquito has been found!");
    v_thres_c = true;
    timerAlarmEnable(My_timer);
  }
}
