/*****************************************************
INCLUDES
*****************************************************/
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_BME680.h>
#include <Adafruit_GPS.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*****************************************************
PIN DEFINITIONS
*****************************************************/
#define IR_SENSOR_PIN 15 //Reads IR Sensor Voltage
#define BATTERY_VOLTAGE_PIN 27 //Reads Battery Voltage
#define FAN_PIN 13 //Used to control the fan
#define GRID_PIN 35 //Reads Voltage from the Grid
#define LED_PIN 12 //LED Control
#define SPEAKER_PIN 14 //Speaker Control
/*****************************************************
STRUCTS
*****************************************************/
struct settings {
  bool data_received = false;
  bool led = false;
  bool speak_tog = false;
  int speak_freq = 500;
};
struct sysdata{
  float temperature = 0.0;
  float humidity = 1.0;
  float pressure = 3.0;
  float latitude;
  float longitude;
  float bat_volt;
  int kill_count;
};
