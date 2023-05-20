 /*****************************************************
INCLUDES
*****************************************************/
#include <nRF24L01.h>
#include <RF24.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
/*****************************************************
PIN DEFINITIONS
*****************************************************/

/*****************************************************
STRUCTS
*****************************************************/
struct settings {
  bool data_received = false;
  bool led = false;
  bool speak_tog = false;
  int speak_freq = 500;
};
struct sysdata {
  float temperature = 0.0;
  float humidity = 1.0;
  float pressure = 2.0;
  float latitude = 3.0;
  float longitude = 4.0;
  float bat_volt = 5.0;
  int kill_count = 0;
};
/*****************************************************
AWS and WiFi Setup
*****************************************************/
#define THINGNAME "EXPO_THING"                         
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pubByteBack" //Use this
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/subByteBack"

const char WIFI_SSID[] = "xxxxx";              
const char WIFI_PASSWORD[] = "xxxx";           
const char AWS_IOT_ENDPOINT[] = "a31yqtwd5sx8lx-ats.iot.us-east-2.amazonaws.com";       
 
// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
xxx
-----END CERTIFICATE-----
)EOF";
 
// Device Certificate                                              
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
xxx
-----END CERTIFICATE-----


 
 
)KEY";
 
// Device Private Key                                               
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
xxx
-----END RSA PRIVATE KEY-----

 
)KEY";
