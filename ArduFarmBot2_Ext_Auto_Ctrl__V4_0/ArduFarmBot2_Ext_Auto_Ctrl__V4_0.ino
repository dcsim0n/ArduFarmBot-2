/***************************************************************************************************************
 *  ArduFarmBot using NodeMCU ESP-12 Develop Kit V1.0
 *  DHT connected to NodeMCU pin D3 (Ambient Temperature and Relative Humidity)
 *  Soil Moister Sensor connected to A0
 *  Sensor Data on local OLED Display
 *  Local Command via buttons 
 *  OLED Display is off as default. Press Sensor Button to update and display data
 *  Introduced the function "waitButtonPress (int waitTime)", to break initial loop
 *  Automatic Local Control
 *  Display automatic set-up parameters at Start-up
 *  Sensor data sent to Blynk app
 *  Control commands received from Blynk app
 *  Downloads, docs, tutorials: http://www.blynk.cc
 *  Blynk library is licensed under MIT license
 *  
 *  V4.0 Invert the logic to use Relay module that works from HIGH to LOW
 *       
 *  MJRoBot Version 4.0 - Automatic and Remote Control - Logic LOW -  Developed by MJRovai 16 Feb 2017
 ********************************************************************************************************************************/
#define SW_VERSION "   SW Ver. 4.0" // SW version will appears at innitial LCD Display
#include "stationDefines.h"       // Project definitions
#include "secret.h"

/* ESP & Blynk */
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
WidgetLED PUMPs(V0);  // Echo signal to Sensors Tab at Blynk App
WidgetLED PUMPa(V5); // Echo signal to Actuators Tab at Blynk App
WidgetLED LAMPs(V1);  // Echo signal to Sensors Tab at Blynk App
WidgetLED LAMPa(V6); // Echo signal to Actuators Tab at Blynk App
WidgetLED LIGHTa(V7);
WiFiClient espClient;

#include <PubSubClient.h>
PubSubClient client(espClient);

/* TIMER */
//#include <SimpleTimer.h>
SimpleTimer timer;

/* OLED */
#include <ACROBOTIC_SSD1306.h> // library for OLED: SCL ==> D1; SDA ==> D2
#include <SPI.h>

/* DHT22*/
//#include "DHT.h"
//DHT dht(DHTPIN, DHTTYPE);
#include <Wire.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;



/* DS18B20 Temperature Sensor */
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println("ArduFarmBot 2");
  Serial.println(".... Starting Setup");
  Serial.println(" ");

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LAMP_PIN, OUTPUT);
  pinMode(PUMP_ON_BUTTON, INPUT_PULLUP);
  pinMode(LAMP_ON_BUTTON, INPUT_PULLUP);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(SENSORS_READ_BUTTON, INPUT_PULLUP);
  pinMode(soilMoisterVcc, OUTPUT);
  
  setup_wifi();
  oledStart();
  //dht.begin();
  Wire.begin(BME_SDA, BME_SCL); 
  bme.begin(BME_ADDR, &Wire);

  DS18B20.begin();

  PUMPs.off();
  LAMPs.off();
  PUMPa.off();
  LAMPa.off();
  
  digitalWrite(PUMP_PIN, HIGH); // To be used with Relay module (inverted logic: normally HIGH)
  digitalWrite(LAMP_PIN, HIGH); // To be used with Relay module (inverted logic: normally HIGH)
  digitalWrite(LIGHT_PIN, HIGH);
  digitalWrite (soilMoisterVcc, LOW);
  
  waitButtonPress (SHOW_SET_UP); // Wait for Sensor Button to be pressed 
  oled.clearDisplay();
  startTimers();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe("/missionControl/lightStatus");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
//if (lampStatus == 1) 
//  {
//    Blynk.notify("ArduFarmBot2: Warning ==>> Lamp ON");
//    digitalWrite(LAMP_PIN, LOW); // To be used with Relay module (inverted logic: activate with LOW)
//    if (!turnOffOLED) displayData();
//    LAMPs.on();
//    LAMPa.on();
//  }
    digitalWrite(LIGHT_PIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    if(!turnOffOLED) displayData();
    LIGHTa.on();
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(LIGHT_PIN, HIGH);  // Turn the LED off by making the voltage HIGH
    if(!turnOffOLED) displayData();
    LIGHTa.off();
  }

}

void loop()
{
  timer.run(); // Initiates SimpleTimer
  Blynk.run();

  if(!client.connected()){
    reconnect();
  }
  client.loop();
}

/****************************************************************
* Read remote commands 
****************************************************************/
BLYNK_WRITE(2)
{
  int i=param.asInt();
  if(i==1)
  {
    lightStatus = !lightStatus;
    aplyCmd();
  }
}
BLYNK_WRITE(3) // Pump remote control
{
  int i=param.asInt();
  if (i==1) 
  {
    pumpStatus = !pumpStatus;
    aplyCmd();
  }
}

BLYNK_WRITE(4) // Lamp remote control
{
  int i=param.asInt();
  if (i==1)
  {
    lampStatus = !lampStatus;
    aplyCmd();
  }
}

/****************************************************************
* Read local commands (Pump and Lamp buttons are normally "HIGH"):
****************************************************************/
void readLocalCmd() 
{  
  boolean digiValue = debounce(PUMP_ON_BUTTON);
  if (!digiValue) 
  {
    pumpStatus = !pumpStatus;
    aplyCmd();
  }

  digiValue = debounce(LAMP_ON_BUTTON);
  if (!digiValue) 
  {
    lampStatus = !lampStatus;
    aplyCmd();
  }

  digiValue = debounce(SENSORS_READ_BUTTON);
  if (!digiValue) 
  {
    turnOffOLED = !turnOffOLED;
    if (!turnOffOLED)
    {
      oled.setTextXY(0,0); oled.putString("UPDATING SENSORS");
      getDhtData();
      getSoilMoisterData();
      getSoilTempData();
      oledStart();
      displayData();
    }else oled.clearDisplay();
  }
}

/***************************************************
* Receive Commands and act on RELAYs - Logic LOW
****************************************************/
void aplyCmd()
{
  if (pumpStatus == 1) 
  {
    Blynk.notify("ArduFarmBot2: Warning ==>> Pump ON"); 
    digitalWrite(PUMP_PIN, LOW); // To be used with Relay module (inverted logic: activate with LOW)
    if (!turnOffOLED) displayData();
    PUMPs.on();
    PUMPa.on();
  }
  else
      {
        digitalWrite(PUMP_PIN, HIGH); // To be used with Relay module (inverted logic: normally HIGH)
        if (!turnOffOLED) displayData();
        PUMPs.off();
        PUMPa.off();
      }
  
  if (lampStatus == 1) 
  {
    Blynk.notify("ArduFarmBot2: Warning ==>> Lamp ON");
    digitalWrite(LAMP_PIN, LOW); // To be used with Relay module (inverted logic: activate with LOW)
    if (!turnOffOLED) displayData();
    LAMPs.on();
    LAMPa.on();
  }
  else
      {
        digitalWrite(LAMP_PIN, HIGH); // To be used with Relay module (inverted logic: normally HIGH)
        if (!turnOffOLED) displayData();
        LAMPs.off();
        LAMPa.off();
      }
  if (lightStatus == 1) 
  {
    Blynk.notify("ArduFarmBot2: Warning ==>> Light ON");
    digitalWrite(LIGHT_PIN, LOW); // To be used with Relay module (inverted logic: activate with LOW)
    if (!turnOffOLED) displayData();
    LIGHTa.on();
  }
  else
      {
        digitalWrite(LIGHT_PIN, HIGH); // To be used with Relay module (inverted logic: normally HIGH)
        if (!turnOffOLED) displayData();
        LIGHTa.off();
      }
}

/***************************************************
* Automatically Control the Plantation based on sensors reading
****************************************************/
void autoControlPlantation(void)
{ 
  if (soilMoister < DRY_SOIL) 
  {
    turnPumpOn();
  }

  if (airTemp < COLD_TEMP) 
  {
    turnLampOn();
  }
}

/***************************************************
* Turn Pump On for a certain amount of time
****************************************************/
void turnPumpOn()
{
  pumpStatus = 1;
  aplyCmd();
  delay (TIME_PUMP_ON*1000);
  pumpStatus = 0;
  aplyCmd();
}

/***************************************************
* Turn Lamp On for a certain amount of time 
****************************************************/
void turnLampOn()
{
  lampStatus = 1;
  aplyCmd();
  delay (TIME_LAMP_ON*1000);
  lampStatus = 0;
  aplyCmd();
}

/***************************************************
 * Send data to Blynk
 **************************************************/
void sendUptime()
{
  Blynk.virtualWrite(10, airTemp); //virtual pin V10
  Blynk.virtualWrite(11, airHum); // virtual pin V11
  Blynk.virtualWrite(12, soilMoister); // virtual pin V12
  Blynk.virtualWrite(13, soilTemp); //virtual pin V13
}
