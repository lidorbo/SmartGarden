//
// Copyright 2015 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// FirebaseDemo_ESP8266 is a sample that demo the different functions
// of the FirebaseArduino API.

//#include <FirebaseArduino.h>
#include <ESP8266WiFi.h>
#include <MAX6675_Thermocouple.h>
#include <EEPROM.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <FirebaseESP8266.h>
#include <addons/RTDBHelper.h>

// Set these to run example
//#define FIREBASE_HOST "smartsmoker-46d1e.firebaseio.com"
//#define FIREBASE_AUTH "0Lcu3Surzz9WwP7kQ6vP0XMmk4zFgTApt09k82Bu"
#define DATABASE_URL "garden-1edc0-default-rtdb.firebaseio.com"

#define WIFI_SSID "LidorB_WIFI"
#define WIFI_PASSWORD "l0526713223b"

#define SO_PIN  D5
#define CS_PIN  D6
#define SCK_PIN D7

#define READINGS_NUMBER 50
#define DELAY_TIME 30
#define WET 140
#define DRY 570

const long utcOffsetInSeconds = 10800;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);



struct {
  char ssid[50] = "";
  char pass[50] = "";
} data;

bool isConnect = false;
const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
const int pinD2 = D2;
const int pinD3 = D3;
const int dhtPin = 12;
const int waterLevelPin = 2;   
const int relayPin = 14;
char realyState = 0;

/* 3. Define the Firebase Data object */
FirebaseData fbdo;

/* 4, Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* Define the FirebaseConfig data for config data */
FirebaseConfig config;

DHT dht(dhtPin, DHT11);

void setup() {
  Serial.begin(9600);
  
  connectToNetwork(WIFI_SSID, WIFI_PASSWORD);
                   
  timeClient.begin();
  timeClient.update();

  pinMode(pinD2, OUTPUT);
  pinMode(pinD3, OUTPUT);
  pinMode(analogInPin, INPUT);
  pinMode(relayPin, OUTPUT_OPEN_DRAIN);
  pinMode(waterLevelPin, INPUT);   

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.database_url = DATABASE_URL;
  config.signer.test_mode = true;

  Firebase.reconnectWiFi(true);
  Firebase.begin(&config, &auth);

  dht.begin();
}

int sensorValue1 = 0;  // value read from the pot
int sensorValue2 = 0;  // value read from the pot

//Always keeps the soil between those values
int wetSoilMaxPrecnt = 80; // We want to watering until we get this value, when we get it stop the pump
int wetSoilMinPrecnt = 35; // When we get this value or lower we must water the plantes until we get to max value.
int waterLevelOk = 0;
 
void loop() {

 if(WiFi.status() == WL_CONNECTED)
 {
   
  sensorValue1 = readAnalogSensor1();
  sensorValue1 = map(sensorValue1, WET, DRY, 100, 0);
 Firebase.RTDB.setInt(&fbdo, "moistureSensor1", sensorValue1);
  // print the readings in the Serial Monitor
  Serial.println("sensor 1 = ");
  Serial.println(sensorValue1);

  delay(100);
  sensorValue2 = readAnalogSensor2();//analogRead(analogInPin);
  sensorValue2 = map(sensorValue2, WET, DRY, 100, 0);
Firebase.RTDB.setInt(&fbdo, "moistureSensor2", sensorValue2);

  Serial.println("sensor 2 = ");
  Serial.println(sensorValue2);
  timeClient.update();

 /* Serial.print(daysOfTheWeek[timeClient.getDay()]);
  Serial.print(", ");
  Serial.print(timeClient.getHours());
  Serial.print(":");
  Serial.print(timeClient.getMinutes());
  Serial.print(":");
  Serial.println(timeClient.getSeconds());
  Serial.println("EPOCH " + timeClient.getFormattedTime());*/
 
  String time = String(timeClient.getHours()) + ":" + String(timeClient.getMinutes());

  Firebase.RTDB.setString(&fbdo, "time", time);

  Firebase.RTDB.setInt(&fbdo, "temperature", dht.readTemperature());
  Firebase.RTDB.setInt(&fbdo, "humidity", dht.readHumidity());

  waterLevelOk = digitalRead(waterLevelPin);   // read the input pin
  Serial.println("Water level " + String(waterLevelOk));
 // Firebase.setInt(fbdo, "waterLevelOk", waterLevelOk);

  // handle error
  /* if (Firebase.failed()) {
      Serial.print("setting /number failed:");
      Serial.println(Firebase.error());  
      return;
  }*/


	/*float t = dht.readTemperature();	// Read temperature
	float h = dht.readTemperature();		// Read humidity

	Serial.print("Temperature = ");
	Serial.print(t);
	Serial.print("°C | ");
	Serial.print((t*9.0)/5.0+32.0);	// Convert celsius to fahrenheit
	Serial.println("°F ");
	Serial.print("Humidity = ");
	Serial.print(h);
	Serial.println("% ");
	Serial.println("");*/

   int command = 0;
   Firebase.RTDB.getInt(&fbdo, "command", &command);
   Serial.println("From FB " + String(command));  

   if(command == 1 && realyState == HIGH)
   {
     digitalWrite(relayPin, LOW);
   	 realyState = LOW;

   }

   else if(command == 0 && realyState == LOW)
   {
      digitalWrite(relayPin, HIGH);
   	  realyState = HIGH;
   }



    delay(1000);
 }
  
 /* // get value 
  Serial.print("number: ");
  Serial.println(Firebase.getFloat("number"));
  delay(1000);*/
}

int readAnalogSensor1()
{
    digitalWrite(pinD2, LOW);
    digitalWrite(pinD3, HIGH);
    
    return analogRead(analogInPin);
}

int readAnalogSensor2()
{
    digitalWrite(pinD2, HIGH);
    digitalWrite(pinD3, LOW);
    
    return analogRead(analogInPin);
}

bool connectToNetwork(char* ssid, char*  password)
{
  WiFi.begin(ssid, password); //begin WiFi connection
  Serial.println("");
  int timeOut = 10000;
  unsigned long currentMillis = millis();  // Wait for connection

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() > (currentMillis + timeOut))
    {
      return false;
    }
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

