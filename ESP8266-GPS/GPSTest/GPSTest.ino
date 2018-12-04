#include <Arduino.h>
// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

#include <ArduinoJson.h>


#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>

#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>
#include <AzureIoTUtility.h>

#include "config.h"

//#include <EasyButton.h>

#define greenButton 16
#define bButton 12
#define greenLED 14
#define blueLED 2



static bool messagePending = false;
static bool messageSending = true;

static char *connectionString;
static char *ssid;
static char *pass;

static int interval = INTERVAL;

float globalCalories = 0;

//EasyButton greenButton(gButton);
//EasyButton blueButton(bButton);

SoftwareSerial mySerial(15, 13);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial mySerial = Serial1;


Adafruit_GPS Adafruit_GPS(&mySerial);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

void onLED()
{
    digitalWrite(4, HIGH);
}

void offLED()
{
    digitalWrite(4, LOW);
}
void initWifi()
{
    // Attempt to connect to Wifi network:
    Serial.printf("Attempting to connect to SSID: %s.\r\n", ssid);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED)
    {
        // Get Mac Address and show it.
        // WiFi.macAddress(mac) save the mac address into a six length array, but the endian may be different. The huzzah board should
        // start from mac[0] to mac[5], but some other kinds of board run in the oppsite direction.
        uint8_t mac[6];
        WiFi.macAddress(mac);
        Serial.printf("You device with MAC address %02x:%02x:%02x:%02x:%02x:%02x connects to %s failed! Waiting 10 seconds to retry.\r\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], ssid);
        WiFi.begin(ssid, pass);
        delay(10000);
    }
    Serial.printf("Connected to wifi %s.\r\n", ssid);
}

void initTime()
{
    time_t epochTime;
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    while (true)
    {
        epochTime = time(NULL);

        if (epochTime == 0)
        {
            Serial.println("Fetching NTP epoch time failed! Waiting 2 seconds to retry.");
            delay(2000);
        }
        else
        {
            Serial.printf("Fetched NTP epoch time is: %lu.\r\n", epochTime);
            break;
        }
    }
}

static IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle;

void setup()  
{

  pinMode(5, OUTPUT); //GPS fix 
  pinMode(4, OUTPUT); //packets being sent to azure IoT hub

  digitalWrite(5,HIGH);

    initSerial();
    delay(2000);
    readCredentials();

    initWifi();
    initTime();
    
   pinMode(15, INPUT); //TX
   pinMode(13, OUTPUT); //RX
   
   //greenButton.begin();
   //blueButton.begin();
   pinMode(greenButton, INPUT);
   //pinMode(blueButton, INPUT);
   pinMode(greenLED, OUTPUT);
   pinMode(blueLED, OUTPUT);
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  Adafruit_GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  Adafruit_GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  Adafruit_GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  Adafruit_GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);

  iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol);
    if (iotHubClientHandle == NULL)
    {
        Serial.println("Failed on IoTHubClient_CreateFromConnectionString.");
        while (1);
    }

    IoTHubClient_LL_SetOption(iotHubClientHandle, "product_info", "HappyPath_AdafruitFeatherHuzzah-C");
    IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, receiveMessageCallback, NULL);
    IoTHubClient_LL_SetDeviceMethodCallback(iotHubClientHandle, deviceMethodCallback, NULL);
    IoTHubClient_LL_SetDeviceTwinCallback(iotHubClientHandle, twinCallback, NULL);
}

uint32_t timer = millis();

float readSpeed()
{
    return knotToVelocity(Adafruit_GPS.speed);
}

long readDay()
{
    uint8_t day = Adafruit_GPS.day;
    return (long)day;
}

uint8_t readMonth()
{
    return Adafruit_GPS.month;
}

uint8_t readYear()
{
    return Adafruit_GPS.year;
}

float readLatitude()
{
    return Adafruit_GPS.latitudeDegrees;
}

float readLongitude()
{
    return Adafruit_GPS.longitudeDegrees;
}

float readAngle()
{
    return Adafruit_GPS.angle;
}

float readAltitude()
{
    return Adafruit_GPS.altitude;
}

int knotToVelocity(int knot)
{
  return knot/1.944;
}
int calcJoules(int velocity){
  //joules = 1/2mass(kg) * velocity^2(m/s)
  return (75/2) + velocity^2; //mass = 75
}

int calcCalories(int velocity)
{
  int calories = calcJoules(velocity)/4184;
  //globalCalories += calories;
  return calories;
}

static int messageCount = 1;
    
bool greenState = false;
int greenValue = 0;
int blueState = 0;


void loop()                     // run over and over again
{
    /*
    if(blueButton.isPressed()){
      
      digitalWrite(blueLED,blueState);
      blueState = !blueState;
      Serial.println("blue pressed");
    }
      */
      /*
    greenValue = digitalRead(greenButton);
    if(greenValue == HIGH){
      //greenState = HIGH;
      digitalWrite(greenLED, LOW);
      Serial.println("green pressed");
    }
    else{
      digitalWrite(greenLED, HIGH);
    }
    */
    // read data from the GPS in the 'main loop'
    char c = Adafruit_GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  
  if (Adafruit_GPS.newNMEAreceived()) {

  
    if (!Adafruit_GPS.parse(Adafruit_GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    /*
    Serial.print("\nTime: ");
    Serial.print(Adafruit_GPS.hour); Serial.print(':');
    Serial.print(Adafruit_GPS.minute); Serial.print(':');
    Serial.print(Adafruit_GPS.seconds); Serial.print('.');
    Serial.println(Adafruit_GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(Adafruit_GPS.day); Serial.print('/');
    Serial.print(Adafruit_GPS.month); Serial.print("/20");
    Serial.println(Adafruit_GPS.year);
    Serial.print("Fix: "); Serial.print((int)Adafruit_GPS.fix);
    Serial.print(" quality: "); Serial.println((int)Adafruit_GPS.fixquality); 
    if (Adafruit_GPS.fix) {
      Serial.print("Location: ");
      Serial.print(Adafruit_GPS.latitude, 4); Serial.print(Adafruit_GPS.lat);
      Serial.print(", "); 
      Serial.print(Adafruit_GPS.longitude, 4); Serial.println(Adafruit_GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(Adafruit_GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(Adafruit_GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(Adafruit_GPS.speed);
      Serial.print("Angle: "); Serial.println(Adafruit_GPS.angle);
      Serial.print("Altitude: "); Serial.println(Adafruit_GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)Adafruit_GPS.satellites);
      */
      Serial.print("\nTime: ");
      Serial.print(Adafruit_GPS.hour); Serial.print(':');
      Serial.print(Adafruit_GPS.minute); Serial.print(':');
      Serial.print(Adafruit_GPS.seconds); Serial.print('.');
      float velocity = readSpeed();
      globalCalories += calcCalories(velocity);
      long day = readDay();
      uint8_t month = readMonth();
      uint8_t year = readYear();
      float latitude = readLatitude();
      float longitude = readLongitude();
      float angle = readAngle();
      float altitude = readAltitude();
      bool fix = Adafruit_GPS.fix;
      Serial.println(fix);
      if(fix){
        digitalWrite(5, LOW);
        Serial.println("GOT A FIX");
      }
      else{
        digitalWrite(5,HIGH);
      }
      StaticJsonBuffer<MESSAGE_MAX_LEN> jsonBuffer;
      JsonObject &root = jsonBuffer.createObject();
      root["deviceId"] = DEVICE_ID;
      root["messageId"] = messageCount;
      root["fix"] = fix;
      bool temperatureAlert = false;
      if (std::isnan(velocity))
      {
          root["velocity"] = NULL;
      }
      else
      {
          root["velocity"] = velocity;
          if (velocity > TEMPERATURE_ALERT)
          {
              temperatureAlert = true;
          }
      }
      /*
      if (std::isnan(day))
      {
          root[day] = NULL;
      }
      else
      {
          root["day"] = day;  
      }
      /*
      if (std::isnan(month))
      {
          root[month] = NULL;
      }
      else
      {
          root["month"] = month;  
      }
      if (std::isnan(year))
      {
          root[year] = NULL;
      }
      else
      {
          root["year"] = year;  
      }
      */
      if (std::isnan(calories))
      {
          root["calories"] = NULL;
      }
      else
      {
          root["calories"] = calories;
      }
      if (std::isnan(latitude))
      {
          root["latitude"] = NULL;
      }
      else
      {
          root["latitude"] = latitude;
      }
      if (std::isnan(longitude))
      {
          root["longitude"] = NULL;
      }
      else
      {
          root["longitude"] = longitude;
      }
      if (std::isnan(angle))
      {
          root["angle"] = NULL;
      }
      else
      {
          root["angle"] = angle;
      }
      if (std::isnan(altitude))
      {
          root["altitude"] = NULL;
      }
      else
      {
          root["altitude"] = altitude;
      }
      //root.printTo(messagePayLoad, MESSAGE_MAX_LEN);

     if (!messagePending && messageSending)
    {
        char messagePayload[MESSAGE_MAX_LEN];
        //bool temperatureAlert = readMessage(messageCount, messagePayload, Adafruit_GPS);
        root.printTo(messagePayload, MESSAGE_MAX_LEN);
        sendMessage(iotHubClientHandle, messagePayload, temperatureAlert);
        messageCount++;
        delay(interval);
    }
    IoTHubClient_LL_DoWork(iotHubClientHandle);
    delay(10);
    //}
  }
}
