#ifndef HELPERS_H
#define HELPERS_H

#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include "secrets.h"

// NTP Server for getting time
extern const char* ntpServer;
extern const long gmtOffset_sec;
extern const int daylightOffset_sec;

// Global Variables
extern const int publishInterval;
extern WiFiClientSecure net;
extern PubSubClient client;

// Global variables to store sensor readings
extern float g_temperature;
extern float g_do_level;
extern float g_ph_value;
extern float g_ammonia_level;
extern float g_turbidity;
extern int g_water_level;
extern int g_feed_level;
extern int g_battery_level;

// Stepper control variables
extern unsigned long lastStepTime1;
extern unsigned long lastStepTime2;
extern const unsigned long stepInterval;
extern volatile long stepper1_steps_remaining;
extern volatile long stepper2_steps_remaining;

// Stepper calibration
extern const float feeder_grams_per_rev;
extern const int steps_per_rev;
extern const int rotSpeed;
extern const int tankArea;
extern const int tankHeight;
extern const int tankHeightAccomodation;

// Pin definitions
#define ONE_WIRE_BUS1 15
#define ONE_WIRE_BUS2 4
#define INLET_VALVE_1_PIN 13
#define OUTLET_VALVE_1_PIN 14
#define INLET_VALVE_2_PIN 15
#define OUTLET_VALVE_2_PIN 17
extern const int fs1;
extern const int slp1;
extern const int dir1;
extern const int fs2;
extern const int slp2;
extern const int dir2;

// WiFiManager constants
#define MAX_RETRIES 5
#define RETRY_DELAY_MS 3000

// WiFiManager Prototypes
void startConfigPortal();
void connectWithRetry();

// Function prototypes for actuators
void actuateFeeder(int feederId, int grams);
void actuateValve(int valve, bool state);

// Sample sensor data (replace with actual sensor readings)
void readAllSensors();
float readTemperature();
float readDO();
float readpH();
float readTurbidity();
float readAmmonia();
float readWaterLevel();
float readFeedLevel();
float readBatteryLevel();

// Function prototypes
void setup_wifi();
void reconnect_wifi();
void connectAWS();
void publishMessage();
void messageHandler(char* topic, byte* payload, unsigned int length);
void setupTime();
String getLocalTimeString();
void stepMotorIfNeeded();
void checkThreshold(const char* parameter, float value, float min_threshold, float max_threshold);
void sendCommandAck(String commandId);

// Thresholds
extern float temperature_min;
extern float temperature_max;
extern float dissolved_oxygen_min;
extern float dissolved_oxygen_max;
extern float ph_min;
extern float ph_max;
extern float ammonia_min;
extern float ammonia_max;
extern float turbidity_min;
extern float turbidity_max;
extern float water_level_min;
extern float water_level_max;
extern float feed_level_min;
extern float feed_level_max;

#endif // HELPERS_H 