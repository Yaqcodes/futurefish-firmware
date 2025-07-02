#include "helpers.h"
#include "webpage.h"
#include <Arduino.h>

// NTP Server for getting time
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// Global Variables
const int publishInterval = 10000; // publish every 10 seconds
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// Global variables to store sensor readings
float g_temperature = 0.0;
float g_do_level = 0.0;
float g_ph_value = 0.0;
float g_ammonia_level = 0.0;
float g_turbidity = 0.0;
int g_water_level = 0;
int g_feed_level = 0;
int g_battery_level = 0;

// DEFINE DATA-HOLDING VARIABLES
const int steps_per_rev = 200;
const int rotSpeed = 200;
const int tankArea = 10925;            // tank CSA in cm
const int tankHeight = 91;             // height of tank in cm
const int tankHeightAccomodation = 8;  // distance between face of sensor and highet level mark in cm

// Stepper control variables
unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;
const unsigned long stepInterval = 5000;  // Microseconds between steps (controls speed)
volatile long stepper1_steps_remaining = 0;
volatile long stepper2_steps_remaining = 0;

// STEPPER CALIBRATION CONSTANTS - ADJUST THESE FOR CALIBRATION
const float feeder_grams_per_rev = 13.0;  // Example: 10 grams dispensed per revolution

// Pin definitions for feeders
const int fs1 = 18;
const int slp1 = 19;
const int dir1 = 16;
const int fs2 = 22;
const int slp2 = 23;
const int dir2 = 21;

// Global threshold variables
float temperature_min = 0.0;
float temperature_max = 100.0;
float dissolved_oxygen_min = 0.0;
float dissolved_oxygen_max = 20.0;
float ph_min = 0.0;
float ph_max = 14.0;
float ammonia_min = 0.0;
float ammonia_max = 10.0;
float turbidity_min = 0.0;
float turbidity_max = 1000.0;
float water_level_min = 0.0;
float water_level_max = 1000.0;
float feed_level_min = 0.0;
float feed_level_max = 1000.0;

// Function definitions (move all from AWS_WiFiManager.ino except setup/loop)
// ... existing code ... 

void setupTime() {
  Serial.println("Setting up NTP time sync");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  struct tm timeinfo;
  int retries = 0;
  while(!getLocalTime(&timeinfo) && retries < 5) {
    Serial.println("Failed to obtain time, retrying...");
    delay(1000);
    retries++;
  }
  
  if (getLocalTime(&timeinfo)) {
    Serial.println("NTP time synchronized");
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    Serial.println(timeStringBuff);
  } else {
    Serial.println("Could not synchronize time. Will continue without accurate time.");
  }
}

String getLocalTimeString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "unknown-time";
  }
  
  char timeString[30];
  strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(timeString);
}

void connectAWS() {
  setup_wifi();
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint
  client.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
 
  // Set the message handler callback
  client.setCallback(messageHandler);
 
  Serial.println("Connecting to AWS IoT...");
 
  // Connect with the AWS IoT MQTT broker
  // Use THINGNAME as client ID for AWS IoT
  // while (!client.connect(THINGNAME, NULL, NULL, NULL, 1, false, NULL, false)) 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(1000);
  }
 
  if (!client.connected()) {
    Serial.println("AWS IoT connection failed!");
    return;
  }
 
  // Subscribe to topics
  Serial.print("Subscribing to topic: ");
  Serial.println(AWS_IOT_SUBSCRIBE_TOPIC);
  
  if (client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC)) {
    Serial.println("Subscription to Sub successful");
  } else {
    Serial.println("Subscription failed");
  }
 
  Serial.println("AWS IoT Connected!");
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
 
  // Create a null-terminated string from the payload
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  // Try to parse JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Print formatted command details
  Serial.println("Command received");
  
  // Check for required fields
  if (!doc.containsKey("command_type")) {
    Serial.println("ERROR: Missing required field 'command_type'");
    return;
  }
  
  if (!doc.containsKey("command_id")) {
    Serial.println("ERROR: Missing required field 'command_id'");
    return;
  }
  
  if (!doc.containsKey("timestamp")) {
    Serial.println("ERROR: Missing required field 'timestamp'");
    return;
  }
  
  if (!doc.containsKey("payload")) {
    Serial.println("ERROR: Missing required field 'payload'");
    return;
  }
  
  // Extract command details
  const char* commandType = doc["command_type"];
  String commandId = doc["command_id"].as<String>();
  String timestamp = doc["timestamp"].as<String>();
  
  // Format timestamp from ISO 8601 (2025-05-07T17:13:04Z) to DD:MM:YYYY HH:MM:SS
  if (timestamp.length() >= 19) {
    String formattedTime = timestamp.substring(8, 10) + ":" + 
                    timestamp.substring(5, 7) + ":" + 
                    timestamp.substring(0, 4) + " " + 
                    timestamp.substring(11, 19);
    Serial.print("Timestamp: ");
    Serial.println(formattedTime);
  } else {
    Serial.println("ERROR: Invalid timestamp format");
    return;
  }
  
  Serial.print("Command Type: ");
  Serial.println(commandType);
  Serial.print("Message: ");
  serializeJson(doc["payload"], Serial);
  Serial.println();
  Serial.print("Command ID: ");
  Serial.println(commandId);
  
  // Handle different command types
  if (strcmp(commandType, "feed") == 0) {
    // Check required fields for feed command
    if (!doc["payload"].containsKey("feed_amount")) {
      Serial.println("ERROR: Missing required field 'feed_amount' in payload");
      return;
    }
    
    if (!doc["payload"].containsKey("id")) {
      Serial.println("ERROR: Missing required field 'id' in payload");
      return;
    }
    
    int feederId = doc["payload"]["id"].as<int>();
    int feedAmount = doc["payload"]["feed_amount"];
    
    // Validate feeder ID
    if (feederId != 1 && feederId != 2) {
      Serial.print("ERROR: Invalid feeder ID: ");
      Serial.println(feederId);
      return;
    }
    
    // Actuate the specific feeder
    actuateFeeder(feederId, feedAmount);
  } 
  else if (strcmp(commandType, "water") == 0) {
    // Check required fields for water command
    if (!doc["payload"].containsKey("valve_state")) {
      Serial.println("ERROR: Missing required field 'valve_state' in payload");
      return;
    }
    
    if (!doc["payload"].containsKey("source")) {
      Serial.println("ERROR: Missing required field 'source' in payload");
      return;
    }
    
    if (!doc["payload"].containsKey("id")) {
      Serial.println("ERROR: Missing required field 'id' in payload");
      return;
    }
    
    bool valveState = doc["payload"]["valve_state"];
    String source = doc["payload"]["source"].as<String>();
    int valveId = doc["payload"]["id"].as<int>();
    
    // Validate source
    if (source != "inlet" && source != "outlet") {
      Serial.print("ERROR: Invalid valve source: ");
      Serial.println(source);
      return;
    }
    
    // Validate valve ID
    if (valveId != 1 && valveId != 2) {
      Serial.print("ERROR: Invalid valve ID: ");
      Serial.println(valveId);
      return;
    }
    
    // Actuate the specific valve
    if (source == "inlet") {
      if (valveId == 1) {
        actuateValve(INLET_VALVE_1_PIN, valveState);
      } else if (valveId == 2) {
        actuateValve(INLET_VALVE_2_PIN, valveState);
      }
    } else if (source == "outlet") {
      if (valveId == 1) {
        actuateValve(OUTLET_VALVE_1_PIN, valveState);
      } else if (valveId == 2) {
        actuateValve(OUTLET_VALVE_2_PIN, valveState);
      }
    }
  }
  else if (strcmp(commandType, "threshold") == 0) {
    // Check required fields for threshold command
    if (!doc["payload"].containsKey("parameter")) {
      Serial.println("ERROR: Missing required field 'parameter' in payload");
      return;
    }
    
    if (!doc["payload"].containsKey("min_value")) {
      Serial.println("ERROR: Missing required field 'min_value' in payload");
      return;
    }
    
    if (!doc["payload"].containsKey("max_value")) {
      Serial.println("ERROR: Missing required field 'max_value' in payload");
      return;
    }
    
    String parameter = doc["payload"]["parameter"].as<String>();
    float minValue = doc["payload"]["min_value"].as<float>();
    float maxValue = doc["payload"]["max_value"].as<float>();
    
    // Validate min and max values
    if (minValue >= maxValue) {
      Serial.println("ERROR: min_value must be less than max_value");
      return;
    }
    
    // Set thresholds based on parameter
    if (parameter == "temperature") {
      temperature_min = minValue;
      temperature_max = maxValue;
      Serial.print("Temperature thresholds set to min: ");
      Serial.print(temperature_min);
      Serial.print(", max: ");
      Serial.println(temperature_max);
    }
    else if (parameter == "dissolved_oxygen") {
      dissolved_oxygen_min = minValue;
      dissolved_oxygen_max = maxValue;
      Serial.print("Dissolved oxygen thresholds set to min: ");
      Serial.print(dissolved_oxygen_min);
      Serial.print(", max: ");
      Serial.println(dissolved_oxygen_max);
    }
    else if (parameter == "ph") {
      ph_min = minValue;
      ph_max = maxValue;
      Serial.print("pH thresholds set to min: ");
      Serial.print(ph_min);
      Serial.print(", max: ");
      Serial.println(ph_max);
    }
    else if (parameter == "ammonia") {
      ammonia_min = minValue;
      ammonia_max = maxValue;
      Serial.print("Ammonia thresholds set to min: ");
      Serial.print(ammonia_min);
      Serial.print(", max: ");
      Serial.println(ammonia_max);
    }
    else if (parameter == "turbidity") {
      turbidity_min = minValue;
      turbidity_max = maxValue;
      Serial.print("Turbidity thresholds set to min: ");
      Serial.print(turbidity_min);
      Serial.print(", max: ");
      Serial.println(turbidity_max);
    }
    else if (parameter == "water_level") {
      water_level_min = minValue;
      water_level_max = maxValue;
      Serial.print("Water level thresholds set to min: ");
      Serial.print(water_level_min);
      Serial.print(", max: ");
      Serial.println(water_level_max);
    }
    else {
      Serial.print("ERROR: Unknown parameter: ");
      Serial.println(parameter);
      return;
    }
  }
  else if (strcmp(commandType, "reboot") == 0) {
    Serial.println("Executing reboot command...");
    ESP.restart();
  }
  else {
    Serial.print("ERROR: Unrecognized command type: ");
    Serial.println(commandType);
    return;
  }
  
  // Send acknowledgment with command ID
  sendCommandAck(commandId);
}

// Function to actuate the specified feeder
void actuateFeeder(int feederId, int grams) {
  // Convert grams to revolutions

  float revs = grams/feeder_grams_per_rev;

  // Convert revolutions to steps
  long steps = long(revs * steps_per_rev);

  Serial.print("Actuating feeder ");
  Serial.print(feederId);
  Serial.print(" to dispense ");
  Serial.print(grams);
  Serial.print(" grams of feed in ");
  Serial.print(revs);
  Serial.println(" revolutions");

  // Select the appropriate feeder based on the ID
  if (feederId == 1) {
    // Wake up the A4988 driver by setting the sleep pin HIGH
    digitalWrite(slp1, HIGH);
    delay(5);  // Small delay for driver to wake up

    // Set direction for feeder1
    digitalWrite(dir1, HIGH);  // Set direction based on your motor wiring
    delay(1);                  // Small delay after changing direction

    // Set steps remaining to trigger the movement
    stepper1_steps_remaining = abs(steps);
  } 
  else if (feederId == 2) {
    // Wake up the A4988 driver by setting the sleep pin HIGH
    digitalWrite(slp2, HIGH);
    delay(5);  // Small delay for driver to wake up

    // Set direction for feeder2
    digitalWrite(dir2, HIGH);  // Set direction based on your motor wiring
    delay(1);                  // Small delay after changing direction

    // Set steps remaining to trigger the movement
    stepper2_steps_remaining = abs(steps);
  }
  else {
    Serial.print("Invalid feeder ID: ");
    Serial.println(feederId);
  }
}

// Function to step motors when needed - called frequently from loop()
void stepMotorIfNeeded() {
  unsigned long currentMicros = micros();

  // Handle Stepper 1
  if (stepper1_steps_remaining > 0) {
    if (currentMicros - lastStepTime1 >= stepInterval) {
      lastStepTime1 = currentMicros;

      // Generate a step pulse for motor 1
      digitalWrite(fs1, HIGH);
      delayMicroseconds(10);  // Minimum pulse width for A4988
      digitalWrite(fs1, LOW);

      stepper1_steps_remaining--;

      // If no more steps, put the driver to sleep
      if (stepper1_steps_remaining == 0) {
        digitalWrite(slp1, LOW);
      }
    }
  }

  // Handle Stepper 2
  if (stepper2_steps_remaining > 0) {
    if (currentMicros - lastStepTime2 >= stepInterval) {
      lastStepTime2 = currentMicros;

      // Generate a step pulse for motor 2
      digitalWrite(fs2, HIGH);
      delayMicroseconds(10);  // Minimum pulse width for A4988
      digitalWrite(fs2, LOW);

      stepper2_steps_remaining--;

      // If no more steps, put the driver to sleep
      if (stepper2_steps_remaining == 0) {
        digitalWrite(slp2, LOW);
      }
    }
  }
}

void actuateValve(int valve, bool state) {
  Serial.print("Setting valve on pin ");
  Serial.print(valve);
  Serial.print(" to ");
  Serial.println(state ? "OPEN" : "CLOSED");
  
  // Set the pin HIGH or LOW based on the state
  digitalWrite(valve, state ? HIGH : LOW);
}

// Function to check if a parameter is outside its threshold and publish alert
void checkThreshold(const char* parameter, float value, float min_threshold, float max_threshold) {
  // Skip invalid readings
  if (value < -100) {
    return;
  }
  
  // Check if value is outside threshold range
  bool isOutOfRange = false;
  const char* alertType = NULL;
  
  if (value < min_threshold) {
    isOutOfRange = true;
    alertType = "low";
  } 
  else if (value > max_threshold) {
    isOutOfRange = true;
    alertType = "high";
  }
  
  // If value is out of range, publish alert
  if (isOutOfRange) {
    // Create JSON document for the alert
    StaticJsonDocument<256> jsonDoc;
    
    // Add alert information
    jsonDoc["device_id"] = THINGNAME;
    jsonDoc["timestamp"] = getLocalTimeString();
    jsonDoc["parameter"] = parameter;
    jsonDoc["data"] = alertType;
    jsonDoc["value"] = value;
    
    // Serialize JSON to string
    char jsonBuffer[256];
    serializeJson(jsonDoc, jsonBuffer);
    
    // Publish alert
    Serial.print("Publishing threshold alert: ");
    Serial.println(jsonBuffer);
    
    if (client.publish(THRESHOLD_ALERT_TOPIC, jsonBuffer)) {
      Serial.println("Alert published successfully");
    } else {
      Serial.println("Failed to publish alert");
    }
    
    // Add a small delay to avoid flooding the MQTT broker
    delay(100);
  }
}

// Function to safely read all sensors with error and threshold checking
void readAllSensors() {
  // Use try-catch blocks for each sensor to prevent crashes
  try {
    g_temperature = readTemperature();
    Serial.print("Temperature read: ");
    Serial.println(g_temperature);
    checkThreshold("temperature", g_temperature, temperature_min, temperature_max);
  } catch(...) {
    Serial.println("Error reading temperature");
    g_temperature = -1.0;
  }
  
  delay(100); // Add delay between sensor reads
  
  try {
    g_do_level = readDO();
    Serial.print("DO read: ");
    Serial.println(g_do_level);
    checkThreshold("dissolved_oxygen", g_do_level, dissolved_oxygen_min, dissolved_oxygen_max);
  } catch(...) {
    Serial.println("Error reading DO");
    g_do_level = -1.0;
  }
  
  delay(100);
  
  try {
    g_ph_value = readpH();
    Serial.print("pH read: ");
    Serial.println(g_ph_value);
    checkThreshold("ph", g_ph_value, ph_min, ph_max);
  } catch(...) {
    Serial.println("Error reading pH");
    g_ph_value = -1.0;
  }
  
  delay(100);
  
  try {
    g_ammonia_level = readAmmonia();
    Serial.print("Ammonia read: ");
    Serial.println(g_ammonia_level);
    checkThreshold("ammonia", g_ammonia_level, ammonia_min, ammonia_max);
  } catch(...) {
    Serial.println("Error reading ammonia");
    g_ammonia_level = -1.0;
  }
  
  delay(100);
  
  try {
    g_turbidity = readTurbidity();
    Serial.print("Turbidity read: ");
    Serial.println(g_turbidity);
    checkThreshold("turbidity", g_turbidity, turbidity_min, turbidity_max);
  } catch(...) {
    Serial.println("Error reading turbidity");
    g_turbidity = -1.0;
  }
  
  delay(100);
  
  try {
    g_water_level = readWaterLevel();
    Serial.print("Water level read: ");
    Serial.println(g_water_level);
    checkThreshold("water_level", g_water_level, water_level_min, water_level_max);
  } catch(...) {
    Serial.println("Error reading water level");
    g_water_level = -1.0;
  }
  
  delay(100);
  
  try {
    g_feed_level = readFeedLevel();
    Serial.print("Feed level read: ");
    Serial.println(g_feed_level);
    checkThreshold("feed_level", g_feed_level, feed_level_min, feed_level_max);
  } catch(...) {
    Serial.println("Error reading feed level");
    g_feed_level = -1.0;
  }
  
  delay(100);
  
  try {
    g_battery_level = readBatteryLevel();
    Serial.print("Battery level read: ");
    Serial.println(g_battery_level);
    // You can add battery thresholds too if needed
  } catch(...) {
    Serial.println("Error reading battery level");
    g_battery_level = 0.0;
  }
}

float readTemperature() { return random(0, 40) / 1.0; }
float readDO() { return random(0, 10) / 1.0; }
float readpH() { return random(0, 14) / 1.0; }
float readTurbidity() { return random(0, 1000) / 1.0; }
float readAmmonia() { return random(0, 3) / 1.0; }
float readWaterLevel() { return random(0, 1000); }
float readFeedLevel() { return random(0, 1000); }
float readBatteryLevel() { return random(0, 100); }

void publishMessage() {
  // Read all sensor data first, completely separate from JSON creation
  readAllSensors();

  // Create JSON document for the message
  StaticJsonDocument<512> jsonDoc;
  
  // Add device information
  jsonDoc["device_id"] = THINGNAME;
  jsonDoc["timestamp"] = getLocalTimeString();
  
  // Add sensor data
  JsonObject data = jsonDoc.createNestedObject("data");
  data["temperature"] = g_temperature;
  data["dissolved_oxygen"] = g_do_level;
  data["ph"] = g_ph_value;
  data["ammonia"] = g_ammonia_level;
  // data["turbidity"] = g_turbidity;
  // data["water_level"] = g_water_level;
  // data["feed_level"] = g_feed_level;
  // data["battery"] = g_battery_level;
  
  // Add metadata
  JsonObject metadata = jsonDoc.createNestedObject("metadata");
  metadata["firmware_version"] = "1.0.0";
  metadata["hardware_version"] = "ESP32-WROOM-32";
  
  // Serialize JSON to string
  char jsonBuffer[1024];
  serializeJson(jsonDoc, jsonBuffer);
  
  // Publish message
  Serial.print("Publishing message: ");
  Serial.println(jsonBuffer);
  
  if (client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer)) {
    Serial.println("Message published successfully");
  } else {
    Serial.println("Failed to publish message");
  }
}

// Function to send acknowledgment back to AWS IoT
void sendCommandAck(String commandId) {
  StaticJsonDocument<200> ackDoc;
  
  ackDoc["device_id"] = THINGNAME;
  ackDoc["command_id"] = commandId;
  ackDoc["status"] = "success";
  ackDoc["timestamp"] = getLocalTimeString();
  
  char ackBuffer[256];
  serializeJson(ackDoc, ackBuffer);
  
  if (client.publish(ACK_TOPIC, ackBuffer)) {
    Serial.println("Command acknowledgment sent successfully");
  } else {
    Serial.println("Failed to send command acknowledgment");
  }
}

// WiFiManager Functions
void setup_wifi() {
  delay(10);
  Serial.println("Trying saved WiFi credentials...");

  WiFi.begin();  // use stored SSID/PASS
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < MAX_RETRIES) {
    delay(RETRY_DELAY_MS);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi!");
  } else {
    Serial.println("\nFailed to connect after retries. Starting AP mode...");
    startConfigPortal();
  }
}

void startConfigPortal() {
  WiFiManager wm;

  // Optional: custom timeout
  wm.setConfigPortalTimeout(0);  // Portal stays open forever

  wm.setConnectTimeout(10);

  // Uncomment to reset saved credentials every boot (for debugging)
  // wm.resetSettings();

  wm.setCustomHeadElement(CUSTOM_HTML_HEAD);  // load styles + logo

  // Attempt to auto-connect using saved credentials, otherwise start AP mode
  if (!wm.autoConnect("Future Fish Smart Pond", "smartfarmer100")) {
    Serial.println("Failed to connect or timeout. Restarting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Connected to WiFi via portal.");
}
