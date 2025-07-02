#include "helpers.h"

void setup() {
  Serial.begin(115200);
  delay(1000);  // allow time for serial

  WiFi.mode(WIFI_STA);  // Ensure we're in station mode
  WiFi.begin();         // Load credentials into memory
  delay(100);           // Small delay to give time to load SSID
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  pinMode(INLET_VALVE_1_PIN, OUTPUT);
  pinMode(OUTLET_VALVE_1_PIN, OUTPUT);
  pinMode(INLET_VALVE_2_PIN, OUTPUT);
  pinMode(OUTLET_VALVE_2_PIN, OUTPUT);
  
  // Initialize valves to closed
  digitalWrite(INLET_VALVE_1_PIN, LOW);
  digitalWrite(OUTLET_VALVE_1_PIN, LOW);
  digitalWrite(INLET_VALVE_2_PIN, LOW);
  digitalWrite(OUTLET_VALVE_2_PIN, LOW);

  delay(1000);
  Serial.println("Starting Smart Pond Firmware...\n");

  // Check if credentials exist at all
  if (!WiFi.SSID()) {
    startConfigPortal();
  }
  else {
    // Connect to AWS IoT
    connectAWS();
    // connectWithRetry();
  }

  // Setup NTP time synchronization first
  setupTime();
}

void loop() {
  // Step the motors if needed (frequently called)
  stepMotorIfNeeded();

  if (!client.connected()) {
    Serial.println("Lost connection to AWS IoT. Reconnecting...");
    connectAWS();
  }
  
  // Publish message every interval
  static unsigned long lastPublishTime = 0;
  unsigned long now = millis();
  
  if (now - lastPublishTime > publishInterval) {
    lastPublishTime = now;
    publishMessage();
  }

  client.loop();  // This is critical for MQTT message processing!
  delay(1000);
}
