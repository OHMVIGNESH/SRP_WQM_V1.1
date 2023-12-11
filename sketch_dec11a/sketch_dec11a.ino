#include <ArduinoJson.h>
const float analogMin = 0.0;    // Minimum analog reading
const float analogMax = 1023.0; // Maximum analog reading
const float tdsMin = 0.0;       // Minimum TDS value for your specific setup
const float tdsMax = 100.0;     // Maximum TDS value for your specific setup

const float targetVoltage1 = 1.30;  // First target voltage for calibration
const float targetTDS1 = 32.0;     // Corresponding TDS value for the first target voltage

const float targetVoltage2 = 2.23;  // Second target voltage for calibration
const float targetTDS2 = 94.0;     // Corresponding TDS value for the second target voltage

void setup() {
  Serial.begin(9600);
}

// Custom map function for float values
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  // Read the analog input on pin A1:
  int sensorValue = analogRead(A1);

  // Convert analog reading to voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  float tdsValue;

  if (voltage <= targetVoltage1) {
    tdsValue = mapFloat(voltage, analogMin, targetVoltage1, tdsMin, targetTDS1);
  } else {
    tdsValue = mapFloat(voltage, targetVoltage1, analogMax, targetTDS1, tdsMax);
  }

StaticJsonDocument<200> jsonDoc; // Adjust the size according to your needs

  // Add data to the JSON document:
  jsonDoc["voltage"] = voltage;
  jsonDoc["tds"] = tdsValue;

  // Serialize the JSON document to a String:
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  // Print the JSON string:
  Serial.println(jsonString);

  delay(1000);  // Delay for better readability, adjust as needed
}
