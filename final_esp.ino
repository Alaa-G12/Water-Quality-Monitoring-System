#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>

// Provide the token generation process info
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions
#include "addons/RTDBHelper.h"

// Wi-Fi credentials
#define WIFI_SSID "alaa"
#define WIFI_PASSWORD "00000000"

// Firebase credentials
#define API_KEY "AIzaSyDpHIIc7N5NRP6AGs88fxNI-6weSYKTpx4"
#define DATABASE_URL "https://nyera-603f0-default-rtdb.firebaseio.com/"
#define FIREBASE_PROJECT_ID "nyera-603f0"

// Firebase instances
FirebaseData fbdoTemp;
FirebaseData fbdoTurb;
FirebaseData fbdoTDS;
FirebaseData fbdoV;
FirebaseData fbdoData;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

// Temperature Sensor
const int tempSensorPin = 4;
OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

// Turbidity Sensor
int turbiditySensorPin = 34;
float turbidityVoltage;
float turbidityNTU;

// TDS Sensor Configuration

#define TdsSensorPin 35
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;       // current temperature for compensation

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}


void uploadDocument(float temperatureC, float turbidityNTU, float TDS) {
  // Create a unique ID for the document
  String documentId = String(millis()); // Use the current time in milliseconds as the document ID
  FirebaseJson json;

  // Set fields with correct Firestore types
  json.set("fields/temperature/doubleValue", temperatureC);
  json.set("fields/turbidity/doubleValue", turbidityNTU);
  json.set("fields/TDS/doubleValue", TDS);

  // Send the data to Firestore
  if (Firebase.Firestore.createDocument(&fbdoData, FIREBASE_PROJECT_ID, "", "water_quality/" + documentId, json.raw())) {
    Serial.println("New document created successfully!");
  } else {
    Serial.println("Error creating new document: " + fbdoTemp.errorReason());
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
  pinMode(turbiditySensorPin, INPUT);

  // Initialize sensors
  sensors.begin();

  Serial.println("Water Quality Monitoring System");

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
    Serial.print(".");
    delay(300);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to Wi-Fi.");
    while (true); // Halt execution if Wi-Fi fails
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Firebase configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Sign up
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase authentication successful.");
    signupOK = true;
  } else {
    Serial.printf("Firebase authentication failed: %s\n", config.signer.signupError.message.c_str());
  }

  // Set token status callback
  config.token_status_callback = tokenStatusCallback;

  if (signupOK) {
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
  }
}

void loop() {
  // Temperature Measurement
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);

  // Turbidity Measurement
  turbidityVoltage = (float)analogRead(turbiditySensorPin) * (3.3 / 4095);
  turbidityNTU = -1000.4 * (turbidityVoltage * turbidityVoltage) + 5742.3 * turbidityVoltage - 4352.9;

  // TDS Measurement
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) { // Every 40 ms
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      //convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

      //Serial.print("voltage:");
      Serial.println(tdsValue, 0);
      Serial.println(",");
      Serial.println(averageVoltage, 2);
      //Serial.print("V   ");
      //      Serial.print("TDS Value:");
      //      Serial.println("ppm");

      if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
        sendDataPrevMillis = millis();
        if (Firebase.RTDB.setInt(&fbdoTDS, "sensors/tds", tdsValue)) {
          Serial.println ("sent");
        }    else {
          Serial.println("FAILED");
          Serial.println("REASON: " + fbdoTDS.errorReason());
        }

        if (Firebase.RTDB.setInt(&fbdoV, "sensors/voltage", averageVoltage)) {
          Serial.println ("sent");
        }    else {
          Serial.println("FAILED");
          Serial.println("REASON: " + fbdoV.errorReason());
        }

        if (Firebase.RTDB.setFloat(&fbdoTemp, "water_quality/temperature", temperatureC)) {
          Serial.println("Temperature sent successfully.");
        } else {
          Serial.println("Temperature upload failed: " + fbdoTemp.errorReason());
        }

        if (Firebase.RTDB.setFloat(&fbdoTurb, "water_quality/turbidity", turbidityNTU)) {
          Serial.println("Turbidity sent successfully.");
        } else {
          Serial.println("Turbidity upload failed: " + fbdoTurb.errorReason());
        }

      }
    }
  }
}
