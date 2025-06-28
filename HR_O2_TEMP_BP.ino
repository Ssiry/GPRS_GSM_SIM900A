#include <Arduino.h>
#include <FirebaseESP32.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Firebase.h>
#include <addons/RTDBHelper.h>

#define MODEM_RX 16  // SIM900A TX -> ESP32 RX
#define MODEM_TX 17  // SIM900A RX -> ESP32 TX
HardwareSerial sim900(2);   // UART2 for SIM900A

#define WIFI_SSID "Hassan Assiry's iPhone"
#define WIFI_PASSWORD "00000011"
#define DATABASE_URL "tset-esp32-default-rtdb.firebaseio.com"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

MAX30105 particleSensor;
#define BUFFER_SIZE 95

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int32_t spo2;
int32_t heartRate;
int8_t validSPO2, validHeartRate;
float temperature;
float systolicBP, diastolicBP;
long unblockedValue;
bool previousPresence = false;

int global_state;
int last_state = -1;  // Used to prevent repeated SMS

void WIFI_INIT() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nWiFi Connected");
}

void FIRESTORE_SETUP() {
  Serial.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);
  config.database_url = DATABASE_URL;
  config.signer.test_mode = true;
  Firebase.reconnectNetwork(true);
  fbdo.setBSSLBufferSize(4096, 1024);
  Firebase.begin(&config, &auth);
}

void MAX30102_INIT() {
  Wire.begin(4, 5);
  Serial.println("\nInitializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found! Check wiring.");
    while (1);
  }
}

void PRESENCE_SET_UP() {
  particleSensor.setup(0xFF, 4, 3, 400, 411, 4096);
  particleSensor.setPulseAmplitudeGreen(0);
  unblockedValue = 0;
  for (byte x = 0; x < 32; x++) {
    unblockedValue += particleSensor.getIR();
  }
  unblockedValue /= 32;
}

void GPRS_INIT() {
  sim900.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  Serial.println("Initializing SIM900A...");
  delay(2000);
}

void fun() {
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);
}

// ✅ Improved hand presence detection logic
bool Presence() {
  long currentIR = particleSensor.getIR();
  long currentDelta = currentIR - unblockedValue;

  if (currentDelta > (unblockedValue * 0.2)) {  // Dynamic threshold
    Serial.print(" 🫳🏻 Hand Accepted! ");
    Firebase.setBool(fbdo, "/Hand_on", true);
    return true;
  } else {
    Serial.print(" 🫸🏻 Hand Rejected! ");
    Firebase.setBool(fbdo, "/Hand_on", false);
    return false;
  }
}

void calculateBloodPressure(int heartRate, int spo2, float &systolicBP, float &diastolicBP) {
  if (heartRate > 30 && spo2 > 80) {
    systolicBP = map(heartRate, 50, 150, 90, 140);
    diastolicBP = map(heartRate, 50, 150, 60, 90);
  } else {
    systolicBP = 0;
    diastolicBP = 0;
  }
}

// ✅ Prevent repeated SMS
// void evaluateHealthState(int hr, int spo2, float temp) {
//   int hrState, spo2State, tempState;

//   if (hr > 180 || hr < 40) hrState = 2;
//   else if (hr > 120 || hr < 50) hrState = 1;
//   else hrState = 0;

//   if (spo2 < 85) spo2State = 2;
//   else if (spo2 < 95) spo2State = 1;
//   else spo2State = 0;

//   if (temp > 40.0 || temp < 34.0) tempState = 2;
//   else if (temp > 37.5 || temp < 35.0) tempState = 1;
//   else tempState = 0;

//   String state = String(hrState) + String(spo2State) + String(tempState);

//   if (state == "000") {
//     Serial.println("Normal");
//     global_state = 0;
//   } else if (state == "100" || state == "010" || state == "001") {
//     global_state = 1;
//     Serial.println("Slight concern");
//   } else if (state == "110" || state == "101" || state == "011") {
//     global_state = 2;
//     if (last_state != global_state) sendSMS("+201066009672", "Abnormal");
//     Serial.println("Abnormal");
//   } else if (state == "111") {
//     global_state = 3;
//     if (last_state != global_state) sendSMS("+201066009672", "Critical");
//     Serial.println("Critical");
//   } else if (state.indexOf('2') >= 0) {
//     global_state = 4;
//     if (last_state != global_state) sendSMS("+201066009672", "Emergency state, need immediate help");
//     Serial.println("Dangerous");
//   } else {
//     global_state = 0;
//     Serial.println("Unknown state");
//   }

//   last_state = global_state;  // Update last sent state
//   Firebase.setString(fbdo, "/test/State", state);
// }

void evaluateHealthState(int hr, int spo2, float temp) {
  int hrState, spo2State, tempState;

  // Heart Rate Evaluation
  if (hr > 180 || hr < 40) hrState = 2;
  else if (hr > 120 || hr < 50) hrState = 1;
  else hrState = 0;

  // SpO2 Evaluation
  if (spo2 < 85) spo2State = 2;
  else if (spo2 < 95) spo2State = 1;
  else spo2State = 0;

  // Temperature Evaluation
  if (temp > 40.0 || temp < 34.0) tempState = 2;
  else if (temp > 37.5 || temp < 35.0) tempState = 1;
  else tempState = 0;

  // Encode states into a 3-digit key
  String state = String(hrState) + String(spo2State) + String(tempState);

  Serial.print("State code: ");
  Serial.println(state);

  Firebase.setString(fbdo, "/test/State", state);  // Always update state in Firebase

  // Default global_state = 0 (normal)
  global_state = 0;

  if (state == "000") {
                      Firebase.setString(fbdo, "/test/State_Msg", "Normal");  // dont forget to make the location as global and separate var HERE OR IN getLoacation()
    Serial.println("✅ Normal");
  } 
  else if (state == "100" || state == "010" || state == "001") {
    global_state = 1;
                  Firebase.setString(fbdo, "/test/State_Msg", "Slight concern");  // dont forget to make the location as global and separate var HERE OR IN getLoacation()
    Serial.println("⚠️ Slight concern");
  } 
  else if (state == "110" || state == "101" || state == "011") {
    global_state = 2;
    Serial.println("❗ Abnormal - Help needed");
          Firebase.setString(fbdo, "/test/State_Msg", "Health Abnormality Detected! Please check the patient.");  // dont forget to make the location as global and separate var HERE OR IN getLoacation()
    sendSMS("+201066009672", "Health Abnormality Detected! Please check the patient.");
        makeCall();


  } 
  else if (state == "111") {
    global_state = 3;
    Serial.println("🚨 Critical - Immediate attention needed!");
              Firebase.setString(fbdo, "/test/State_Msg", "CRITICAL health warning! Act immediately.");  // dont forget to make the location as global and separate var HERE OR IN getLoacation()

    sendSMS("+201066009672", " CRITICAL health warning! Act immediately.");
        makeCall();

  } 
  else if (state.indexOf('2') >= 0) {
    global_state = 4;
    Serial.println("🆘 Emergency - Danger!");
                  Firebase.setString(fbdo, "/test/State_Msg", "Emergency state detected! Immediate help is needed.");  // dont forget to make the location as global and separate var HERE OR IN getLoacation()
    sendSMS("+201066009672", "Emergency state detected! Immediate help is needed.");
    makeCall();
  } 
  else {
    Serial.println("❓ Unknown state");
  }
}


void HR_SPO2_TEMP() {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    while (!particleSensor.check());
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    delay(10);
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  temperature = particleSensor.readTemperature();

  if (heartRate < 40 || heartRate > 130 || spo2 == 0) {
    return;
  }

  if (validHeartRate && validSPO2) {
    calculateBloodPressure(heartRate, spo2, systolicBP, diastolicBP);
  } else {
    systolicBP = 0;
    diastolicBP = 0;
  }

  evaluateHealthState(heartRate, spo2, temperature);

  Serial.print("Heart Rate: "); Serial.print(heartRate);
  Firebase.setInt(fbdo, "/test/HeartRate", heartRate);

  Serial.print(" bpm | SpO2: "); Serial.print(spo2);
  Firebase.setInt(fbdo, "/test/Spo2", spo2);

  Serial.print(" % | Temp: "); Serial.print(temperature); Serial.print(" °C | ");
  Firebase.setInt(fbdo, "/test/Temperature", temperature);

  Serial.print("Systolic BP: "); Serial.print(systolicBP); Serial.print(" mmHg | ");
  Firebase.setInt(fbdo, "/test/High_puls", systolicBP);

  Serial.print("Diastolic BP: "); Serial.print(diastolicBP); Serial.println(" mmHg");
  Firebase.setInt(fbdo, "/test/Low_puls", diastolicBP);
}

// String getLocationFromSIM900() {
//   sim900.println("AT+CLBS=1,1");
//   delay(5000); // Allow time for response

//   String response = "";
//   while (sim900.available()) {
//     char c = sim900.read();
//     response += c;
//   }

//   Serial.println("LBS Response: " + response);

//   int latIndex = response.indexOf(':');
//   if (latIndex != -1) {
//     int firstComma = response.indexOf(',', latIndex);
//     int secondComma = response.indexOf(',', firstComma + 1);
//     int thirdComma = response.indexOf(',', secondComma + 1);

//     String lat = response.substring(firstComma + 1, secondComma);
//     String lon = response.substring(secondComma + 1, thirdComma);

//     return "http://maps.google.com/?q=" + lat + "," + lon;
//   }

//   return "Location not available";
// }

String getLocationFromSIM900() {
  sim900.println("AT+CLBS=1,1");
  delay(5000); // Allow time for response

  String response = "";
  while (sim900.available()) {
    char c = sim900.read();
    response += c;
  }

  Serial.println("LBS Response: " + response);

  int colonIndex = response.indexOf(':');
  if (colonIndex != -1) {
    int firstComma = response.indexOf(',', colonIndex);
    int secondComma = response.indexOf(',', firstComma + 1);

    if (firstComma != -1 && secondComma != -1) {
      String lat = response.substring(firstComma + 1, secondComma);
      String lon = response.substring(secondComma + 1);

      lat.trim(); lon.trim(); // Clean any newlines or spaces

      return "http://maps.google.com/?q=" + lat + "," + lon;
    }
  }

  return "Location not available";
}


String sendAT(const char *cmd, unsigned long timeout = 2000) {
  sim900.flush();
  sim900.println(cmd);
  // Serial.print("Command: "); Serial.println(cmd);

  String response = "";
  unsigned long startTime = millis();

  while (millis() - startTime < timeout) {
    while (sim900.available()) {
      char c = sim900.read();
      response += c;
    }
  }

  // Serial.print("Response: ");
  // Serial.println(response);
  return response;
}

// void sendSMS(String phoneNumber, String message) {
//   sendAT("AT+CMGF=1");  // Set SMS text mode
//   delay(500);

//   sim900.print("AT+CMGS=\"");
//   sim900.print(phoneNumber);
//   sim900.println("\"");
//   delay(500);

//   sim900.println(message);
//   sim900.write(26);  // Ctrl+Z
//   delay(6000);

//   Serial.println("SMS Sent (or attempted)."); 
// }

void sendSMS(String phoneNumber, String message) {
  // Get location
  String location = getLocationFromSIM900();  // Call location function
  
  // Append location to message
  message += "\nLocation: " + location;

  sendAT("AT+CMGF=1");  // Set SMS text mode
  delay(500);

  sim900.print("AT+CMGS=\"");
  sim900.print(phoneNumber);
  sim900.println("\"");
  delay(500);

  sim900.println(message);
  sim900.write(26);  // Ctrl+Z to send
  delay(6000);       // Allow time to send

  Serial.println("SMS Sent (or attempted).");
}

void makeCall(){
  sim900.println("ATD+201066009672;"); // Replace with the actual phone number
delay(20000);  // Wait for 20 seconds (duration of call before hanging up)

sim900.println("ATH"); // Hang up the call
}


void setup() {
  Serial.begin(115200);
  MAX30102_INIT();
  // WIFI_INIT();  // Enable this for Firebase
  // FIRESTORE_SETUP();
  GPRS_INIT();
  PRESENCE_SET_UP();
  fun();
}

void loop() {
  bool isHandPresent = Presence();
  if (isHandPresent) {
    HR_SPO2_TEMP();
    Serial.println();
  }

  Serial.println(global_state);
  delay(3000);  // Reduce usage of Firebase/SIM
}
