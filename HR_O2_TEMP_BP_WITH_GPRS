#include <Arduino.h>
#include <FirebaseESP32.h>// lib == Firebase ESP32 Client by Mobizt
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"  

#include <Firebase.h>
// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>


/* 1. Define the WiFi credentials */
#define WIFI_SSID "Aya"
#define WIFI_PASSWORD "1234567899"

/* 2. Define the RTDB URL */
#define DATABASE_URL "tset-esp32-default-rtdb.firebaseio.com" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app

/* 3. Define the Firebase Data object */
FirebaseData fbdo;

/* 4, Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* Define the FirebaseConfig data for config data */
FirebaseConfig config;

unsigned long dataMillis = 0;
int count = 0;

MAX30105 particleSensor;


#define BUFFER_SIZE 100  // Number of samples for averaging
uint32_t irBuffer[BUFFER_SIZE];  // Infrared LED sensor data
uint32_t redBuffer[BUFFER_SIZE]; // Red LED sensor data
int32_t spo2;       // Calculated SpO2 value (must be int32_t)
int32_t heartRate;  // Calculated Heart Rate (must be int32_t)
int8_t validSPO2, validHeartRate; // Must be int8_t
float temperature; // Temperature from MAX30102
float systolicBP, diastolicBP;

// Presence Detection
long unblockedValue; // Baseline IR value


void WIFI_INIT(){
  
  #if !defined(ARDUINO_UNOWIFIR4)
    WiFi.mode(WIFI_STA);
  #else
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
  #endif
  WiFi.disconnect();
  delay(1000);

  /* Connect to WiFi */
  Serial.println();
  Serial.println();
  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("-");
    delay(500);
  }

  Serial.println();
  Serial.println("WiFi Connected");
  Serial.println();

  #if defined(ARDUINO_UNOWIFIR4)
    digitalWrite(LED_BUILTIN, HIGH);
  #endif

}

void FIRESTORE_SETUP(){
  
    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the certificate file (optional) */
    // config.cert.file = "/cert.cer";
    // config.cert.file_storage = StorageType::FLASH;

    /* Assign the database URL(required) */
    config.database_url = DATABASE_URL;

    config.signer.test_mode = true;

    /**
     Set the database rules to allow public read and write.

       {
          "rules": {
              ".read": true,
              ".write": true
          }
        }

    */

    // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
    Firebase.reconnectNetwork(true);

    // Since v4.4.x, BearSSL engine was used, the SSL buffer need to be set.
    // Large data transmission may require larger RX buffer, otherwise connection issue or data read time out can be occurred.
    fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

    /* Initialize the library with the Firebase authen and config */
    Firebase.begin(&config, &auth);
}

// MAX30102 Initialization
void MAX30102_INIT() {
    Serial.begin(115200);
    Wire.begin(4, 5); 

    Serial.println("\n Initializing MAX30102...");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 not found! Check wiring.");

        while (1);
    }
}
// Presence Setup
void PRESENCE_SET_UP() {
    byte ledBrightness = 0xFF; 
    byte sampleAverage = 4;
    byte ledMode = 3;
    int sampleRate = 400;
    int pulseWidth = 411;
    int adcRange = 4096;

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    // particleSensor.setPulseAmplitudeRed(0);    
    particleSensor.setPulseAmplitudeGreen(0);  

    unblockedValue = 0;
    for (byte x = 0; x < 32; x++) {
        unblockedValue += particleSensor.getIR();
    }
    unblockedValue /= 32;
}

// Function To Init SIM900A
void GPRS_INIT(){
  Serial.begin(115200);      // Debug Serial
  sim900.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);  // SIM900 Serial

  Serial.println("Initializing SIM900A...");
  delay(2000);

  // Send SMS
  sendSMS("+201008673758", "Hello from ESP32 via SIM900A!");
} 


// heart-rate & Temp & blood Pressure
void fun(){
    // particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A); // Adjust LED brightness
    particleSensor.setPulseAmplitudeIR(0x0A);
}

void setup() {

    MAX30102_INIT();
    WIFI_INIT();
    FIRESTORE_SETUP();
    GPRS_INIT();

    PRESENCE_SET_UP();
    fun();
}

// **✅ Presence Detection**
void Presence() {
    long currentDelta = particleSensor.getIR() - unblockedValue;
    if (currentDelta > unblockedValue * 1.2) {  // Dynamic threshold (20% increase)
        Serial.print(" 🫳🏻 Hand Accepted! ");
        Firebase.setBool(fbdo, "/Hand_on", true);
    } else {
        Serial.print(" 🫸🏻 Hand Rejected! ");
        Firebase.setBool(fbdo, "/Hand_on", false);
    }
}
// ✅ Function to estimate Blood Pressure from Heart Rate and SpO2
void calculateBloodPressure(int heartRate, int spo2, float &systolicBP, float &diastolicBP) {
    if (heartRate > 30 && spo2 > 80) { // Avoid invalid readings
        systolicBP = map(heartRate, 50, 150, 90, 140);   // Scale HR to realistic BP range
        diastolicBP = map(heartRate, 50, 150, 60, 90);
    } else {
        systolicBP = 0;
        diastolicBP = 0;
    }
}

void HR_SPO2_TEMP() {
  
    // Read raw IR and Red LED values
    for (int i = 0; i < BUFFER_SIZE; i++) {
        while (!particleSensor.check());  // Wait for new data
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
    }

    // Calculate Heart Rate and SpO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Read temperature from MAX30102
    temperature = particleSensor.readTemperature();

    if(validHeartRate &&  validSPO2){
      calculateBloodPressure(heartRate, spo2, systolicBP, diastolicBP);
    }else{
        systolicBP = 0;
        diastolicBP = 0;
    }

    Serial.print("Heart Rate: "); Serial.print(validHeartRate ? heartRate : 0); 
    Firebase.setInt(fbdo, "/test/HeartRate", heartRate);
    Serial.print(" bpm | SpO2: ");  Serial.print(validSPO2 ? spo2 : 0);
    Firebase.setInt(fbdo, "/test/Spo2", spo2);

    Serial.print(" % | Temperature: "); Serial.print(temperature);  Serial.print(" °C | ");
    Firebase.setInt(fbdo, "/test/Temperature", temperature);


    Serial.print("Systolic BP: "); Serial.print(systolicBP); Serial.print(" mmHg | ");
    Firebase.setInt(fbdo, "/test/High_puls", systolicBP);

    Serial.print("Diastolic BP: "); Serial.print(diastolicBP); Serial.println(" mmHg");
    Firebase.setInt(fbdo, "/test/Low_puls", diastolicBP);

}




// Function To Init SIM900A


// Function to send AT commands to SIM900A
String sendAT(const char *cmd, unsigned long timeout = 2000) {
  sim900.flush();  // Clear the serial buffer before sending the command
  sim900.println(cmd);  // Send the AT command
  Serial.print("Command: ");
  Serial.println(cmd);

  String response = "";
  unsigned long startTime = millis();

  // Wait for response
  while (millis() - startTime < timeout) {
    while (sim900.available()) {
      char c = sim900.read();
      response += c;
    }
  }

  // Print out the response for debugging
  Serial.print("Response: ");
  Serial.println(response);
  return response;
}
// Function to send SMS
void sendSMS(String phoneNumber, String message) {
  // Set SMS mode to text
  sendAT("AT+CMGF=1");

  // Send the message
  sim900.print("AT+CMGS=\"");
  sim900.print(phoneNumber);
  sim900.println("\"");
  delay(100);

  sim900.println(message); // Send message text
  sim900.write(26);        // Send Ctrl+Z to indicate the end of the message
  delay(1000);

  Serial.println("SMS Sent!");
}




// **✅ Main Loop**
void loop() {
    Presence();
    HR_SPO2_TEMP();
    Serial.println();
}

