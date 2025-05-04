#define MODEM_RX 16  // Connect to SIM900A TX
#define MODEM_TX 17  // Connect to SIM900A RX

HardwareSerial sim900(2);  // Use UART2

void setup() {
  Serial.begin(115200);      // Debug Serial
  sim900.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);  // SIM900 Serial

  Serial.println("Initializing SIM900A...");
  delay(2000);

  // Send SMS
  sendSMS("+201008673758", "Hello from ESP32 via SIM900A!");
}

void loop() {
  // Nothing to do in the loop
}



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