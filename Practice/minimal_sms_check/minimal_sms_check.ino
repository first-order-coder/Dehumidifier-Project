#include <Arduino.h>
#include <HardwareSerial.h>

#define MODEM_RX 16
#define MODEM_TX 17
HardwareSerial Modem(2); //use UART 2

void setup() {
  Serial.begin(115200);
  Modem.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(2000); // small settle

  const char* NUMBER = "+37128799838";
  const char* TEXT   = "Finally it fucking works";

  // Modem.print("ATD");
  // Modem.print(NUMBER);
  // Modem.println(";");

  Modem.println("AT+CMGF=1");    delay(200);

  // Start SMS
  Modem.print("AT+CMGS=\""); 
  Modem.print(NUMBER); 
  Modem.println("\"");

  // Wait a moment for '>' prompt (keep it simple)
  delay(1000);

  // Body
  Modem.print(TEXT);
  delay(100);

  // Send Ctrl+Z (ASCII 26) to submit the SMS
  Modem.write((uint8_t)0x1A);
}

void loop() {
  // Forward SIM808 → PC
  if (Modem.available()) {
    Serial.write(Modem.read());
    // delay(2000);
  }   
  // Forward PC → SIM808
  if (Serial.available()) {
    Modem.write(Serial.read());
    // delay(2000);
  }
}

// to recieve --> AT+CNMI=2,2,0,0,0
// to send --> AT+CMGS="+371"
