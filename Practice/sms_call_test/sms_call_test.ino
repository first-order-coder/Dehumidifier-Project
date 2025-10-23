#include <HardwareSerial.h>
HardwareSerial sim(2);                  // UART2 on ESP32-S3
const int RX2 = 16, TX2 = 17;           // SIM808 TX->ESP32 RX2(16), SIM808 RX<-ESP32 TX2(17)
const uint32_t SIM_BAUD = 9600;         // or 115200 if that's your module

// Simple line reader
String readLineFromSim(uint32_t timeout_ms = 3000) {
  String line; unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    while (sim.available()) {
      char c = (char)sim.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (line.length()) return line;
      } else {
        line += c;
      }
    }
  }
  return line;
}

void sendAT(const char* cmd, uint32_t wait=1500) {
  sim.print(cmd); sim.print("\r");
  unsigned long t0 = millis();
  while (millis() - t0 < wait) {
    if (sim.available()) Serial.write(sim.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  sim.begin(SIM_BAUD, SERIAL_8N1, RX2, TX2);
  delay(500);

  // Init modem for direct-delivery of SMS (+CMT:)
  sendAT("AT");
  sendAT("ATE0");
  sendAT("AT+CMGF=1");
  sendAT("AT+CSCS=\"GSM\"");
  sendAT("AT+IFC=0,0");
  sendAT("AT+CNMI=2,2,0,0,0");

  Serial.println("\nReady. Send an SMS with text PING to the SIM808 number.");
}

void loop() {
  // Parse unsolicited lines
  if (sim.available()) {
    String line = readLineFromSim(50);
    if (line.length() == 0) return;

    // Example header:
    // +CMT: "+3712xxxxxxx","","25/09/20,18:42:01+12"
    if (line.startsWith("+CMT:")) {
      // Next line is the message body
      String body = readLineFromSim(2000);
      Serial.print("Got SMS: "); Serial.println(body);

      // Mini task behavior
      if (body == "PING") {
        Serial.println("-> Got SMS: PING");
      }
    } else {
      // Print other unsolicited info
      Serial.println(line);
    }
  }
}
