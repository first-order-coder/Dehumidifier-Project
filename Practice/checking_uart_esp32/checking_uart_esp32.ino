#include <HardwareSerial.h>
HardwareSerial TestSerial(2); // Use UART2

void setup() {
  Serial.begin(115200);  // USB debug
  TestSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("Loopback test ready. Type something:");
}

void loop() {
  // Anything from PC → send to UART2
  if (Serial.available()) {
    char c = Serial.read();
    TestSerial.write(c);
  }

  // Anything from UART2 (looped back) → print to PC
  if (TestSerial.available()) {
    char c = TestSerial.read();
    Serial.write(c);
  }
}
