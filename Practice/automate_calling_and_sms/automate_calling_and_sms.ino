#include <HardwareSerial.h>
#define ESP32_TX 17
#define ESP32_RX 16

HardwareSerial sim808(1); //use uart 1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  sim808.begin(115200, SERIAL_8N1, ESP32_RX, ESP32_TX);
  
  delay(2000);
  sim808.println("AT"); // have to write to the sim not to the Serial monitor --> sim808.write("AT\r") this works.
  delay(2000);
  sim808.println("AT+CCID");

}

void loop() {
  // put your main code here, to run repeatedly:
  if (sim808.available() > 0){ //if there is anything available like "OK" then they go in to sim808.available() ESP32 forwards them with Serial.write()
    Serial.write(sim808.read()); //write to Serial monitor through ESP32 
  }

  if (Serial.available() > 0){ //when we type something on serial montior then they are sent to Serial.available() and then ESP32 forwards them raw with sim808.write()
    sim808.write(Serial.read()); // write to sim808 through ESP32
  }

}
