#include <HardwareSerial.h>

#define MODEM_RX 16
#define MODEM_TX 17

HardwareSerial sim808(2);

String modemLine = ""; //text buffer to collect characters from the modem until we hit "end of line".
bool expectingSmsBody = false; // if we see +CMT then next line is the SMS text.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  sim808.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  Serial.println("Init SIMICOM for SMS RECEPTION...");

  sim808.println("AT+CMGF=1"); //put modem in to text mode
  delay(500);

  sim808.println("AT+CNMI=2,1,0,0,0");
  delay(500);

  Serial.println("Ready. Send and SMS");

}

void loop() {
  // put your main code here, to run repeatedly:
  while (sim808.available()){
    char c = sim808.read();

    if (c == '\n'){ // using \n to detect end-of-line // if the char is '\n' new line --> then 
      modemLine.trim(); // clean \r the carriage return and any whitespaces at the ends

      if (modemLine.startsWith("+CMT:")){ // if the line starts with +CMT: then that means a new SMS arrived the actaul message text will be on the next line
        expectingSmsBody = true; // the next line we read should be treates as the SMS body (text)
      } else if (expectingSmsBody) { // if the line did not start with +CMT: but we are expecting and SMS body (because we saw a +CMT: just before)...
        Serial.print("Got SMS: "); 
        Serial.println(modemLine);
        expectingSmsBody = false; // we handled the SMS body so clear the flag we no longer expect it
      }
      modemLine = "";
    } else if (c != '\r'){
      modemLine += c; // if the char is not newline '\n' and not carriage return '\r' append it.
    }
  }

}
