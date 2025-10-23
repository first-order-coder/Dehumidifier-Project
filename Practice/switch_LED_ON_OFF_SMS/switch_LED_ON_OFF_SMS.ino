#include <Arduino.h>

// ==== Wiring (ESP32-S3 Mini-1U <-> SIM808) ====
// SIM808 TX -> ESP32-S3 GPIO16 (MODEM_RX)
// SIM808 RX -> ESP32-S3 GPIO17 (MODEM_TX)
// GND <-> GND
#define MODEM_RX 16
#define MODEM_TX 17
HardwareSerial Modem(2);

// ==== LED ====
#define LED_PIN 2   // change to your LED GPIO

// ---- helpers ----
String readLine(uint32_t timeout_ms = 3000) { // to read one line of text from the modem (sim808) uart.
  String line;
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) { // keep the reciving sms under 3 seconds
    while (Modem.available()) {
      char c = (char)Modem.read();
      if (c == '\r') continue;
      if (c == '\n') { if (line.length()) return line; }
      else line += c;
    }
    delay(1);
  }
  return line; // may be empty if timeout
}

int parseIndexFromCMTI(const String& s) {
  int comma = s.lastIndexOf(',');
  return (comma >= 0) ? s.substring(comma + 1).toInt() : -1;
}

void modemInit() {
  Modem.println("AT");             readLine();
  Modem.println("ATE0");           readLine();     // no echo
  Modem.println("AT+CMEE=2");      readLine();     // verbose errors
  Modem.println("AT+CMGF=1");      readLine();     // text mode
  Modem.println("AT+CNMI=2,1,0,0,0"); readLine();  // push +CMTI when new SMS
  // Optional: clear storage once at boot
  // Modem.println("AT+CMGD=1,4"); readLine();
}

void handleBody(const String& bodyRaw) {
  String u = bodyRaw; u.trim();
  for (size_t i=0;i<u.length();++i) u[i] = toupper((unsigned char)u[i]);

  if (u.indexOf("LED ON") >= 0) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("[LED] ON");
  } else if (u.indexOf("LED OFF") >= 0) {
    digitalWrite(LED_PIN, LOW);
    Serial.println("[LED] OFF");
  } else {
    Serial.println("[INFO] Unknown command");
  }
}

void setup() {
  Serial.begin(115200);
  Modem.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n[Init] ESP32-S3 + SIM808 SMS LED controller");
  delay(2000);
  modemInit();
  Serial.println("[Ready] Send SMS: LED ON  /  LED OFF");
}

void loop() {
  if (!Modem.available()) return;

  String line = readLine();                 // unsolicited notifications
  if (line.length() == 0) return;
  Serial.println("[MODEM] " + line);

  if (line.startsWith("+CMTI:")) {
    int idx = parseIndexFromCMTI(line);
    if (idx < 0) return;

    // Read the SMS at that index
    Modem.print("AT+CMGR="); Modem.println(idx);

    // CMGR returns:
    //   +CMGR: "REC UNREAD","+xxx",,"yy/mm/dd,hh:mm:ss+tz"
    //   <message body>
    //   OK
    String header = readLine(5000);         // consume header
    String body   = readLine(5000);         // should be the message body

    // Guard: if timing gave us the header again, grab next non-empty line
    if (body.startsWith("+CMGR:") || body.length() == 0) {
      String maybe = readLine(5000);
      if (maybe.length()) body = maybe;
    }

    Serial.println("[BODY] " + body);
    handleBody(body);

    // Optional: delete SMS to keep storage clean
    Modem.print("AT+CMGD="); Modem.println(idx);
    readLine(2000); // consume OK
  }

  // (Basic version ignores +CMT: direct body push; +CMTI path is enough)
}
