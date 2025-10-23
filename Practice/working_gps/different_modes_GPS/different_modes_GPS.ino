#include <HardwareSerial.h>

HardwareSerial modem(2);                 // UART2
constexpr uint8_t PIN_RX = 16;           // SIM808 TX -> ESP32 RX
constexpr uint8_t PIN_TX = 17;           // SIM808 RX <- ESP32 TX

// Try these UART speeds to find the modem's
uint32_t BAUDS[] = {9600, 115200, 57600, 38400};

String readAll(uint32_t ms=1500){
  String s; unsigned long t0=millis();
  while (millis()-t0 < ms){
    while (modem.available()) s += (char)modem.read();
  }
  return s;
}
bool sendAT(const String& cmd, const char* expect="OK", uint32_t ms=2000, bool echo=true){
  if (echo) { Serial.print("> "); Serial.println(cmd); }
  modem.print(cmd); modem.print("\r");
  String resp = readAll(ms);
  if (echo) { Serial.print(resp); }
  return resp.indexOf(expect) >= 0;
}

// Simple CSV field after the colon, 1-based index
String csvField(const String& line, int idxAfterColon){
  int p=line.indexOf(':'); if (p<0) return "";
  int i=p+1; while (i<(int)line.length() && line[i]==' ') i++;
  int field=1, start=i;
  for (; i<= (int)line.length(); ++i){
    if (i==(int)line.length() || line[i]==','){
      if (field==idxAfterColon) return line.substring(start, i);
      field++; start=i+1;
    }
  }
  return "";
}

enum GnssMode { MODE_NONE, MODE_CGNS, MODE_CGPS };
GnssMode gnssMode = MODE_NONE;

bool autodetectBaud(){
  for (uint32_t b: BAUDS){
    modem.begin(b, SERIAL_8N1, PIN_RX, PIN_TX);
    delay(200);
    // wake a noisy modem, try a few ATs
    for (int k=0;k<3;k++){
      modem.print("AT\r");
      String r = readAll(400);
      if (r.indexOf("OK")>=0){
        Serial.print("Detected modem baud: "); Serial.println(b);
        return true;
      }
      delay(150);
    }
  }
  return false;
}

bool initCommon(){
  // Clean echo & verbose errors help
  sendAT("ATE0");
  sendAT("AT+CMEE=2");
  // Avoid continuous NMEA streaming over UART
  sendAT("AT+CGNSTST=0"); // Will just be ignored if CGNSTST not supported
  // Some firmwares need this to allow commands freely
  sendAT("AT+IFC=0,0");
  return true;
}

bool chooseGnssMode(){
  // Probe CGNS first
  if (sendAT("AT+CGNSPWR?", "OK", 1000)){
    // Try power on
    if (sendAT("AT+CGNSPWR=1")) { gnssMode = MODE_CGNS; return true; }
  }
  // Fallback: CGPS family
  if (sendAT("AT+CGPSPWR?", "OK", 1000) || true){
    if (sendAT("AT+CGPSPWR=1")) { gnssMode = MODE_CGPS; return true; }
  }
  return false;
}

void setup(){
  Serial.begin(115200);
  Serial.println("\nAuto-detecting modem baud...");
  if (!autodetectBaud()){
    Serial.println("Could not detect modem baud. Check TX/RX cross, GND, power.");
    return;
  }
  initCommon();

  Serial.println("Enabling GNSS...");
  if (!chooseGnssMode()){
    Serial.println("Failed to power GNSS (CGNS and CGPS both failed).");
    Serial.println("-> Check power (5V/2A+), antenna, or firmware support for GPS.");
    return;
  }
  Serial.print("GNSS mode: ");
  Serial.println(gnssMode==MODE_CGNS ? "CGNS" : "CGPS");
  Serial.println("Waiting for fix (cold start can take 30–120 s)...");
}

void loop(){
  static uint32_t last=0;
  if (millis()-last < 2000) return;
  last = millis();

  if (gnssMode==MODE_CGNS){
    modem.print("AT+CGNSINF\r");
    String all = readAll(1000);
    if (all.indexOf("+CGNSINF:")>=0){
      int ln = all.indexOf("+CGNSINF:");
      int end = all.indexOf('\n', ln);
      String line = all.substring(ln, end>ln?end:all.length());
      String pwr = csvField(line,1);
      String fix = csvField(line,2);
      String lat = csvField(line,5);
      String lon = csvField(line,6);
      if (pwr=="1" && fix=="1" && lat.length() && lon.length()){
        Serial.print("GPS FIX  Lat="); Serial.print(lat);
        Serial.print("  Lon="); Serial.println(lon);
        Serial.print("Maps: https://maps.google.com/?q=");
        Serial.print(lat); Serial.print(","); Serial.println(lon);
      } else {
        Serial.println("No fix yet...");
      }
    } else {
      Serial.print(all); // show whatever we got (OK/ERROR)
    }
  } else if (gnssMode==MODE_CGPS){
    // Ask status first
    sendAT("AT+CGPSSTATUS?", "OK", 800, false);
    // Get a snapshot (mode 0)
    modem.print("AT+CGPSINF=0\r");
    String all = readAll(1200);
    if (all.indexOf("+CGPSINF:")>=0){
      int ln = all.indexOf("+CGPSINF:");
      int end = all.indexOf('\n', ln);
      String line = all.substring(ln, end>ln?end:all.length());
      // +CGPSINF: 0,<lat>,<lon>,<alt>,<speed>,<course>,<UTCdate>,<UTCtime>,...
      String lat = csvField(line,2);
      String lon = csvField(line,3);
      if (lat.length() && lon.length() && lat != "0" && lon != "0"){
        Serial.print("GPS FIX  Lat="); Serial.print(lat);
        Serial.print("  Lon="); Serial.println(lon);
        Serial.print("Maps: https://maps.google.com/?q=");
        Serial.print(lat); Serial.print(","); Serial.println(lon);
      } else {
        Serial.println("No fix yet...");
      }
    } else {
      Serial.print(all);
    }
  }
}
