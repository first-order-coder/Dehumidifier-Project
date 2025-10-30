#include <Arduino.h>
#include <HardwareSerial.h>   // for SIM module UART
#include <EEPROM.h>
#include <Wire.h>             // I2C for RTC
#include <RTClib.h>           // DS3231 RTC (Adafruit RTClib)
#include <Preferences.h>
#include "esp_system.h"       // esp_restart()

/* ======= SIMPLE SMS CONFIG ======= */
#define USE_SIM808 1
#if USE_SIM808
  // UART for SIM module
  static const int  SIM_RX_PIN = 16;     // ESP32 RX  <- SIM TX
  static const int  SIM_TX_PIN = 17;     // ESP32 TX  -> SIM RX
  static const long SIM_BAUD   = 115200; // your module’s baud

  // Your phone number for ON alert
  static const char* const SMS_PHONE   = "+37128799838";
  static const char* const SMS_TEXT_ON = "Pump ON at";

  HardwareSerial& Modem = Serial1;

  // --- tiny helpers for modem text I/O ---
  bool readLine(String &out, uint32_t to=1500){
    out = "";
    uint32_t start = millis();
    while (millis() - start < to) {
      while (Modem.available()) {
        char c = (char)Modem.read();
        if (c == '\r') continue;
        if (c == '\n') { if (out.length()) return true; }
        else out += c;
      }
      delay(2);
    }
    return out.length();
  }
  bool waitFor(const char* expect, uint32_t to=3000){
    String line; uint32_t s=millis();
    while (millis()-s < to) {
      if (readLine(line, 600)) {
        if (line.indexOf(expect) >= 0) return true;
        if (line.indexOf("ERROR") >= 0 || line.indexOf("+CMS ERROR") >= 0) return false;
      }
    }
    return false;
  }
  bool cmd(const char* c, const char* expect="OK", uint32_t to=2000){
    Modem.println(c);
    return waitFor(expect, to);
  }
  void simSimpleInit(){
    for (int i=0;i<5;i++){ Modem.println("AT"); if (waitFor("OK", 800)) break; delay(200); }
    cmd("ATE0");
    cmd("AT+CMEE=0");
    cmd("AT+CMGF=1");
    cmd("AT+CSCS=\"GSM\"");
    cmd("AT+CPMS=\"SM\",\"SM\",\"SM\"");
    cmd("AT+CFUN=1", "OK", 3000);
    cmd("AT+COPS=0", "OK", 5000);
  }
  bool simSendOneSMS(const char* number, const char* body){
    Modem.print("AT+CMGS=\""); Modem.print(number); Modem.println("\"");
    if (!waitFor(">", 5000)) return false;
    Modem.print(body);
    Modem.write((uint8_t)0x1A);
    return waitFor("OK", 30000);
  }

  // Find the first UNREAD SMS index via CMGL
  bool simFindFirstUnreadIndex(int &outIndex){
    outIndex = -1;
    Modem.println("AT+CMGL=\"REC UNREAD\"");
    uint32_t t0 = millis();
    String line;
    while (millis() - t0 < 2500) {
      if (!readLine(line, 800)) break;
      // Example: +CMGL: 3,"REC UNREAD","+371...","",...
      if (line.startsWith("+CMGL:")) {
        int colon = line.indexOf(':');
        int comma = line.indexOf(',', colon+1);
        if (colon>=0 && comma>colon) {
          outIndex = line.substring(colon+1, comma).toInt();
          waitFor("OK", 1000);
          return (outIndex >= 0);
        }
      }
    }
    return false;
  }
  bool simReadSMS(int index, String &from, String &body){
    from=""; body="";
    char buf[24]; snprintf(buf, sizeof(buf), "AT+CMGR=%d", index);
    Modem.println(buf);
    uint32_t t0=millis();
    String line;
    bool gotHeader=false, gotBody=false;
    while (millis()-t0<3000) {
      if (!readLine(line, 1000)) break;
      if (line.startsWith("+CMGR:")) {
        int firstQuote=line.indexOf('"');
        if (firstQuote>=0){
          int nextQuote=line.indexOf('"', firstQuote+1);
          int numStart = line.indexOf('"', nextQuote+1);
          int numEnd   = line.indexOf('"', numStart+1);
          if (numStart>=0 && numEnd>numStart) { from = line.substring(numStart+1, numEnd); gotHeader=true; }
        }
        continue;
      }
      if (gotHeader && !gotBody) {
        if (line.length()>0) { body=line; gotBody=true; }
        continue;
      }
      if (line=="OK") return gotHeader;
    }
    return gotHeader;
  }
  void simDeleteSMS(int index){
    char buf[24]; snprintf(buf, sizeof(buf), "AT+CMGD=%d,0", index);
    cmd(buf);
  }

  // Send long (multi-line) text as multiple SMS parts (~150 chars each)
  void smsSendLong(const String& number, const String& text){
    const size_t MAX_SMS = 150; // conservative per part
    // split by lines greedily
    String part = "";
    int start = 0;
    while (start < text.length()){
      int nl = text.indexOf('\n', start);
      String line = (nl < 0) ? text.substring(start) : text.substring(start, nl+1);
      if (part.length() + line.length() > MAX_SMS && part.length() > 0) {
        simSendOneSMS(number.c_str(), part.c_str());
        delay(800);
        part = "";
      }
      if (line.length() > MAX_SMS) {
        // hard cut very long line
        int pos = 0;
        while (pos < line.length()){
          int take = min((int)MAX_SMS, (int)(line.length()-pos));
          simSendOneSMS(number.c_str(), line.substring(pos, pos+take).c_str());
          delay(800);
          pos += take;
        }
      } else {
        part += line;
      }
      if (nl < 0) break;
      start = nl+1;
    }
    if (part.length()) simSendOneSMS(number.c_str(), part.c_str());
  }
#endif

/* ======= BUILD-TIME SERIAL SWITCH ======= */
#ifndef useSerialTerminal
#define useSerialTerminal 1
#endif
#if useSerialTerminal
  #define SER_BEGIN(...)    Serial.begin(__VA_ARGS__)
  #define SER_PRINT(...)    Serial.print(__VA_ARGS__)
  #define SER_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define SER_AVAILABLE()   Serial.available()
  #define SER_READ()        Serial.read()
#else
  #define SER_BEGIN(...)    ((void)0)
  #define SER_PRINT(...)    ((void)0)
  #define SER_PRINTLN(...)  ((void)0)
  #define SER_AVAILABLE()   (0)
  #define SER_READ()        (0)
#endif

/* ======================= ESP32 PIN MAP ======================= */
const int FLOAT_TOP_PIN       = 7;
const int FLOAT_BOTTOM_PIN    = 33;

const int RELAY_PUMP_PIN      = 38;   // keep off SIM pins
const int RELAY_HUMID_PIN     = 15;
const bool RELAY_PUMP_ACTIVE_LOW  = false;
const bool RELAY_HUMID_ACTIVE_LOW = true;

const int SHUNT_ADC_PIN       = 4;

const int  FAULT_LED_PIN         = 5;
const bool FAULT_LED_ACTIVE_HIGH = true;

const int  VALVE_RELAY_PIN         = 12;
const bool VALVE_ACTIVE_LOW        = false;
const int  VALVE_LED_PIN           = 13;
const bool VALVE_LED_ACTIVE_HIGH   = true;
const uint32_t VALVE_OPEN_PULSE_MS = 30000UL;

bool     valveIsOpen      = false;
uint32_t valveOpenUntilMs = 0;

const int  MOTOR_LED_PIN         = 38;
const bool MOTOR_LED_ACTIVE_HIGH = true;

/* ======= HEARTBEAT ======= */
const int  ALIVE_LED_PIN           = 41;
const uint32_t BOOT_SOLID_MS       = 2000UL;
const uint32_t HEARTBEAT_PERIOD_MS = 10000UL;
const uint16_t HEARTBEAT_ON_MS     = 100;
uint32_t bootMs = 0;

/* ======= RTC ======= */
const int I2C_SDA = 8;
const int I2C_SCL = 9;
RTC_DS3231 rtc;
bool rtcOk = false;

/* ======= GUARDS / VERIFY ======= */
const uint16_t FLOAT_FILTER_MS = 150;
const uint32_t MIN_DWELL_MS    = 3000;
const uint32_t FAULT_BLANK_MS  = 700;

const uint16_t FLOW_BAD_mV    = 500;
const uint32_t FLOW_VERIFY_MS = 5000;
const uint8_t  ADC_SAMPLES    = 12;

/* ======= EEPROM ======= */
#define EEPROM_SIZE         1024
const int EE_ADDR_FAULT      = 0;
const uint8_t EE_FAULT_CLEAR = 0x00;
const uint8_t EE_FAULT_SET   = 0xF1;
const int EE_ADDR_TOTAL_SEC  = 1;
const int EE_ADDR_BOOT_COUNT = 10;
const int EE_ADDR_RTC_FLAG   = 20;
const uint8_t EE_RTC_SET     = 0xA5;

/* ======= SESSION (EEPROM) ======= */
const int EE_ADDR_SESSION_BASE = 200;
const uint32_t SESSION_MAGIC   = 0xC0FFEE51;
const uint8_t  MAX_SESSIONS    = 30;

struct SessionHeader { uint32_t magic; uint8_t head; uint8_t count; uint16_t rsv; };
struct SessionEntry  { uint32_t bootNo; uint32_t runSec; uint16_t starts; };
SessionHeader sessHdr;
uint8_t sessIndexCurrent = 0xFF;

/* ======= BUTTONS ======= */
const bool ENABLE_CLEAR_BTN = false;
const int  CLEAR_BTN_PIN    = 18;

const int  MANUAL_BTN_PIN        = 46;
const bool MANUAL_BTN_ACTIVE_LOW = true;
const uint32_t MANUAL_MAX_MS     = 1UL * 60UL * 1000UL;

const int  CAL_BTN_PIN        = 14;
const bool CAL_BTN_ACTIVE_LOW = true;

/* ======= CALIBRATION ======= */
const float    CAL_VOLUME_L      = 5.0f;
const uint32_t CAL_TIMEOUT_MS    = 5UL * 60UL * 1000UL;

/* ======= NVS calibration ======= */
Preferences prefs;
const char* PREF_NS  = "pumpcal";
const char* PREF_KEY = "flowLpm";
float       flow_L_per_min = 0.0f;
const float DEFAULT_FLOW_LPM     = 1.216216f;
const bool  SAVE_DEFAULT_TO_NVS  = true;

/* ======= DAY METRICS ======= */
uint32_t secondsToday = 0;
float    litersToday  = 0.0f;
uint32_t lastYMD_today = 0;

/* ======= STATE ======= */
bool pumpRunning  = false;
bool faultLatched = false;
uint32_t lastSwitchMs = 0;
uint32_t pumpOnAtMs   = 0;

/* ======= RUNTIME METRICS ======= */
uint32_t lastRunMs   = 0;
uint32_t totalRunSec = 0;
uint32_t bootCount   = 0;

/* ======= RUN HISTORY (RAM) with liters ======= */
struct RunLog {
  uint32_t start_ms;
  uint32_t dur_ms;
  char     reason;
  uint32_t start_epoch;   // UNIX seconds at pump ON (0 if unknown)
  float    liters;        // water removed in this run
};
const uint8_t MAX_RUN_LOGS = 40;
RunLog   logs[MAX_RUN_LOGS];
uint16_t logsCount = 0, logsHead = 0;
uint32_t totalOns = 0, totalOffs = 0;
char     pendingStopReason = 'U';
uint32_t currentRunStartEpoch = 0;

/* ======= FLOW VERIFY STATE ======= */
bool     flowCheckActive    = false;
uint32_t flowWindowStartMs  = 0;
uint32_t flowSum_mV         = 0;
uint16_t flowSamples        = 0;
uint16_t flowMin_mV         = 65535;

/* ======= SCHEDULE ======= */
const uint8_t SCHED_HOUR   = 6;
const uint8_t SCHED_MINUTE = 30;
uint32_t lastSchedYMD      = 0;
bool scheduledRunActive    = false;

/* ======= MODES ======= */
bool     manualDrainMode = false;
uint32_t manualStartMs   = 0;

enum CalState : uint8_t { CAL_IDLE=0, CAL_WAIT_TOP, CAL_TIMING };
CalState calState = CAL_IDLE;
uint32_t calStartMs = 0;

/* ======= SELF-RESET ======= */
const bool    ENABLE_DAILY_RESET = true;
const uint8_t RESET_AT_HOUR      = 3;
const uint8_t RESET_AT_MINUTE    = 0;
uint32_t      lastResetYMD       = 0;

const bool     ENABLE_INTERVAL_RESET = false;
const uint32_t RESET_INTERVAL_MS     = 24UL * 60UL * 60UL * 1000UL;
uint32_t       resetDueAt            = 0;

/* ======================= HELPERS ======================= */
inline void setMotorLED(bool on){
  if (MOTOR_LED_PIN < 0) return;
  digitalWrite(MOTOR_LED_PIN, MOTOR_LED_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                                    : (on ? LOW  : HIGH));
}
inline void eepromCommit() { EEPROM.commit(); }
inline void setFaultLED(bool on) {
  if (FAULT_LED_PIN < 0) return;
  digitalWrite(FAULT_LED_PIN, FAULT_LED_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                                    : (on ? LOW  : HIGH));
}
inline void setRelayLevel_raw(int pin, bool on, bool activeLow) {
  digitalWrite(pin, activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}
inline void setRelayLevel(int pin, bool on, bool activeLow) {
  setRelayLevel_raw(pin, on, activeLow);
}
inline void relayPumpOn()  { setRelayLevel(RELAY_PUMP_PIN,  true,  RELAY_PUMP_ACTIVE_LOW); }
inline void relayPumpOff() { setRelayLevel(RELAY_PUMP_PIN,  false, RELAY_PUMP_ACTIVE_LOW); }
inline void setHumid(bool on) { setRelayLevel(RELAY_HUMID_PIN, on, RELAY_HUMID_ACTIVE_LOW); }

inline void setValveLED(bool valveOpen) {
  if (VALVE_LED_PIN < 0) return;
  digitalWrite(VALVE_LED_PIN,
               VALVE_LED_ACTIVE_HIGH ? (valveOpen ? HIGH : LOW)
                                     : (valveOpen ? LOW  : HIGH));
}
inline void setValveCoil(bool energize) { setRelayLevel_raw(VALVE_RELAY_PIN, energize, VALVE_ACTIVE_LOW); }
inline void valveOpen()  { setValveCoil(true);  valveIsOpen = true;  setValveLED(true); }
inline void valveClose() { setValveCoil(false); valveIsOpen = false; setValveLED(false); }

struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p){
    pin=p; pinMode(pin, INPUT_PULLUP);
    stable=candidate=digitalRead(pin);
    tChange=millis();
  }
  int read(uint16_t hold_ms){
    int raw=digitalRead(pin);
    if (raw!=candidate){ candidate=raw; tChange=millis(); }
    if ((millis()-tChange)>=hold_ms && stable!=candidate) stable=candidate;
    return stable;
  }
};
Debounce topF, botF, manBtn, calBtn;

uint32_t readShunt_mV() {
  static bool init=false;
  if (!init) {
#if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
    analogSetPinAttenuation(SHUNT_ADC_PIN, ADC_11db);
#endif
    init = true;
  }
#if defined(ARDUINO_ARCH_ESP32)
  uint32_t acc = 0;
  for (uint8_t i=0;i<ADC_SAMPLES;i++){ acc += analogReadMilliVolts(SHUNT_ADC_PIN); delayMicroseconds(200); }
  return acc / ADC_SAMPLES;
#else
  return 0;
#endif
}

/* ======= EEPROM helpers ======= */
void saveFaultToEEPROM(bool set){
  uint8_t want = set ? EE_FAULT_SET : EE_FAULT_CLEAR;
  if (EEPROM.read(EE_ADDR_FAULT) != want) { EEPROM.write(EE_ADDR_FAULT, want); eepromCommit(); }
}
bool faultStoredInEEPROM(){ return EEPROM.read(EE_ADDR_FAULT) == EE_FAULT_SET; }
inline void saveTotalRunToEEPROM() {
  static uint32_t lastSaved = 0xFFFFFFFF;
  if (totalRunSec != lastSaved) { EEPROM.put(EE_ADDR_TOTAL_SEC, totalRunSec); eepromCommit(); lastSaved = totalRunSec; }
}
inline void loadBootCount()  { EEPROM.get(EE_ADDR_BOOT_COUNT, bootCount); }
inline void saveBootCount()  { EEPROM.put(EE_ADDR_BOOT_COUNT, bootCount); eepromCommit(); }

/* ======= RTC helpers ======= */
bool rtcWasEverSet() { return EEPROM.read(EE_ADDR_RTC_FLAG) == EE_RTC_SET; }
void markRtcSet()    { EEPROM.write(EE_ADDR_RTC_FLAG, EE_RTC_SET); eepromCommit(); }
bool rtcTimeLooksSane() {
  if (!rtcOk) return false;
  DateTime n = rtc.now();
  return (n.year() >= 2023);
}
bool parseAndSetRTC(const String& s) {
  int eq = s.indexOf('=');
  if (eq < 0 || eq + 1 + 19 > s.length()) return false;
  String t = s.substring(eq + 1).substring(0, 19);
  int Y=t.substring(0,4).toInt();
  int M=t.substring(5,7).toInt();
  int D=t.substring(8,10).toInt();
  int h=t.substring(11,13).toInt();
  int m=t.substring(14,16).toInt();
  int S=t.substring(17,19).toInt();
  if (Y<2023||M<1||M>12||D<1||D>31||h>23||m>59||S>59) return false;
  rtc.adjust(DateTime(Y,M,D,h,m,S));
  markRtcSet();
  return true;
}

/* ======= SESSION STORE ======= */
struct SessionStore {
  static int entryAddr(uint8_t idx) {
    return EE_ADDR_SESSION_BASE + sizeof(SessionHeader) + idx * sizeof(SessionEntry);
  }
  static void readHeader() {
    EEPROM.get(EE_ADDR_SESSION_BASE, sessHdr);
    if (sessHdr.magic != SESSION_MAGIC || sessHdr.head >= MAX_SESSIONS || sessHdr.count > MAX_SESSIONS) {
      sessHdr.magic = SESSION_MAGIC; sessHdr.head = 0; sessHdr.count = 0; sessHdr.rsv = 0;
      EEPROM.put(EE_ADDR_SESSION_BASE, sessHdr); eepromCommit();
    }
  }
  static void writeHeader() { EEPROM.put(EE_ADDR_SESSION_BASE, sessHdr); eepromCommit(); }
  static void writeAt(uint8_t idx, const SessionEntry &e) { EEPROM.put(entryAddr(idx), e); eepromCommit(); }
  static void readAt(uint8_t idx, SessionEntry &e) { EEPROM.get(entryAddr(idx), e); }
  static int findByBoot(uint32_t bootNo) {
    if (sessHdr.count == 0) return -1;
    uint8_t first = (sessHdr.head + MAX_SESSIONS - sessHdr.count) % MAX_SESSIONS;
    for (uint8_t i=0;i<sessHdr.count;i++){
      uint8_t idx = (first + i) % MAX_SESSIONS;
      SessionEntry e; readAt(idx, e);
      if (e.bootNo == bootNo) return idx;
    }
    return -1;
  }
  static uint8_t startNew(uint32_t bootNo) {
    SessionEntry e{bootNo, 0, 0};
    uint8_t idx = sessHdr.head;
    writeAt(idx, e);
    if (sessHdr.count < MAX_SESSIONS) sessHdr.count++;
    sessHdr.head = (sessHdr.head + 1) % MAX_SESSIONS;
    writeHeader();
    return idx;
  }
  static void addStarts(uint16_t delta=1) {
    if (sessIndexCurrent == 0xFF) return;
    SessionEntry e; readAt(sessIndexCurrent, e);
    e.starts += delta; writeAt(sessIndexCurrent, e);
  }
  static void addRunSec(uint32_t addSec) {
    if (sessIndexCurrent == 0xFF) return;
    SessionEntry e; readAt(sessIndexCurrent, e);
    e.runSec += addSec; writeAt(sessIndexCurrent, e);
  }
};

/* ======= Timestamped logging ======= */
void printStamp() {
  if (rtcOk) {
    DateTime n = rtc.now();
    char buf[24];
    snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u ",
             n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second());
    SER_PRINT(buf);
  } else {
    SER_PRINT(F("[no-RTC] "));
  }
}
void printlnTS(const __FlashStringHelper* s){ printStamp(); SER_PRINTLN(s); }
void printTS(const __FlashStringHelper* s){ printStamp(); SER_PRINT(s); }

/* ======= RUN HISTORY log helper ======= */
void logRun(uint32_t start_ms, uint32_t dur_ms, char reason, float liters) {
  logs[logsHead] = RunLog{ start_ms, dur_ms, reason, currentRunStartEpoch, liters };
  logsHead = (logsHead + 1) % MAX_RUN_LOGS;
  if (logsCount < MAX_RUN_LOGS) logsCount++;
  totalOffs++;
}

/* ======= NVS calibration helpers ======= */
void loadCalibration() {
  prefs.begin(PREF_NS, true);
  flow_L_per_min = prefs.getFloat(PREF_KEY, 0.0f);
  prefs.end();

  if (flow_L_per_min == 0.0f) {
    flow_L_per_min = DEFAULT_FLOW_LPM;
    SER_PRINT(F("Flow calibration missing; using DEFAULT (L/min): "));
    SER_PRINTLN(flow_L_per_min, 6);
    if (SAVE_DEFAULT_TO_NVS) {
      prefs.begin(PREF_NS, false);
      prefs.putFloat(PREF_KEY, flow_L_per_min);
      prefs.end();
      SER_PRINTLN(F("Default flow saved to NVS."));
    }
  } else {
    SER_PRINT(F("Flow calibration (L/min) from NVS: "));
    SER_PRINTLN(flow_L_per_min, 6);
  }
}
void saveCalibration(float lpm) {
  prefs.begin(PREF_NS, false);
  prefs.putFloat(PREF_KEY, lpm);
  prefs.end();
  flow_L_per_min = lpm;
  SER_PRINT(F("Flow calibration SAVED (L/min): "));
  SER_PRINTLN(lpm, 6);
}

/* ======= Build SMS strings ======= */
void buildPumpOnSMS(char* out, size_t outlen) {
  if (rtcOk) {
    DateTime n = rtc.now();
    snprintf(out, outlen, "%s %04u-%02u-%02u %02u:%02u:%02u",
             SMS_TEXT_ON,
             n.year(), n.month(), n.day(),
             n.hour(), n.minute(), n.second());
  } else {
    snprintf(out, outlen, "%s (no RTC) uptime %lus",
             SMS_TEXT_ON, (unsigned long)(millis()/1000UL));
  }
}

// --- Reason code -> short label (single word)
static const char* reasonLabel(char r) {
  switch (r) {
    case 'B': return "Bottom";     // stopped: bottom float high
    case 'T': return "Timer";      // scheduled/manual timeout
    case 'M': return "Manual";     // user manual stop
    case 'C': return "Calib";      // calibration stop
    case 'F': return "FAULT";      // fault/emergency
    case 'U': return "Other";      // unspecified/unknown
    default:  return "?";
  }
}

// --- Clean, SMS-friendly history (compact & aligned)
void buildHistoryCleanSMS(String& out, uint8_t show = 7) {
  out = "";
  const uint16_t have = logsCount;
  const uint16_t take = min<uint16_t>(show, have);

  // Header with today totals (if RTC known) and calibration
  if (rtcOk) {
    DateTime n = rtc.now();
    char hdr[96];
    snprintf(hdr, sizeof(hdr),
             "HISTORY  (%04u-%02u-%02u %02u:%02u)\n",
             n.year(), n.month(), n.day(), n.hour(), n.minute());
    out += hdr;
  } else {
    out += "HISTORY  (RTC?)\n";
  }

  char line[96];
  // Today counters + calibration
  snprintf(line, sizeof(line),
           "Today: %lus, %.3fL   Cal: %.3f L/min\n",
           (unsigned long)secondsToday, litersToday, flow_L_per_min);
  out += line;

  // Divider + columns
  out += "Last ";
  out += String(take);
  out += " runs\n";
  out += "No  Date       Time    Dur    Ltrs  Why\n";
  out += "-----------------------------------------\n";

  if (have == 0) {
    out += "(no runs yet)\n";
  } else {
    // print newest first
    uint16_t first = (logsHead + MAX_RUN_LOGS - logsCount) % MAX_RUN_LOGS;
    for (uint16_t printed = 0; printed < take; ++printed) {
      int i = (logsCount - 1) - printed;
      uint16_t idx = (first + i) % MAX_RUN_LOGS;
      const RunLog &L = logs[idx];

      char dstr[11]  = "----------";
      char tstr[9]   = "--------";
      if (rtcOk && L.start_epoch != 0) {
        DateTime t((uint32_t)L.start_epoch);
        snprintf(dstr, sizeof(dstr), "%04u-%02u-%02u", t.year(), t.month(), t.day());
        snprintf(tstr, sizeof(tstr), "%02u:%02u:%02u", t.hour(), t.minute(), t.second());
      } else {
        // fallback to uptime-based stamp
        unsigned long s = L.start_ms / 1000UL;
        snprintf(dstr, sizeof(dstr), "t+%8lus", s);
        strcpy(tstr, "   --  ");
      }

      // Duration (s) and liters, fixed width
      float dur_s   = L.dur_ms / 1000.0f;
      float liters  = L.liters;
      const char* why = reasonLabel(L.reason);

      snprintf(line, sizeof(line),
               "%-2u %-10s %-8s %6.1fs %6.2f %-6s\n",
               (unsigned)(printed + 1), dstr, tstr, dur_s, liters, why);
      out += line;
    }
  }

  // Tiny legend & tail
  out += "Why: Bottom/Timer/Manual/Calib/FAULT/Other\n";
  if (rtcOk) {
    DateTime n = rtc.now();
    snprintf(line, sizeof(line),
             "%04u-%02u-%02u %02u:%02u  END",
             n.year(), n.month(), n.day(), n.hour(), n.minute());
    out += line;
  } else {
    out += "END";
  }
}

/* ======= PUMP STATE APPLY (send SMS on ON; record liters on OFF) ======= */
void applyPumpState(bool on) {
  if (on == pumpRunning) return;
  if (millis() - lastSwitchMs < MIN_DWELL_MS) return;

  if (on) {
    relayPumpOn();
    pumpOnAtMs = millis();
    totalOns++;
    SessionStore::addStarts(1);

    if (rtcOk) currentRunStartEpoch = rtc.now().unixtime();
    else       currentRunStartEpoch = 0;

    valveClose();
    valveOpenUntilMs = 0;
    setMotorLED(true);

    flowCheckActive   = true;
    flowWindowStartMs = pumpOnAtMs + FAULT_BLANK_MS;
    flowSum_mV        = 0;
    flowSamples       = 0;
    flowMin_mV        = 65535;

    printlnTS(F("PUMP -> ON (latched) | Valve CLOSED | Starting 5 s flow verification after inrush."));

#if USE_SIM808
    char msg[96];
    buildPumpOnSMS(msg, sizeof(msg));
    bool ok = simSendOneSMS(SMS_PHONE, msg);
    printStamp(); SER_PRINTLN(ok ? F("SMS sent.") : F("SMS FAILED."));
#endif

  } else {
    if (pumpRunning) {
      uint32_t now = millis();
      lastRunMs = (now >= pumpOnAtMs) ? (now - pumpOnAtMs) : 0;
      uint32_t addSec = (lastRunMs + 999) / 1000;
      totalRunSec += addSec; saveTotalRunToEEPROM();

      float thisRunLiters = (flow_L_per_min / 60.0f) * addSec;
      secondsToday += addSec;
      litersToday  += thisRunLiters;

      SessionStore::addRunSec(addSec);

      logRun(pumpOnAtMs, lastRunMs, pendingStopReason, thisRunLiters);
      pendingStopReason='U';

      printStamp();
      SER_PRINT(F("PUMP -> OFF (latched). This run: "));
      SER_PRINT(lastRunMs / 1000.0, 3);
      SER_PRINT(F(" s, ~"));
      SER_PRINT(thisRunLiters, 3);
      SER_PRINT(F(" L  |  Lifetime: "));
      SER_PRINT(totalRunSec);
      SER_PRINTLN(F(" s"));
    }
    relayPumpOff();
    setMotorLED(false);

    if (VALVE_OPEN_PULSE_MS > 0) {
      valveOpen();
      valveOpenUntilMs = millis() + VALVE_OPEN_PULSE_MS;
      printlnTS(F("Valve OPEN (LED ON) for post-stop pulse."));
    } else {
      valveClose();
      valveOpenUntilMs = 0;
      printlnTS(F("Valve CLOSED (no post-stop pulse)."));
    }

    flowCheckActive = false;
  }
  pumpRunning = on;
  lastSwitchMs = millis();
}

/* ======= Handle incoming SMS commands (polling) ======= */
void processSmsCommands(){
#if USE_SIM808
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll < 4000) return; // poll every 4 s
  lastPoll = millis();

  int idx;
  if (!simFindFirstUnreadIndex(idx)) return;

  String from, body;
  if (!simReadSMS(idx, from, body)) { simDeleteSMS(idx); return; }

  body.trim();
  body.toUpperCase();

  if (body == "H") {
    String reply;
    buildHistoryCleanSMS(reply, 7);   // last 7 runs in a tidy layout
    smsSendLong(from, reply);
  }
  // add other commands here (e.g., 'R', 'T=...', etc.)

  simDeleteSMS(idx);  // avoid re-processing
#endif
}

/* ======= Misc helpers ======= */
bool checkClearButtonOnce(){
  if (!ENABLE_CLEAR_BTN) return false;
  static int last=HIGH; static uint32_t t=0;
  int v = digitalRead(CLEAR_BTN_PIN);
  if (v != last){ last=v; t=millis(); }
  if (last==LOW && millis()-t>20){ while(digitalRead(CLEAR_BTN_PIN)==LOW) delay(5); return true; }
  return false;
}
void printHMS(uint32_t secs) {
  uint32_t h = secs / 3600UL;
  uint32_t m = (secs / 60UL) % 60UL;
  uint32_t s = secs % 60UL;
  SER_PRINT(h); SER_PRINT(':');
  if (m<10) SER_PRINT('0'); SER_PRINT(m); SER_PRINT(':');
  if (s<10) SER_PRINT('0'); SER_PRINT(s);
}
uint32_t ymdFromDT(const DateTime& dt){
  return (uint32_t)dt.year()*10000UL + (uint32_t)dt.month()*100UL + (uint32_t)dt.day();
}
bool timeIsAtOrAfter(const DateTime& dt, uint8_t hh, uint8_t mm, uint8_t ss=0){
  if (dt.hour() > hh) return true;
  if (dt.hour() < hh) return false;
  if (dt.minute() > mm) return true;
  if (dt.minute() < mm) return false;
  return dt.second() >= ss;
}

/* ======= Manual & Calibration ======= */
void startManualDrain(){
  if (faultLatched) { printlnTS(F("Manual DRAIN ignored: fault latched.")); return; }
  if (calState != CAL_IDLE) { printlnTS(F("Manual DRAIN ignored: calibration in progress.")); return; }
  manualDrainMode = true; manualStartMs = millis();
  lastSwitchMs = millis() - MIN_DWELL_MS;
  pendingStopReason = 'M';
  applyPumpState(true);
  printlnTS(F("Manual DRAIN: started (force ON until bottom float HIGH or timeout)."));
}
void stopManualDrain(char reasonCode){
  manualDrainMode = false;
  pendingStopReason = reasonCode;
  applyPumpState(false);
  printStamp(); SER_PRINT(F("Manual DRAIN: stop (reason=")); SER_PRINT(reasonCode); SER_PRINTLN(F(")."));
}
void startCalibration() {
  if (faultLatched) { printlnTS(F("Calibration ignored: fault latched.")); return; }
  if (manualDrainMode) { printlnTS(F("Calibration ignored: manual drain active.")); return; }
  if (calState != CAL_IDLE) { printlnTS(F("Calibration already running.")); return; }
  int topNow = digitalRead(FLOAT_TOP_PIN);
  if (topNow != LOW) {
    calState = CAL_WAIT_TOP;
    printlnTS(F("Calibration: waiting for tank to reach TOP float (top->LOW)..."));
  } else {
    calState = CAL_TIMING;
    calStartMs = millis();
    lastSwitchMs = millis() - MIN_DWELL_MS;
    pendingStopReason = 'C';
    applyPumpState(true);
    printlnTS(F("Calibration: starting timing (TOP reached) -> draining to BOTTOM (bottom->HIGH)..."));
  }
}
void cancelCalibration(const __FlashStringHelper* why) {
  printlnTS(why);
  if (pumpRunning) { pendingStopReason='C'; applyPumpState(false); }
  calState = CAL_IDLE;
}
void completeCalibration(uint32_t elapsedMs) {
  if (elapsedMs < 1000UL) { cancelCalibration(F("Calibration aborted: elapsed time too short (<1 s).")); return; }
  float cal_seconds = elapsedMs / 1000.0f;
  float lpm = (CAL_VOLUME_L / cal_seconds) * 60.0f;
  printStamp(); SER_PRINT(F("Calibration: volume=")); SER_PRINT(CAL_VOLUME_L, 3);
  SER_PRINT(F(" L, time=")); SER_PRINT(cal_seconds, 3);
  SER_PRINT(F(" s  -> flow=")); SER_PRINT(lpm, 6); SER_PRINTLN(F(" L/min"));
  saveCalibration(lpm);
  calState = CAL_IDLE;
}

/* ======================= SETUP ======================= */
void setup() {
  SER_BEGIN(115200);
  delay(500);
  SER_PRINTLN(F("\n[BOOT] ESP32 starting… (115200 baud)"));

  Wire.begin(I2C_SDA, I2C_SCL);
  if (rtc.begin()) {
    if (rtc.lostPower()) {
      if (!rtcWasEverSet()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        markRtcSet();
        rtcOk = true;
        printlnTS(F("RTC had lost power -> initialized to compile time (first boot)."));
      } else {
        if (!rtcTimeLooksSane()) {
          rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
          printlnTS(F("RTC looked invalid -> reinitialized to compile time."));
        } else {
          DateTime keep = rtc.now();
          rtc.adjust(keep);
          printlnTS(F("RTC lostPower flag cleared without changing time."));
        }
        rtcOk = true;
      }
    } else {
      rtcOk = true;
      if (!rtcWasEverSet() && rtcTimeLooksSane()) markRtcSet();
      printlnTS(F("RTC ready."));
    }
  } else {
    rtcOk = false;
    printlnTS(F("RTC not found on I2C (check wiring)."));
  }

  bootMs = millis();

  if (ALIVE_LED_PIN >= 0) pinMode(ALIVE_LED_PIN, OUTPUT), digitalWrite(ALIVE_LED_PIN, HIGH);
  if (FAULT_LED_PIN >= 0) pinMode(FAULT_LED_PIN, OUTPUT), setFaultLED(false);
  if (VALVE_LED_PIN >= 0) pinMode(VALVE_LED_PIN, OUTPUT);
  pinMode(MOTOR_LED_PIN, OUTPUT); setMotorLED(false);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EE_ADDR_TOTAL_SEC, totalRunSec);
  loadBootCount();

  SessionStore::readHeader();
  bootCount++; saveBootCount();

  int exist = SessionStore::findByBoot(bootCount);
  if (exist >= 0) sessIndexCurrent = (uint8_t)exist;
  else            sessIndexCurrent = SessionStore::startNew(bootCount);

  printStamp(); SER_PRINT(F("Boot count: ")); SER_PRINTLN(bootCount);
  printStamp(); SER_PRINT(F("Lifetime pump ON time (EEPROM): ")); SER_PRINT(totalRunSec); SER_PRINTLN(F(" s"));

  topF.begin(FLOAT_TOP_PIN);
  botF.begin(FLOAT_BOTTOM_PIN);
  manBtn.begin(MANUAL_BTN_PIN);
  calBtn.begin(CAL_BTN_PIN);

  pinMode(RELAY_PUMP_PIN,  OUTPUT);
  pinMode(RELAY_HUMID_PIN, OUTPUT);
  pinMode(VALVE_RELAY_PIN, OUTPUT);

  valveClose();
  relayPumpOff();
  setHumid(true);
  setFaultLED(false);

  saveFaultToEEPROM(false);
  faultLatched = false;

  loadCalibration();

  if (rtcOk) lastYMD_today = ymdFromDT(rtc.now());
  if (rtcOk) lastResetYMD = ymdFromDT(rtc.now());
  if (ENABLE_INTERVAL_RESET) resetDueAt = millis() + RESET_INTERVAL_MS;

#if USE_SIM808
  Modem.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(600);
  simSimpleInit();
#endif

  printlnTS(F("\nPower-up: fault cleared."));
  printlnTS(F("Serial cmds: 'C' clear fault, 'R' report, 'H' history, 'X' clr hist, 'B' boot, 'Z' reset boot, 'S' sessions, 't' now, 'T=YYYY-MM-DD HH:MM:SS' set RTC, 'M' manual, 'K' calibrate, 'k' clr cal, 'f=1.23' set L/min, 'd' reset today"));

  delay(FLOAT_FILTER_MS);
  int bottomNow = botF.read(FLOAT_FILTER_MS);
  if (!faultLatched && bottomNow == LOW) {
    printlnTS(F("Startup drain: water detected. Pump ON until bottom float goes HIGH."));
    lastSwitchMs = millis() - MIN_DWELL_MS;
    applyPumpState(true);
  } else {
    printlnTS(F("Startup drain skipped: no water or fault latched."));
  }
}

/* ======================= LOOP ======================= */
void loop() {
  // Heartbeat
  if (ALIVE_LED_PIN >= 0) {
    uint32_t t = millis() - bootMs;
    if (t <= BOOT_SOLID_MS) digitalWrite(ALIVE_LED_PIN, HIGH);
    else {
      uint32_t phase = (t - BOOT_SOLID_MS) % HEARTBEAT_PERIOD_MS;
      digitalWrite(ALIVE_LED_PIN, (phase < HEARTBEAT_ON_MS) ? HIGH : LOW);
    }
  }

  // Daily rollover
  if (rtcOk) {
    uint32_t ymdNow = ymdFromDT(rtc.now());
    if (ymdNow != lastYMD_today) {
      secondsToday = 0;
      litersToday  = 0.0f;
      lastYMD_today = ymdNow;
      printlnTS(F("New day -> Today counters reset."));
    }
  }

  // Self-reset windows
  if (ENABLE_DAILY_RESET && rtcOk) {
    DateTime nowdt = rtc.now();
    uint32_t ymd   = ymdFromDT(nowdt);
    bool idle = !pumpRunning && !manualDrainMode && (calState == CAL_IDLE);
    if (ymd != lastResetYMD && timeIsAtOrAfter(nowdt, RESET_AT_HOUR, RESET_AT_MINUTE, 0) && idle) {
      printlnTS(F("Daily self-reset: conditions OK -> restarting MCU..."));
      delay(200);
      esp_restart();
    }
  }
  if (ENABLE_INTERVAL_RESET) {
    if ((int32_t)(millis() - resetDueAt) >= 0) {
      bool idle = !pumpRunning && !manualDrainMode && (calState == CAL_IDLE);
      if (idle) { printlnTS(F("Interval self-reset (24h) -> restarting MCU...")); delay(200); esp_restart(); }
      else resetDueAt = millis() + 60000UL;
    }
  }

  // Button clear
  if (ENABLE_CLEAR_BTN && checkClearButtonOnce()){
    faultLatched = false; saveFaultToEEPROM(false);
    setFaultLED(false);
    printlnTS(F("Fault CLEARED via button."));
  }

  // Serial console commands (unchanged except 'H' prints liters too)
  if (SER_AVAILABLE()){
    char c = SER_READ();
    switch (c) {
      case 'C': faultLatched=false; saveFaultToEEPROM(false); setFaultLED(false); printlnTS(F("Fault CLEARED via Serial.")); break;

      case 'R': {
        printStamp(); SER_PRINT(F("Pump running: ")); SER_PRINTLN(pumpRunning ? F("YES") : F("NO"));
        if (pumpRunning) { uint32_t curMs = millis() - pumpOnAtMs; printStamp(); SER_PRINT(F("Current run so far: ")); SER_PRINT(curMs / 1000.0, 3); SER_PRINTLN(F(" s")); }
        printStamp(); SER_PRINT(F("Last completed run: ")); SER_PRINT(lastRunMs / 1000.0, 3); SER_PRINTLN(F(" s"));
        printStamp(); SER_PRINT(F("Lifetime ON time: ")); SER_PRINT(totalRunSec); SER_PRINTLN(F(" s"));
        printStamp(); SER_PRINT(F("Calibration (L/min): ")); SER_PRINTLN(flow_L_per_min, 6);
        printStamp(); SER_PRINT(F("Today: ")); SER_PRINT(secondsToday); SER_PRINT(F(" s, ")); SER_PRINT(litersToday, 3); SER_PRINTLN(F(" L"));
        if (sessIndexCurrent != 0xFF) { SessionEntry e; SessionStore::readAt(sessIndexCurrent, e);
          printStamp(); SER_PRINT(F("This power cycle (boot #")); SER_PRINT(e.bootNo); SER_PRINT(F("): starts="));
          SER_PRINT(e.starts); SER_PRINT(F(", runSec=")); SER_PRINT(e.runSec); SER_PRINTLN(F(" s"));
        }
      } break;

      case 'H': {
        // Serial-side history dump (kept similar to your previous version)
        printStamp(); SER_PRINT(F("\n=== RUN HISTORY (latest ")); SER_PRINT(logsCount); SER_PRINTLN(F(") ==="));
        SER_PRINTLN(F("#\tStart(Local)\t\tDur[s]\tLiters\tReason"));
        uint16_t first = (logsHead + MAX_RUN_LOGS - logsCount) % MAX_RUN_LOGS;
        for (uint16_t i=0; i<logsCount; i++){
          uint16_t idx = (first + i) % MAX_RUN_LOGS;
          SER_PRINT(i+1); SER_PRINT('\t');
          if (rtcOk && logs[idx].start_epoch != 0) {
            DateTime t((uint32_t)logs[idx].start_epoch);
            char when[24];
            snprintf(when, sizeof(when), "%04u-%02u-%02u %02u:%02u:%02u",
                     t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
            SER_PRINT(when); SER_PRINT('\t');
          } else {
            SER_PRINT(logs[idx].start_ms / 1000.0, 3); SER_PRINT('\t');
          }
          SER_PRINT(logs[idx].dur_ms / 1000.0, 3); SER_PRINT('\t');
          SER_PRINT(logs[idx].liters, 3); SER_PRINT('\t');
          SER_PRINTLN(logs[idx].reason);
          delay(2);
        }
        printStamp(); SER_PRINTLN(F("=== END RUN HISTORY ===\n"));
      } break;

      case 'X': logsCount=0; logsHead=0; totalOns=0; totalOffs=0; printlnTS(F("Run history cleared (RAM counters reset).")); break;

      case 'B': { printStamp(); SER_PRINT(F("Boot count: ")); SER_PRINTLN(bootCount);
                  printStamp(); SER_PRINT(F("Uptime: ")); printHMS(millis()/1000UL); SER_PRINTLN(); } break;

      case 'Z': bootCount=0; saveBootCount(); printlnTS(F("Boot counter reset to 0.")); break;

      case 'S': {
        SessionStore::readHeader();
        printStamp(); SER_PRINT(F("\n=== SESSION HISTORY (last ")); SER_PRINT(sessHdr.count); SER_PRINTLN(F(" power cycles) ==="));
        uint8_t first2 = (sessHdr.head + MAX_SESSIONS - sessHdr.count) % MAX_SESSIONS;
        for (uint8_t i=0; i<sessHdr.count; i++){
          uint8_t idx = (first2 + i) % MAX_SESSIONS; SessionEntry e; SessionStore::readAt(idx,e);
          SER_PRINTLN(F("----------------------------------------"));
          SER_PRINT(F("Power cycle #")); SER_PRINTLN(e.bootNo);
          SER_PRINT(F("  Starts: ")); SER_PRINTLN(e.starts);
          SER_PRINT(F("  Total run: ")); printHMS(e.runSec); SER_PRINTLN();
        }
        SER_PRINTLN(F("----------------------------------------"));
        printStamp(); SER_PRINTLN(F("=== END SESSION HISTORY ===\n"));
      } break;

      case 't': {
        if (rtcOk) {
          DateTime n = rtc.now();
          char buf[32];
          snprintf(buf,sizeof(buf),"%04u-%02u-%02u %02u:%02u:%02u",
                   n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second());
          printStamp(); SER_PRINT(F("RTC now: ")); SER_PRINTLN(buf);
        } else {
          printlnTS(F("RTC not OK."));
        }
      } break;

      case 'T': {
        String rest = Serial.readStringUntil('\n');
        String full = String("T") + rest;
        if (parseAndSetRTC(full)) { printlnTS(F("RTC time set.")); }
        else { printlnTS(F("Bad format. Use: T=YYYY-MM-DD HH:MM:SS")); }
      } break;

      case 'M': startManualDrain(); break;
      case 'K': startCalibration(); break;

      case 'k': {
        prefs.begin(PREF_NS, false);
        prefs.remove(PREF_KEY);
        prefs.end();
        SER_PRINTLN(F("Calibration cleared from NVS."));
        loadCalibration();
      } break;

      case 'f': {
        String rest = Serial.readStringUntil('\n');
        int eq = rest.indexOf('=');
        float val = (eq >= 0) ? rest.substring(eq+1).toFloat() : NAN;
        if (!isnan(val)) saveCalibration(val);
        else SER_PRINTLN(F("Use: f=1.23  (L/min)"));
      } break;

      case 'd': secondsToday=0; litersToday=0.0f; printlnTS(F("Today counters reset.")); break;

      default: break;
    }
  }

  // Debounced inputs
  int top    = topF.read(FLOAT_FILTER_MS);
  int bottom = botF.read(FLOAT_FILTER_MS);
  int man    = manBtn.read(FLOAT_FILTER_MS);
  int cal    = calBtn.read(FLOAT_FILTER_MS);

  // Rising edges
  static int lastMan = HIGH, lastCal = HIGH;
  bool manPressed = MANUAL_BTN_ACTIVE_LOW ? (lastMan==HIGH && man==LOW) : (lastMan==LOW && man==HIGH);
  bool calPressed = CAL_BTN_ACTIVE_LOW   ? (lastCal==HIGH && cal==LOW) : (lastCal==LOW && cal==HIGH);
  lastMan = man; lastCal = cal;

  if (manPressed) startManualDrain();
  if (calPressed) startCalibration();

  // Scheduled drain 06:30
  if (rtcOk && !faultLatched && calState==CAL_IDLE && !manualDrainMode) {
    DateTime nowdt = rtc.now();
    uint32_t ymd = ymdFromDT(nowdt);
    if (ymd != lastSchedYMD && timeIsAtOrAfter(nowdt, SCHED_HOUR, SCHED_MINUTE, 0)) {
      printlnTS(F("Scheduled drain start (06:30). Forcing pump ON until bottom float goes HIGH."));
      lastSwitchMs = millis() - MIN_DWELL_MS;
      pendingStopReason = 'T';
      scheduledRunActive = true;
      applyPumpState(true);
      lastSchedYMD = ymd;
    }
  }

  if (faultLatched) {
    if (pumpRunning) { pendingStopReason='F'; applyPumpState(false); }
    setHumid(false);
    setFaultLED(true);
    valveClose();
    valveOpenUntilMs = 0;
    manualDrainMode = false;
    if (calState != CAL_IDLE) cancelCalibration(F("Calibration canceled due to FAULT."));
  } else {
    // Calibration FSM
    if (calState == CAL_WAIT_TOP) {
      if (top == LOW) {
        calState = CAL_TIMING;
        calStartMs = millis();
        lastSwitchMs = millis() - MIN_DWELL_MS;
        pendingStopReason = 'C';
        applyPumpState(true);
        printlnTS(F("Calibration: TOP reached -> starting timing; draining to BOTTOM (bottom->HIGH)..."));
      }
    } else if (calState == CAL_TIMING) {
      if (bottom == HIGH) {
        uint32_t elapsed = millis() - calStartMs;
        pendingStopReason = 'C';
        applyPumpState(false);
        completeCalibration(elapsed);
      } else if (CAL_TIMEOUT_MS > 0 && (millis() - calStartMs) >= CAL_TIMEOUT_MS) {
        cancelCalibration(F("Calibration TIMEOUT -> stopping pump."));
      }
    }

    // Manual / normal control
    if (calState == CAL_IDLE) {
      if (manualDrainMode) {
        if (!pumpRunning) { lastSwitchMs = millis() - MIN_DWELL_MS; applyPumpState(true); }
        if (bottom == HIGH) stopManualDrain('M');
        else if (MANUAL_MAX_MS > 0 && (millis() - manualStartMs) >= MANUAL_MAX_MS) stopManualDrain('T');
      } else {
        if (!pumpRunning) {
          if (top == LOW) { applyPumpState(true); printlnTS(F("Reason: TOP UP -> start pump")); }
        } else {
          if (bottom == HIGH) {
            if (scheduledRunActive) scheduledRunActive = false;
            pendingStopReason = (pendingStopReason=='T') ? 'T' : 'B';
            applyPumpState(false); printlnTS(F("Reason: BOTTOM DOWN -> stop pump"));
          }
        }
      }
    }

    // Flow verification window
    if (pumpRunning && flowCheckActive) {
      uint32_t now = millis();
      if (now >= flowWindowStartMs) {
        uint32_t mv = readShunt_mV();
        flowSum_mV   += mv;
        flowSamples  += 1;
        if (mv < flowMin_mV) flowMin_mV = mv;

        if (now - flowWindowStartMs >= FLOW_VERIFY_MS) {
          float avg_mV = (flowSamples == 0) ? 0.0f : (float)flowSum_mV / (float)flowSamples;
          printStamp(); SER_PRINT(F("Flow avg over 5 s: ")); SER_PRINT(avg_mV, 1);
          SER_PRINT(F(" mV (min=")); SER_PRINT(flowMin_mV); SER_PRINTLN(F(" mV"));

          if (avg_mV >= FLOW_BAD_mV) {
            pendingStopReason='F';
            applyPumpState(false);
            faultLatched = true; saveFaultToEEPROM(true); setFaultLED(true);
            valveClose(); valveOpenUntilMs = 0;
            printlnTS(F("EMERGENCY: No-flow average >= 500 mV. Fault LATCHED and STORED. Pump inhibited until manual CLEAR."));
            manualDrainMode = false;
            if (calState != CAL_IDLE) calState = CAL_IDLE;
          } else {
            printlnTS(F("Flow OK: average below 500 mV."));
          }
          flowCheckActive = false;
        }
      }
    }

    setHumid(!faultLatched);
    setFaultLED(faultLatched);
  }

  // Auto-close valve
  if (valveIsOpen && valveOpenUntilMs != 0 && millis() >= valveOpenUntilMs) {
    valveClose();
    valveOpenUntilMs = 0;
    printlnTS(F("Valve auto-closed after pulse window."));
  }

  // Debug (every 5 s)
  const uint32_t DEBUG_PERIOD_MS = 5000;
  static uint32_t lastDbg=0;
  if (millis() - lastDbg >= DEBUG_PERIOD_MS) {
    lastDbg = millis();
    printStamp();
    SER_PRINT(F("Pump="));  SER_PRINT(pumpRunning?F("ON"):F("OFF"));
    SER_PRINT(F("  FaultLatched=")); SER_PRINT(faultLatched?F("YES"):F("NO"));
    uint32_t mv = readShunt_mV();
    SER_PRINT(F("  Vshunt=")); SER_PRINT(mv); SER_PRINT(F(" mV"));
    SER_PRINT(F("  Valve=")); SER_PRINT(valveIsOpen ? F("OPEN") : F("CLOSED"));
    if (valveIsOpen && valveOpenUntilMs) {
      long rem = (long)(valveOpenUntilMs - millis()); if (rem < 0) rem = 0;
      SER_PRINT(F(" (auto-close in ")); SER_PRINT(rem/1000); SER_PRINT(F(" s)"));
    }
    if (pumpRunning) {
      uint32_t curMs = millis() - pumpOnAtMs;
      SER_PRINT(F("  RunSoFar=")); SER_PRINT(curMs / 1000.0, 3); SER_PRINT(F(" s"));
    }
    SER_PRINT(F("  Cal(L/min)=")); SER_PRINT(flow_L_per_min, 6);
    SER_PRINT(F("  Today=")); SER_PRINT(secondsToday); SER_PRINT(F(" s, ")); SER_PRINT(litersToday, 3); SER_PRINT(F(" L"));
    SER_PRINTLN();
  }

  // Poll for incoming SMS (e.g., "H")
  processSmsCommands();

  delay(5);
}
