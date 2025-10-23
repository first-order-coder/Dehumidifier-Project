#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>            // I2C for RTC
#include <RTClib.h>          // DS3231 RTC (Adafruit RTClib)

/* ======================= ESP32 PIN MAP (ADJUST IF NEEDED) ======================= */
// Floats (open = HIGH, closed-to-GND = LOW)
const int FLOAT_TOP_PIN       = 7;     // e.g., ESP32-S3 GPIO7
const int FLOAT_BOTTOM_PIN    = 16;    // e.g., ESP32-S3 GPIO16

// Relays
const int RELAY_PUMP_PIN      = 17;    // pump relay IN/driver
const int RELAY_HUMID_PIN     = 15;    // humidifier relay IN/driver
const bool RELAY_PUMP_ACTIVE_LOW  = false;  // false=active-HIGH, true=active-LOW
const bool RELAY_HUMID_ACTIVE_LOW = true;   // active-LOW module

// Shunt ADC (ESP32 ADC1-capable pin recommended)
const int SHUNT_ADC_PIN       = 4;     // tap at Node X (pump side of 1Ω)

// Fault LED
const int  FAULT_LED_PIN         = 14;
const bool FAULT_LED_ACTIVE_HIGH = true;

// ======= VALVE CONTROL (12V NC valve via relay; LED shows OPEN state) =======
const int  VALVE_RELAY_PIN         = 12;       // avoid GPIO6..11 on classic ESP32
const bool VALVE_ACTIVE_LOW        = false;    // true if relay board is LOW-trigger
const int  VALVE_LED_PIN           = 13;       // valve-status LED; set -1 to disable
const bool VALVE_LED_ACTIVE_HIGH   = true;
const uint32_t VALVE_OPEN_PULSE_MS = 30000UL;  // 30 s post-stop pulse

bool     valveIsOpen      = false;
uint32_t valveOpenUntilMs = 0;

/* ======= HEARTBEAT ======= */
const int  ALIVE_LED_PIN           = 2;        // change if your board needs another pin
const uint32_t BOOT_SOLID_MS       = 2000UL;   // 2 s solid ON after boot
const uint32_t HEARTBEAT_PERIOD_MS = 10000UL;  // blink every 10 s
const uint16_t HEARTBEAT_ON_MS     = 100;      // LED ON for 100 ms
uint32_t bootMs                    = 0;

/* ======= RTC ======= */
const int I2C_SDA = 8;                          // change for your board
const int I2C_SCL = 9;                          // change for your board
RTC_DS3231 rtc;
bool rtcOk = false;

/* ======= NOISE / TIMING GUARDS ======= */
const uint16_t FLOAT_FILTER_MS   = 150;     // must be stable this long
const uint32_t MIN_DWELL_MS      = 3000;    // min ON/OFF runtime (anti-chatter)
const uint32_t FAULT_BLANK_MS    = 700;     // ignore inrush for sensing

/* ======= FLOW VERIFY (AVERAGE over 5 s) ======= */
const uint16_t FLOW_BAD_mV    = 500;        // threshold for no-flow (avg >= 0.50 V)
const uint32_t FLOW_VERIFY_MS = 5000;       // averaging window
const uint8_t  ADC_SAMPLES    = 12;         // per-read averaging

/* ======= EEPROM LAYOUT ======= */
#define EEPROM_SIZE         1024
const int EE_ADDR_FAULT      = 0;
const uint8_t EE_FAULT_CLEAR = 0x00;
const uint8_t EE_FAULT_SET   = 0xF1;

const int EE_ADDR_TOTAL_SEC  = 1;           // 4 bytes at 1..4
const int EE_ADDR_BOOT_COUNT = 10;          // 4 bytes at 10..13

// Remember whether the RTC has been explicitly set before
const int EE_ADDR_RTC_FLAG   = 20;          // 1 byte
const uint8_t EE_RTC_SET     = 0xA5;

/* ======= SESSION LOG (per power cycle) ======= */
const int EE_ADDR_SESSION_BASE = 200;
const uint32_t SESSION_MAGIC   = 0xC0FFEE51;
const uint8_t  MAX_SESSIONS    = 30;

struct SessionHeader { uint32_t magic; uint8_t head; uint8_t count; uint16_t rsv; };
struct SessionEntry  { uint32_t bootNo; uint32_t runSec; uint16_t starts; };

SessionHeader sessHdr;
uint8_t sessIndexCurrent = 0xFF;

/* ======= OPTIONAL MANUAL CLEAR BUTTON ======= */
const bool ENABLE_CLEAR_BTN = false;
const int  CLEAR_BTN_PIN    = 18;           // INPUT_PULLUP; button to GND

/* ======= STATE ======= */
bool pumpRunning  = false;
bool faultLatched = false;
uint32_t lastSwitchMs = 0;
uint32_t pumpOnAtMs   = 0;

/* ======= RUNTIME METRICS ======= */
uint32_t lastRunMs   = 0;                   // ms
uint32_t totalRunSec = 0;                   // seconds
uint32_t bootCount   = 0;                   // power cycles

/* ======= RUN HISTORY (RAM for current boot) ======= */
struct RunLog {
  uint32_t start_ms;
  uint32_t dur_ms;
  char     reason;
  uint32_t start_epoch;   // UNIX seconds at pump ON (0 if unknown / no RTC)
};
const uint8_t MAX_RUN_LOGS = 40;
RunLog logs[MAX_RUN_LOGS];
uint16_t logsCount = 0, logsHead = 0;
uint32_t totalOns = 0, totalOffs = 0;
char pendingStopReason = 'U';

/* Track current run's start time (epoch) */
uint32_t currentRunStartEpoch = 0;

/* ======= FLOW VERIFY STATE ======= */
bool     flowCheckActive    = false;
uint32_t flowWindowStartMs  = 0;
uint32_t flowSum_mV         = 0;
uint16_t flowSamples        = 0;
uint16_t flowMin_mV         = 65535;

/* ======= DAILY SCHEDULE ======= */
const uint8_t SCHED_HOUR     = 6;           // 06:30
const uint8_t SCHED_MINUTE   = 30;
uint32_t lastSchedYMD        = 0;           // YYYYMMDD of last completed schedule
bool scheduledRunActive      = false;       // true while timer-driven drain is in progress

/* ======================= HELPERS ======================= */
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
inline void relayPumpOn()        { setRelayLevel(RELAY_PUMP_PIN,  true,  RELAY_PUMP_ACTIVE_LOW); }
inline void relayPumpOff()       { setRelayLevel(RELAY_PUMP_PIN,  false, RELAY_PUMP_ACTIVE_LOW); }
inline void setHumid(bool on)    { setRelayLevel(RELAY_HUMID_PIN, on,    RELAY_HUMID_ACTIVE_LOW); }

/* ======= VALVE HELPERS ======= */
inline void setValveLED(bool valveOpen) {
  if (VALVE_LED_PIN < 0) return;
  digitalWrite(VALVE_LED_PIN,
               VALVE_LED_ACTIVE_HIGH ? (valveOpen ? HIGH : LOW)
                                     : (valveOpen ? LOW  : HIGH));
}
inline void setValveCoil(bool energize) { setRelayLevel_raw(VALVE_RELAY_PIN, energize, VALVE_ACTIVE_LOW); }
inline void valveOpen()  { setValveCoil(true);  valveIsOpen = true;  setValveLED(true); }
inline void valveClose() { setValveCoil(false); valveIsOpen = false; setValveLED(false); }

/* ======= Debounce ======= */
struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p){
    pin=p;
    // If you use external pull-ups on floats, change to INPUT.
    pinMode(pin, INPUT_PULLUP);
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
Debounce topF, botF;

/* ======= ADC: ESP32 uses analogReadMilliVolts ======= */
uint32_t readShunt_mV() {
  static bool init=false;
  if (!init) {
#if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
    analogSetPinAttenuation(SHUNT_ADC_PIN, ADC_11db); // ~0..3.3 V
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

// sanity: consider time valid if year >= 2023
bool rtcTimeLooksSane() {
  if (!rtcOk) return false;
  DateTime n = rtc.now();
  return (n.year() >= 2023);
}

// Parse "T=YYYY-MM-DD HH:MM:SS" and set RTC
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
    Serial.print(buf);
  } else {
    Serial.print("[no-RTC] ");
  }
}
void printlnTS(const __FlashStringHelper* s){ printStamp(); Serial.println(s); }
void printTS(const __FlashStringHelper* s){ printStamp(); Serial.print(s); }

/* ======= LOGGING (stores epoch for nicer history) ======= */
void logRun(uint32_t start_ms, uint32_t dur_ms, char reason) {
  logs[logsHead] = RunLog{ start_ms, dur_ms, reason, currentRunStartEpoch };
  logsHead = (logsHead + 1) % MAX_RUN_LOGS;
  if (logsCount < MAX_RUN_LOGS) logsCount++;
  totalOffs++;
}

/* ======= PUMP STATE APPLY (with valve & LED) ======= */
void applyPumpState(bool on) {
  if (on == pumpRunning) return;
  if (millis() - lastSwitchMs < MIN_DWELL_MS) return; // dwell guard

  if (on) {
    relayPumpOn();
    pumpOnAtMs = millis();
    totalOns++;
    SessionStore::addStarts(1);

    // Capture wall-clock start time (if RTC present)
    if (rtcOk) currentRunStartEpoch = rtc.now().unixtime();
    else       currentRunStartEpoch = 0;

    // Keep water out while pumping
    valveClose();
    valveOpenUntilMs = 0;

    // Start 5 s flow verify after inrush
    flowCheckActive   = true;
    flowWindowStartMs = pumpOnAtMs + FAULT_BLANK_MS;
    flowSum_mV        = 0;
    flowSamples       = 0;
    flowMin_mV        = 65535;

    printlnTS(F("PUMP -> ON (latched) | Valve CLOSED | Starting 5 s flow verification after inrush."));
  } else {
    if (pumpRunning) {
      uint32_t now = millis();
      lastRunMs = (now >= pumpOnAtMs) ? (now - pumpOnAtMs) : 0;
      uint32_t addSec = (lastRunMs + 999) / 1000;
      totalRunSec += addSec; saveTotalRunToEEPROM();
      SessionStore::addRunSec(addSec);

      logRun(pumpOnAtMs, lastRunMs, pendingStopReason); pendingStopReason='U';

      printStamp();
      Serial.print(F("PUMP -> OFF (latched). This run: "));
      Serial.print(lastRunMs / 1000.0, 3);
      Serial.print(F(" s  |  Lifetime: "));
      Serial.print(totalRunSec);
      Serial.println(F(" s"));
    }
    relayPumpOff();

    // Post-stop valve open pulse (LED ON while open)
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

bool checkClearButtonOnce(){
  if (!ENABLE_CLEAR_BTN) return false;
  static int last=HIGH; static uint32_t t=0;
  int v = digitalRead(CLEAR_BTN_PIN);
  if (v != last){ last=v; t=millis(); }
  if (last==LOW && millis()-t>20){ while(digitalRead(CLEAR_BTN_PIN)==LOW) delay(5); return true; }
  return false;
}

/* ======= HH:MM:SS printer ======= */
void printHMS(uint32_t secs) {
  uint32_t h = secs / 3600UL;
  uint32_t m = (secs / 60UL) % 60UL;
  uint32_t s = secs % 60UL;
  Serial.print(h); Serial.print(':');
  if (m<10) Serial.print('0'); Serial.print(m); Serial.print(':');
  if (s<10) Serial.print('0'); Serial.print(s);
}

/* ======= Daily schedule helper ======= */
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

/* ======================= SETUP ======================= */
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n[BOOT] ESP32 starting… (115200 baud)");

  // I2C + RTC
  Wire.begin(I2C_SDA, I2C_SCL);
  if (rtc.begin()) {
    if (rtc.lostPower()) {
      if (!rtcWasEverSet()) {
        // First-time init only: seed from compile time
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
          rtc.adjust(keep); // clears OSF without altering time
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

  // LEDs
  if (ALIVE_LED_PIN >= 0) pinMode(ALIVE_LED_PIN, OUTPUT), digitalWrite(ALIVE_LED_PIN, HIGH);
  if (FAULT_LED_PIN >= 0) pinMode(FAULT_LED_PIN, OUTPUT), setFaultLED(false);
  if (VALVE_LED_PIN >= 0) pinMode(VALVE_LED_PIN, OUTPUT);

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EE_ADDR_TOTAL_SEC, totalRunSec);
  loadBootCount();

  // Session header BEFORE incrementing boot
  SessionStore::readHeader();
  bootCount++; saveBootCount();

  int exist = SessionStore::findByBoot(bootCount);
  if (exist >= 0) sessIndexCurrent = (uint8_t)exist;
  else            sessIndexCurrent = SessionStore::startNew(bootCount);

  printStamp(); Serial.print(F("Boot count: ")); Serial.println(bootCount);
  printStamp(); Serial.print(F("Lifetime pump ON time (EEPROM): ")); Serial.print(totalRunSec); Serial.println(F(" s"));

  // Inputs/outputs
  topF.begin(FLOAT_TOP_PIN);
  botF.begin(FLOAT_BOTTOM_PIN);
  pinMode(RELAY_PUMP_PIN,  OUTPUT);
  pinMode(RELAY_HUMID_PIN, OUTPUT);
  pinMode(VALVE_RELAY_PIN, OUTPUT);

  // Defaults
  valveClose();  // keep water out
  relayPumpOff();
  setHumid(true);
  setFaultLED(false);

  // Clear any stored fault at boot
  saveFaultToEEPROM(false);
  faultLatched = false;

  printlnTS(F("\nPower-up: fault cleared."));
  printlnTS(F("Serial cmds: 'C' clear fault, 'R' report, 'H' run history, 'X' clear run history, 'B' boot info, 'Z' reset boot count, 'S' session history, 't' show RTC, 'T=YYYY-MM-DD HH:MM:SS' set RTC"));

  // Startup drain ONLY if water present (bottom float LOW)
  delay(FLOAT_FILTER_MS);
  int bottomNow = botF.read(FLOAT_FILTER_MS);
  if (!faultLatched && bottomNow == LOW) {
    printlnTS(F("Startup drain: water detected. Pump ON until bottom float goes HIGH."));
    lastSwitchMs = millis() - MIN_DWELL_MS; // bypass dwell
    applyPumpState(true);
  } else {
    printlnTS(F("Startup drain skipped: no water or fault latched."));
  }
}

/* ======================= LOOP ======================= */
void loop() {
  // ===== Heartbeat (2 s solid, then short blink every 10 s) =====
  if (ALIVE_LED_PIN >= 0) {
    uint32_t t = millis() - bootMs;
    if (t <= BOOT_SOLID_MS) {
      digitalWrite(ALIVE_LED_PIN, HIGH);
    } else {
      uint32_t phase = (t - BOOT_SOLID_MS) % HEARTBEAT_PERIOD_MS;
      digitalWrite(ALIVE_LED_PIN, (phase < HEARTBEAT_ON_MS) ? HIGH : LOW);
    }
  }

  // Manual clear button
  if (ENABLE_CLEAR_BTN && checkClearButtonOnce()){
    faultLatched = false; saveFaultToEEPROM(false);
    setFaultLED(false);
    printlnTS(F("Fault CLEARED via button."));
  }

  // Serial commands
  if (Serial.available()){
    char c = Serial.read();
    switch (c) {
      case 'C': faultLatched=false; saveFaultToEEPROM(false); setFaultLED(false); printlnTS(F("Fault CLEARED via Serial.")); break;

      case 'R': {
        printStamp(); Serial.print(F("Pump running: ")); Serial.println(pumpRunning ? F("YES") : F("NO"));
        if (pumpRunning) { uint32_t curMs = millis() - pumpOnAtMs; printStamp(); Serial.print(F("Current run so far: ")); Serial.print(curMs / 1000.0, 3); Serial.println(F(" s")); }
        printStamp(); Serial.print(F("Last completed run: ")); Serial.print(lastRunMs / 1000.0, 3); Serial.println(F(" s"));
        printStamp(); Serial.print(F("Lifetime ON time: ")); Serial.print(totalRunSec); Serial.println(F(" s"));
        if (sessIndexCurrent != 0xFF) { SessionEntry e; SessionStore::readAt(sessIndexCurrent, e);
          printStamp(); Serial.print(F("This power cycle (boot #")); Serial.print(e.bootNo); Serial.print(F("): starts="));
          Serial.print(e.starts); Serial.print(F(", runSec=")); Serial.print(e.runSec); Serial.println(F(" s"));
        }
      } break;

      case 'H': {
        // show wall-clock start time if available
        printStamp(); Serial.print(F("\n=== RUN HISTORY (latest ")); Serial.print(logsCount); Serial.println(F(") ==="));
        Serial.println(F("#\tStart(Local)\t\tDur[s]\tReason"));
        uint16_t first = (logsHead + MAX_RUN_LOGS - logsCount) % MAX_RUN_LOGS;
        for (uint16_t i=0; i<logsCount; i++){
          uint16_t idx = (first + i) % MAX_RUN_LOGS;
          Serial.print(i+1); Serial.print('\t');
          if (rtcOk && logs[idx].start_epoch != 0) {
            DateTime t((uint32_t)logs[idx].start_epoch);
            char when[24];
            snprintf(when, sizeof(when), "%04u-%02u-%02u %02u:%02u:%02u",
                     t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
            Serial.print(when); Serial.print('\t');
          } else {
            Serial.print(logs[idx].start_ms / 1000.0, 3); Serial.print('\t');
          }
          Serial.print(logs[idx].dur_ms / 1000.0, 3); Serial.print('\t');
          Serial.println(logs[idx].reason);
          delay(2);
        }
        printStamp(); Serial.println(F("=== END RUN HISTORY ===\n"));
      } break;

      case 'X': logsCount=0; logsHead=0; totalOns=0; totalOffs=0; printlnTS(F("Run history cleared (RAM counters reset).")); break;

      case 'B': { printStamp(); Serial.print(F("Boot count: ")); Serial.println(bootCount);
                  printStamp(); Serial.print(F("Uptime: ")); printHMS(millis()/1000UL); Serial.println(); } break;

      case 'Z': bootCount=0; saveBootCount(); printlnTS(F("Boot counter reset to 0.")); break;

      case 'S': {
        SessionStore::readHeader();
        printStamp(); Serial.print(F("\n=== SESSION HISTORY (last ")); Serial.print(sessHdr.count); Serial.println(F(" power cycles) ==="));
        uint8_t first = (sessHdr.head + MAX_SESSIONS - sessHdr.count) % MAX_SESSIONS;
        for (uint8_t i=0; i<sessHdr.count; i++){
          uint8_t idx = (first + i) % MAX_SESSIONS; SessionEntry e; SessionStore::readAt(idx,e);
          Serial.println(F("----------------------------------------"));
          Serial.print(F("Power cycle #")); Serial.println(e.bootNo);
          Serial.print(F("  Starts: ")); Serial.println(e.starts);
          Serial.print(F("  Total run: ")); printHMS(e.runSec); Serial.println();
        }
        Serial.println(F("----------------------------------------"));
        printStamp(); Serial.println(F("=== END SESSION HISTORY ===\n"));
      } break;

      // show RTC time
      case 't': {
        if (rtcOk) {
          DateTime n = rtc.now();
          char buf[32];
          snprintf(buf,sizeof(buf),"%04u-%02u-%02u %02u:%02u:%02u",
                   n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second());
          printStamp(); Serial.print(F("RTC now: ")); Serial.println(buf);
        } else {
          printlnTS(F("RTC not OK."));
        }
      } break;

      // set RTC time exactly: T=YYYY-MM-DD HH:MM:SS
      case 'T': {
        String rest = Serial.readStringUntil('\n'); // reads rest of line
        String full = String("T") + rest;
        if (parseAndSetRTC(full)) { printlnTS(F("RTC time set.")); }
        else { printlnTS(F("Bad format. Use: T=YYYY-MM-DD HH:MM:SS")); }
      } break;

      default: break;
    }
  }

  // Debounced floats
  int top    = topF.read(FLOAT_FILTER_MS);
  int bottom = botF.read(FLOAT_FILTER_MS);

  // ======= Daily schedule at 06:30 =======
  if (rtcOk && !faultLatched) {
    DateTime nowdt = rtc.now();
    uint32_t ymd = ymdFromDT(nowdt);
    if (ymd != lastSchedYMD && timeIsAtOrAfter(nowdt, SCHED_HOUR, SCHED_MINUTE, 0)) {
      printlnTS(F("Scheduled drain start (06:30). Forcing pump ON until bottom float goes HIGH."));
      lastSwitchMs = millis() - MIN_DWELL_MS;   // bypass dwell
      pendingStopReason = 'T';                  // Timer
      scheduledRunActive = true;
      applyPumpState(true);
      lastSchedYMD = ymd;                       // mark as executed for today
    }
  }

  if (faultLatched) {
    if (pumpRunning) { pendingStopReason='F'; applyPumpState(false); }
    setHumid(false);
    setFaultLED(true);
    valveClose();
    valveOpenUntilMs = 0;
  } else {
    // Normal control
    if (!pumpRunning) {
      if (top == LOW) {
        applyPumpState(true);
        printlnTS(F("Reason: TOP UP -> start pump"));
      }
    } else {
      if (bottom == HIGH) {
        if (scheduledRunActive) scheduledRunActive = false;
        pendingStopReason = (pendingStopReason=='T') ? 'T' : 'B';
        applyPumpState(false);
        printlnTS(F("Reason: BOTTOM DOWN -> stop pump"));
      }
    }

    // ======= FLOW VERIFICATION WINDOW (AVERAGE) =======
    if (pumpRunning && flowCheckActive) {
      uint32_t now = millis();
      if (now >= flowWindowStartMs) {
        uint32_t mv = readShunt_mV();
        flowSum_mV   += mv;
        flowSamples  += 1;
        if (mv < flowMin_mV) flowMin_mV = mv;

        if (now - flowWindowStartMs >= FLOW_VERIFY_MS) {
          float avg_mV = (flowSamples == 0) ? 0.0f : (float)flowSum_mV / (float)flowSamples;
          printStamp(); Serial.print(F("Flow avg over 5 s: ")); Serial.print(avg_mV, 1);
          Serial.print(F(" mV (min=")); Serial.print(flowMin_mV); Serial.println(F(" mV)"));

          if (avg_mV >= FLOW_BAD_mV) {
            pendingStopReason='F';
            applyPumpState(false);
            faultLatched = true; saveFaultToEEPROM(true); setFaultLED(true);
            valveClose(); valveOpenUntilMs = 0;
            printlnTS(F("EMERGENCY: No-flow average >= 500 mV. Fault LATCHED and STORED. Pump inhibited until manual CLEAR."));
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

  // Auto-close valve after the post-stop open pulse
  if (valveIsOpen && valveOpenUntilMs != 0 && millis() >= valveOpenUntilMs) {
    valveClose();
    valveOpenUntilMs = 0;
    printlnTS(F("Valve auto-closed after pulse window."));
  }

  // Debug print (every 5 s) – timestamped
  const uint32_t DEBUG_PERIOD_MS = 5000;
  static uint32_t lastDbg=0;
  if (millis() - lastDbg >= DEBUG_PERIOD_MS) {
    lastDbg = millis();
    printStamp();
    Serial.print(F("Pump="));  Serial.print(pumpRunning?F("ON"):F("OFF"));
    Serial.print(F("  FaultLatched=")); Serial.print(faultLatched?F("YES"):F("NO"));
    uint32_t mv = readShunt_mV();
    Serial.print(F("  Vshunt=")); Serial.print(mv); Serial.print(F(" mV"));
    Serial.print(F("  Valve=")); Serial.print(valveIsOpen ? F("OPEN") : F("CLOSED"));
    if (valveIsOpen && valveOpenUntilMs) {
      long rem = (long)(valveOpenUntilMs - millis()); if (rem < 0) rem = 0;
      Serial.print(F(" (auto-close in ")); Serial.print(rem/1000); Serial.print(F(" s)"));
    }
    if (pumpRunning) {
      uint32_t curMs = millis() - pumpOnAtMs;
      Serial.print(F("  RunSoFar=")); Serial.print(curMs / 1000.0, 3); Serial.print(F(" s"));
    }
    Serial.println();
  }

  delay(5);
}
