#include <Arduino.h>
#include <EEPROM.h>

/*************** VERSION SWITCH (compile-time) ***************/
#define EQUIPMENT_VERSION 21   // <- set to 10, 11, 20, or 21

#define EV_MAJOR    (EQUIPMENT_VERSION / 10)   // 1x or 2x
#define EV_MINOR    (EQUIPMENT_VERSION % 10)   // x0 or x1
#define HAS_TANK2   (EV_MAJOR == 2)            // v20 / v21
#define HAS_DEFROST (EV_MINOR == 1)            // v11 / v21

#if (EQUIPMENT_VERSION!=10 && EQUIPMENT_VERSION!=11 && \
     EQUIPMENT_VERSION!=20 && EQUIPMENT_VERSION!=21)
  #error "EQUIPMENT_VERSION must be 10, 11, 20, or 21"
#endif

/*************** BUILD-TIME SERIAL SWITCH ***************/
#ifndef useSerialTerminal
#define useSerialTerminal 1   // 1=enable logs, 0=strip all serial I/O
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

/*************** ESP32-S3 PIN MAP (your list) ***************
  0  out  air_V1_open           (unused here -> safe LOW)
  1  out  air_V2_open           (unused here -> safe LOW)
  2  out  air_V1_closed         (unused here -> safe LOW)
  3  out  air_V2_closed         (unused here -> safe LOW)
  4  IN_A shunt_resistance (ADC)  (J5 -> shunt_resistance; add zener; no R8)
  5  SPI  SS
  6  out  valve5                (Tank-2 bypass -> fill T2)
  7  IN_J top_sensor            (T1 HIGH; add zener)
  8  I2C  SDA
  9  I2C  SCL                   (marked SDL in note; R6 not necessary)
 10  IN_J top_sensor2           (T2 HIGH)
 11  SPI3 CLK
 12  SPI3 MISO
 13  out empty pipe             (Valve-4 main discharge / flush)
 14  IN_DB calibration switch   (not used here; reserved)
 15  OUT HUMIDIFIER ON          (add R + zener)
 16  SIMCOM (UART RX/TX pins used by your SIM module)
 17  SIMCOM
 18  SPI1 CLK
 20  IN_J bottom_sensor2        (T2 LOW)
 21  Available
 26  OUT FAN1                   (unused here -> safe LOW)
 33  IN_J bottom_sensor         (T1 LOW; two resistors + zeners)
 36  SPI3 SS1
 37  OUT FAN2                   (unused here -> safe LOW)
 38  OUT pump / relay
 39  OUT valve_8                (Tank-2 drain)
 40  OUT HEATER (defrost)
 41  OUT HeartBeat LED
 46  IN_DB empty tank (already used elsewhere in your system; unused here)
************************************************************/

/*************** PINS (remapped to ESP32-S3) ***************/
// Tank-1 floats
const int PIN_T1_HIGH   = 7;    // top_sensor (upper)
const int PIN_T1_LOW    = 33;   // bottom_sensor (lower)

// Actuators
const int PIN_PUMP      = 38;   // pump / relay
const int PIN_HUMID     = 15;   // HUMIDIFIER ON (energized = humidifier OFF in logic below)
const int PIN_VALVE4    = 13;   // "empty pipe" -> main discharge valve (flush)

// Indicators
const int PIN_FAULT_LED  = 41;                 // use heartbeat LED as fault indicator too
const int PIN_MOTOR_LED  = 41;                 // reuse same LED for "motor" on indication
const int PIN_VALVE4_LED = -1;                 // no dedicated LED (set to >=0 to enable)

// Optional button(s)
const bool ENABLE_CLEAR_BTN = false;
const int  PIN_CLEAR_BTN    = 4;               // reserved (not used)
const int  PIN_MANUAL_BTN   = 14;              // you can wire calibration button as manual if desired

// Relay polarities (adjust to your boards)
const bool RELAY_PUMP_ACTIVE_LOW   = false;
const bool RELAY_HUMID_ACTIVE_LOW  = true;     // active-LOW module
const bool RELAY_VALVE4_ACTIVE_LOW = false;

// Tank-2 hardware (only used in v20/v21)
#if HAS_TANK2
  const int PIN_VALVE5    = 6;    // bypass valve to Tank-2 (energize=open) -> fill T2
  const int PIN_VALVE8    = 39;   // switching valve -> drain T2
  const bool RELAY_VALVE5_ACTIVE_LOW = false;
  const bool RELAY_VALVE8_ACTIVE_LOW = false;

  // Tank-2 sensors
  const int PIN_T2_LOW    = 20;   // bottom_sensor2  (LOW=water present)
  const int PIN_T2_HIGH   = 10;   // top_sensor2     (HIGH=full)
#endif

// Defrost spiral (only used in v11/v21)
#if HAS_DEFROST
  const int PIN_DEFROST   = 40;   // heater driver (energize to heat)
  const bool DEFROST_ACTIVE_LOW = false;
#endif

// ADC for shunt (pump current proxy)
const int   SHUNT_ADC_PIN        = 4;      // GPIO4 analog input
const bool  USE_INTERNAL_ADC_REF = false;  // ESP32 uses its own ADC ref; ignore
const float VREF_mV_CAL          = 3300.0; // not used with analogReadMilliVolts()

/*************** TIMING & THRESHOLDS ***************/
const uint16_t FLOAT_FILTER_MS     = 150;     // debounce hold time
const uint32_t MIN_DWELL_MS        = 3000;    // anti-chatter ON/OFF min dwell
const uint32_t FAULT_BLANK_MS      = 700;     // ignore inrush before sampling
const uint32_t VALVE4_PULSE_MS     = 30000UL; // open valve-4 this long after pump stops (flush)

// Frozen handling timing
const uint32_t DEFROST_MS          = 30000UL;  // adjust as needed
const uint32_t RETRY_24H_MS        = 24UL*60UL*60UL*1000UL;
const uint32_t DEFROST_CHECK_MS    = 6UL*60UL*60UL*1000UL;

// Flow/current check (rolling avg)
const uint16_t FLOW_BAD_mV         = 500;     // rolling avg trip level
const uint8_t  AVG_WINDOW          = 5;       // 5-sample rolling average
const uint32_t SAMPLE_MS           = 1000UL;  // 1 Hz

// Manual drain
const bool     MANUAL_BTN_ACTIVE_LOW = true;
const uint32_t MANUAL_MAX_MS         = 1UL * 60UL * 1000UL; // 1 minute hard timeout

/*************** EEPROM LAYOUT ***************/
const int EE_ADDR_FAULT      = 0;            // 1 byte
const uint8_t EE_FAULT_CLEAR = 0x00;
const uint8_t EE_FAULT_SET   = 0xF1;
const int EE_ADDR_TOTAL_SEC  = 1;            // 4 bytes at 1..4
const int EE_ADDR_BOOT_COUNT = 10;           // 4 bytes at 10..13

/*************** SESSION LOG (per power cycle) ***************/
const int EE_ADDR_SESSION_BASE = 200;
const uint32_t SESSION_MAGIC   = 0xC0FFEE51;
const uint8_t  MAX_SESSIONS    = 30;
struct SessionHeader { uint32_t magic; uint8_t head; uint8_t count; uint16_t rsv; };
struct SessionEntry  { uint32_t bootNo; uint32_t runSec; uint16_t starts; };
SessionHeader sessHdr; uint8_t sessIndexCurrent = 0xFF;

struct SessionStore {
  static int entryAddr(uint8_t idx) { return EE_ADDR_SESSION_BASE + sizeof(SessionHeader) + idx * sizeof(SessionEntry); }
  static void readHeader() {
    EEPROM.get(EE_ADDR_SESSION_BASE, sessHdr);
    if (sessHdr.magic != SESSION_MAGIC || sessHdr.head >= MAX_SESSIONS || sessHdr.count > MAX_SESSIONS) {
      sessHdr.magic = SESSION_MAGIC; sessHdr.head = 0; sessHdr.count = 0; sessHdr.rsv = 0;
      EEPROM.put(EE_ADDR_SESSION_BASE, sessHdr);
      EEPROM.commit();
    }
  }
  static void writeHeader() { EEPROM.put(EE_ADDR_SESSION_BASE, sessHdr); EEPROM.commit(); }
  static void writeAt(uint8_t idx, const SessionEntry &e) { EEPROM.put(entryAddr(idx), e); EEPROM.commit(); }
  static void readAt(uint8_t idx, SessionEntry &e) { EEPROM.get(entryAddr(idx), e); }
  static int findByBoot(uint32_t bootNo) {
    if (sessHdr.count == 0) return -1;
    uint8_t first = (sessHdr.head + MAX_SESSIONS - sessHdr.count) % MAX_SESSIONS;
    for (uint8_t i=0;i<sessHdr.count;i++){ uint8_t idx=(first + i) % MAX_SESSIONS; SessionEntry e; readAt(idx, e); if (e.bootNo == bootNo) return idx; }
    return -1;
  }
  static uint8_t startNew(uint32_t bootNo) {
    SessionEntry e{bootNo, 0, 0};
    uint8_t idx = sessHdr.head; writeAt(idx, e);
    if (sessHdr.count < MAX_SESSIONS) sessHdr.count++;
    sessHdr.head = (sessHdr.head + 1) % MAX_SESSIONS; writeHeader(); return idx;
  }
  static void addStarts(uint16_t delta=1) {
    if (sessIndexCurrent == 0xFF) return; SessionEntry e; readAt(sessIndexCurrent, e); e.starts += delta; writeAt(sessIndexCurrent, e);
  }
  static void addRunSec(uint32_t addSec) {
    if (sessIndexCurrent == 0xFF) return; SessionEntry e; readAt(sessIndexCurrent, e); e.runSec += addSec; writeAt(sessIndexCurrent, e);
  }
};

/*************** STATE & METRICS ***************/
bool pumpRunning  = false;
bool faultLatched = false;

bool valve4Open   = false;
uint32_t valve4_close_at = 0;

uint32_t lastSwitchMs = 0;
uint32_t pumpOnAtMs   = 0;
uint32_t faultLatchedAtMs = 0;

uint32_t lastRunMs   = 0;   // ms
uint32_t totalRunSec = 0;   // s
uint32_t bootCount   = 0;

inline void saveTotalRunToEEPROM() {
  static uint32_t lastSaved = 0xFFFFFFFF;
  if (totalRunSec != lastSaved) { EEPROM.put(EE_ADDR_TOTAL_SEC, totalRunSec); EEPROM.commit(); lastSaved = totalRunSec; }
}

/*************** RUN HISTORY (RAM ring) ***************/
struct RunLog { uint32_t start_ms, dur_ms; char reason; };
const uint8_t MAX_RUN_LOGS = 40;
RunLog logs[MAX_RUN_LOGS]; uint16_t logsCount = 0, logsHead = 0;
uint32_t totalOns = 0;
char pendingStopReason = 'U';

/*************** 5-SAMPLE ROLLING AVG ***************/
bool     flowAvgActive      = false;
uint32_t flowStartAfterMs   = 0;
uint16_t shuntBuf[AVG_WINDOW];
uint8_t  shuntCount = 0, shuntHead = 0;
uint32_t lastSampleMs = 0;

/*************** Optional manual drain ***************/
bool     manualDrainMode     = false;
bool     manualVerifyPending = false;
uint32_t manualStartMs       = 0;

/*************** Tank-2 drain orchestration ***************/
#if HAS_TANK2
bool t2DrainPending = false;      // set at stop if T2 has water (sensor 6 LOW)
#endif
bool suppressNextMainFlush = false; // skip valve-4 flush once when we will drain T2

/*************** Debounce helper ***************/
struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p, bool pullup=false){ pin=p; pinMode(pin, pullup?INPUT_PULLUP:INPUT); stable=candidate=digitalRead(pin); tChange=millis(); }
  int read(uint16_t hold_ms){ int raw=digitalRead(pin); if(raw!=candidate){candidate=raw; tChange=millis();} if((millis()-tChange)>=hold_ms && stable!=candidate) stable=candidate; return stable; }
};
Debounce t1HighF, t1LowF, manualBtn;
#if HAS_TANK2
  Debounce t2LowF, t2HighF;
#endif

/*************** Relay & IO helpers ***************/
inline void setOut(int pin, bool on, bool activeLow=false){
  digitalWrite(pin, activeLow ? (on?LOW:HIGH) : (on?HIGH:LOW));
}
inline void pumpOn()       { setOut(PIN_PUMP, true, RELAY_PUMP_ACTIVE_LOW); }
inline void pumpOff()      { setOut(PIN_PUMP, false, RELAY_PUMP_ACTIVE_LOW); }
inline void humidOff()     { setOut(PIN_HUMID, true, RELAY_HUMID_ACTIVE_LOW); }  // energized = humidifier OFF
inline void humidOn()      { setOut(PIN_HUMID, false, RELAY_HUMID_ACTIVE_LOW); } // de-energized = humidifier ON

inline void valve4_set(bool open){
  setOut(PIN_VALVE4, open, RELAY_VALVE4_ACTIVE_LOW);
  valve4Open = open;
  if (PIN_VALVE4_LED >= 0) digitalWrite(PIN_VALVE4_LED, open ? HIGH : LOW);
}
inline void valve4_openTimed(uint32_t ms){
  valve4_set(true);
  valve4_close_at = millis() + ms;
}
inline void valve4_autoCloseTick(){
  if (valve4Open && valve4_close_at && (long)(millis() - valve4_close_at) >= 0){
    valve4_set(false);
    valve4_close_at = 0;
    SER_PRINTLN(F("Valve-4 auto-closed after flush window."));
  }
}

#if HAS_TANK2
  inline void valve5_set(bool open){ setOut(PIN_VALVE5, open, RELAY_VALVE5_ACTIVE_LOW); }
  inline void valve8_set(bool open){ setOut(PIN_VALVE8, open, RELAY_VALVE8_ACTIVE_LOW); }
#endif

#if HAS_DEFROST
  inline void defrost_set(bool on){ setOut(PIN_DEFROST, on, DEFROST_ACTIVE_LOW); }
#endif

inline void setFaultLED(bool on){ digitalWrite(PIN_FAULT_LED, on ? HIGH : LOW); }
inline void setMotorLED(bool on){ digitalWrite(PIN_MOTOR_LED, on ? HIGH : LOW); }

/*************** ADC (ESP32: read in mV) ***************/
const uint8_t ADC_SAMPLES = 12;
uint32_t readShunt_mV() {
  uint32_t acc=0;
  for(uint8_t i=0;i<ADC_SAMPLES;i++){
#if defined(ARDUINO_ARCH_ESP32)
    acc += analogReadMilliVolts(SHUNT_ADC_PIN); // returns mV
#else
    acc += analogRead(SHUNT_ADC_PIN) * 3300UL / 4095UL;
#endif
    delayMicroseconds(200);
  }
  return (acc + (ADC_SAMPLES/2)) / ADC_SAMPLES;
}

/*************** EEPROM fault flag ***************/
void saveFaultToEEPROM(bool set){
  uint8_t want = set ? EE_FAULT_SET : EE_FAULT_CLEAR;
  if (EEPROM.read(EE_ADDR_FAULT) != want) { EEPROM.write(EE_ADDR_FAULT, want); EEPROM.commit(); }
}
bool faultStoredInEEPROM(){ return EEPROM.read(EE_ADDR_FAULT) == EE_FAULT_SET; }

/*************** Logging ***************/
void logRun(uint32_t start_ms, uint32_t dur_ms, char reason) {
  logs[logsHead] = RunLog{start_ms, dur_ms, reason};
  logsHead = (logsHead + 1) % MAX_RUN_LOGS;
  if (logsCount < MAX_RUN_LOGS) logsCount++;
}

/*************** Apply pump state (handles sampling & flush) ***************/
void applyPumpState(bool on){
  if (on == pumpRunning) return;
  if (millis() - lastSwitchMs < MIN_DWELL_MS) return;

  if (on){
    pumpOn(); pumpOnAtMs = millis(); totalOns++;
    SessionStore::addStarts(1);
    valve4_set(false); valve4_close_at = 0;   // keep main valve closed while pumping (normal start)
    flowAvgActive = true; flowStartAfterMs = pumpOnAtMs + FAULT_BLANK_MS;
    shuntCount = 0; shuntHead = 0; lastSampleMs = flowStartAfterMs;
    setMotorLED(true);
    SER_PRINTLN(F("PUMP -> ON | Valve4 CLOSED | Rolling average will start after inrush."));
  } else {
    if (pumpRunning){
      uint32_t now = millis();
      lastRunMs = (now>=pumpOnAtMs)?(now-pumpOnAtMs):0;
      uint32_t addSec = (lastRunMs + 999)/1000;
      totalRunSec += addSec; saveTotalRunToEEPROM();
      SessionStore::addRunSec(addSec);
      logRun(pumpOnAtMs, lastRunMs, pendingStopReason); pendingStopReason='U';
      SER_PRINT(F("PUMP -> OFF. This run: ")); SER_PRINT(lastRunMs/1000.0,3);
      SER_PRINT(F(" s  |  Lifetime: ")); SER_PRINT(totalRunSec); SER_PRINTLN(F(" s"));
    }
    pumpOff(); setMotorLED(false); flowAvgActive=false;

#if HAS_TANK2
    int t2low_now = digitalRead(PIN_T2_LOW);
    if (t2low_now == LOW) {
      t2DrainPending = true;
      suppressNextMainFlush = true;   // skip immediate main flush; we'll flush after T2 drain
      valve4_set(false);              // ensure main is closed before rerouting
      SER_PRINTLN(F("Post-run: Tank-2 has water -> schedule T2 drain; skipping immediate main flush."));
    }
#endif

    if (!suppressNextMainFlush) {
      if (VALVE4_PULSE_MS > 0){
        valve4_openTimed(VALVE4_PULSE_MS);
        SER_PRINTLN(F("Valve-4 OPEN for post-stop flush."));
      } else {
        valve4_set(false);
        SER_PRINTLN(F("Valve-4 CLOSED (no flush window)."));
      }
    } else {
      suppressNextMainFlush = false;
    }
  }
  pumpRunning = on;
  lastSwitchMs = millis();
}

/*************** Frozen handling helpers ***************/
bool policyAllowsDefrostNow(bool t2High){
#if HAS_DEFROST
  #if HAS_TANK2
    return t2High;  // defrost allowed only when Tank-2 is full (v21 policy)
  #else
    return true;
  #endif
#else
  (void)t2High; return false;
#endif
}

/*************** NEW: Frozen → FILL Tank-2 only (Valve-5), never Valve-8 ***************/
bool routeFrozenToTank2_UntilT1Empty(uint32_t timeout_ms = 5UL*60UL*1000UL) {
#if HAS_TANK2
  SER_PRINTLN(F("[FROZEN] Routing to Tank-2 via Valve-5 only (no Valve-8)."));
  valve4_set(false);
  valve5_set(true);        // open bypass to Tank-2 (fill)

  lastSwitchMs = millis() - MIN_DWELL_MS;
  applyPumpState(true);

  // Disable rolling-average watchdog during this special run
  flowAvgActive = false;

  uint32_t cap = millis() + timeout_ms;
  bool emptied = false;
  while (millis() < cap) {
    if (digitalRead(PIN_T1_LOW) == HIGH) { emptied = true; break; }
    delay(10);
  }

  pendingStopReason = 'F';  // 'F' = frozen reroute
  applyPumpState(false);
  valve5_set(false);
  valve4_openTimed(5000);   // short flush to empty pipe

  if (emptied) SER_PRINTLN(F("[FROZEN] Tank-1 emptied to Tank-2 successfully (Valve-8 stayed OFF)."));
  else         SER_PRINTLN(F("[FROZEN] Timeout while routing to Tank-2. Stopped without emergency."));
  return emptied;
#else
  SER_PRINTLN(F("[FROZEN] Tank-2 hardware not present; cannot route."));
  return false;
#endif
}

/*************** Emergency trip ***************/
void tripFaultInstant(const char* why){
  pendingStopReason = 'F';
  applyPumpState(false);
  faultLatched = true; saveFaultToEEPROM(true); setFaultLED(true);
  valve4_set(false); valve4_close_at=0;
  manualDrainMode=false;
  faultLatchedAtMs = millis();
  SER_PRINT(F("EMERGENCY: ")); SER_PRINTLN(why);
}

/*************** Manual drain ***************/
void startManualDrain(){
  if (faultLatched) { SER_PRINTLN(F("Manual DRAIN ignored: fault latched.")); return; }
  manualDrainMode = true; manualVerifyPending=true; manualStartMs=millis();

  if (pumpRunning){
    flowAvgActive=true; flowStartAfterMs=millis()+FAULT_BLANK_MS; shuntCount=0; shuntHead=0; lastSampleMs=flowStartAfterMs;
    SER_PRINTLN(F("Manual DRAIN: verification window restarted while running."));
  } else {
    lastSwitchMs = millis() - MIN_DWELL_MS;
    applyPumpState(true);
  }
  SER_PRINTLN(F("Manual DRAIN: started (runs until T1 low=HIGH or timeout)."));
}
void stopManualDrain(char reasonCode){
  manualDrainMode=false; pendingStopReason=reasonCode; applyPumpState(false);
  SER_PRINT(F("Manual DRAIN: stop (reason=")); SER_PRINT(reasonCode); SER_PRINTLN(F(")."));
}

/*************** Setup ***************/
void setup(){
  SER_BEGIN(115200);

#if defined(ARDUINO_ARCH_ESP32)
  EEPROM.begin(512);  // flash-backed EEPROM emulation
#endif

  // Inputs (most “IN_J/IN_DB” are wired with external resistors + zeners -> still safe to enable pullups)
  pinMode(PIN_T1_HIGH, INPUT_PULLUP);
  pinMode(PIN_T1_LOW,  INPUT_PULLUP);
  t1HighF.begin(PIN_T1_HIGH, true);
  t1LowF.begin(PIN_T1_LOW,  true);

  pinMode(PIN_MANUAL_BTN, INPUT_PULLUP);
  manualBtn.begin(PIN_MANUAL_BTN, true);

#if HAS_TANK2
  pinMode(PIN_T2_LOW,  INPUT_PULLUP);
  pinMode(PIN_T2_HIGH, INPUT_PULLUP);
  t2LowF.begin(PIN_T2_LOW,  true);
  t2HighF.begin(PIN_T2_HIGH, true);
#endif

  // ADC pin
  pinMode(SHUNT_ADC_PIN, INPUT);

  // Outputs
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_HUMID, OUTPUT);
  pinMode(PIN_VALVE4, OUTPUT);
  if (PIN_VALVE4_LED >= 0) pinMode(PIN_VALVE4_LED, OUTPUT);
  pinMode(PIN_FAULT_LED, OUTPUT);
  pinMode(PIN_MOTOR_LED, OUTPUT);
#if HAS_TANK2
  pinMode(PIN_VALVE5, OUTPUT);
  pinMode(PIN_VALVE8, OUTPUT);
#endif
#if HAS_DEFROST
  pinMode(PIN_DEFROST, OUTPUT);
  defrost_set(false);
#endif

  // Safe defaults
  pumpOff(); humidOn(); valve4_set(false); setFaultLED(false); setMotorLED(false);
#if HAS_TANK2
  valve5_set(false); valve8_set(false);
#endif

  // Quiet unused declared outputs (keep LOW)
  for (int p : {0,1,2,3,26,37}) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  // Load counters / sessions
  EEPROM.get(EE_ADDR_TOTAL_SEC, totalRunSec);
  EEPROM.get(EE_ADDR_BOOT_COUNT, bootCount); bootCount++; EEPROM.put(EE_ADDR_BOOT_COUNT, bootCount); EEPROM.commit();

  SessionStore::readHeader();
  {
    int exist = SessionStore::findByBoot(bootCount);
    if (exist >= 0) sessIndexCurrent = (uint8_t)exist;
    else            sessIndexCurrent = SessionStore::startNew(bootCount);
  }

  // Clear stored fault at boot
  saveFaultToEEPROM(false); faultLatched=false; faultLatchedAtMs=0;

  SER_PRINT(F("Boot #")); SER_PRINTLN(bootCount);
  SER_PRINT(F("Equip version: ")); SER_PRINTLN(EQUIPMENT_VERSION);
  SER_PRINT(F("Lifetime pump ON (s): ")); SER_PRINTLN(totalRunSec);

  delay(FLOAT_FILTER_MS);
  if (!faultLatched && t1LowF.read(FLOAT_FILTER_MS) == LOW){
    SER_PRINTLN(F("Startup drain: water in Tank-1. Pump ON until T1 low = HIGH."));
    lastSwitchMs = millis() - MIN_DWELL_MS;
    applyPumpState(true);
  } else {
    SER_PRINTLN(F("Startup drain skipped."));
  }
}

/*************** Loop ***************/
void loop(){
  // Serial commands
  if (SER_AVAILABLE()){
    char c = SER_READ();
    switch(c){
      case 'C': faultLatched=false; saveFaultToEEPROM(false); setFaultLED(false); faultLatchedAtMs=0; SER_PRINTLN(F("Fault CLEARED.")); break;
      case 'R': {
        SER_PRINT(F("Pump: ")); SER_PRINTLN(pumpRunning?F("ON"):F("OFF"));
        SER_PRINT(F("Last run (s): ")); SER_PRINTLN(lastRunMs/1000.0,3);
        SER_PRINT(F("Lifetime (s): ")); SER_PRINTLN(totalRunSec);
        SER_PRINT(F("Version: ")); SER_PRINTLN(EQUIPMENT_VERSION);
#if HAS_TANK2
        SER_PRINT(F("T2 low=")); SER_PRINT(digitalRead(PIN_T2_LOW)==LOW?F("WATER"):F("DRY"));
        SER_PRINT(F("  T2 high=")); SER_PRINTLN(digitalRead(PIN_T2_HIGH)==HIGH?F("FULL"):F("OK"));
#endif
      } break;
      case 'H': {
        SER_PRINT(F("\n=== RUN HISTORY (latest ")); SER_PRINT(logsCount); SER_PRINTLN(F(") ==="));
        SER_PRINTLN(F("#\tStart[s]\tDur[s]\tReason"));
        uint16_t first = (logsHead + MAX_RUN_LOGS - logsCount) % MAX_RUN_LOGS;
        for(uint16_t i=0;i<logsCount;i++){ uint16_t idx=(first+i)%MAX_RUN_LOGS;
          SER_PRINT(i+1); SER_PRINT('\t'); SER_PRINT(logs[idx].start_ms/1000.0,3); SER_PRINT('\t');
          SER_PRINT(logs[idx].dur_ms/1000.0,3); SER_PRINT('\t'); SER_PRINTLN(logs[idx].reason); delay(2);
        }
        SER_PRINTLN(F("=== END RUN HISTORY ===\n"));
      } break;
      case 'X': logsCount=0; logsHead=0; totalOns=0; SER_PRINTLN(F("Run history cleared.")); break;
      case 'B': SER_PRINT(F("Boot #")); SER_PRINTLN(bootCount); SER_PRINT(F("Uptime (s): ")); SER_PRINTLN(millis()/1000.0,3); break;
      case 'Z': bootCount=0; EEPROM.put(EE_ADDR_BOOT_COUNT, bootCount); EEPROM.commit(); SER_PRINTLN(F("Boot counter reset.")); break;
      case 'M': startManualDrain(); break;
      default: break;
    }
  }

  // Debounce reads
  int t1High = t1HighF.read(FLOAT_FILTER_MS);
  int t1Low  = t1LowF.read(FLOAT_FILTER_MS);
  int man    = manualBtn.read(FLOAT_FILTER_MS);

#if HAS_TANK2
  int t2Low  = t2LowF.read(FLOAT_FILTER_MS);   // LOW = water present
  int t2High = t2HighF.read(FLOAT_FILTER_MS);  // HIGH = full
#else
  int t2Low = HIGH, t2High = LOW;
#endif

  // Manual button rising edge
  static int lastMan = HIGH;
  bool pressedEdge = MANUAL_BTN_ACTIVE_LOW ? (lastMan==HIGH && man==LOW) : (lastMan==LOW && man==HIGH);
  lastMan = man;
  if (pressedEdge) startManualDrain();

  // Faulted?
  if (faultLatched){
    if (pumpRunning){ pendingStopReason='F'; applyPumpState(false); }
    humidOff(); setFaultLED(true); setMotorLED(false);
    valve4_set(false); valve4_close_at=0;
    if (faultLatchedAtMs!=0 && (millis()-faultLatchedAtMs)>=RETRY_24H_MS){
      faultLatched=false; saveFaultToEEPROM(false); setFaultLED(false); faultLatchedAtMs=0;
      SER_PRINTLN(F("Fault auto-cleared after 24h."));
    }
  } else {
#if HAS_TANK2
    if (t2High==HIGH) humidOff(); else humidOn();
#else
    humidOn();
#endif

    if (manualDrainMode){
      if (!pumpRunning){ lastSwitchMs = millis() - MIN_DWELL_MS; applyPumpState(true); }
      if (t1Low==HIGH){ stopManualDrain('M'); }
      else if (MANUAL_MAX_MS>0 && (millis()-manualStartMs)>=MANUAL_MAX_MS){ stopManualDrain('T'); }
    } else {
      if (!pumpRunning){
        if (t1High==LOW){ applyPumpState(true); SER_PRINTLN(F("Reason: T1 HIGH -> start pump")); }
      } else {
        if (t1Low==HIGH){ pendingStopReason='B'; applyPumpState(false); SER_PRINTLN(F("Reason: T1 LOW -> stop pump")); }
      }
    }

    /***** Rolling 1Hz average while pumping *****/
    if (pumpRunning && flowAvgActive){
      uint32_t now=millis();
      if (now>=flowStartAfterMs && (now-lastSampleMs)>=SAMPLE_MS){
        lastSampleMs += SAMPLE_MS;
        uint16_t mv = (uint16_t)readShunt_mV();
        shuntBuf[shuntHead]=mv; shuntHead=(shuntHead+1)%AVG_WINDOW; if(shuntCount<AVG_WINDOW) shuntCount++;

        if (shuntCount>=AVG_WINDOW){
          uint32_t sum=0; for(uint8_t i=0;i<AVG_WINDOW;i++) sum+=shuntBuf[i];
          float avg_mV = sum/(float)AVG_WINDOW;

          SER_PRINT(F("Rolling avg(5): ")); SER_PRINT(avg_mV,1);
          SER_PRINT(F(" mV | latest=")); SER_PRINTLN(mv);

          if (avg_mV >= FLOW_BAD_mV){
            SER_PRINTLN(F("Frozen/abnormal flow detected by rolling average."));
            (void)routeFrozenToTank2_UntilT1Empty(); // do not emergency-trip
            flowAvgActive = false; // let normal logic continue in next loop pass
          } else if (manualDrainMode && manualVerifyPending){
            manualVerifyPending=false; SER_PRINTLN(F("Manual DRAIN: verification PASSED."));
          }
        } else {
          SER_PRINT(F("Collecting (" )); SER_PRINT(shuntCount); SER_PRINTLN(F("/5)..."));
        }
      }
    }

#if HAS_TANK2
    if (!faultLatched && !pumpRunning && t2DrainPending) {
      SER_PRINTLN(F("Executing scheduled Tank-2 drain..."));
      valve4_set(false);
      valve8_set(true);
      delay(50);

      lastSwitchMs = millis() - MIN_DWELL_MS;
      applyPumpState(true);

      uint32_t cap = millis() + 5UL*60UL*1000UL;
      while (millis() < cap) {
        if (t2LowF.read(FLOAT_FILTER_MS) == HIGH) break; // empty
        if (faultLatched) break;
        delay(10);
      }

      pendingStopReason = 't';
      applyPumpState(false);
      valve8_set(false);
      valve4_openTimed(5000);
      SER_PRINTLN(F("Tank-2 drain complete. Post-drain main flush done."));
      t2DrainPending = false;
    }
#endif
  }

  // Valve-4 auto-close
  valve4_autoCloseTick();

  // Debug every 5s
  static uint32_t lastDbg=0;
  if (millis()-lastDbg>=5000){
    lastDbg=millis();
    SER_PRINT(F("Pump=")); SER_PRINT(pumpRunning?F("ON"):F("OFF"));
    SER_PRINT(F("  Fault=")); SER_PRINT(faultLatched?F("YES"):F("NO"));
    uint32_t mv=readShunt_mV(); SER_PRINT(F("  Vshunt=")); SER_PRINT(mv); SER_PRINT(F(" mV"));
    SER_PRINT(F("  V4=")); SER_PRINT(valve4Open?F("OPEN"):F("CLOSED"));
#if HAS_TANK2
    SER_PRINT(F("  T2low=")); SER_PRINT(digitalRead(PIN_T2_LOW)==LOW?F("WET"):F("DRY"));
    SER_PRINT(F("  T2high=")); SER_PRINTLN(digitalRead(PIN_T2_HIGH)==HIGH?F("FULL"):F("OK"));
#else
    SER_PRINTLN();
#endif
  }

  delay(5);
}
