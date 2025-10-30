#include <Arduino.h>
#include <EEPROM.h>

/* ======= BUILD-TIME SERIAL SWITCH ======= */
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

/* ======= PINS (Nano/UNO) ======= */
const int FLOAT_TOP_PIN    = 2;   // external pull-up to +5V; switch to GND
const int FLOAT_BOTTOM_PIN = 3;   // external pull-up to +5V; switch to GND
const int RELAY_PUMP_PIN   = 8;   // pump relay IN/driver
const int RELAY_HUMID_PIN  = 9;   // humidifier relay IN/driver

// >>> Relay polarity
const bool RELAY_PUMP_ACTIVE_LOW  = false;  // false=active-HIGH, true=active-LOW
const bool RELAY_HUMID_ACTIVE_LOW = true;   // active-LOW module

/* ======= FAULT LED ======= */
const int  FAULT_LED_PIN          = 6;
const bool FAULT_LED_ACTIVE_HIGH  = true;
inline void setFaultLED(bool on) {
  digitalWrite(FAULT_LED_PIN, FAULT_LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
}

/* ======= MOTOR RUN LED (D13) ======= */
const int  MOTOR_LED_PIN         = LED_BUILTIN; // D13 on Nano/UNO
const bool MOTOR_LED_ACTIVE_HIGH = true;
inline void setMotorLED(bool on) {
  digitalWrite(MOTOR_LED_PIN, MOTOR_LED_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                                   : (on ? LOW  : HIGH));
}

/* ======= VALVE CONTROL (12V NC valve via relay; LED shows OPEN state) ======= */
const int  VALVE_RELAY_PIN         = 5;       // drives valve relay IN
const bool VALVE_ACTIVE_LOW        = false;   // true if relay board LOW-trigger
const int  VALVE_LED_PIN           = 7;       // valve-status LED; set -1 to disable
const bool VALVE_LED_ACTIVE_HIGH   = true;
const uint32_t VALVE_OPEN_PULSE_MS = 30000UL; // 30 s valve open after pump stops

bool     valveIsOpen      = false;
uint32_t valveOpenUntilMs = 0;

inline void setRelayLevel_raw(int pin, bool on, bool activeLow) {
  digitalWrite(pin, activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}
inline void setValveLED(bool valveOpen) {
  if (VALVE_LED_PIN < 0) return;
  digitalWrite(VALVE_LED_PIN,
               VALVE_LED_ACTIVE_HIGH ? (valveOpen ? HIGH : LOW)
                                     : (valveOpen ? LOW  : HIGH));
}
inline void setValveCoil(bool energize) { setRelayLevel_raw(VALVE_RELAY_PIN, energize, VALVE_ACTIVE_LOW); }
inline void valveOpen()  { setValveCoil(true);  valveIsOpen = true;  setValveLED(true); }
inline void valveClose() { setValveCoil(false); valveIsOpen = false; setValveLED(false); }

/* ======= ADC (shunt on Node X) ======= */
const int   SHUNT_ADC_PIN        = A0;     // Node X (pump side of 1Ω)
const bool  USE_INTERNAL_ADC_REF = true;   // internal ref for small voltages
const float VREF_mV_CAL          = 1100.0; // measure & adjust (typ. 1040–1090 mV)

/* ======= NOISE / TIMING GUARDS ======= */
const uint16_t FLOAT_FILTER_MS   = 150;    // debounce hold time
const uint32_t MIN_DWELL_MS      = 3000;   // anti-chatter ON/OFF min dwell
const uint32_t FAULT_BLANK_MS    = 700;    // ignore inrush before sampling

/* ======= NEW: AUTO-CLEAR WINDOW FOR FAULT LATCH ======= */
const uint32_t FAULT_AUTO_CLEAR_MS = 6UL * 60UL * 60UL * 1000UL; // 24 hours
uint32_t faultLatchedAtMs = 0;  // timestamp when fault latched (millis)

/* ======= FLOW CHECK (rolling average of last 5 samples @1 Hz) ======= */
const uint16_t FLOW_BAD_mV = 500;          // trip if 5-sample avg >= threshold
const uint8_t  AVG_WINDOW  = 5;            // rolling window length (seconds)
const uint32_t SAMPLE_MS   = 1000UL;       // sample period (1 Hz)

/* ======= EEPROM ADDRS ======= */
const int EE_ADDR_FAULT      = 0;            // 1 byte
const uint8_t EE_FAULT_CLEAR = 0x00;
const uint8_t EE_FAULT_SET   = 0xF1;
const int EE_ADDR_TOTAL_SEC  = 1;            // 4 bytes at 1..4 (lifetime ON seconds)
const int EE_ADDR_BOOT_COUNT = 10;           // 4 bytes at 10..13 (power-up counter)

/* ======= SESSION LOG (per power cycle) ======= */
const int EE_ADDR_SESSION_BASE = 200;
const uint32_t SESSION_MAGIC   = 0xC0FFEE51;
const uint8_t  MAX_SESSIONS    = 30;

struct SessionHeader { uint32_t magic; uint8_t head; uint8_t count; uint16_t rsv; };
struct SessionEntry  { uint32_t bootNo; uint32_t runSec; uint16_t starts; };

SessionHeader sessHdr;
uint8_t sessIndexCurrent = 0xFF;

struct SessionStore {
  static int entryAddr(uint8_t idx) {
    return EE_ADDR_SESSION_BASE + sizeof(SessionHeader) + idx * sizeof(SessionEntry);
  }
  static void readHeader() {
    EEPROM.get(EE_ADDR_SESSION_BASE, sessHdr);
    if (sessHdr.magic != SESSION_MAGIC || sessHdr.head >= MAX_SESSIONS || sessHdr.count > MAX_SESSIONS) {
      sessHdr.magic = SESSION_MAGIC; sessHdr.head = 0; sessHdr.count = 0; sessHdr.rsv = 0;
      EEPROM.put(EE_ADDR_SESSION_BASE, sessHdr);
    }
  }
  static void writeHeader() { EEPROM.put(EE_ADDR_SESSION_BASE, sessHdr); }
  static void writeAt(uint8_t idx, const SessionEntry &e) { EEPROM.put(entryAddr(idx), e); }
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
    e.starts += delta;
    writeAt(sessIndexCurrent, e);
  }
  static void addRunSec(uint32_t addSec) {
    if (sessIndexCurrent == 0xFF) return;
    SessionEntry e; readAt(sessIndexCurrent, e);
    e.runSec += addSec;
    writeAt(sessIndexCurrent, e);
  }
};

/* ======= OPTIONAL MANUAL CLEAR BUTTON ======= */
const bool ENABLE_CLEAR_BTN = false;
const int  CLEAR_BTN_PIN    = 4;            // ext pull-up to +5V; button to GND

/* ======= MANUAL DRAIN BUTTON (uses internal pull-up; wire D10 ↔ GND) ======= */
const int  MANUAL_BTN_PIN        = 10;
const bool MANUAL_BTN_ACTIVE_LOW = true;
const uint32_t MANUAL_MAX_MS     = 1UL * 60UL * 1000UL; // 1 minute hard timeout

/* ======= OPTIONAL CALIBRATION (seconds -> liters) ======= */
const float LITERS_PER_SEC_CAL = 0.0f;      // set after calibration (e.g., 0.0333 for 2 L/min)
float     litersToday   = 0.0f;
uint32_t  secondsToday  = 0;

/* ======= STATE ======= */
bool pumpRunning  = false;
bool faultLatched = false;
bool manualDrainMode = false;
bool manualVerifyPending = false;
uint32_t manualStartMs = 0;

uint32_t lastSwitchMs = 0;
uint32_t pumpOnAtMs   = 0;

/* ======= RUNTIME METRICS ======= */
uint32_t lastRunMs   = 0;               // duration of most recent ON cycle (ms)
uint32_t totalRunSec = 0;               // lifetime accumulated ON-time (sec)
inline void saveTotalRunToEEPROM() {
  static uint32_t lastSaved = 0xFFFFFFFF;
  if (totalRunSec != lastSaved) { EEPROM.put(EE_ADDR_TOTAL_SEC, totalRunSec); lastSaved = totalRunSec; }
}

/* ======= POWER CYCLE COUNTER ======= */
uint32_t bootCount = 0;
inline void loadBootCount()  { EEPROM.get(EE_ADDR_BOOT_COUNT, bootCount); }
inline void saveBootCount()  { EEPROM.put(EE_ADDR_BOOT_COUNT, bootCount); }

/* ======= RUN HISTORY (ring buffer for current boot only) ======= */
struct RunLog { uint32_t start_ms, dur_ms; char reason; };
const uint8_t MAX_RUN_LOGS = 40;
RunLog logs[MAX_RUN_LOGS];
uint16_t logsCount = 0, logsHead = 0;
uint32_t totalOns = 0, totalOffs = 0;
char pendingStopReason = 'U';

/* ======= ROLLING 5-SAMPLE SHUNT AVERAGE (1 Hz) ======= */
bool     flowAvgActive      = false;     // collecting 1 Hz samples now?
uint32_t flowStartAfterMs   = 0;         // time sampling is allowed to start (after inrush blank)
uint16_t shuntBuf[AVG_WINDOW];
uint8_t  shuntCount         = 0;         // how many valid samples currently in buffer (0..5)
uint8_t  shuntHead          = 0;         // next index to write (circular)
uint32_t lastSampleMs       = 0;         // last 1 Hz sample timestamp

/* ======= HELPERS ======= */
inline void setRelayLevel(int pin, bool on, bool activeLow) {
  digitalWrite(pin, (activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW)));
}
inline void relayPumpOn()        { setRelayLevel(RELAY_PUMP_PIN,  true,  RELAY_PUMP_ACTIVE_LOW); }
inline void relayPumpOff()       { setRelayLevel(RELAY_PUMP_PIN,  false, RELAY_PUMP_ACTIVE_LOW); }
inline void setHumid(bool on)    { setRelayLevel(RELAY_HUMID_PIN, on,    RELAY_HUMID_ACTIVE_LOW); }

/* ======= Debounce with optional INPUT_PULLUP ======= */
struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p, bool usePullup=false){
    pin = p;
    pinMode(pin, usePullup ? INPUT_PULLUP : INPUT);
    stable = candidate = digitalRead(pin);
    tChange = millis();
  }
  int read(uint16_t hold_ms){
    int raw = digitalRead(pin);
    if (raw != candidate){ candidate = raw; tChange = millis(); }
    if ((millis() - tChange) >= hold_ms && stable != candidate) stable = candidate;
    return stable;
  }
};
Debounce topF, botF, manBtn;

/* ======= ADC read (averaged) ======= */
const uint8_t ADC_SAMPLES = 12;
uint32_t readShunt_mV() {
  static bool refInit=false;
  if (!refInit) {
    analogReference(USE_INTERNAL_ADC_REF ? INTERNAL : DEFAULT);
    delay(5);
    (void)analogRead(SHUNT_ADC_PIN); // dummy
    refInit=true;
  }
  uint32_t acc=0;
  for (uint8_t i=0;i<ADC_SAMPLES;i++){
    acc += analogRead(SHUNT_ADC_PIN);
    delayMicroseconds(200);
  }
  const float Vref_mV = USE_INTERNAL_ADC_REF ? VREF_mV_CAL : 5000.0f;
  return (uint32_t)((acc/(float)ADC_SAMPLES) * Vref_mV / 1023.0f + 0.5f);
}

/* ======= EEPROM fault flag ======= */
void saveFaultToEEPROM(bool set){
  uint8_t want = set ? EE_FAULT_SET : EE_FAULT_CLEAR;
  if (EEPROM.read(EE_ADDR_FAULT) != want) EEPROM.update(EE_ADDR_FAULT, want);
}
bool faultStoredInEEPROM(){ return EEPROM.read(EE_ADDR_FAULT) == EE_FAULT_SET; }

/* ======= LOGGING ======= */
void logRun(uint32_t start_ms, uint32_t dur_ms, char reason) {
  logs[logsHead] = RunLog{start_ms, dur_ms, reason};
  logsHead = (logsHead + 1) % MAX_RUN_LOGS;
  if (logsCount < MAX_RUN_LOGS) logsCount++;
  totalOffs++;
}

/* ======= Emergency fault helper ======= */
void tripFaultInstant(const char* why) {
  pendingStopReason = 'F';
  // Stop pump (also turns off motor LED in applyPumpState(false))
  applyPumpState(false);
  faultLatched = true;
  saveFaultToEEPROM(true);
  setFaultLED(true);
  valveClose();
  valveOpenUntilMs = 0;
  flowAvgActive = false;
  manualDrainMode = false;       // cancel manual mode if active
  faultLatchedAtMs = millis();   // <<< NEW: start 24h auto-clear timer
  SER_PRINT(F("EMERGENCY: ")); SER_PRINTLN(why);
}

/* ======= PUMP STATE APPLY (with valve & LEDs) ======= */
void applyPumpState(bool on) {
  if (on == pumpRunning) return;
  if (millis() - lastSwitchMs < MIN_DWELL_MS) return; // dwell guard

  if (on) {
    // ----- PUMP ON -----
    relayPumpOn();
    pumpOnAtMs = millis();
    totalOns++;
    SessionStore::addStarts(1);

    // Keep water out while pumping
    valveClose();                // Valve LED OFF
    valveOpenUntilMs = 0;

    // Start rolling 1 Hz sampling after inrush blank
    flowAvgActive    = true;
    flowStartAfterMs = pumpOnAtMs + FAULT_BLANK_MS;
    shuntCount = 0;
    shuntHead  = 0;
    lastSampleMs = flowStartAfterMs; // first 1 Hz tick happens ~1s after blank

    // Motor LED ON
    setMotorLED(true);

    SER_PRINTLN(F("PUMP -> ON | Valve CLOSED | Rolling 5-sample average will start after inrush."));
  } else {
    // ----- PUMP OFF -----
    if (pumpRunning) {
      uint32_t now = millis();
      lastRunMs = (now >= pumpOnAtMs) ? (now - pumpOnAtMs) : 0;
      uint32_t addSec = (lastRunMs + 999) / 1000; // ceil seconds

      // Accrue uniform counters (also used for liters)
      totalRunSec += addSec; saveTotalRunToEEPROM();
      secondsToday += addSec;
      if (LITERS_PER_SEC_CAL > 0.0f) litersToday += addSec * LITERS_PER_SEC_CAL;
      SessionStore::addRunSec(addSec);

      logRun(pumpOnAtMs, lastRunMs, pendingStopReason); pendingStopReason='U';

      SER_PRINT(F("PUMP -> OFF. This run: "));
      SER_PRINT(lastRunMs / 1000.0, 3);
      SER_PRINT(F(" s  |  Lifetime: "));
      SER_PRINT(totalRunSec);
      SER_PRINTLN(F(" s"));
    }
    relayPumpOff();

    // Motor LED OFF
    setMotorLED(false);

    // Post-stop valve open pulse
    if (VALVE_OPEN_PULSE_MS > 0) {
      valveOpen();
      valveOpenUntilMs = millis() + VALVE_OPEN_PULSE_MS;
      SER_PRINTLN(F("Valve OPEN for post-stop pulse."));
    } else {
      valveClose();
      valveOpenUntilMs = 0;
      SER_PRINTLN(F("Valve CLOSED (no post-stop pulse)."));
    }

    // stop rolling average
    flowAvgActive = false;
  }
  pumpRunning = on;
  lastSwitchMs = millis();
}

/* ======= Manual drain helpers ======= */
void startManualDrain() {
  if (faultLatched) { SER_PRINTLN(F("Manual DRAIN ignored: fault latched.")); return; }

  manualDrainMode = true;
  manualVerifyPending = true;
  manualStartMs = millis();

  // If pump is already running, restart the verification window without power-cycling
  if (pumpRunning) {
    flowAvgActive    = true;
    flowStartAfterMs = millis() + FAULT_BLANK_MS;
    shuntCount = 0; shuntHead = 0;
    lastSampleMs = flowStartAfterMs;
    SER_PRINTLN(F("Manual DRAIN: verification window restarted while running."));
  } else {
    // bypass dwell and turn on
    lastSwitchMs = millis() - MIN_DWELL_MS;
    applyPumpState(true);
  }

  SER_PRINTLN(F("Manual DRAIN: started (will run until bottom float HIGH or timeout)."));
}

void stopManualDrain(char reasonCode) {
  manualDrainMode = false;
  pendingStopReason = reasonCode; // 'M' = manual empty, 'T' = timeout
  applyPumpState(false);
  SER_PRINT(F("Manual DRAIN: stop (reason="));
  SER_PRINT(reasonCode);
  SER_PRINTLN(F(")."));
}

/* ======= SETUP ======= */
void setup() {
  SER_BEGIN(115200);

  // Load counters
  EEPROM.get(EE_ADDR_TOTAL_SEC, totalRunSec);
  loadBootCount();

  // Session init
  SessionStore::readHeader();
  bootCount++;
  saveBootCount();
  int exist = SessionStore::findByBoot(bootCount);
  if (exist >= 0) sessIndexCurrent = (uint8_t)exist;
  else            sessIndexCurrent = SessionStore::startNew(bootCount);

  SER_PRINT(F("Boot count: ")); SER_PRINTLN(bootCount);
  SER_PRINT(F("Lifetime pump ON time (EEPROM): ")); SER_PRINT(totalRunSec); SER_PRINTLN(F(" s"));

  topF.begin(FLOAT_TOP_PIN);              // external pull-up on floats
  botF.begin(FLOAT_BOTTOM_PIN);
  manBtn.begin(MANUAL_BTN_PIN, true);     // **internal pull-up enabled** (button D10 ↔ GND)

  pinMode(RELAY_PUMP_PIN,  OUTPUT);
  pinMode(RELAY_HUMID_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);

  pinMode(MOTOR_LED_PIN, OUTPUT);
  setMotorLED(false);

  if (ENABLE_CLEAR_BTN) pinMode(CLEAR_BTN_PIN, INPUT);

  pinMode(VALVE_RELAY_PIN, OUTPUT);
  if (VALVE_LED_PIN >= 0) pinMode(VALVE_LED_PIN, OUTPUT);
  valveClose();

  relayPumpOff();
  setHumid(true);
  setFaultLED(false);

  // Clear any stored fault at boot (and reset timer)
  saveFaultToEEPROM(false);
  faultLatched = false;
  faultLatchedAtMs = 0;

  SER_PRINTLN(F("\nPower-up: fault cleared."));
  SER_PRINTLN(F("Serial cmds: 'C' clear fault, 'R' report, 'H' run history, 'X' clear run history, 'B' boot info, 'Z' reset boot count, 'S' session history, 'M' manual drain"));

  // Startup drain ONLY if water present (bottom float LOW)
  delay(FLOAT_FILTER_MS);
  int bottomNow = botF.read(FLOAT_FILTER_MS);
  if (!faultLatched && bottomNow == LOW) {
    SER_PRINTLN(F("Startup drain: water detected. Pump ON until bottom float goes HIGH."));
    lastSwitchMs = millis() - MIN_DWELL_MS; // bypass dwell
    applyPumpState(true);
  } else {
    SER_PRINTLN(F("Startup drain skipped: no water or fault latched."));
  }
}

/* ======= LOOP ======= */
void loop() {
  // Serial commands (trimmed)
  if (SER_AVAILABLE()){
    char c = SER_READ();
    switch (c) {
      case 'C': 
        faultLatched=false; 
        saveFaultToEEPROM(false); 
        setFaultLED(false); 
        faultLatchedAtMs = 0;      // <<< NEW: reset timer on manual clear
        SER_PRINTLN(F("Fault CLEARED via Serial.")); 
        break;
      case 'R': {
        SER_PRINT(F("Pump running: ")); SER_PRINTLN(pumpRunning ? F("YES") : F("NO"));
        if (pumpRunning) { uint32_t curMs = millis() - pumpOnAtMs; SER_PRINT(F("Current run so far: ")); SER_PRINT(curMs / 1000.0, 3); SER_PRINTLN(F(" s")); }
        SER_PRINT(F("Last completed run: ")); SER_PRINT(lastRunMs / 1000.0, 3); SER_PRINTLN(F(" s"));
        SER_PRINT(F("Lifetime ON time: ")); SER_PRINT(totalRunSec); SER_PRINTLN(F(" s"));
        if (LITERS_PER_SEC_CAL > 0.0f) { SER_PRINT(F("Today: ")); SER_PRINT(secondsToday); SER_PRINT(F(" s, ")); SER_PRINT(litersToday, 3); SER_PRINTLN(F(" L")); }
        if (sessIndexCurrent != 0xFF) { SessionEntry e; SessionStore::readAt(sessIndexCurrent, e);
          SER_PRINT(F("This power cycle (boot #")); SER_PRINT(e.bootNo); SER_PRINT(F("): starts="));
          SER_PRINT(e.starts); SER_PRINT(F(", runSec=")); SER_PRINT(e.runSec); SER_PRINTLN(F(" s"));
        }
        if (faultLatched) {
          uint32_t elapsed = millis() - faultLatchedAtMs;
          uint32_t remain  = (elapsed >= FAULT_AUTO_CLEAR_MS) ? 0 : (FAULT_AUTO_CLEAR_MS - elapsed);
          SER_PRINT(F("Fault auto-clear in ~ ")); SER_PRINT(remain/1000UL); SER_PRINTLN(F(" s"));
        }
      } break;
      case 'H': {
        SER_PRINT(F("\n=== RUN HISTORY (latest ")); SER_PRINT(logsCount); SER_PRINTLN(F(") ==="));
        SER_PRINTLN(F("#\tStart[s]\tDur[s]\tReason"));
        uint16_t first = (logsHead + MAX_RUN_LOGS - logsCount) % MAX_RUN_LOGS;
        for (uint16_t i=0;i<logsCount; i++){ uint16_t idx=(first+i)%MAX_RUN_LOGS;
          SER_PRINT(i+1); SER_PRINT('\t');
          SER_PRINT(logs[idx].start_ms/1000.0,3); SER_PRINT('\t');
          SER_PRINT(logs[idx].dur_ms/1000.0,3);   SER_PRINT('\t');
          SER_PRINTLN(logs[idx].reason);
          delay(2);
        }
        SER_PRINTLN(F("=== END RUN HISTORY ===\n"));
      } break;
      case 'X': logsCount=0; logsHead=0; totalOns=0; totalOffs=0; SER_PRINTLN(F("Run history cleared (RAM counters reset).")); break;
      case 'B': { SER_PRINT(F("Boot count: ")); SER_PRINTLN(bootCount);
                  SER_PRINT(F("Uptime (s): ")); SER_PRINTLN(millis()/1000.0,3); } break;
      case 'Z': bootCount=0; saveBootCount(); SER_PRINTLN(F("Boot counter reset to 0.")); break;
      case 'S': {
        SessionStore::readHeader();
        SER_PRINT(F("\n=== SESSION HISTORY (last ")); SER_PRINT(sessHdr.count); SER_PRINTLN(F(" power cycles) ==="));
        uint8_t first = (sessHdr.head + MAX_SESSIONS - sessHdr.count) % MAX_SESSIONS;
        for (uint8_t i=0;i<sessHdr.count;i++){ uint8_t idx=(first+i)%MAX_SESSIONS; SessionEntry e; SessionStore::readAt(idx,e);
          SER_PRINTLN(F("----------------------------------------"));
          SER_PRINT(F("Power cycle #")); SER_PRINTLN(e.bootNo);
          SER_PRINT(F("  Starts: ")); SER_PRINTLN(e.starts);
          SER_PRINT(F("  Total run: "));
          uint32_t h=e.runSec/3600UL, m=(e.runSec/60UL)%60UL, s=e.runSec%60UL;
          SER_PRINT(h); SER_PRINT(':'); if(m<10)SER_PRINT('0'); SER_PRINT(m); SER_PRINT(':'); if(s<10)SER_PRINT('0'); SER_PRINTLN(s);
        }
        SER_PRINTLN(F("----------------------------------------"));
        SER_PRINTLN(F("=== END SESSION HISTORY ===\n"));
      } break;
      case 'M': startManualDrain(); break;  // optional Serial trigger
      default: break;
    }
  }

  // Debounced inputs
  int top    = topF.read(FLOAT_FILTER_MS);
  int bottom = botF.read(FLOAT_FILTER_MS);
  int man    = manBtn.read(FLOAT_FILTER_MS);

  // Rising edge on manual button -> start manual drain
  static int lastMan = HIGH;
  bool pressedEdge = MANUAL_BTN_ACTIVE_LOW ? (lastMan==HIGH && man==LOW) : (lastMan==LOW && man==HIGH);
  lastMan = man;
  if (pressedEdge) startManualDrain();

  // Fault path
  if (faultLatched) {
    if (pumpRunning) { pendingStopReason='F'; applyPumpState(false); }
    setHumid(false);
    setFaultLED(true);
    setMotorLED(false);
    valveClose();
    valveOpenUntilMs = 0;

    // auto-clear after 24 hours of continuous time since latch
    if (faultLatchedAtMs != 0 && (millis() - faultLatchedAtMs) >= FAULT_AUTO_CLEAR_MS) {
      faultLatched = false;
      saveFaultToEEPROM(false);
      setFaultLED(false);
      faultLatchedAtMs = 0;
      SER_PRINTLN(F("Fault auto-cleared after 24 hours."));
    }

  } else {
    // ----- Manual drain overrides normal control -----
    if (manualDrainMode) {
      // force/keep pump ON
      if (!pumpRunning) {
        lastSwitchMs = millis() - MIN_DWELL_MS; // bypass dwell to honor manual command
        applyPumpState(true);
      }
      // stop conditions: empty or timeout
      if (bottom == HIGH) {
        stopManualDrain('M'); // emptied
      } else if (MANUAL_MAX_MS > 0 && (millis() - manualStartMs) >= MANUAL_MAX_MS) {
        stopManualDrain('T'); // timeout
      }
    } else {
      // ----- Normal control -----
      if (!pumpRunning) {
        if (top == LOW) { applyPumpState(true); SER_PRINTLN(F("Reason: TOP UP -> start pump")); }
      } else {
        if (bottom == HIGH) { pendingStopReason='B'; applyPumpState(false); SER_PRINTLN(F("Reason: BOTTOM DOWN -> stop pump")); }
      }
    }

    /* ======= Continuous rolling 5-sample average (1 Hz) ======= */
    if (pumpRunning && flowAvgActive) {
      uint32_t now = millis();

      // Only start sampling after the inrush blank
      if (now >= flowStartAfterMs) {
        if ((now - lastSampleMs) >= SAMPLE_MS) {
          lastSampleMs += SAMPLE_MS;

          // Take a 1 Hz shunt sample
          uint16_t mv = (uint16_t)readShunt_mV();

          // Write into circular buffer
          shuntBuf[shuntHead] = mv;
          shuntHead = (uint8_t)((shuntHead + 1) % AVG_WINDOW);
          if (shuntCount < AVG_WINDOW) {
            shuntCount++;
          }

          // Once we have 5 samples, compute rolling average every second
          if (shuntCount >= AVG_WINDOW) {
            uint32_t sum = 0;
            for (uint8_t i = 0; i < AVG_WINDOW; i++) sum += shuntBuf[i];
            float avg_mV = sum / (float)AVG_WINDOW;

            SER_PRINT(F("Rolling avg(5): ")); SER_PRINT(avg_mV, 1);
            SER_PRINT(F(" mV  | latest=")); SER_PRINT(mv); SER_PRINTLN(F(" mV"));

            if (avg_mV >= FLOW_BAD_mV) {
              tripFaultInstant("Rolling 5-sample average >= threshold.");
            } else if (manualDrainMode && manualVerifyPending) {
              // verification passed the first 5-sample gate
              manualVerifyPending = false;
              SER_PRINTLN(F("Manual DRAIN: verification PASSED."));
            }
          } else {
            SER_PRINT(F("Collecting (")); SER_PRINT(shuntCount); SER_PRINTLN(F("/5)..."));
          }
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
    SER_PRINTLN(F("Valve auto-closed after pulse window."));
  }

  // Debug (every 5 s)
  static uint32_t lastDbg=0;
  if (millis() - lastDbg >= 5000) {
    lastDbg = millis();
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
    if (LITERS_PER_SEC_CAL > 0.0f) {
      SER_PRINT(F("  Today=")); SER_PRINT(secondsToday); SER_PRINT(F(" s, ")); SER_PRINT(litersToday, 3); SER_PRINT(F(" L"));
    }
    SER_PRINTLN();
  }

  delay(5);
}
