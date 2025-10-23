#include <Arduino.h>
#include <EEPROM.h>

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
const bool FAULT_LED_ACTIVE_HIGH  = true;  // per your last message
inline void setFaultLED(bool on) {
  digitalWrite(FAULT_LED_PIN, FAULT_LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
}

/* ======= MOTOR RUN LED (moved off pin 9 to avoid conflict) ======= */
const int  MOTOR_LED_PIN         = LED_BUILTIN; // D13 on Nano
const bool MOTOR_LED_ACTIVE_HIGH = true;        // set false if your LED is active-LOW
inline void setMotorLED(bool on) {
  digitalWrite(MOTOR_LED_PIN, MOTOR_LED_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                                   : (on ? LOW  : HIGH));
}

/* ======= VALVE CONTROL (12V NC valve via relay; LED shows OPEN state) ======= */
/* NC valve: de-energized = CLOSED (default); energized = OPEN */
const int  VALVE_RELAY_PIN         = 5;       // drives valve relay IN
const bool VALVE_ACTIVE_LOW        = false;   // true if relay board is LOW-trigger (LOW=ON)
const int  VALVE_LED_PIN           = 7;       // valve-status LED; set -1 to disable
const bool VALVE_LED_ACTIVE_HIGH   = true;    // true: LED HIGH = ON; false: LED LOW = ON
const uint32_t VALVE_OPEN_PULSE_MS = 30000UL; // 30 s valve open after pump stops (0 = disabled)

bool     valveIsOpen      = false;            // logical valve state (OPEN/CLOSED)
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
const int   SHUNT_ADC_PIN     = A0;           // Node X (pump side of 1Ω)
const bool  USE_INTERNAL_ADC_REF = true;      // use internal ref for small voltages
const float VREF_mV_CAL       = 1100.0;       // <-- measure & adjust (typ. 1040–1090 mV)

/* ======= NOISE / TIMING GUARDS ======= */
const uint16_t FLOAT_FILTER_MS   = 150;     // must be stable this long
const uint32_t MIN_DWELL_MS      = 3000;    // min ON/OFF runtime (anti-chatter)
const uint32_t FAULT_BLANK_MS    = 700;     // ignore inrush for sensing

/* ======= FLOW VERIFY ======= */
const uint16_t FLOW_BAD_mV    = 400;        // INSTANT TRIP if any sample >= 400 mV
const uint32_t FLOW_VERIFY_MS = 5000;       // also check average over 5 s
const uint8_t  ADC_SAMPLES    = 12;         // per-read averaging

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

/* ======= STATE ======= */
bool pumpRunning  = false;
bool faultLatched = false;
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

/* ======= FLOW VERIFY STATE (average mode) ======= */
bool     flowCheckActive    = false;
uint32_t flowWindowStartMs  = 0;      // = pumpOnAtMs + FAULT_BLANK_MS
uint32_t flowSum_mV         = 0;
uint16_t flowSamples        = 0;
uint16_t flowMin_mV         = 65535;  // kept for info/debug

/* ======= HELPERS ======= */
inline void setRelayLevel(int pin, bool on, bool activeLow) {
  digitalWrite(pin, (activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW)));
}
inline void relayPumpOn()        { setRelayLevel(RELAY_PUMP_PIN,  true,  RELAY_PUMP_ACTIVE_LOW); }
inline void relayPumpOff()       { setRelayLevel(RELAY_PUMP_PIN,  false, RELAY_PUMP_ACTIVE_LOW); }
inline void setHumid(bool on)    { setRelayLevel(RELAY_HUMID_PIN, on,    RELAY_HUMID_ACTIVE_LOW); }

struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p){ pin=p; pinMode(pin, INPUT); stable=candidate=digitalRead(pin); tChange=millis(); }
  int read(uint16_t hold_ms){
    int raw=digitalRead(pin);
    if (raw!=candidate){ candidate=raw; tChange=millis(); }
    if ((millis()-tChange)>=hold_ms && stable!=candidate) stable=candidate;
    return stable;
  }
};
Debounce topF, botF;

/* ======= ADC read (averaged) ======= */
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

/* ======= Instant fault helper ======= */
void tripFaultInstant(const char* why) {
  pendingStopReason = 'F';
  applyPumpState(false);
  faultLatched = true;
  saveFaultToEEPROM(true);
  setFaultLED(true);
  valveClose();
  valveOpenUntilMs = 0;
  flowCheckActive = false;
  Serial.print(F("EMERGENCY: "));
  Serial.println(why);
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

    // Start 5 s flow verify after inrush
    flowCheckActive   = true;
    flowWindowStartMs = pumpOnAtMs + FAULT_BLANK_MS;
    flowSum_mV        = 0;
    flowSamples       = 0;
    flowMin_mV        = 65535;

    // Motor LED ON
    setMotorLED(true);

    Serial.println(F("PUMP -> ON (latched) | Valve CLOSED (LED OFF) | Starting 5 s flow verification after inrush."));
  } else {
    // ----- PUMP OFF -----
    if (pumpRunning) {
      uint32_t now = millis();
      lastRunMs = (now >= pumpOnAtMs) ? (now - pumpOnAtMs) : 0;
      uint32_t addSec = (lastRunMs + 999) / 1000; // ceil seconds
      totalRunSec += addSec; saveTotalRunToEEPROM();
      SessionStore::addRunSec(addSec);

      logRun(pumpOnAtMs, lastRunMs, pendingStopReason); pendingStopReason='U';

      Serial.print(F("PUMP -> OFF (latched). This run: "));
      Serial.print(lastRunMs / 1000.0, 3);
      Serial.print(F(" s  |  Lifetime: "));
      Serial.print(totalRunSec);
      Serial.println(F(" s"));
    }
    relayPumpOff();

    // Motor LED OFF
    setMotorLED(false);

    // Post-stop valve open pulse (LED ON while open)
    if (VALVE_OPEN_PULSE_MS > 0) {
      valveOpen();                                           // Valve LED ON
      valveOpenUntilMs = millis() + VALVE_OPEN_PULSE_MS;
      Serial.println(F("Valve OPEN (LED ON) for post-stop pulse."));
    } else {
      valveClose();                                          // Valve LED OFF
      valveOpenUntilMs = 0;
      Serial.println(F("Valve CLOSED (no post-stop pulse)."));
    }

    // cancel flow verify
    flowCheckActive = false;
  }
  pumpRunning = on;
  lastSwitchMs = millis();
}

/* ======= SETUP ======= */
void setup() {
  Serial.begin(115200);

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

  Serial.print(F("Boot count: ")); Serial.println(bootCount);
  Serial.print(F("Lifetime pump ON time (EEPROM): "));
  Serial.print(totalRunSec); Serial.println(F(" s"));

  topF.begin(FLOAT_TOP_PIN);
  botF.begin(FLOAT_BOTTOM_PIN);

  pinMode(RELAY_PUMP_PIN,  OUTPUT);
  pinMode(RELAY_HUMID_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);

  // Motor LED pin
  pinMode(MOTOR_LED_PIN, OUTPUT);
  setMotorLED(false); // start OFF

  if (ENABLE_CLEAR_BTN) pinMode(CLEAR_BTN_PIN, INPUT);

  // Valve pins
  pinMode(VALVE_RELAY_PIN, OUTPUT);
  if (VALVE_LED_PIN >= 0) pinMode(VALVE_LED_PIN, OUTPUT);
  valveClose();  // default: keep water out (LED OFF)

  relayPumpOff();
  setHumid(true);
  setFaultLED(false);

  // Clear any stored fault at boot
  saveFaultToEEPROM(false);
  faultLatched = false;

  Serial.println(F("\nPower-up: fault cleared."));
  Serial.println(F("Serial cmds: 'C' clear fault, 'R' report, 'H' run history, 'X' clear run history, 'B' boot info, 'Z' reset boot count, 'S' session history"));

  // Startup drain ONLY if water present (bottom float LOW)
  delay(FLOAT_FILTER_MS);
  int bottomNow = botF.read(FLOAT_FILTER_MS);
  if (!faultLatched && bottomNow == LOW) {
    Serial.println(F("Startup drain: water detected. Pump ON until bottom float goes HIGH."));
    lastSwitchMs = millis() - MIN_DWELL_MS; // bypass dwell
    applyPumpState(true);
  } else {
    Serial.println(F("Startup drain skipped: no water or fault latched."));
  }
}

/* ======= LOOP ======= */
void loop() {

  if (!faultLatched) {
    uint16_t mv = readShunt_mV();   // or read_mV()
    if (mv >= 450) {
      faultLatched = true;
      setFaultLED(true);
      relayPumpOff();               // make sure pump is OFF
      setMotorLED(false);           // ensure motor LED off on direct trip
      valveClose();                 // optional
      Serial.println(F("FAULT LATCHED: A0 >= 450 mV"));
    }
  }

  // Serial commands – trimmed
  if (Serial.available()){
    char c = Serial.read();
    switch (c) {
      case 'C': faultLatched=false; saveFaultToEEPROM(false); setFaultLED(false); Serial.println(F("Fault CLEARED via Serial.")); break;
      case 'R': {
        Serial.print(F("Pump running: ")); Serial.println(pumpRunning ? F("YES") : F("NO"));
        if (pumpRunning) { uint32_t curMs = millis() - pumpOnAtMs; Serial.print(F("Current run so far: ")); Serial.print(curMs / 1000.0, 3); Serial.println(F(" s")); }
        Serial.print(F("Last completed run: ")); Serial.print(lastRunMs / 1000.0, 3); Serial.println(F(" s"));
        Serial.print(F("Lifetime ON time: ")); Serial.print(totalRunSec); Serial.println(F(" s"));
        if (sessIndexCurrent != 0xFF) { SessionEntry e; SessionStore::readAt(sessIndexCurrent, e);
          Serial.print(F("This power cycle (boot #")); Serial.print(e.bootNo); Serial.print(F("): starts="));
          Serial.print(e.starts); Serial.print(F(", runSec=")); Serial.print(e.runSec); Serial.println(F(" s"));
        }
      } break;
      case 'H': {
        Serial.print(F("\n=== RUN HISTORY (latest ")); Serial.print(logsCount); Serial.println(F(") ==="));
        Serial.println(F("#\tStart[s]\tDur[s]\tReason"));
        uint16_t first = (logsHead + MAX_RUN_LOGS - logsCount) % MAX_RUN_LOGS;
        for (uint16_t i=0;i<logsCount; i++){ uint16_t idx=(first+i)%MAX_RUN_LOGS;
          Serial.print(i+1); Serial.print('\t');
          Serial.print(logs[idx].start_ms/1000.0,3); Serial.print('\t');
          Serial.print(logs[idx].dur_ms/1000.0,3);   Serial.print('\t');
          Serial.println(logs[idx].reason);
          delay(2);
        }
        Serial.println(F("=== END RUN HISTORY ===\n"));
      } break;
      case 'X': logsCount=0; logsHead=0; totalOns=0; totalOffs=0; Serial.println(F("Run history cleared (RAM counters reset).")); break;
      case 'B': { Serial.print(F("Boot count: ")); Serial.println(bootCount);
                  Serial.print(F("Uptime (s): ")); Serial.println(millis()/1000.0,3); } break;
      case 'Z': bootCount=0; saveBootCount(); Serial.println(F("Boot counter reset to 0.")); break;
      case 'S': {
        SessionStore::readHeader();
        Serial.print(F("\n=== SESSION HISTORY (last ")); Serial.print(sessHdr.count); Serial.println(F(" power cycles) ==="));
        uint8_t first = (sessHdr.head + MAX_SESSIONS - sessHdr.count) % MAX_SESSIONS;
        for (uint8_t i=0;i<sessHdr.count;i++){ uint8_t idx=(first+i)%MAX_SESSIONS; SessionEntry e; SessionStore::readAt(idx,e);
          Serial.println(F("----------------------------------------"));
          Serial.print(F("Power cycle #")); Serial.println(e.bootNo);
          Serial.print(F("  Starts: ")); Serial.println(e.starts);
          Serial.print(F("  Total run: ")); 
          uint32_t h=e.runSec/3600UL, m=(e.runSec/60UL)%60UL, s=e.runSec%60UL;
          Serial.print(h); Serial.print(':'); if(m<10)Serial.print('0'); Serial.print(m); Serial.print(':'); if(s<10)Serial.print('0'); Serial.println(s);
        }
        Serial.println(F("----------------------------------------"));
        Serial.println(F("=== END SESSION HISTORY ===\n"));
      } break;
      default: break;
    }
  }

  // Debounced floats
  int top    = topF.read(FLOAT_FILTER_MS);
  int bottom = botF.read(FLOAT_FILTER_MS);

  // Fault path
  if (faultLatched) {
    if (pumpRunning) { pendingStopReason='F'; applyPumpState(false); } // will also turn motor LED OFF
    setHumid(false);
    setFaultLED(true);
    setMotorLED(false);              // extra safety
    valveClose();                    // LED OFF
    valveOpenUntilMs = 0;
  } else {
    // Normal control
    if (!pumpRunning) {
      if (top == LOW) { applyPumpState(true); Serial.println(F("Reason: TOP UP -> start pump")); }
    } else {
      if (bottom == HIGH) { pendingStopReason='B'; applyPumpState(false); Serial.println(F("Reason: BOTTOM DOWN -> stop pump")); }
    }

    /* ======= Flow verification window (average) + instant trip ======= */
    if (pumpRunning && flowCheckActive) {
      static uint32_t lastFlowSampleMs = 0;
      uint32_t now = millis();

      if (now >= flowWindowStartMs) {
        if (now - lastFlowSampleMs >= 10) {  // ~100 Hz sampling
          lastFlowSampleMs = now;
          uint32_t mv = readShunt_mV();

          // INSTANT TRIP during the window
          if (mv >= FLOW_BAD_mV) {
            tripFaultInstant("Instant shunt >= threshold during verify window.");
          } else {
            flowSum_mV  += mv;
            flowSamples += 1;
            if (mv < flowMin_mV) flowMin_mV = mv;
          }
        }

        if (!faultLatched && (now - flowWindowStartMs >= FLOW_VERIFY_MS)) {
          float avg_mV = flowSamples ? (float)flowSum_mV / (float)flowSamples : 0.f;
          Serial.print(F("Flow avg over 5 s: ")); Serial.print(avg_mV,1);
          Serial.print(F(" mV (min=")); Serial.print(flowMin_mV); Serial.println(F(" mV)"));

          if (avg_mV >= FLOW_BAD_mV) {
            tripFaultInstant("Avg shunt >= threshold after 5 s window.");
          }
          flowCheckActive = false;
        }
      }
    }

    // ======= Continuous instant watcher (outside verify window too) =======
    if (pumpRunning && !faultLatched && !flowCheckActive) {
      static uint32_t lastInstantMs = 0;
      uint32_t now = millis();
      if (now - lastInstantMs >= 10) {          // ~100 Hz
        lastInstantMs = now;
        uint32_t mv = readShunt_mV();
        if (mv >= FLOW_BAD_mV) {
          tripFaultInstant("Instant shunt >= threshold (continuous monitor).");
        }
      }
    }

    setHumid(!faultLatched);
    setFaultLED(faultLatched);
  }

  // Auto-close valve after the post-stop open pulse
  if (valveIsOpen && valveOpenUntilMs != 0 && millis() >= valveOpenUntilMs) {
    valveClose();                 // LED OFF
    valveOpenUntilMs = 0;
    Serial.println(F("Valve auto-closed after pulse window."));
  }

  // Debug (every 5 s)
  static uint32_t lastDbg=0;
  if (millis() - lastDbg >= 5000) {
    lastDbg = millis();
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
