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
const bool FAULT_LED_ACTIVE_HIGH  = true;
inline void setFaultLED(bool on) {
  digitalWrite(FAULT_LED_PIN, FAULT_LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
}

/* ======= VALVE CONTROL (12V NC valve via relay; LED mirror) ======= */
// NC valve: de-energized = CLOSED (default); energized = OPEN
const int  VALVE_RELAY_PIN        = 5;       // <--- set to your free pin that drives valve relay IN
const bool VALVE_ACTIVE_LOW       = false;   // true if your relay board is low-trigger (LOW=ON)
const int  VALVE_LED_PIN          = 7;       // LED mirror on breadboard; set -1 to disable
const bool VALVE_LED_ACTIVE_HIGH  = true;    // false if LED ON when pin LOW
const uint32_t VALVE_OPEN_PULSE_MS = 60000UL; // 60 s valve open after pump stops (set 0 to disable)
bool     valveIsOpen       = false;          // true when coil energized (valve OPEN)
uint32_t valveOpenUntilMs  = 0;

inline void setValveCoil(bool energize) {
  digitalWrite(VALVE_RELAY_PIN,
               VALVE_ACTIVE_LOW ? (energize ? LOW : HIGH)
                                : (energize ? HIGH : LOW));
  if (VALVE_LED_PIN >= 0) {
    digitalWrite(VALVE_LED_PIN,
                 VALVE_LED_ACTIVE_HIGH ? (energize ? HIGH : LOW)
                                       : (energize ? LOW  : HIGH));
  }
}
inline void valveOpen()  { setValveCoil(true);  valveIsOpen = true; }
inline void valveClose() { setValveCoil(false); valveIsOpen = false; }

/* ======= ADC (shunt on Node X) ======= */
const int  SHUNT_ADC_PIN    = A0;           // Node X (pump side of 1Ω)
const bool USE_INTERNAL_ADC_REF = true;     // use internal ref for small voltages

/* ======= NOISE / TIMING GUARDS ======= */
const uint16_t FLOAT_FILTER_MS   = 150;     // must be stable this long
const uint32_t MIN_DWELL_MS      = 3000;    // min ON/OFF runtime (anti-chatter)
const uint32_t FAULT_BLANK_MS    = 700;     // ignore inrush for sensing

/* ======= FLOW VERIFY (AVERAGE over 5 s) ======= */
/* After inrush, sample for 5 s. If average >= 500 mV (0.50 V) => no-flow/blocked => EMERGENCY STOP */
const uint16_t FLOW_BAD_mV    = 500;        // threshold for no-flow (avg >= 0.50 V)
const uint32_t FLOW_VERIFY_MS = 5000;       // averaging window
const uint8_t  ADC_SAMPLES    = 12;         // per-read averaging

/* ======= EEPROM ADDRS ======= */
const int EE_ADDR_FAULT      = 0;            // 1 byte
const uint8_t EE_FAULT_CLEAR = 0x00;
const uint8_t EE_FAULT_SET   = 0xF1;
const int EE_ADDR_TOTAL_SEC  = 1;            // 4 bytes at 1..4 (lifetime ON seconds)
const int EE_ADDR_BOOT_COUNT = 10;           // 4 bytes at 10..13 (power-up counter)

/* ======= SESSION LOG (per power cycle) ======= */
/*
  EEPROM layout from EE_ADDR_SESSION_BASE:
  header: magic(uint32), head(uint8), count(uint8), rsv(uint16) = 8 bytes
  entry : bootNo(uint32), runSec(uint32), starts(uint16)       = 10 bytes
*/
const int EE_ADDR_SESSION_BASE = 200;
const uint32_t SESSION_MAGIC   = 0xC0FFEE51;
const uint8_t  MAX_SESSIONS    = 30;

struct SessionHeader {
  uint32_t magic;
  uint8_t  head;
  uint8_t  count;
  uint16_t rsv;
};
struct SessionEntry {
  uint32_t bootNo;   // which boot this session corresponds to
  uint32_t runSec;   // total ON seconds in this power cycle
  uint16_t starts;   // times pump turned ON in this power cycle
};

/* Global session state stored in EEPROM */
SessionHeader sessHdr;
uint8_t sessIndexCurrent = 0xFF; // where current session lives

/* Wrap helpers in a class to avoid Arduino auto-prototypes */
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

/* ======= HH:MM:SS printer ======= */
void printHMS(uint32_t secs) {
  uint32_t h = secs / 3600UL;
  uint32_t m = (secs / 60UL) % 60UL;
  uint32_t s = secs % 60UL;
  Serial.print(h); Serial.print(':');
  if (m<10) Serial.print('0'); Serial.print(m); Serial.print(':');
  if (s<10) Serial.print('0'); Serial.print(s);
}

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

uint32_t readShunt_mV() {
  static bool refInit=false;
  if (!refInit) { analogReference(USE_INTERNAL_ADC_REF ? INTERNAL : DEFAULT); delay(5); (void)analogRead(SHUNT_ADC_PIN); refInit=true; }
  uint32_t acc=0; for (uint8_t i=0;i<ADC_SAMPLES;i++){ acc += analogRead(SHUNT_ADC_PIN); delayMicroseconds(200); }
  const float Vref_mV = USE_INTERNAL_ADC_REF ? 1100.0f : 5000.0f;
  return (uint32_t)((acc/(float)ADC_SAMPLES) * Vref_mV / 1023.0f + 0.5f);
}

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

/* ======= PUMP STATE APPLY (with valve control) ======= */
void applyPumpState(bool on) {
  if (on == pumpRunning) return;
  if (millis() - lastSwitchMs < MIN_DWELL_MS) return; // dwell guard

  if (on) {
    // ----- PUMP TURNING ON -----
    relayPumpOn();
    pumpOnAtMs = millis();
    totalOns++;
    SessionStore::addStarts(1);

    // Close valve while pump runs (NC valve de-energized = CLOSED)
    valveClose();
    valveOpenUntilMs = 0; // cancel any pending pulse

    // Start average-based 5s flow verify (after inrush)
    flowCheckActive   = true;
    flowWindowStartMs = pumpOnAtMs + FAULT_BLANK_MS;
    flowSum_mV        = 0;
    flowSamples       = 0;
    flowMin_mV        = 65535;

    Serial.println(F("PUMP -> ON (latched) | Valve CLOSED | Starting 5 s flow verification after inrush."));
  } else {
    // ----- PUMP TURNING OFF -----
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

    // Open valve briefly after stopping, then auto-close
    if (VALVE_OPEN_PULSE_MS > 0) {
      valveOpen();
      valveOpenUntilMs = millis() + VALVE_OPEN_PULSE_MS;
      Serial.println(F("Valve OPEN (post-stop pulse)."));
    } else {
      valveClose();
      valveOpenUntilMs = 0;
      Serial.println(F("Valve CLOSED (no post-stop pulse configured)."));
    }

    // cancel flow verify
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

/* ======= SETUP ======= */
void setup() {
  Serial.begin(115200);

  // Load counters
  EEPROM.get(EE_ADDR_TOTAL_SEC, totalRunSec);
  loadBootCount();

  // Init/read session header BEFORE incrementing bootCount
  SessionStore::readHeader();

  // Increment boot count for THIS power-up and save
  bootCount++;
  saveBootCount();

  // Start a new session entry for this boot
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
  if (ENABLE_CLEAR_BTN) pinMode(CLEAR_BTN_PIN, INPUT);

  // Valve pins
  pinMode(VALVE_RELAY_PIN, OUTPUT);
  if (VALVE_LED_PIN >= 0) pinMode(VALVE_LED_PIN, OUTPUT);
  valveClose();  // default: keep water out

  relayPumpOff();
  setHumid(true);
  setFaultLED(false);

  // Clear any stored fault at boot
  saveFaultToEEPROM(false);
  faultLatched = false;

  Serial.println(F("\nPower-up: fault cleared."));
  Serial.println(F("Serial cmds: 'C' clear fault, 'R' report, 'H' run history (this boot), 'X' clear run history, 'B' boot info, 'Z' reset boot count, 'S' session history"));

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
  // Manual clear button
  if (ENABLE_CLEAR_BTN && checkClearButtonOnce()){
    faultLatched = false; saveFaultToEEPROM(false);
    setFaultLED(false);
    Serial.println(F("Fault CLEARED via button."));
  }

  // Serial commands
  if (Serial.available()){
    char c = Serial.read();
    switch (c) {
      case 'C': // clear fault
        faultLatched=false; saveFaultToEEPROM(false); setFaultLED(false);
        Serial.println(F("Fault CLEARED via Serial."));
        break;

      case 'R': { // runtime report (includes current session snapshot only)
        Serial.print(F("Pump running: ")); Serial.println(pumpRunning ? F("YES") : F("NO"));
        if (pumpRunning) {
          uint32_t curMs = millis() - pumpOnAtMs;
          Serial.print(F("Current run so far: ")); Serial.print(curMs / 1000.0, 3); Serial.println(F(" s"));
        }
        Serial.print(F("Last completed run: ")); Serial.print(lastRunMs / 1000.0, 3); Serial.println(F(" s"));
        Serial.print(F("Lifetime ON time: ")); Serial.print(totalRunSec); Serial.println(F(" s"));

        // Current session snapshot
        if (sessIndexCurrent != 0xFF) {
          SessionEntry e; SessionStore::readAt(sessIndexCurrent, e);
          Serial.print(F("This power cycle (boot #")); Serial.print(e.bootNo); Serial.print(F("): starts="));
          Serial.print(e.starts); Serial.print(F(", runSec=")); Serial.print(e.runSec); Serial.println(F(" s"));
        }
      } break;

      case 'H': { // individual run history dump (RAM for current boot)
        Serial.print(F("\n=== RUN HISTORY (latest ")); Serial.print(logsCount); Serial.println(F(") ==="));
        Serial.println(F("#\tStart[s]\tDur[s]\tReason"));
        uint16_t first = (logsHead + MAX_RUN_LOGS - logsCount) % MAX_RUN_LOGS;
        for (uint16_t i=0; i<logsCount; i++){
          uint16_t idx = (first + i) % MAX_RUN_LOGS;
          Serial.print(i+1); Serial.print('\t');
          Serial.print(logs[idx].start_ms / 1000.0, 3); Serial.print('\t');
          Serial.print(logs[idx].dur_ms   / 1000.0, 3); Serial.print('\t');
          Serial.println(logs[idx].reason);
          delay(2);
        }
        Serial.println(F("=== END RUN HISTORY ===\n"));
      } break;

      case 'X': // clear run history (RAM counters reset)
        logsCount=0; logsHead=0; totalOns=0; totalOffs=0;
        Serial.println(F("Run history cleared (RAM counters reset)."));
        break;

      case 'B': { // boot info (with HH:MM:SS uptime)
        Serial.print(F("Boot count: ")); Serial.println(bootCount);
        Serial.print(F("Uptime: ")); printHMS(millis() / 1000UL); Serial.println();
      } break;

      case 'Z': // reset boot counter
        bootCount = 0; saveBootCount();
        Serial.println(F("Boot counter reset to 0."));
        break;

      case 'S': { // SESSION HISTORY: grouped per power cycle ONLY on request
        SessionStore::readHeader();
        Serial.print(F("\n=== SESSION HISTORY (last "));
        Serial.print(sessHdr.count);
        Serial.println(F(" power cycles) ==="));

        uint8_t first = (sessHdr.head + MAX_SESSIONS - sessHdr.count) % MAX_SESSIONS;
        for (uint8_t i=0; i<sessHdr.count; i++){
          uint8_t idx = (first + i) % MAX_SESSIONS;
          SessionEntry e; SessionStore::readAt(idx, e);

          Serial.println(F("----------------------------------------"));
          Serial.print(F("Power cycle #")); Serial.println(e.bootNo);
          Serial.print(F("  Starts: ")); Serial.println(e.starts);

          Serial.print(F("  Total run: "));
          printHMS(e.runSec);
          Serial.println();
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

  if (faultLatched) {
    if (pumpRunning) { pendingStopReason='F'; applyPumpState(false); }
    setHumid(false);
    setFaultLED(true);
    // On fault, keep valve CLOSED and cancel any pending open pulse
    valveClose();
    valveOpenUntilMs = 0;
  } else {
    // Normal control
    if (!pumpRunning) {
      if (top == LOW) { // TOP UP -> start pump
        applyPumpState(true);
        Serial.println(F("Reason: TOP UP -> start pump"));
      }
    } else {
      if (bottom == HIGH) { // BOTTOM DOWN -> stop
        pendingStopReason = 'B';
        applyPumpState(false);
        Serial.println(F("Reason: BOTTOM DOWN -> stop pump"));
      }
    }

    // ======= FLOW VERIFICATION WINDOW (AVERAGE) =======
    if (pumpRunning && flowCheckActive) {
      uint32_t now = millis();

      // begin sampling only after inrush blanking
      if (now >= flowWindowStartMs) {
        uint32_t mv = readShunt_mV();
        flowSum_mV   += mv;
        flowSamples  += 1;
        if (mv < flowMin_mV) flowMin_mV = mv;

        // When full window elapsed, evaluate average
        if (now - flowWindowStartMs >= FLOW_VERIFY_MS) {
          float avg_mV = (flowSamples == 0) ? 0.0f : (float)flowSum_mV / (float)flowSamples;
          Serial.print(F("Flow avg over 5 s: "));
          Serial.print(avg_mV, 1);
          Serial.print(F(" mV (min=")); Serial.print(flowMin_mV); Serial.println(F(" mV)"));

          if (avg_mV >= FLOW_BAD_mV) {
            // Average indicates no-flow / blocked
            pendingStopReason = 'F';
            applyPumpState(false);
            faultLatched = true;              // latch
            saveFaultToEEPROM(true);
            setFaultLED(true);
            valveClose();                     // keep water out
            valveOpenUntilMs = 0;
            Serial.println(F("EMERGENCY: No-flow average >= 500 mV. Fault LATCHED and STORED. Pump inhibited until manual CLEAR."));
          } else {
            Serial.println(F("Flow OK: average below 500 mV."));
          }

          // End the check either way
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
    Serial.println(F("Valve auto-closed after pulse window."));
  }

  // Debug print (every 5 s; NO session snapshot here)
  const uint32_t DEBUG_PERIOD_MS = 5000; // print every 5 s
  static uint32_t lastDbg=0;
  if (millis() - lastDbg >= DEBUG_PERIOD_MS) {
    lastDbg = millis();
    Serial.print(F("Pump="));  Serial.print(pumpRunning?F("ON"):F("OFF"));
    Serial.print(F("  FaultLatched=")); Serial.print(faultLatched?F("YES"):F("NO"));
    uint32_t mv = readShunt_mV();
    Serial.print(F("  Vshunt=")); Serial.print(mv); Serial.print(F(" mV"));
    Serial.print(F("  Uptime=")); printHMS(millis()/1000UL);
    Serial.print(F("  Boot#=")); Serial.print(bootCount);
    Serial.print(F("  Valve=")); Serial.print(valveIsOpen ? F("OPEN") : F("CLOSED"));
    if (valveIsOpen && valveOpenUntilMs) {
      long rem = (long)(valveOpenUntilMs - millis());
      if (rem < 0) rem = 0;
      Serial.print(F(" (auto-close in ")); Serial.print(rem/1000); Serial.print(F(" s)"));
    }
    if (pumpRunning && flowCheckActive) {
      Serial.print(F("  [FlowCheck samples=")); Serial.print(flowSamples);
      Serial.print(F(" min=")); Serial.print(flowMin_mV); Serial.print(F(" mV]"));
    }
    if (pumpRunning) {
      uint32_t curMs = millis() - pumpOnAtMs;
      Serial.print(F("  RunSoFar=")); Serial.print(curMs / 1000.0, 3); Serial.print(F(" s"));
    }
    Serial.println();
  }

  delay(5);
}
