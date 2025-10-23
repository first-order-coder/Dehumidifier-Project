#include <Arduino.h>

// ================== USER SETTINGS ==================
const char* PHONE = "+37128799838";     // <-- your phone number

// Your sensor range: DRY ~0 ... WET ~750
const int W1_ON_ADC  = 600;             // >= this -> start pump
const int W2_OFF_ADC = 120;             // <= this -> stop pump

// ================== PINOUT ==================
#define WATER_ADC_PIN  5                // Sensor SIG -> GPIO5 (ADC)
#define PUMP_LED_PIN   2                // (optional) shows pump state
#define MODEM_RX      16                // SIM808 TX -> ESP32 RX
#define MODEM_TX      17                // SIM808 RX <- ESP32 TX

HardwareSerial Modem(2);

// ================== STATE ==================
bool pumpOn = false;

// ================== MODEM (minimal) ==================
bool waitFor(const char* token, uint32_t ms) {
  uint32_t t0 = millis(); String buf;
  while (millis() - t0 < ms) {
    while (Modem.available()) {
      char c = (char)Modem.read();
      buf += c;
      if (buf.indexOf(token) >= 0) return true;
    }
    delay(1);
  }
  return false;
}

void modemInit() {
  Modem.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(600);
  Modem.println("AT");        waitFor("OK", 1000);
  Modem.println("ATE0");      waitFor("OK", 500);    // cleaner responses
  Modem.println("AT+CMGF=1"); waitFor("OK", 1000);   // SMS text mode
}

void sendSMS(const String& text) {
  Modem.print("AT+CMGS=\""); Modem.print(PHONE); Modem.println("\"");
  if (!waitFor(">", 60000)) return;
  Modem.print(text);
  delay(60);
  Modem.write((uint8_t)0x1A);                   // Ctrl+Z
  waitFor("+CMGS:", 60000); waitFor("OK", 10000);
}

// ================== SENSOR ==================
int readADCavg(uint8_t n = 8) {
  long s = 0;
  for (uint8_t i = 0; i < n; ++i) { s += analogRead(WATER_ADC_PIN); delay(2); }
  return (int)(s / n);
}

// ================== ACTUATION ==================
void startPump(int adcAtStart) {
  pumpOn = true;
  digitalWrite(PUMP_LED_PIN, HIGH);
  sendSMS("Pump STARTED at ADC=" + String(adcAtStart));
}

void stopPump(int adcAtStop) {
  pumpOn = false;
  digitalWrite(PUMP_LED_PIN, LOW);
  sendSMS("Pump STOPPED at ADC=" + String(adcAtStop));
}

// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(115200);
  pinMode(PUMP_LED_PIN, OUTPUT);
  digitalWrite(PUMP_LED_PIN, LOW);

  analogReadResolution(12);                         // 0..4095
  analogSetPinAttenuation(WATER_ADC_PIN, ADC_11db); // ~0..3.3V

  modemInit();

  Serial.println("\nWater controller (SMS only on START/STOP)");
  Serial.printf("Start >= %d, Stop <= %d (sensor ~0..750)\n", W1_ON_ADC, W2_OFF_ADC);
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 100) return;   // 10 Hz
  last = millis();

  int adc = readADCavg();              // stable reading

  if (!pumpOn && adc >= W1_ON_ADC) {
    startPump(adc);
  } else if (pumpOn && adc <= W2_OFF_ADC) {
    stopPump(adc);
  }

  // (Optional: view live value for tuning)
  // Serial.printf("ADC=%d\n", adc);
}
