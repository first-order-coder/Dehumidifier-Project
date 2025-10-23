int sensorPin = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(sensorPin, ADC_11db);

}

void loop() {

  if (Serial.available()){
    char c = Serial.read();
    if (c == "s"){
      while (1);
    }
  }
  
  // put your main code here, to run repeatedly:
  int raw = analogRead(sensorPin);
  Serial.println(raw);
  delay(500);
}
