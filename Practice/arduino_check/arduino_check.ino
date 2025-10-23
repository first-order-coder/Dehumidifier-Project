void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(115200);
  // Do NOT wait on Serial here; Nanos don't need while(!Serial)
  Serial.println("\nNano alive. .");
}

void loop() {
  digitalWrite(13, !digitalRead(13)); // blink on-board LED
  Serial.println("Tick");
  delay(500);
}
