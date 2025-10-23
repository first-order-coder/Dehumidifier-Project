String inputString = "";

void setup() { //setup runs once after the board is powered
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Type something and press Enter:");
}

void loop() {
  // put your main code here, to run repeatedly:

  while (Serial.available() > 0) { //how many characters are waiting to be read from the PC
    char c = Serial.read();

    if (c == '\n'){
      Serial.print("You said:");
      Serial.println(inputString);
      inputString = "";
    } else if (c != '\r') {
      inputString += c;
    }
    // if the char was'\r' then we skip it (do nothing), because its just a line ending helper.
  }

}
