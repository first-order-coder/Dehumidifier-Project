String line = "";
unsigned long baudRate = 115200;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(baudRate);
  Serial.println("Serial Command Console ready. Type HELP...");
}

int addNums (int x, int y){
  return x + y;
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()){
    char c = Serial.read();

    if (c == '\n'){
      line.trim(); // trim() will clean spaces + \r

      if (line.equalsIgnoreCase("HELLO")){ //ignores lowercase and upper cases
        Serial.println("Hi there!");
      } else if (line.startsWith("NAME ")){
        // Grab everything AFTER "NAME" (which is 5 characters long)
        String name = line.substring(5);
        Serial.print("Nice to meet you, ");
        Serial.print(name);
        Serial.println("!");
      
      } else if (line.startsWith("ADD ")) {
        int firstSpace = line.indexOf("", 4); // first space is 5 after A D D _ x _

        if (firstSpace > 0) {
          int x = line.substring(4, firstSpace).toInt();
          int y = line.substring(firstSpace + 1).toInt();

          Serial.print("Result:");
          Serial.println(x+y);
        } else {
          Serial.println("Usage: ADD x y");
        }
      } else if (line.equalsIgnoreCase("HELP")){
        Serial.println("Commands: HELLO, NAME <text>, ADD x y, HELP");
      
      } else if (line.length() > 0) {
        Serial.println("Unknown command. Try HELP.");
      } 

      line = "";
    } else if (c != '\r') {

      line += c;
    }
    
  }

}
