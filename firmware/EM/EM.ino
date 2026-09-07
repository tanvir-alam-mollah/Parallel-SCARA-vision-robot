#define MAGNET_PIN 26 

void setup() {
  Serial.begin(115200);
  delay(500);

  // Start with the magnet OFF (High-Impedance mode)
  pinMode(MAGNET_PIN, INPUT); 

  Serial.println(F("\n======================================="));
  Serial.println(F(" PINMODE HACK ELECTROMAGNET TEST"));
  Serial.println(F("======================================="));
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();          
    cmd.toUpperCase();   
    
    if (cmd == "ON") {
      // Turn ON: Set as output and pull to GND
      pinMode(MAGNET_PIN, OUTPUT);
      digitalWrite(MAGNET_PIN, LOW); 
      Serial.println(F("[STATUS] Magnet ON (Green LED should be ON)"));
    } 
    else if (cmd == "OFF") {
      // Turn OFF: Set as input (disconnects the pin internally)
      pinMode(MAGNET_PIN, INPUT); 
      Serial.println(F("[STATUS] Magnet OFF (Green LED should be OFF)"));
    }
  }
}