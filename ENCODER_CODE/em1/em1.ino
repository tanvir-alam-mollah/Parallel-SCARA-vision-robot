// Single Motor Single ENCODER
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Wire.h>

// --- Pins for Motor 1 & Encoder 1 ---
#define MOTOR_STEP_PIN    18
#define MOTOR_DIR_PIN     19
#define ENABLE_PIN        21
#define ENCODER_SDA       32  // Updated to match your wiring
#define ENCODER_SCL       33  // Updated to match your wiring

// --- Objects ---
TwoWire I2C_Encoder = TwoWire(0);
const int AS5600_ADDRESS = 0x36; 
const int RAW_ANGLE_REGISTER = 0x0C; 

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// --- Global Variable ---
int startingOffset = 0; // Treats startup position as 0

// --- Functions ---
uint16_t readEncoder() {
  I2C_Encoder.beginTransmission(AS5600_ADDRESS);
  I2C_Encoder.write(RAW_ANGLE_REGISTER);
  I2C_Encoder.endTransmission(false);
  I2C_Encoder.requestFrom(AS5600_ADDRESS, 2);
  if (I2C_Encoder.available() == 2) {
    uint8_t highByte = I2C_Encoder.read();
    uint8_t lowByte = I2C_Encoder.read();
    return ((highByte << 8) | lowByte) & 0x0FFF; 
  }
  return 0; 
}

int getZeroedTicks() {
  int rawTicks = readEncoder();
  // Subtract startup offset, add 4096 to prevent negative math, wrap at 4096
  return (rawTicks - startingOffset + 4096) % 4096; 
}

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize Encoder
  I2C_Encoder.begin(ENCODER_SDA, ENCODER_SCL, 400000); 
  delay(500); // Let sensor boot up
  
  // Record current physical position as "0"
  startingOffset = readEncoder();
  
  // 2. Initialize Stepper
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); 

  engine.init();
  stepper = engine.stepperConnectToPin(MOTOR_STEP_PIN);
  stepper->setDirectionPin(MOTOR_DIR_PIN, true); // Try changing to 'true' if direction is backwards
  stepper->setSpeedInHz(3200); 
  stepper->setAcceleration(2000);
  
  Serial.println("========================================");
  Serial.println("Single Motor Encoder Test Ready.");
  Serial.print("Startup Offset Recorded: "); Serial.println(startingOffset);
  Serial.println("Type 'M' in the Serial Monitor and press Enter to move 90 degrees.");
  Serial.println("========================================");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // If user types 'M' or 'm'
    if (cmd == 'M' || cmd == 'm') {
      
      Serial.println("\nCommand Received: Moving Stepper 1600 steps (90 deg)...");
      
      // Read where we are BEFORE moving
      int startTicks = getZeroedTicks();
      
      // Tell motor to move 1600 steps relative to where it is right now
      stepper->move(1600); 
      
      delay(10);
      while (stepper->isRunning()) {
        // Wait for motor to finish moving
        delay(1); 
      }
      
      // Give the physical arm half a second to stop vibrating
      delay(500); 
      
      // Read the physical result AFTER moving
      int rawAfter = readEncoder();
      int endTicks = getZeroedTicks();
      
      // Calculate the delta (change), handling the 360-degree wrap-around
      int deltaTicks = endTicks - startTicks;
      if (deltaTicks < -2048) deltaTicks += 4096;
      if (deltaTicks > 2048) deltaTicks -= 4096;
      
      Serial.println("--- MOVEMENT COMPLETE ---");
      Serial.print("Raw AS5600 Reading (If 0 = Loose Wire!) : "); Serial.println(rawAfter);
      Serial.print("Expected Encoder Ticks for 90 deg       : 1024\n");
      Serial.print("Actual Ticks Moved (Delta)              : "); Serial.println(deltaTicks);
      
      // Calculate the difference between expected and actual
      int error = 1024 - deltaTicks;
      Serial.print("Error / Difference                      : "); Serial.print(error); Serial.println(" ticks");
      
      Serial.println("\nType 'M' to move ANOTHER 90 degrees forward.");
    }
  }
}