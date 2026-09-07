/// ANGLE INPUT 

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Pin Allocations for ESP32 DevKit V1 and DRV8825 Drivers
// -----------------------------------------------------------------------------
#define MOTOR1_STEP_PIN    18
#define MOTOR1_DIR_PIN     19
#define MOTOR2_STEP_PIN    22
#define MOTOR2_DIR_PIN     23
#define SHARED_ENABLE_PIN  21

// Stepper Configuration: 6,400 steps per revolution
const double STEPS_PER_DEGREE = 6400.0 / 360.0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { delay(10); }

  Serial.println(F("=================================================="));
  Serial.println(F(" SMOOTH DIRECT ANGLE CONTROL MODE"));
  Serial.println(F("=================================================="));

  // FORCE DRIVERS ON: Keep the DRV8825s permanently engaged for holding torque
  pinMode(SHARED_ENABLE_PIN, OUTPUT);
  digitalWrite(SHARED_ENABLE_PIN, LOW); 

  engine.init();
  stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);
  stepper2 = engine.stepperConnectToPin(MOTOR2_STEP_PIN);

  if (!stepper1 || !stepper2) {
    Serial.println(F("[ERROR] Hardware Timer Allocation Failed!"));
    while (true) { delay(1000); }
  }

  // --- SMOOTH MOTION PROFILE SETTINGS ---
  // Speed reduced to 3,200 (half revolution per second)
  // Acceleration dramatically reduced to 2,000 for soft starts and stops
  
  // Configure Motor 1 
  stepper1->setDirectionPin(MOTOR1_DIR_PIN, false);
  stepper1->setSpeedInHz(3200); 
  stepper1->setAcceleration(2000);

  // Configure Motor 2
  stepper2->setDirectionPin(MOTOR2_DIR_PIN, false);
  stepper2->setSpeedInHz(3200); 
  stepper2->setAcceleration(2000);

  // Home position is manually set to 90 degrees vertical
  stepper1->setCurrentPosition(1600);
  stepper2->setCurrentPosition(1600);

  Serial.println(F("[READY] Arms are assumed to be at 90 degrees."));
  Serial.println(F("Enter Target Angles in degrees: [Theta1] [Theta2]"));
  Serial.println(F("--------------------------------------------------"));
}

void loop() {
  if (Serial.available() > 0) {
    float targetAngle1 = Serial.parseFloat();
    float targetAngle2 = Serial.parseFloat();

    while (Serial.available() > 0) {
      Serial.read();
      delay(2);
    }

    int32_t targetSteps1 = round(targetAngle1 * STEPS_PER_DEGREE);
    int32_t targetSteps2 = round(targetAngle2 * STEPS_PER_DEGREE);

    Serial.printf("Command Received: Move to %.1f Deg, %.1f Deg\n", targetAngle1, targetAngle2);
    
    stepper1->moveTo(targetSteps1);
    stepper2->moveTo(targetSteps2);

    while (stepper1->isRunning() || stepper2->isRunning()) {
      vTaskDelay(1 / portTICK_PERIOD_MS); 
    }

    Serial.println(F("Move Complete. Motors are holding position..."));
  }
}