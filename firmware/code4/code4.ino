#include <Arduino.h>
#include <FastAccelStepper.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Pin Allocations for ESP32 DevKit V1 and DRV8825 Drivers
// -----------------------------------------------------------------------------
#define MOTOR1_STEP_PIN    18 //
#define MOTOR1_DIR_PIN     19 //
#define MOTOR2_STEP_PIN    22 //
#define MOTOR2_DIR_PIN     23 //[cite: 1]
#define SHARED_ENABLE_PIN  21 //[cite: 1]

// -----------------------------------------------------------------------------
// Physical Arm Parameters and Kinematic Constants
// -----------------------------------------------------------------------------
const double BASE1_X = -40.0; //[cite: 1]
const double BASE1_Y =   0.0; //[cite: 1]
const double BASE2_X =  40.0; //[cite: 1]
const double BASE2_Y =   0.0; //[cite: 1]

const double LINK1 = 110.0;   //[cite: 1]
const double LINK2 = 110.0;   //[cite: 1]
const double LINK3 = 160.0;   //[cite: 1]
const double LINK4 = 160.0;   //[cite: 1]

const double STEPS_PER_REV = 6400.0; //[cite: 1]
const double STEPS_PER_RAD = STEPS_PER_REV / (2.0 * M_PI); //[cite: 1]

// Engine & Stepper Objects
FastAccelStepperEngine engine = FastAccelStepperEngine(); //[cite: 1]
FastAccelStepper *stepper1 = NULL; //[cite: 1]
FastAccelStepper *stepper2 = NULL; //[cite: 1]

// Function Prototypes
bool solveInverseKinematics(double x, double y, double &out_theta1, double &out_theta2); //[cite: 1]
void moveToTarget(int32_t targetSteps1, int32_t targetSteps2); //[cite: 1]

void setup() {
  Serial.begin(115200); //[cite: 1]
  while (!Serial && millis() < 2000) { delay(10); } //[cite: 1]

  Serial.println(F("=================================================="));
  Serial.println(F(" SCARA VISION SYSTEM READY "));
  Serial.println(F("=================================================="));

  // FORCE DRIVERS ON for holding torque
  pinMode(SHARED_ENABLE_PIN, OUTPUT); //[cite: 1]
  digitalWrite(SHARED_ENABLE_PIN, LOW); //[cite: 1]

  engine.init(); //[cite: 1]
  stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN); //[cite: 1]
  stepper2 = engine.stepperConnectToPin(MOTOR2_STEP_PIN); //[cite: 1]

  if (!stepper1 || !stepper2) { //[cite: 1]
    Serial.println(F("[ERROR] Hardware Timer Allocation Failed!")); //[cite: 1]
    while (true) { delay(1000); } //[cite: 1]
  }

  // --- SMOOTH MOTION PROFILE & CORRECTED DIRECTION ---
  stepper1->setDirectionPin(MOTOR1_DIR_PIN, false); //[cite: 1]
  stepper1->setSpeedInHz(3200);  //[cite: 1]
  stepper1->setAcceleration(2000); //[cite: 1]

  stepper2->setDirectionPin(MOTOR2_DIR_PIN, false); //[cite: 1]
  stepper2->setSpeedInHz(3200);  //[cite: 1]
  stepper2->setAcceleration(2000); //[cite: 1]

  // Set home position (90 degrees vertical = 1,600 steps)
  stepper1->setCurrentPosition(1600); //[cite: 1]
  stepper2->setCurrentPosition(1600); //[cite: 1]

  Serial.println(F("[READY] Arms manually homed at 90 degrees.")); //[cite: 1]
  Serial.println(F("[INFO] Waiting for Python vision commands..."));
}

void loop() {
  // Listen for Serial commands from Python
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Check if the command starts with "VPICK "
    if (cmd.startsWith("VPICK ")) {
      // Find the comma to split X and Y
      int commaIndex = cmd.indexOf(',');
      
      if (commaIndex > 0) {
        // Extract the numbers from the string (starts at index 6 after "VPICK ")
        float targetX = cmd.substring(6, commaIndex).toFloat();
        float targetY = cmd.substring(commaIndex + 1).toFloat();
        
        Serial.printf("[VISION] Target Received: X = %.1f, Y = %.1f\n", targetX, targetY);

        double theta1_rad = 0.0, theta2_rad = 0.0; //[cite: 1]

        // Calculate IK
        if (solveInverseKinematics(targetX, targetY, theta1_rad, theta2_rad)) { //[cite: 1]
          int32_t targetSteps1 = round(theta1_rad * STEPS_PER_RAD); //[cite: 1]
          int32_t targetSteps2 = round(theta2_rad * STEPS_PER_RAD); //[cite: 1]

          // Execute Move
          moveToTarget(targetSteps1, targetSteps2); //[cite: 1]
          
          Serial.println(F("[VISION] Move Complete. Ready for next object."));
        } else {
          Serial.println(F("[IK ERROR] Target coordinate out of reach!")); //[cite: 1]
        }
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Helper Functions (Unchanged)
// -----------------------------------------------------------------------------
bool solveInverseKinematics(double x, double y, double &out_theta1, double &out_theta2) { //[cite: 1]
  double dx1 = x - BASE1_X; //[cite: 1]
  double dy1 = y - BASE1_Y; //[cite: 1]
  double dx2 = x - BASE2_X; //[cite: 1]
  double dy2 = y - BASE2_Y; //[cite: 1]

  double D1 = hypot(dx1, dy1); //[cite: 1]
  double D2 = hypot(dx2, dy2); //[cite: 1]

  if (D1 > (LINK1 + LINK3) || D1 < fabs(LINK1 - LINK3) || //[cite: 1]
      D2 > (LINK2 + LINK4) || D2 < fabs(LINK2 - LINK4)) { //[cite: 1]
    return false;  //[cite: 1]
  }

  double alpha1 = atan2(dy1, dx1); //[cite: 1]
  double alpha2 = atan2(dy2, dx2); //[cite: 1]

  double cos_beta1 = constrain((LINK1 * LINK1 + D1 * D1 - LINK3 * LINK3) / (2.0 * LINK1 * D1), -1.0, 1.0); //[cite: 1]
  double cos_beta2 = constrain((LINK2 * LINK2 + D2 * D2 - LINK4 * LINK4) / (2.0 * LINK2 * D2), -1.0, 1.0); //[cite: 1]

  double beta1 = acos(cos_beta1); //[cite: 1]
  double beta2 = acos(cos_beta2); //[cite: 1]

  out_theta1 = alpha1 + beta1; //[cite: 1]
  out_theta2 = alpha2 - beta2; //[cite: 1]

  return true; //[cite: 1]
}

void moveToTarget(int32_t targetSteps1, int32_t targetSteps2) { //[cite: 1]
  stepper1->moveTo(targetSteps1); //[cite: 1]
  stepper2->moveTo(targetSteps2); //[cite: 1]
  while (stepper1->isRunning() || stepper2->isRunning()) { //[cite: 1]
    vTaskDelay(1 / portTICK_PERIOD_MS);  //[cite: 1]
  }
}