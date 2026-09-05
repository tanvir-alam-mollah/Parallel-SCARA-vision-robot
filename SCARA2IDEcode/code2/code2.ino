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

// -----------------------------------------------------------------------------
// Physical Arm Parameters and Kinematic Constants
// -----------------------------------------------------------------------------
const double BASE1_X = -40.0; 
const double BASE1_Y =   0.0; 
const double BASE2_X =  40.0; 
const double BASE2_Y =   0.0; 

const double LINK1 = 110.0;   
const double LINK2 = 110.0;   
const double LINK3 = 160.0;   
const double LINK4 = 160.0;   

// Stepper Configuration
const double STEPS_PER_REV = 6400.0;
const double STEPS_PER_RAD = STEPS_PER_REV / (2.0 * M_PI);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

// Function prototype for IK
bool solveInverseKinematics(double x, double y, double &out_theta1, double &out_theta2);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { delay(10); }

  Serial.println(F("=================================================="));
  Serial.println(F(" CARTESIAN (X,Y) CONTROL MODE WITH IK"));
  Serial.println(F("=================================================="));

  // FORCE DRIVERS ON for holding torque
  pinMode(SHARED_ENABLE_PIN, OUTPUT);
  digitalWrite(SHARED_ENABLE_PIN, LOW); 

  engine.init();
  stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);
  stepper2 = engine.stepperConnectToPin(MOTOR2_STEP_PIN);

  if (!stepper1 || !stepper2) {
    Serial.println(F("[ERROR] Hardware Timer Allocation Failed!"));
    while (true) { delay(1000); }
  }

  // --- SMOOTH MOTION PROFILE & CORRECTED DIRECTION ---
  
  // Configure Motor 1
  stepper1->setDirectionPin(MOTOR1_DIR_PIN, false);
  stepper1->setSpeedInHz(3200); 
  stepper1->setAcceleration(2000);

  // Configure Motor 2
  stepper2->setDirectionPin(MOTOR2_DIR_PIN, false);
  stepper2->setSpeedInHz(3200); 
  stepper2->setAcceleration(2000);

  // Set home position (90 degrees vertical = 1,600 steps)
  stepper1->setCurrentPosition(1600);
  stepper2->setCurrentPosition(1600);

  Serial.println(F("[READY] Arms are assumed to be at 90 degrees."));
  Serial.println(F("Enter Target Coordinates in mm: [X] [Y]"));
  Serial.println(F("Example: 0 200"));
  Serial.println(F("--------------------------------------------------"));
}

void loop() {
  if (Serial.available() > 0) {
    // Read the X and Y inputs
    float targetX = Serial.parseFloat();
    float targetY = Serial.parseFloat();

    // Clear buffer
    while (Serial.available() > 0) {
      Serial.read();
      delay(2);
    }

    Serial.printf("\nCommand Received: Move to X = %.1f mm, Y = %.1f mm\n", targetX, targetY);

    double theta1_rad = 0.0;
    double theta2_rad = 0.0;

    // Run the Inverse Kinematics Math
    if (solveInverseKinematics(targetX, targetY, theta1_rad, theta2_rad)) {
      
      // Convert radians to steps
      int32_t targetSteps1 = round(theta1_rad * STEPS_PER_RAD);
      int32_t targetSteps2 = round(theta2_rad * STEPS_PER_RAD);

      // Print debug angles in degrees for your reference
      double deg1 = theta1_rad * (180.0 / M_PI);
      double deg2 = theta2_rad * (180.0 / M_PI);
      Serial.printf("[IK SUCCESS] Joint Angles: Theta1 = %.2f deg, Theta2 = %.2f deg\n", deg1, deg2);

      // Execute movement
      stepper1->moveTo(targetSteps1);
      stepper2->moveTo(targetSteps2);

      // Wait for movement to finish
      while (stepper1->isRunning() || stepper2->isRunning()) {
        vTaskDelay(1 / portTICK_PERIOD_MS); 
      }

      Serial.println(F("Move Complete. Motors holding..."));
    } else {
      Serial.println(F("[IK ERROR] That coordinate is outside the physical reach of the arms!"));
    }
    Serial.println(F("--------------------------------------------------"));
  }
}

// -----------------------------------------------------------------------------
// Real-Time Inverse Kinematics Implementation
// -----------------------------------------------------------------------------
bool solveInverseKinematics(double x, double y, double &out_theta1, double &out_theta2) {
  // Delta distance components from base joints to target coordinate
  double dx1 = x - BASE1_X;
  double dy1 = y - BASE1_Y;
  double dx2 = x - BASE2_X;
  double dy2 = y - BASE2_Y;

  // Linear Euclidean spans from base points to target
  double D1 = hypot(dx1, dy1);
  double D2 = hypot(dx2, dy2);

  // Boundary check against link extension envelope constraints
  if (D1 > (LINK1 + LINK3) || D1 < fabs(LINK1 - LINK3) ||
      D2 > (LINK2 + LINK4) || D2 < fabs(LINK2 - LINK4)) {
    return false; // Point lies outside kinematic workspace reach
  }

  // Angular orientations of linear distance vectors
  double alpha1 = atan2(dy1, dx1);
  double alpha2 = atan2(dy2, dx2);

  // Cosine rule triangle interior angle calculations
  double cos_beta1 = constrain((LINK1 * LINK1 + D1 * D1 - LINK3 * LINK3) / (2.0 * LINK1 * D1), -1.0, 1.0);
  double cos_beta2 = constrain((LINK2 * LINK2 + D2 * D2 - LINK4 * LINK4) / (2.0 * LINK2 * D2), -1.0, 1.0);

  double beta1 = acos(cos_beta1);
  double beta2 = acos(cos_beta2);

  // Outer branch kinematic combination ("elbow-up" configuration)
  out_theta1 = alpha1 + beta1;
  out_theta2 = alpha2 - beta2;

  return true;
}