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

const double STEPS_PER_REV = 6400.0;
const double STEPS_PER_RAD = STEPS_PER_REV / (2.0 * M_PI);

// Structure for Target Coordinates
struct Point2D {
  float x;
  float y;
};

// Target Points Array
const Point2D TARGET_POINTS[] = {
  { -200.0f, 0.0f },
  { -153.0f,  0.0f },
  { -100.0f, 70.0f },
  {    0.0f, 70.0f },
  {  100.0f,  70.0f },
  {  153.0f,  0.0f },
  {  200.0f,  0.0f },
  {  200.0f, 112.0f },
  {  100.0f, 225.0f },
  { -200.0f, 112.0f },
  
  
};

const uint8_t NUM_POINTS = sizeof(TARGET_POINTS) / sizeof(TARGET_POINTS[0]);
const uint8_t MAX_LOOPS = 5;

// Engine & Stepper Objects
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

// Loop Tracking Variables
uint8_t currentPointIndex = 0;
uint8_t loopCount = 0;
bool sequenceComplete = false;

// Function Prototypes
bool solveInverseKinematics(double x, double y, double &out_theta1, double &out_theta2);
void moveToTarget(int32_t targetSteps1, int32_t targetSteps2);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { delay(10); }

  Serial.println(F("=================================================="));
  Serial.println(F(" 3-POINT TARGET REPETITION SEQUENCE (5 LOOPS)"));
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
  stepper1->setDirectionPin(MOTOR1_DIR_PIN, false);
  stepper1->setSpeedInHz(3200); 
  stepper1->setAcceleration(2000);

  stepper2->setDirectionPin(MOTOR2_DIR_PIN, false);
  stepper2->setSpeedInHz(3200); 
  stepper2->setAcceleration(2000);

  // Set home position (90 degrees vertical = 1,600 steps)
  stepper1->setCurrentPosition(1600);
  stepper2->setCurrentPosition(1600);

  Serial.println(F("[READY] Arms manually homed at 90 degrees."));
  Serial.println(F("[INFO] Starting sequence in 5 seconds..."));
  delay(5000);
}

void loop() {
  // If all 5 loops are finished, keep holding position and do nothing
  if (sequenceComplete) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }

  // Fetch current point coordinates
  float targetX = TARGET_POINTS[currentPointIndex].x;
  float targetY = TARGET_POINTS[currentPointIndex].y;

  Serial.printf("[LOOP %d/%d] Moving to Point %d: X = %.1f, Y = %.1f\n", 
                loopCount + 1, MAX_LOOPS, currentPointIndex + 1, targetX, targetY);

  double theta1_rad = 0.0, theta2_rad = 0.0;

  // Calculate IK
  if (solveInverseKinematics(targetX, targetY, theta1_rad, theta2_rad)) {
    int32_t targetSteps1 = round(theta1_rad * STEPS_PER_RAD);
    int32_t targetSteps2 = round(theta2_rad * STEPS_PER_RAD);

    // Execute Move
    moveToTarget(targetSteps1, targetSteps2);
    delay(500); // 0.5 second dwell time at each corner
  } else {
    Serial.println(F("[IK ERROR] Target coordinate out of reach!"));
  }

  // Advance to next point
  currentPointIndex++;

  // Check if a full triangle loop has been completed
  if (currentPointIndex >= NUM_POINTS) {
    currentPointIndex = 0;
    loopCount++;

    // Check if 5 total loops have been completed
    if (loopCount >= MAX_LOOPS) {
      Serial.println(F("=================================================="));
      Serial.println(F("[SEQUENCE COMPLETE] 5 Loops Finished!"));
      Serial.println(F("Returning to Home Position (90 Deg)..."));
      Serial.println(F("=================================================="));
      
      // Return arms to vertical home position
      moveToTarget(1600, 1600); 
      sequenceComplete = true; // Lock execution
    }
  }
}

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------
bool solveInverseKinematics(double x, double y, double &out_theta1, double &out_theta2) {
  double dx1 = x - BASE1_X;
  double dy1 = y - BASE1_Y;
  double dx2 = x - BASE2_X;
  double dy2 = y - BASE2_Y;

  double D1 = hypot(dx1, dy1);
  double D2 = hypot(dx2, dy2);

  if (D1 > (LINK1 + LINK3) || D1 < fabs(LINK1 - LINK3) ||
      D2 > (LINK2 + LINK4) || D2 < fabs(LINK2 - LINK4)) {
    return false; 
  }

  double alpha1 = atan2(dy1, dx1);
  double alpha2 = atan2(dy2, dx2);

  double cos_beta1 = constrain((LINK1 * LINK1 + D1 * D1 - LINK3 * LINK3) / (2.0 * LINK1 * D1), -1.0, 1.0);
  double cos_beta2 = constrain((LINK2 * LINK2 + D2 * D2 - LINK4 * LINK4) / (2.0 * LINK2 * D2), -1.0, 1.0);

  double beta1 = acos(cos_beta1);
  double beta2 = acos(cos_beta2);

  out_theta1 = alpha1 + beta1;
  out_theta2 = alpha2 - beta2;

  return true;
}

void moveToTarget(int32_t targetSteps1, int32_t targetSteps2) {
  stepper1->moveTo(targetSteps1);
  stepper2->moveTo(targetSteps2);
  while (stepper1->isRunning() || stepper2->isRunning()) {
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}