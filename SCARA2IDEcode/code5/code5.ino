#include <Arduino.h>
#include <FastAccelStepper.h>
#include <math.h>

// --- Pins ---
#define MOTOR1_STEP_PIN    18
#define MOTOR1_DIR_PIN     19
#define MOTOR2_STEP_PIN    22
#define MOTOR2_DIR_PIN     23
#define SHARED_ENABLE_PIN  21
#define MAGNET_PIN         26 // Your working relay pin

// --- Kinematics ---
const double BASE1_X = -40.0, BASE1_Y = 0.0; 
const double BASE2_X =  40.0, BASE2_Y = 0.0; 
const double LINK1 = 110.0, LINK2 = 110.0;   
const double LINK3 = 160.0, LINK4 = 160.0;   
const double STEPS_PER_REV = 6400.0;
const double STEPS_PER_RAD = STEPS_PER_REV / (2.0 * M_PI);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

bool solveInverseKinematics(double x, double y, double &out_theta1, double &out_theta2);
void moveToTarget(int32_t targetSteps1, int32_t targetSteps2);
void setMagnet(bool state);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { delay(10); }

  // Ensure Magnet is OFF initially using the High-Impedance trick
  pinMode(MAGNET_PIN, INPUT);

  pinMode(SHARED_ENABLE_PIN, OUTPUT);
  digitalWrite(SHARED_ENABLE_PIN, LOW); 

  engine.init();
  stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);
  stepper2 = engine.stepperConnectToPin(MOTOR2_STEP_PIN);

  stepper1->setDirectionPin(MOTOR1_DIR_PIN, false);
  stepper1->setSpeedInHz(3200); 
  stepper1->setAcceleration(2000);

  stepper2->setDirectionPin(MOTOR2_DIR_PIN, false);
  stepper2->setSpeedInHz(3200); 
  stepper2->setAcceleration(2000);

  stepper1->setCurrentPosition(1600);
  stepper2->setCurrentPosition(1600);
  
  // Wait a moment, then tell Python we are ready!
  delay(2000);
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("VPICK ")) {
      int commaIndex = cmd.indexOf(',');
      if (commaIndex > 0) {
        float targetX = cmd.substring(6, commaIndex).toFloat();
        float targetY = cmd.substring(commaIndex + 1).toFloat();
        
        double theta1_rad, theta2_rad;

        // 1. MOVE TO OBJECT
        if (solveInverseKinematics(targetX, targetY, theta1_rad, theta2_rad)) {
          moveToTarget(round(theta1_rad * STEPS_PER_RAD), round(theta2_rad * STEPS_PER_RAD));
          
          // INCREASED: Let the arm completely stop shaking before grabbing
          delay(1000); 
          
          // 2. GRAB OBJECT
          setMagnet(true); 
          
          // INCREASED: Give the electromagnet plenty of time to build maximum flux
          delay(1500); 
          
          // 3. MOVE TO DROP-OFF (X=200, Y=0)
          if (solveInverseKinematics(200.0, 0.0, theta1_rad, theta2_rad)) {
            moveToTarget(round(theta1_rad * STEPS_PER_RAD), round(theta2_rad * STEPS_PER_RAD));
            
            // INCREASED: Wait for arm to settle at destination
            delay(500);
            
            // 4. DROP OBJECT
            setMagnet(false);
            
            // INCREASED: Ensure object has completely fallen before moving away
            delay(1000); 
            
            // 5. RETURN HOME
            moveToTarget(1600, 1600);
          }
        }
        // Tell Python we are done and ready for the next one!
        Serial.println("READY"); 
      }
    }
  }
}

void setMagnet(bool state) {
  if (state) {
    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW); // ON
  } else {
    pinMode(MAGNET_PIN, INPUT);    // OFF
  }
}

// (Keep your existing solveInverseKinematics and moveToTarget functions down here exactly as they were)
bool solveInverseKinematics(double x, double y, double &out_theta1, double &out_theta2) {
  double dx1 = x - BASE1_X;
  double dy1 = y - BASE1_Y;
  double dx2 = x - BASE2_X;
  double dy2 = y - BASE2_Y;
  double D1 = hypot(dx1, dy1);
  double D2 = hypot(dx2, dy2);
  if (D1 > (LINK1 + LINK3) || D1 < fabs(LINK1 - LINK3) || D2 > (LINK2 + LINK4) || D2 < fabs(LINK2 - LINK4)) {
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