#include <Bluepad32.h>

// Motor control pins for L298N
const int FL_IN1 = 12; // Front Left Motor IN1
const int FL_IN2 = 13; // Front Left Motor IN2

const int FR_IN1 = 25; // Front Right Motor IN1
const int FR_IN2 = 26; // Front Right Motor IN2

const int RL_IN1 = 27; // Rear Left Motor IN1
const int RL_IN2 = 14; // Rear Left Motor IN2

const int RR_IN1 = 32; // Rear Right Motor IN1
const int RR_IN2 = 33; // Rear Right Motor IN2

#define PS4_DEADZONE 5

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Function prototypes
void setMotors(int FL1, int FL2, int FR1, int FR2, int RL1, int RL2, int RR1, int RR2);
void controlMotorsWithJoystick(ControllerPtr gamepad);

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}


void setup() {
  Serial.begin(115200);
  Serial.println("PS4 Controlled Mecanum Wheel Car with Rotation");

  // Initialize motor pins as OUTPUT
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);

  // Setup Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
  bool dataUpdated = BP32.update();

  // Check for connected gamepads and control motors
  if (dataUpdated){
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      ControllerPtr controller = myControllers[i];
      if (controller && controller->isConnected()) {
        controlMotorsWithJoystick(controller);
      }
    }
  }

  delay(100);
}

// Function to control motors using joystick
void controlMotorsWithJoystick(ControllerPtr gamepad) {
  int x = gamepad->axisX();   // Left joystick X-axis (-512 to 512)
  int y = gamepad->axisY();   // Left joystick Y-axis (-512 to 512)
  int rx = gamepad->axisRX(); // Right joystick X-axis for rotation (-512 to 512)

  dumpGamepad(gamepad);

  // Deadzone to prevent small joystick movements from triggering motors
  if (abs(x) < PS4_DEADZONE) x = 0;
  if (abs(y) < PS4_DEADZONE) y = 0;
  if (abs(rx) < PS4_DEADZONE) rx = 0;

  // Rotation Control (Right Joystick X-axis)
  if (rx > 0) {  // Rotate Right (Clockwise)
    Serial.println("Right Rotate");
    setMotors(LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW);
  } else if (rx < 0) {  // Rotate Left (Counter-Clockwise)
    Serial.println("Left Rotate");
    setMotors(HIGH, LOW, LOW, HIGH, HIGH, LOW, LOW, HIGH);
  }
  // Movement Control (Left Joystick)
  else if (y > 0) {  // Forward
    Serial.println("Forward");
    setMotors(HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW);
  } else if (y < 0) {  // Backward
    Serial.println("Backward");
    setMotors(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH);
  } else if (x > 0) {  // Right
    Serial.println("Right");
    setMotors(LOW, HIGH, HIGH, LOW, HIGH, LOW, LOW, HIGH);
  } else if (x < 0) {  // Left
    Serial.println("Left");
    setMotors(HIGH, LOW, LOW, HIGH, LOW, HIGH, HIGH, LOW);
  } else {  // Stop
    Serial.println("Stop");
    setMotors(LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW);
  }   
}

// Function to set motor directions
void setMotors(int FL1, int FL2, int FR1, int FR2, int RL1, int RL2, int RR1, int RR2) {
  digitalWrite(FL_IN1, FL1);
  digitalWrite(FL_IN2, FL2);  
  digitalWrite(FR_IN1, FR1);
  digitalWrite(FR_IN2, FR2);
  digitalWrite(RL_IN1, RL1);
  digitalWrite(RL_IN2, RL2);
  digitalWrite(RR_IN1, RR1);
  digitalWrite(RR_IN2, RR2);
}
