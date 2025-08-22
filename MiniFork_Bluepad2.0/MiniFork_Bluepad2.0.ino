#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington !!!!!MUST USE 3.0.7!!!!!
#include <Bluepad32.h>

#include <ArduinoController.h> // setColorLED()


// Values that may need to be adjusted between vehicles/controllers
// LED colors config values for PS4
const uint8_t ledRed = 0;
const uint8_t ledGreen = 255; // green LED for example
const uint8_t ledBlue = 0;
// mast tilt servo config values
const uint mastTiltMinUS = 1050;  // Minimum pulse width (e.g., fully back)
const int mastTiltMaxUS = 1900;  // Maximum pulse width (e.g., fully forward)
int mastTiltValueUS = 1500;      // Starting midpoint
// steering config values
float steeringExpo = 1.0; // default steering expo (see onConnectedController for custom config)
// throttle config values
const float throttleDeadband = 40.0;
float throttleExpoFullSpeed = 1.0; // default throttle expo (see onConnectedController for custom config)
float throttleExpoHalfSpeed = 0.6; // default throttle expo (see onConnectedController for custom config)


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define steeringServoPin 23
#define mastTiltServoPin 22
#define cabLights 32
#define auxLights 33

#define mastMotor0 25  // Used for controlling auxiliary attachment movement
#define mastMotor1 26  // Used for controlling auxiliary attachment movement
#define auxAttach0 18  // Used for controlling auxiliary attachment movement
#define auxAttach1 17  // Used for controlling auxiliary attachment movement

#define leftMotor0 21   // Used for controlling the left motor movement
#define leftMotor1 19   // Used for controlling the left motor movement
#define rightMotor0 33  // Used for controlling the right motor movement
#define rightMotor1 32  // Used for controlling the right motor movement


#define FORWARD 1
#define BACKWARD -1
#define STOP 0

Servo steeringServo;
Servo mastTiltServo;

int lightSwitchTime = 0;
float adjustedSteeringValue = 86;
float steeringAdjustment = 1;
int steeringTrim = 0;

bool lightsOn = false;
bool hardLeft;
bool hardRight;

bool flagXBOX = false;
bool flagPS4 = false;

bool ledOn = false;
bool flagHalfSpeed = false;

int32_t GlobalAxisRXValue = 0;
uint8_t GlobalDpadValue = 0;
unsigned long GlobalCurrentTimeMS = 0;

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
                    
      if (properties.vendor_id == 0x045e) {
          // Microsoft Xbox controller
          printf("flagXBOX = true\n");
          flagXBOX = true;
          flagPS4 = false;
      } else if (properties.vendor_id == 0x054c) {
          // Sony DualShock or DualSense
          printf("flagPS4 = true\n");
          flagXBOX = false;
          flagPS4 = true;
          steeringExpo = 2.0; // expo for cheap PS4 controllers
          throttleExpoFullSpeed = 0.9; // expo for cheap PS4 controllers
          throttleExpoHalfSpeed = 0.4; // expo for cheap PS4 controllers
          ctl->setColorLED(ledRed, ledGreen, ledBlue);
      } else {
          printf("Unknown controller");
          flagXBOX = false;
          flagPS4 = false;
      }

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

void processGamepad(ControllerPtr ctl) {
  //Throttle
  processThrottle(ctl->axisY());
  //Steering
  processSteering(ctl->axisRX());
  GlobalAxisRXValue = ctl->axisRX();
  //Rasing and lowering of mast
  processMast(ctl->axisRY());
  //MastTilt
  GlobalDpadValue = ctl->dpad();
  //Aux
  processAux(ctl->thumbR());

  processTrimRight(ctl->r1());
  processTrimLeft(ctl->l1());

  if (ctl->l2()) {
    hardLeft = true;
  } else {
    hardLeft = false;
  }
  if (ctl->r2()) {
    hardRight = true;
  } else {
    hardRight = false;
  }

  // A button to toggle half speed
  static bool aButtonLastState = false;
  bool aCurrent = ctl->a();
  if (aCurrent && !aButtonLastState) {
    flagHalfSpeed = !flagHalfSpeed;
    if(flagHalfSpeed){
      Serial.println("half speed");
      ledOn = true;
      ctl->playDualRumble(0, 500, 0x00, 0xFF); // we are slow so play a long, strong (low freq) rumble
    } else {
      Serial.println("full speed");
      ctl->setColorLED(ledRed, ledGreen, ledBlue);
      ctl->playDualRumble(0, 200, 0xFF, 0x00); // we are fast so play a fast, weak (high freq) rumble
    }
  }
  aButtonLastState = aCurrent;
  // if we have a dual shock, flash the LED slow in half speed
  if (flagPS4 && flagHalfSpeed) {
    static int32_t blinkInterval = 1;
    static unsigned long lastLedToggleTime = 0;
    if (GlobalCurrentTimeMS - lastLedToggleTime >= blinkInterval) {
      lastLedToggleTime = GlobalCurrentTimeMS;
      ledOn = !ledOn;
      if (ledOn) {
        ctl->setColorLED(ledRed, ledGreen, ledBlue);
        blinkInterval = 1000;
      } else {
        ctl->setColorLED(0, 0, 0);
        blinkInterval = 500;
      }
    }
  }
}

void processThrottle(int axisYValue) {
  float normalized = axisYValue / 512.0;
  float curved = 0;
  float adjustedThrottleValue = 0;
  if (flagHalfSpeed) {
    curved = normalized * pow(fabs(normalized), throttleExpoHalfSpeed - 1);  // Preserves sign
    adjustedThrottleValue = curved * 128;
  } else {
    curved = normalized * pow(fabs(normalized), throttleExpoFullSpeed - 1);  // Preserves sign
    adjustedThrottleValue = curved * 256;
  }
  
  if (fabs(adjustedThrottleValue) > throttleDeadband) {
    if (hardRight) {
      moveMotor(rightMotor0, rightMotor1, -1 * (adjustedThrottleValue * steeringAdjustment));
    } else if (hardLeft) {
      moveMotor(leftMotor0, leftMotor1, -1 * (adjustedThrottleValue * steeringAdjustment));
    } else if (adjustedSteeringValue > 100) {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue * steeringAdjustment);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
    } else if (adjustedSteeringValue < 80) {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue * steeringAdjustment);
    } else {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
    }
  } else {
    moveMotor(leftMotor0, leftMotor1, 0);
    moveMotor(rightMotor0, rightMotor1, 0);
  }
}

void processMast(int axisRYValue) {
  int adjustedMastValue = axisRYValue / 2;
  if (adjustedMastValue > 100 || adjustedMastValue < -100) {
    moveMotor(mastMotor0, mastMotor1, adjustedMastValue);
  } else {
    moveMotor(mastMotor0, mastMotor1, 0);
  }
}
void processTrimRight(int trimValue) {
  static int previousTrimValue = 0;
  if (trimValue == 1 && previousTrimValue == 0 && trimValue < 20) {
    steeringTrim = steeringTrim + 2;
  }
  previousTrimValue = trimValue;
}

void processTrimLeft(int trimValue) {
  static int previousTrimValue = 0;
  if (trimValue == 1 && previousTrimValue == 0 && trimValue > -20) {
    steeringTrim = steeringTrim - 2;
  }
  previousTrimValue = trimValue;
}

void processSteering(int axisRXValue) {
  adjustedSteeringValue = (90 - (axisRXValue / 9));
  steeringServo.write(adjustedSteeringValue - steeringTrim);

  if (adjustedSteeringValue > 100) {
    steeringAdjustment = ((200 - adjustedSteeringValue) / 100);
  } else if (adjustedSteeringValue < 80) {
    steeringAdjustment = ((200 - (90 + (90 - adjustedSteeringValue))) / 100);
  }
}
void controlSteeringServo(int32_t axisRXValue) {
  const unsigned long steeringIntervalMS = 20; // using 50Hz servos, max speed here
  static unsigned long lastSteeringTimeMS = 0;
  if (GlobalCurrentTimeMS - lastSteeringTimeMS >= steeringIntervalMS) {
    float normalized = axisRXValue / 512.0;  // Range: -1.0 to +1.0
    float curved = normalized * pow(fabs(normalized), steeringExpo - 1);
    steeringServo.write((90 - (int)(curved * 55)) - steeringTrim);
    lastSteeringTimeMS = GlobalCurrentTimeMS;
  }
}

// Different controllers send data at different times, 
// so removing dependency on controller updates means more consistant mast movement
// For instance, XBOX One controllers do not send constant updates, they only send when they have new data
// this means if you hold a direction, you will only get 1 movement.
// Other controllers may send at 60Hz, or others at 135Hz!
void controlMastTiltServo(int dpadValue) {
  static bool initialPress = true;
  const int initialMastTiltStepUS = 30;
  const unsigned long initialPressDelayMS = 333;
  static unsigned long lastInitialTiltTimeMS = 0;
  const int mastTiltStepUS = 5;
  const unsigned long tiltIntervalMS = 20; // using 50Hz servos, max speed here
  static unsigned long lastTiltTimeMS = 0;
  if (dpadValue != 0) {
    if (initialPress) {
      initialPress = false;
      if (dpadValue == 1 && mastTiltValueUS < mastTiltMaxUS) {
        mastTiltValueUS += initialMastTiltStepUS;
        mastTiltServo.writeMicroseconds(mastTiltValueUS);
      } else if (dpadValue == 2 && mastTiltValueUS > mastTiltMinUS) {
        mastTiltValueUS -= initialMastTiltStepUS;
        mastTiltServo.writeMicroseconds(mastTiltValueUS);
      }
      //Serial.println("initialPress"); // uncomment if testing servo range
      //Serial.println(mastTiltServo.readMicroseconds()); // uncomment if testing servo range
      lastInitialTiltTimeMS = GlobalCurrentTimeMS;
    }
    else if (GlobalCurrentTimeMS - lastInitialTiltTimeMS >= initialPressDelayMS) {
      if (GlobalCurrentTimeMS - lastTiltTimeMS >= tiltIntervalMS) {
        if (dpadValue == 1 && mastTiltValueUS < mastTiltMaxUS) {
          mastTiltValueUS += mastTiltStepUS;
          mastTiltServo.writeMicroseconds(mastTiltValueUS);
        } else if (dpadValue == 2 && mastTiltValueUS > mastTiltMinUS) {
          mastTiltValueUS -= mastTiltStepUS;
          mastTiltServo.writeMicroseconds(mastTiltValueUS);
        }
        //Serial.println(mastTiltServo.readMicroseconds()); // uncomment if testing servo range
        lastTiltTimeMS = GlobalCurrentTimeMS;
      }
    }
  } else initialPress = true;
}
void processAux(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchTime) > 200) {
    if (lightsOn) {
      digitalWrite(auxAttach0, LOW);
      digitalWrite(auxAttach1, LOW);
      lightsOn = false;
    } else {
      digitalWrite(auxAttach0, HIGH);
      digitalWrite(auxAttach1, LOW);
      lightsOn = true;
    }

    lightSwitchTime = millis();
  }
}
void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 1) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -1) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(mastMotor0, OUTPUT);
  pinMode(mastMotor1, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
  digitalWrite(auxAttach0, LOW);
  digitalWrite(auxAttach1, LOW);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);


  steeringServo.attach(steeringServoPin);
  steeringServo.write(adjustedSteeringValue);
  mastTiltServo.attach(mastTiltServoPin);
  mastTiltServo.writeMicroseconds(mastTiltValueUS);

  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  // You could add additional error handling here,
  // such as logging the error or attempting to recover.
  // For example, you might attempt to reset the MCP23X17
  // and retry initialization before giving up completely.
  // Then, you could gracefully exit the program or continue
  // running with limited functionality.
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  GlobalCurrentTimeMS = millis();

  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  
  // control servos with a steady timebase instead of on controller updates
  controlMastTiltServo(GlobalDpadValue);
  controlSteeringServo(GlobalAxisRXValue);

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
  vTaskDelay(1);
}
