#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
int minval = 300;
int maxval = 575;
int pulsval;
int servoval;

int l1 = 0;
int l2 = 1;
int l3 = 2;
int l4 = 3;


int r1 = 4;
int r2 = 5;


int r3 = 6;
int r4 = 7;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
void calib() {
    // a
   pwm.setPWM(l1,0,servoval);
       // a

     pwm.setPWM(l2,90,servoval);
      pwm.setPWM(l3,0,servoval);
       pwm.setPWM(l4,90,servoval);
    
     pwm.setPWM(r1,180,servoval);
     pwm.setPWM(r2,90,servoval);
     pwm.setPWM(r3,180,servoval);
     pwm.setPWM(r4,90,servoval);

       Serial.print("servo calibrated");

}
void setl1() {

 pwm.setPWM(l1,90,servoval);
 pwm.setPWM(l2,90,servoval);
     pwm.setPWM(0,90,servoval);
     
Serial.println("servo angle goes to 90 ange degrees");
}


void setl2() {
  // b button
 pwm.setPWM(l2,0,servoval);
     pwm.setPWM(l2,0,servoval);
Serial.println("servo angle goes to 0 ange degrees");
}
void sweepl2() {

 pwm.setPWM(l1,0,servoval);

 pwm.setPWM(l2,0,servoval);
 Serial.println("sweep" );
 delay(200);

pwm.setPWM(l2,90,servoval);
 Serial.println("sweep to 90 angle");

}


void sweepback() {
// b button
 pwm.setPWM(l1,180,servoval);

 pwm.setPWM(l2,90,servoval);


 pwm.setPWM(l3,140,servoval);

 pwm.setPWM(l4,90,servoval);

  pwm.setPWM(r1,180,servoval);


 pwm.setPWM(r2,0,servoval);
   pwm.setPWM(r3,180,servoval);
 pwm.setPWM(r4,0,servoval);
 Serial.println("sweep" );
delay(200);
pwm.setPWM(l2,90,servoval);
pwm.setPWM(l4,90,servoval);

pwm.setPWM(r2,90,servoval);
pwm.setPWM(r4,90,servoval);
 Serial.println("sweep to 90 angle");

}
void sweepforward() {

 pwm.setPWM(l1,0,servoval);

 pwm.setPWM(l2,0,servoval);


 pwm.setPWM(l3,0,servoval);

 pwm.setPWM(l4,0,servoval);

  pwm.setPWM(r1,180,servoval);


 pwm.setPWM(r2,180,servoval);
   pwm.setPWM(r3,180,servoval);
 pwm.setPWM(r4,180,servoval);
 Serial.println("sweep" );
 delay(200);

pwm.setPWM(l2,90,servoval);
pwm.setPWM(l4,90,servoval);

pwm.setPWM(r2,90,servoval);
pwm.setPWM(r4,90,servoval);
 Serial.println("sweep to 90 angle");

}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
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



   



void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
  calib();
    }

    if (ctl->b()) {
       
sweepback();
    }

    if (ctl->x()) {
    

    sweepl2();

    }

if (ctl->y()) {
   sweepforward();

}
    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
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
   
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
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
    servoval = map(pulsval,0,180,minval,maxval);
     pwm.begin();
    pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}
