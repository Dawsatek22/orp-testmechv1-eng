
// This is the code for: orp_testmechv1. It can currently walk only forwards and backwards.
// This code is supposed to be used with  a esp32 boards with arduino uno r3 like form and a bluetooth gamepad(only Xbox Wireless model 1708 is tested with
// Like for example the : WeMos D1 R32 ESP32 4Mb Development Board WiFi Bluetooth Dual Core Arduino Uno R3,
// Or  the : Keyestudio ESP32 PLUS Development Board WROOM-32 module WIFI+Bluetooth Compatible with Arduino
// Links  for reference are here : https://www.keyestudio.com/products/keyestudio-esp32-plus-development-board-woroom-32-module-wifibluetooth-compatible-with-arduino
// And 
// Below are the required headers

#include <Bluepad32.h> // It is recomended to test and try the Bluepad32 out that handles bluetooth connectivity.
// Link for more info about Bluepad32 is here : https://bluepad32.readthedocs.io/en/latest/
#include <Wire.h> // for i2c communication 
#include <Adafruit_PWMServoDriver.h> // to be used with the Adafruit 16-Channel 12-bit PWM/Servo Shield - I2C interface
// link for reference is here: https://www.adafruit.com/product/1411?srsltid=AfmBOooFUhg39ym1zlCGINfeo1_jCtuc5Lzm2m5x-fd4L2a-6eSOJQRq


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // default address for the PWM/servoshield
// below are the minium and maximum values for the mg996R servo motors
int minval = 300; // minium
int maxval = 575; // maximum 
// below are the puls values and servo values for the mapping of servo position
int pulsval; 
int servoval;
// below are the pin integers for the actuators. l stand for leftside of the servos   and r stand for rightside servos of the robot
int l1 = 0; // left upper servo front
int l2 = 1; // left lower servo front
int l3 = 2; // left upper servo back
int l4 = 3; // left lower servo back


int r1 = 4; // right upper servo front 
int r2 = 5; // right lower servo front
int r3 = 6; // right upper servo back
int r4 = 7; // right lower servo back

// Below are the button values for the supported xbox wireless gamepad that are used for this project
int a_btn = 0x0001; // a
int b_btn = 0x0002; // b
int y_btn = 0x0008; // y
int x_btn = 0x0004; // x

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
void calib() { // calibrates servo position to 90 degrees
    // a
   pwm.setPWM(l1,90,servoval);
       // a

     pwm.setPWM(l2,90,servoval);
      pwm.setPWM(l3,90,servoval);
       pwm.setPWM(l4,90,servoval);
    
     pwm.setPWM(r1,90,servoval);
     pwm.setPWM(r2,90,servoval);
     pwm.setPWM(r3,90,servoval);
     pwm.setPWM(r4,90,servoval);

       Serial.print("servo calibrated");

}


}


void sweepback() { // walks backwards
// b button


pwm.setPWM(l1,150,servoval);
pwm.setPWM(l3,150,servoval);

pwm.setPWM(r1,30,servoval);
pwm.setPWM(r3,30,servoval);

pwm.setPWM(l2,90,servoval);
pwm.setPWM(l4,90,servoval);

pwm.setPWM(r2,90,servoval);
pwm.setPWM(r4,90,servoval);
delay(200);

 pwm.setPWM(l1,90,servoval);

 pwm.setPWM(l2,90,servoval);


 pwm.setPWM(l3,90,servoval);

 pwm.setPWM(l4,90,servoval);

  pwm.setPWM(r1,90,servoval);


 pwm.setPWM(r2,90,servoval);
   pwm.setPWM(r3,90,servoval);
 pwm.setPWM(r4,90,servoval);
 Serial.println("sweep" );
delay(500);

pwm.setPWM(l1,90,servoval);
pwm.setPWM(l3,90,servoval);

pwm.setPWM(r1,90,servoval);
pwm.setPWM(r3,90,servoval);


pwm.setPWM(l2,90,servoval);
pwm.setPWM(l4,90,servoval);

pwm.setPWM(l2,90,servoval);
pwm.setPWM(l4,90,servoval);

pwm.setPWM(r2,90,servoval);
pwm.setPWM(r4,90,servoval);
 Serial.println("sweep to 90 angle");

}
void sweepforward() {  // walks forwards
// y button
 Serial.println("going_forward");


pwm.setPWM(l1,20,servoval);
pwm.setPWM(l3,20,servoval);

pwm.setPWM(r1,150,servoval);
pwm.setPWM(r3,150,servoval);

pwm.setPWM(l2,90,servoval);
pwm.setPWM(l4,90,servoval);

pwm.setPWM(r2,90,servoval);
pwm.setPWM(r4,90,servoval);
delay(200);


pwm.setPWM(l1,90,servoval);
pwm.setPWM(l3,90,servoval);

pwm.setPWM(r1,90,servoval);
pwm.setPWM(r3,90,servoval);

pwm.setPWM(l2,90,servoval);
pwm.setPWM(l4,90,servoval);

pwm.setPWM(r2,90,servoval);
pwm.setPWM(r4,90,servoval);
delay(500);


pwm.setPWM(l1,90,servoval);
pwm.setPWM(l3,90,servoval);

pwm.setPWM(r1,90,servoval);
pwm.setPWM(r3,90,servoval);

pwm.setPWM(l2,90,servoval);
pwm.setPWM(l4,90,servoval);

pwm.setPWM(r2,90,servoval);
pwm.setPWM(r4,90,servoval);


/*
 pwm.setPWM(l1,70,servoval);
pwm.setPWM(l3,70,servoval);

pwm.setPWM(r1,110,servoval);
pwm.setPWM(r3,110,servoval);
delay(200); */
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

void dumpGamepad(ControllerPtr ctl) { // this print the button and joystick values
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
    if (ctl->buttons() == a_btn) {
  calib();
    }

    if (ctl->buttons() == b_btn) {
       
sweepback();
    }


if (ctl->buttons() ==  y_btn) {
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

    servoval = map(pulsval,0,180,minval,maxval); // the mapped servo values
     pwm.begin(); // wm begins
    pwm.setOscillatorFrequency(27000000); // oscillator frequency set
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
