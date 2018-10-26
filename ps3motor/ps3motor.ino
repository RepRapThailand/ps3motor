/*
 Example sketch for the PS3 Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

/*
 * Some code is from Adafruit's Motor Shield V2 DCMotorTest.ino.
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M2
Adafruit_DCMotor *myMotor = AFMS.getMotor(2);

#include <PS3BT.h>
#include <usbhub.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
//USBHub Hub1(&UsbH); // Some dongles have a hub inside

BTD Btd(&UsbH); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;

void setup_motor() {
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(0);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  SerialDebug.println(F("\r\nMotor controller started"));
}

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); //halt
  }
  SerialDebug.print(F("\r\nPS3 Bluetooth Library Started"));

  setup_motor();
}

void go_forward(bool forward) {
  static bool dir_backward = false;
  if (dir_backward && forward) {
    myMotor->run(FORWARD);
    dir_backward = false;
    SerialDebug.print(F("\tdirection: "));
    SerialDebug.print("forward");
  }
  if (!dir_backward && !forward) {
    myMotor->run(BACKWARD);
    dir_backward = true;
    SerialDebug.print(F("\tdirection: "));
    SerialDebug.print("backward");
  }
}

uint8_t lastLeftHatX, lastLeftHatY;

void loop() {
  UsbH.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    uint8_t joyX = PS3.getAnalogHat(LeftHatX);
    uint8_t joyY = PS3.getAnalogHat(LeftHatY);
    if (lastLeftHatX != joyX || lastLeftHatY != joyY) {
      lastLeftHatX = joyX; lastLeftHatY = joyY;
      SerialDebug.print(F("\r\nLeftHatX: "));
      SerialDebug.print(joyX);
      SerialDebug.print(F("\tLeftHatY: "));
      SerialDebug.print(joyY);
      // Joystick/hat y-axis controls motor speed and direction
      uint8_t speed;
      if (joyY < 128) {         // forward
        go_forward(true);
        speed = (uint8_t)map(joyY, 127, 0, 0, 255);
      }
      else {                    // backwards
        go_forward(false);
        joyY -= 128;
        speed = (uint8_t)map(joyY, 0, 127, 0, 255);
      }
      SerialDebug.print(F("\tjoyY: "));
      SerialDebug.print(joyY);
      SerialDebug.print(F("\tspeed: "));
      SerialDebug.print(speed);
      myMotor->setSpeed(speed);
    }
    if (PS3.PS3Connected) { // The Navigation controller only have one joystick
      joyX = PS3.getAnalogHat(RightHatX);
      joyY = PS3.getAnalogHat(RightHatY);
      if (joyX > 137 || joyX < 117 || joyY > 137 || joyY < 117) {
        SerialDebug.print(F("\tRightHatX: "));
        SerialDebug.print(joyX);
        SerialDebug.print(F("\tRightHatY: "));
        SerialDebug.print(joyY);
      }
    }

    // Analog button values can be read from almost all buttons
    if (PS3.getAnalogButton(L2) || PS3.getAnalogButton(R2)) {
      SerialDebug.print(F("\r\nL2: "));
      SerialDebug.print(PS3.getAnalogButton(L2));
      if (PS3.PS3Connected) {
        SerialDebug.print(F("\tR2: "));
        SerialDebug.print(PS3.getAnalogButton(R2));
      }
    }

    if (PS3.getButtonClick(PS)) {
      SerialDebug.print(F("\r\nPS"));
      PS3.disconnect();
    }
    else {
      if (PS3.getButtonClick(TRIANGLE)) {
        SerialDebug.print(F("\r\nTraingle"));
        PS3.setRumbleOn(RumbleLow);
      }
      if (PS3.getButtonClick(CIRCLE)) {
        SerialDebug.print(F("\r\nCircle"));
        PS3.setRumbleOn(RumbleHigh);
      }
      if (PS3.getButtonClick(CROSS))
        SerialDebug.print(F("\r\nCross"));
      if (PS3.getButtonClick(SQUARE))
        SerialDebug.print(F("\r\nSquare"));

      if (PS3.getButtonClick(UP)) {
        SerialDebug.print(F("\r\nUp"));
        if (PS3.PS3Connected) {
          PS3.setLedOff();
          PS3.setLedOn(LED4);
        }
      }
      if (PS3.getButtonClick(RIGHT)) {
        SerialDebug.print(F("\r\nRight"));
        if (PS3.PS3Connected) {
          PS3.setLedOff();
          PS3.setLedOn(LED1);
        }
      }
      if (PS3.getButtonClick(DOWN)) {
        SerialDebug.print(F("\r\nDown"));
        if (PS3.PS3Connected) {
          PS3.setLedOff();
          PS3.setLedOn(LED2);
        }
      }
      if (PS3.getButtonClick(LEFT)) {
        SerialDebug.print(F("\r\nLeft"));
        if (PS3.PS3Connected) {
          PS3.setLedOff();
          PS3.setLedOn(LED3);
        }
      }

      if (PS3.getButtonClick(L1))
        SerialDebug.print(F("\r\nL1"));
      if (PS3.getButtonClick(L3))
        SerialDebug.print(F("\r\nL3"));
      if (PS3.getButtonClick(R1))
        SerialDebug.print(F("\r\nR1"));
      if (PS3.getButtonClick(R3))
        SerialDebug.print(F("\r\nR3"));

      if (PS3.getButtonClick(SELECT)) {
        SerialDebug.print(F("\r\nSelect - "));
        PS3.printStatusString();
      }
      if (PS3.getButtonClick(START)) {
        SerialDebug.print(F("\r\nStart"));
        printAngle = !printAngle;
      }
    }
  }
}
