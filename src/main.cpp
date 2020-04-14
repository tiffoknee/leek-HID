#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



//  "OUTPUT_READABLE_YAWPITCHROLL" outpurs the yaw
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float yaw;
float pitch;
float roll;

const int backMagnetPin = 1;  // Used to monitor which buttons to press.
const int forwardMagnetPin = 7;
#define GYRO_INTERRUPT_PIN 0
const int buttonPin = 16;

volatile byte magnetHits;


unsigned long timeold;
// unsigned long timeold_back;
// unsigned long timeold_forward;

// void magnet_detect_back();
void magnet_detect_forward();

unsigned long lastHallSend;
unsigned int rpm;
bool direction;
bool lastDirection;
const long HOLD_MILLIS = 40;
const bool forward = true;
const bool backward = false;


void sendKeyPress(int key);
int getHidStatus(int pin);

int hidStatus;

int forwardStatus;  // Used to monitor which buttons to press.
int backwardStatus;
int leftStatus;
int rightStatus;
int nitroStatus;
int fireStatus;

unsigned long timeout;  // timeout variable used in loop
unsigned long forwardLastSent;  // Used to monitor which buttons to press.
unsigned long backwardLastSent;
unsigned long leftLastSent;
unsigned long rightLastSent;
unsigned long nitroLastSent;
unsigned long fireLastSent;
unsigned long buttonDebounce;

bool leftPressed;
bool rightPressed;
bool firePressed;
bool forwardPressed;
bool backwardPressed;
bool nitroPressed;

bool enableHID;
bool buttonState;
unsigned long buttonPressedLast;

// slow down the serial output
unsigned long debounce;
unsigned long lastSent;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  rpm = 0;
  magnetHits = 0;
  // timeold = 0;
  direction = forward;
  lastDirection = backward;

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(backMagnetPin, INPUT_PULLUP);
  pinMode(forwardMagnetPin, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(backMagnetPin), magnet_detect_back, RISING);//Initialize the interrupt pin (Arduino digital pin 2)
  attachInterrupt(digitalPinToInterrupt(forwardMagnetPin), magnet_detect_forward, RISING);//Initialize the interrupt pin (Arduino digital pin 2)

  forwardLastSent = 0;
  backwardLastSent = 0;
  leftLastSent = 0;
  rightLastSent = 0;
  nitroLastSent = 0;
  fireLastSent = 0;

  leftPressed = false;
  rightPressed = false;

  firePressed = false;
  nitroPressed = false;
  forwardPressed = false;
  backwardPressed = false;


  enableHID = false;
  buttonPressedLast = 0;

  //Keyboard.begin();

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(GYRO_INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(GYRO_INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(GYRO_INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    debounce = 1000;
}


void mpu_loop() {

  if ((millis() - lastSent) > debounce) { //slow this down and don't overload the buffer..
    lastSent = millis();
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            yaw = (ypr[0] * 180/M_PI);
            pitch = (ypr[1] * 180/M_PI);
            roll = (ypr[2] * 180/M_PI);

            // Serial.print("ypr\t");
            // Serial.print(yaw);
            // Serial.print("\t");
            // Serial.print(pitch);
            // Serial.print("\t");
            // Serial.println(roll);
            //
            // if (roll <= -11) {
            //   Serial.print("LEFT\t");
            //   leftStatus = true;
            // }else{
            //   Serial.print("---\t");
            //   leftStatus = false;
            // }
            //
            // if (roll > -11 && roll < 11) {
            //   Serial.print("CENTER\t");
            // }else{
            //   Serial.print("---\t");
            // }
            //
            // if (roll >= 18) {
            //   Serial.print("RIGHT\t");
            //   rightStatus = true;
            // }else{
            //   Serial.print("---\t");
            //   rightStatus = false;
            // }
            //
            // if (pitch > 42) {
            //   Serial.print("FIRE\t");
            //   fireStatus = true;
            // }else{
            //   Serial.print("---\t");
            //   fireStatus = false;
            // }
         #endif



    }
        // blink LED to indicate activity
        // blinkState = !blinkState;
        // digitalWrite(LED_PIN, blinkState);
    }
}

void hall_loop(){

  if(millis() - lastHallSend > debounce){ //Change speed every quarter second

    if(direction == forward){
      Serial.print(millis());
      Serial.print(" ");
      Serial.print(timeold);
      Serial.print(" ");
      Serial.print("FORWARDS:");
      Serial.print(rpm);
      Serial.print("\thits:\t");
      Serial.print(magnetHits);
      if(digitalRead(backMagnetPin) == HIGH){Serial.print("\tBACK:HIGH");}else{Serial.print("\tBACK:LOW");}
      if(digitalRead(forwardMagnetPin) == HIGH){Serial.println("\tFWD:HIGH");}else{Serial.println("\tFWD:LOW");}

      if(rpm > 50){
        forwardStatus = true;
        nitroStatus = true;
      }else if(rpm > 35 && rpm < 50){
        forwardStatus = true;
        nitroStatus = false;
      }else{
        forwardStatus = false;
        nitroStatus = false;
      }
    }

    if(direction == backward){
      Serial.print(millis());
      Serial.print(" ");
      Serial.print(timeold);
      Serial.print(" ");
      Serial.print("BACKWARDS:");
      Serial.print(rpm);
      Serial.print("\thits:\t");
      Serial.print(magnetHits);
      if(digitalRead(backMagnetPin) == HIGH){Serial.print("\tBACK:HIGH");}else{Serial.print("\tBACK:LOW");}
      if(digitalRead(forwardMagnetPin) == HIGH){Serial.println("\tFWD:HIGH");}else{Serial.println("\tFWD:LOW");}

      if(rpm > 35){

        backwardStatus = true;
        nitroStatus = false;
      }else{
        backwardStatus = false;
        nitroStatus = false;
      }
    }

    lastHallSend = millis();

  }

  if(millis() - (timeold) > 3000){ //timeout if no pedalling. TODO - Make this more granular
    rpm = 0;
  }

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop(void)
{

  mpu_loop();
  hall_loop();

}

void magnet_detect_forward()//This function is called whenever a magnet/interrupt is detected by the arduino
{
  if(digitalRead(backMagnetPin) == HIGH){
    direction = forward;
  }else{
    direction = backward;  
  }

  if(direction != lastDirection){
    rpm = 0;
    magnetHits = 1;
  }else{
    magnetHits++;
    rpm = 60L*1000L/((millis() - timeold));
    rpm = (rpm > 400) ? 0 : rpm;
  }

  timeold = millis();
  lastDirection = direction;


}
