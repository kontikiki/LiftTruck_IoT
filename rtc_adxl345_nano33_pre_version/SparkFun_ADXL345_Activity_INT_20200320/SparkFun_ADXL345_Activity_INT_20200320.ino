/*  *********************************************
    SparkFun_ADXL345_Example
    Triple Axis Accelerometer Breakout - ADXL345
    Hook Up Guide Example

    Utilizing Sparkfun's ADXL345 Library
    Bildr ADXL345 source file modified to support
    both I2C and SPI Communication

    E.Robert @ SparkFun Electronics
    Created: Jul 13, 2016
    Updated: Sep 06, 2016

    Development Environment Specifics:
    Arduino 1.6.11

    Hardware Specifications:
    SparkFun ADXL345
    Arduino Uno
 *  *********************************************/

#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library

/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
//ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

/****************** INTERRUPT ******************/
/*      Uncomment If Attaching Interrupt       */
int interruptPin = 2;                 // Setup pin 2 to be the interrupt pin (for most Arduino Boards)
volatile bool flag;

typedef struct {
  float avg_svg;
  float svg_max;
} AccelData;

float base_accx, base_accy, base_accz;

void calibAccel() {
  int x, y, z;
  int32_t sumAcX = 0, sumAcY = 0, sumAcZ = 0;

  for (int i = 0; i < 10; i++) {
    adxl.readAccel(&x, &y, &z);
    Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
    sumAcX += x;
    sumAcY += y;
    sumAcZ += z;
    delay(640);
  }

  base_accx = sumAcX / 10;
  base_accy = sumAcY / 10;
  base_accz = sumAcZ / 10;


  //  for debugging
  Serial.print("base_accx :");
  Serial.println(base_accx);
  Serial.print("base_accy :");
  Serial.println(base_accy);
  Serial.print("base_accz :");
  Serial.println(base_accz);

}

void calculAccel(AccelData& accel) {
  calibAccel();
  int x, y, z;
  float cal_x = 0, cal_y = 0, cal_z = 0;

  uint32_t total_svg = 0;
  float svg_acc = 0;
  float avg_svg = 0;
  float svg_max = 0;


  //1.563Hz sampling
  for (int i = 0; i < 20 ; i++) {

    adxl.readAccel(&x, &y, &z);

    cal_x = x - base_accx;
    cal_y = y - base_accy;
    cal_z = z - base_accz;

    svg_acc = sqrt(pow(cal_x, 2.0) + pow(cal_y, 2.0) + pow(cal_z, 2.0));

    if (svg_acc > svg_max) {
      svg_max = svg_acc;
    }

    total_svg += svg_acc;

    // Output Results to Serial
    /*
          Serial.print(cal_x);
          Serial.print(", ");
          Serial.print(cal_y);
          Serial.print(", ");
          Serial.println(cal_z);

          Serial.print("svg= ");
          Serial.println(svg_acc);
    */
    delay(640);
  }

  avg_svg = total_svg / 20.0;

  Serial.println("------------------");
  Serial.print("avg_svg =");
  Serial.println(avg_svg);
  Serial.print("svg_max=");
  Serial.println(svg_max);
  Serial.println("------------------");

  accel.avg_svg = avg_svg;
  accel.svg_max = svg_max;
}

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup() {
  Serial.begin(9600);// Start the serial terminal
  while (!Serial);
  Serial.println("SparkFun ADXL345 Accelerometer Hook Up Guide Example");
  Serial.println();
  flag = false;
  pinMode(interruptPin, INPUT_PULLUP);

  delayMicroseconds(1100);

  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(8);           // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity
  adxl.setRate(1.563);
  // adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
  // Default: Set to 1
  // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library

  adxl.setActivityXYZ(0, 0, 1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
  adxl.setInterruptLevelBit(1);
  adxl.setInactivityXYZ(0, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(0, 0, 0); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)

  //  Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment

  // Set values for what is considered FREE FALL (0-255)
  // adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  // adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment

  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(0, 0, 0, 1, 0);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
  // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
  // This library may have a problem using INT2 pin. Default to INT1 pin.

  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(0);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);

  attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, FALLING);   // Attach Interrupt

}

/****************** MAIN CODE ******************/
/*     Accelerometer Readings and Interrupt    */
void loop() {
  
  if(flag){
  // Accelerometer Readings
/*
  bool bit_state=adxl.getRegisterBit(ADXL345_FIFO_CTL,5);
  Serial.print("FIFO Triggered INT (0 -> INT1, 1-> INT2) : ");
  Serial.println(bit_state);

  Serial.print("FIFO Mode setting : ");
   bit_state=adxl.getRegisterBit(ADXL345_FIFO_CTL,7);
 Serial.print(bit_state);
 bit_state=adxl.getRegisterBit(ADXL345_FIFO_CTL,6);
 Serial.println(bit_state);

 Serial.print("FIFO Status bit(1 -> occur, 0 -> not yet) : ");
 bit_state=adxl.getRegisterBit(ADXL345_FIFO_STATUS,7);
 Serial.println(bit_state);
 */
 flag = false;
 delay(640);
  AccelData accel;
    calculAccel(accel);
 /*
  int x, y, z;
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

  // Output Results to Serial
  
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
  */
  
//  adxl.ActivityINT(1);
//  adxl.setActivityXYZ(0, 0, 1);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, FALLING);
}

}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  detachInterrupt(interruptPin);
//  adxl.ActivityINT(0);
//adxl.setActivityXYZ(0, 0, 0);
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();

  // Free Fall Detection
  if (adxl.triggered(interrupts, ADXL345_FREE_FALL)) {
    Serial.println("*** FREE FALL ***");
    //add code here to do when free fall is sensed
  }

  // Inactivity
  if (adxl.triggered(interrupts, ADXL345_INACTIVITY)) {
    Serial.println("*** INACTIVITY ***");
    //add code here to do when inactivity is sensed
  }

  // Activity
  if (adxl.triggered(interrupts, ADXL345_ACTIVITY)) {
    Serial.println("*** ACTIVITY ***");
    //add code here to do when activity is sensed
    flag = true;
  }

  // Double Tap Detection
  if (adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)) {
    Serial.println("*** DOUBLE TAP ***");
    //add code here to do when a 2X tap is sensed
  }

  // Tap Detection
  if (adxl.triggered(interrupts, ADXL345_SINGLE_TAP)) {
    Serial.println("*** TAP ***");
    //add code here to do when a tap is sensed
  }
}
