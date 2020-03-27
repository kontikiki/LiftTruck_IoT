#include <SparkFun_ADXL345.h>
#include <math.h>

#define SERIAL_BAUDRATE 9600  //serial baud rate

#define ACCEL_RANGE 8 //accelerometer range setting value

#define ACCEL_RATE 1.56  //accelerometer data rate setting value
#define CALIB_RATE 1.56  //data rate when accel calibration
#define SAMPLING_NUM 20.0 // number of sampling
#define CALIB_DELAY 640  //calibration sampling timing
#define ACCEL_DELAY 640 //measurement sampling timing

ADXL345 adxl = ADXL345();   //I2C
//ADXL345 adxl = ADXL345(3);  // SPI

typedef struct {
  float avg_svg;
  float svg_max;
} AccelData;

float base_accx, base_accy, base_accz;

void calibAccel();
void calculAccel(AccelData& accel);

void calibAccel() {
  int x, y, z;
  int32_t sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  adxl.setRate(CALIB_RATE);
  delay(CALIB_DELAY);

  for (int i = 0; i < 10; i++) {
    adxl.readAccel(&x, &y, &z);
    sumAcX += x;
    sumAcY += y;
    sumAcZ += z;
    delay(CALIB_DELAY);
  }

  base_accx = sumAcX / 10;
  base_accy = sumAcY / 10;
  base_accz = sumAcZ / 10.0;


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
  float cal_x, cal_y, cal_z;

  float total_svg = 0;
  float svg_acc = 0;
  float avg_svg = 0;
  float svg_max = 0;

  adxl.setRate(ACCEL_RATE);
  delay(ACCEL_DELAY);

  //1.563Hz sampling
  for (int i = 0; i < SAMPLING_NUM ; i++) {

    adxl.readAccel(&x, &y, &z);

    cal_x = (float)x - base_accx;
    cal_y = (float)y - base_accy;
    cal_z = (float)z - base_accz;

    svg_acc = sqrt(pow(cal_x, 2.0) + pow(cal_y, 2.0) + pow(cal_z, 2.0));

    // Output Results to Serial

    Serial.print(cal_x);
    Serial.print(", ");
    Serial.print(cal_y);
    Serial.print(", ");
    Serial.println(cal_z);

    Serial.print("svg= ");
    Serial.println(svg_acc);


    if (svg_acc > svg_max) {
      svg_max = svg_acc;
    }

    total_svg += svg_acc;

    delay(ACCEL_DELAY);
  }

  avg_svg = total_svg / SAMPLING_NUM;

  Serial.println("------------------");
  Serial.print("avg_svg =");
  Serial.println(avg_svg);
  Serial.print("svg_max=");
  Serial.println(svg_max);
  Serial.println("------------------");

  accel.avg_svg = avg_svg;
  accel.svg_max = svg_max;
}

void setup() {
  while (!Serial);
  Serial.begin(SERIAL_BAUDRATE);
  delayMicroseconds(1100);
  //accelerometer setting
  adxl.powerOn();
  adxl.setRangeSetting(ACCEL_RANGE);  // range settings : Accepted values are 2g, 4g, 8g or 16g
  adxl.setActivityXYZ(0, 0, 0); // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  //adxl.setActivityThreshold(ACT_THRESHOLD);  // 62.5mg per increment   // Set activity   // activity thresholds (0-255)

  adxl.setInactivityXYZ(0, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)

  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(20);

  adxl.setTapDetectionOnXYZ(0, 0, 0);

  adxl.setInterruptLevelBit(1);   //set INT pin HIGH (active:low, non:high)
  //adxl.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, ADXL345_INT1_PIN);

  adxl.InactivityINT(0);
  adxl.ActivityINT(0);  //Turn on Interrupts for Activity mode(1 == ON, 0 == OFF)
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);
}

void loop() {
  AccelData accel;
  if (Serial.available()) {
    int ch = Serial.read();
    if (ch =='1') {
      calculAccel(accel);
    } else if (ch =='2') {
      int x, y, z;
      adxl.readAccel(&x, &y, &z);
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
    }
  }
}
