
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;
uint8_t devStatus;  

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
  }
}

void loop() {

}