/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

  This software may be distributed and modified under the terms of the GNU
  General Public License version 2 (GPL2) as published by the Free Software
  Foundation and appearing in the file GPL2.TXT included in the packaging of
  this file. Please note that GPL2 Section 2[b] requires that all works based
  on this software must also be made publicly available under the terms of
  the GPL2 ("Copyleft").

  Contact information
  -------------------

  Kristian Lauszus, TKJ Electronics
  Web      :  http://www.tkjelectronics.com
  e-mail   :  kristianl@tkjelectronics.com
*/
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Stepper.h>

//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

Stepper myStepper(800, 12, 13);
double actual, kP = 30, kI = 1, kD = 30, kPcons = 25, kDcons = 1.1, kIcons = 28, drive = 0, last, integral = 0, intTresh = 25, error, P, I, D, abserror;
int lenght, lenghtcm, left, right, pomak;
double t, lasttime = 0, pidtimer;
int sampletime = 50; // frekvencija reguliranja milisekunde, 20hz
int motorSpeed = 160;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine
void setup() {
  Serial.begin(115200);
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(3000); // Stabilizacija senzora

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {

  if (Serial.available() > 0) {   //unosenje duljine pomaka u centimetrima
    lenght = Serial.parseInt();
    Serial.println(lenght);
    myStepper.setSpeed(motorSpeed);
    lenghtcm = lenght * 130;
    myStepper.step(lenghtcm);
  }

  measure();
  kutni_pid();
}


void measure () {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Delta vrijeme -  vrijeme jednog ciklusa loopa u sekundama
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;   //konvertiranje iz radijana u stupnjeve

  double gyroXrate = gyroX / 131.0; // gyroXrate - konveriranje u deg/s
  double gyroYrate = gyroY / 131.0; // gryoYrate - konvertiranje u deg/s

  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // kut X s Kalmanovim filterom

  // Ovo popravlja problem tranzicije kada kut akcelerometra skoci od -180 do 180
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // kut Y s Kalmanovim filterom

  if (abs(kalAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
    kalAngleX = kalAngleX;
  }// Calculate the angle using a Kalman filter

  gyroXangle += gyroXrate * dt; // gyro angle bez filtera
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  /* Print Data */
  /*#if 0 // Set to 1 to activate
    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");

    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ); Serial.print("\t");

    Serial.print("\t");
    #endif */

  //Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  Serial.print("\t"); Serial.print(kalAngleX); Serial.print("\n"); //s ovime treba raditi

  //Serial.print("\t");

  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  //Serial.print(kalAngleY); Serial.print("\t"); //s ovime treba raditi

  //Serial.print("\r\n");

}

int kutni_pid () {

  t = millis();
  pidtimer = (double)(t - lasttime);
  if (pidtimer >= sampletime) {    //osigurava frekvenciju reguliranja 50hz

    actual = abs(kalAngleX);  //kut X bez predznaka
    error = 0 - actual;     //greska

    if (error > -0.6) {  //dopusteno odstupanje
      integral = 0;
      return 0;
    }

    if (error > intTresh) { //prevencija integralnog windupa, treshold = 45 stupnjeva
      integral = integral + error;
    }
    else integral = 0;

    if ( error > 15) {
      P = error * kP;
      I = integral * kI;
      D = (last - actual) * kD;
    }
    else {                  //ako smo blize zadanoj vrijednosti - koristi konzervativne parametre regulatora
      P = error * kPcons;
      I = integral * kIcons;
      D = (last - actual) * kDcons;
    }
    drive = abs(P + I + D);
    drive = doubleMap(drive, 0, 300, 50, 170);  //mapiranje izlaza
    drive = constrain(drive, 0, 170);  // ogranicavanje izlaza da ne prelazi maksimalnu brzinu

    myStepper.setSpeed(drive);
    if (kalAngleX < 0) {     // smjer kretanja
      myStepper.step(20); // duljina kretanja
    }
    else
      myStepper.step(-20);

    last = actual;  // kut za koristenje u iducoj iteraciji
    lasttime = millis();
  }
  //else Serial.print ("nije vrijeme");
}

double doubleMap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

