// Time Variables
unsigned long time;
long lastGyroTime = -999999999;
long lastUltraSonicTime = -999999999;

unsigned long lastLogTime = 0;
const int LOG_TIME_DELAY = 5000;

// MIN/MAX Value variables
bool ballReset = true;
unsigned int ballMin, ballMax;

bool motionReset = true;
unsigned int motionMin, motionMax;

bool ultraReset = true;
unsigned long ultraMin, ultraMax;

bool gyroReset = true;
int16_t AcXMin, AcXMax, AcYMin, AcYMax, AcZMin, AcZMax, TmpMin, TmpMax, GyXMin, GyXMax, GyYMin, GyYMax, GyZMin, GyZMax;


// Ball Switch Constants

// HC-SR501 Constants
int pirPin = 7; // Input for HC-S501
int pirValue; // Place to store read PIR Value

// Gyroscope Constants
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// Ultrasonic Constants
#include "SR04.h"
#define TRIG_PIN 12
#define ECHO_PIN 11
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Ball Switch Setup
  ballSwitchSetup();

  // Motion Sensor Setup
  motionSensorSetup();

  // Gyro-Sensor Setup
  gyroSensorSetup();

  // Ultrasonic Sensor Setup
  ultrasonicSensorSetup();
}

void loop() {
  attemptLog();
  
  ballSwitchLoop();

  motionSensorLoop();

  gyroSensorLoop();

  ultrasonicSensorLoop();
}

void attemptLog() {
  time = millis();
  if(time - lastLogTime > LOG_TIME_DELAY) {
    Serial.print("Ball Switch Delta: "); Serial.println(ballMax - ballMin);
    Serial.print("Motion Delta: "); Serial.println(motionMax - motionMin);
    Serial.print("Ultrasonic Delta: "); Serial.print(ultraMax - ultraMin); Serial.println(" cm");
    
    Serial.println("Gyro Deltas: ");
    Serial.print("   AcX: "); Serial.println(AcXMax - AcXMin);
    Serial.print("   AcY: "); Serial.println(AcYMax - AcYMin);
    Serial.print("   AcZ: "); Serial.println(AcZMax - AcZMin);
    Serial.print("   Tmp: "); Serial.println(TmpMax - TmpMin);  //equation for temperature in degrees C from datasheet
    Serial.print("   GyX: "); Serial.println(GyXMax - GyXMin);
    Serial.print("   GyY: "); Serial.println(GyYMax - GyYMin);
    Serial.print("   GyZ: "); Serial.println(GyZMax - GyZMin);


    lastLogTime = time;
  }
}

void ballSwitchSetup() {
  pinMode(2,INPUT);
  digitalWrite(2, HIGH);
}

// Every Tick
void ballSwitchLoop() {
  int digitalVal = digitalRead(2);
  
  if(ballReset) {
    ballMin = digitalVal;
    ballMax = digitalVal;
  }
  
  if(digitalVal > ballMax) {
    ballMax = digitalVal;
  }

  if(digitalVal < ballMin) {
    ballMin = digitalVal;
  }
  
}

void gyroSensorSetup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

// Every 333 millieconds
const int GYRO_TIME_DELAY = 333;
void gyroSensorLoop() {
  time = millis();
  if(time - lastGyroTime > GYRO_TIME_DELAY) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Tmp = Tmp/340.00+36.53;
//    Serial.print("AcX = "); Serial.print(AcX);
//    Serial.print(" | AcY = "); Serial.print(AcY);
//    Serial.print(" | AcZ = "); Serial.print(AcZ);
//    Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//    Serial.print(" | GyX = "); Serial.print(GyX);
//    Serial.print(" | GyY = "); Serial.print(GyY);
//    Serial.print(" | GyZ = "); Serial.println(GyZ);


    if(gyroReset) {
      AcXMin = AcX;
      AcXMax = AcX;
      AcYMin = AcY;
      AcYMax = AcY;
      AcZMin = AcZ;
      AcZMax = AcZ;
      TmpMin = Tmp;
      TmpMax = Tmp;
      GyXMin = GyX;
      GyXMax = GyX;
      GyYMin = GyY;
      GyYMax = GyY;
      GyZMin = GyZ;
      GyZMax = GyZ;
    }
    
    if(AcX > AcXMax) {
      AcXMax = AcX;
    }
  
    if(AcX < AcXMin) {
      AcXMin = AcX;
    }

    if(AcY > AcYMax) {
      AcYMax = AcY;
    }
  
    if(AcY < AcYMin) {
      AcYMin = AcY;
    }

    if(AcZ > AcZMax) {
      AcZMax = AcZ;
    }
  
    if(AcZ < AcZMin) {
      AcZMin = AcZ;
    }

    if(Tmp > TmpMax) {
      TmpMax = Tmp;
    }
  
    if(Tmp < TmpMin) {
      TmpMin = Tmp;
    }

    if(GyX > GyXMax) {
      GyXMax = GyX;
    }
  
    if(GyX < GyXMin) {
      GyXMin = GyX;
    }

    if(GyY > GyYMax) {
      GyYMax = GyY;
    }
  
    if(GyY < GyYMin) {
      GyYMin = GyY;
    }

    if(GyZ > GyZMax) {
      GyZMax = GyZ;
    }
  
    if(GyZ < GyZMin) {
      GyZMin = GyZ;
    }
    
    lastGyroTime = time;
  }
  
}

void motionSensorSetup() {
  pinMode(pirPin, INPUT);
}

// Every Tick
void motionSensorLoop() {
  pirValue = digitalRead(pirPin);

  if(motionReset) {
    motionMin = pirValue;
    motionMax = pirValue;
  } 
    
   if(pirValue > motionMax) {
     motionMax = pirValue;
   }

   if(pirValue < motionMin) {
    motionMin = pirValue;
   } 
}

void ultrasonicSensorSetup() {}

// Every X Thousand milliseconds (try 500)
const int ULTRASONIC_TIME_DELAY = 1000;
void ultrasonicSensorLoop() {
  time = millis();
  if(time - lastUltraSonicTime > ULTRASONIC_TIME_DELAY) {
    a=sr04.Distance();
     
    lastUltraSonicTime = time;

    if(ultraReset) {
      ultraMin = a;
      ultraMax = a;
    }
    
    if(a > ultraMax) {
      ultraMax = a;
    }
  
    if(a < ultraMin) {
      ultraMin = a;
    }
  }
}
