#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/FlySkyIBus/FlySkyIBus.h"
#include <Adafruit_PWMServoDriver.h>

// Constants
#define PWMnbr 16
const int numInputChannels = 10;
const int failsafelimit = 100;
const int offVal = 900;

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int GminVal=265;
int GmaxVal=402;

double x;
double y;
double z;

int cX=0;
int cY=0;
int cZ=0;

SoftwareSerial iBusSerial(8, 9); // RX, TX
FlySkyIBus iBus;
int LastInput[numInputChannels];
int channelInput[numInputChannels];
int motorVals[6]; // 6 motors values
int minVal = 1000;
int maxVal = 2000;
unsigned long ts;
int status = 0;
bool armed = false;
bool change=false;

Adafruit_PWMServoDriver mdv = Adafruit_PWMServoDriver(0x40);

int lrm=millis();

void setup() {
  mdv.begin();
  mdv.setPWMFreq(200); // 60, 400

  iBusSerial.begin(115200);
  iBus.begin(iBusSerial);

  Wire.begin();
  Wire.setClock(400000);
  ts = millis();

  for(int i=0; i<6; i++) {
    mdv.writeMicroseconds(i, offVal);
    motorVals[i]=offVal;
  }

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Callibrate();

  Serial.begin(115200);
  Serial.println("\nReady!");
}

void loop() {
  change=false;

  // GYRO data
  readGyro();
  if ((millis()-lrm) > 1000) {
    Serial.print("X:");
    Serial.print(int(x));

    Serial.print(",Y:");
    Serial.print(int(y));

    Serial.print(",Z:");
    Serial.print(int(z));
    Serial.println("");
  }

  // RX data
  if (iBus.loop()) {
    if (status==0) {
      Serial.println("Connected");
    }
    status=1;
    ts = millis();

    for (int i = 0; i < numInputChannels; i++) { // récupération des channels
      LastInput[i]=channelInput[i];
      channelInput[i] = iBus.readChannel(i);
      if (LastInput[i]!=channelInput[i]) {
        change=true;
      }
      //Serial.print(channelInput[i]);
      //Serial.print(",");
    }
    //Serial.println("");

    if (channelInput[4]!=LastInput[4] && (channelInput[4]>1500)) { // arm stick
      if (armed==false) {
        if (channelInput[2] < 1025) { // throttle down         
          Serial.println("Armed");
          armed=true;
        } else {
          Serial.println("Failed to arm, throttle not down");
        }
      }
    } else if ((channelInput[4]<=1500) && armed==true) {
      Serial.println("Disarmed");
      for(int i=0; i<6; i++) {
        motorVals[i]=offVal;
      }
      armed=false;
    }

    if (armed) {
      if (true || channelInput[2]!=LastInput[2]) { // throttle
        if (channelInput[2] < 1025) {
          //channelInput[2]=900;
        }
        for(int i=0; i<6; i++) {
          motorVals[i]=channelInput[2];
        }
      }

      if (true || channelInput[1]!=LastInput[1]) { // forward - backward
        if (channelInput[1] > 1500) { // forward
          int fv = channelInput[1]-1500;
          motorVals[0] += fv;
          motorVals[5] += fv;
        } else if (channelInput[1] < 1500) { // backward
          int fv = 1500-channelInput[1];
          motorVals[0] -= fv;
          motorVals[5] -= fv;
        }
      }

      if (true || channelInput[0]!=LastInput[0]) { // left - right
        if (channelInput[0] < 1500) { // left
          int fv = 1500-channelInput[0];
          fv/=2; // divide by 2
          motorVals[4] += fv;
          motorVals[1] -= fv/2;
        } else if (channelInput[0] > 1500) { // right
          int fv = channelInput[0]-1500;
          fv/=2; // divide by 2
          motorVals[1] += fv;
          motorVals[4] -= fv/2;
        } 
      }

      if (channelInput[3] < 1500) { // turn left
        int fv = 1500-channelInput[3];
        for(int i=0; i<6; i+=2) {
          motorVals[i] += fv;
          if (motorVals[i] > 2000) {
            int a = motorVals[i]-2000;
            motorVals[i]=2000;
            motorVals[(i+3)%6] -= a;
          }
        }
      } else if (channelInput[3] > 1500) { // turn right
        int fv = channelInput[3]-1500;
        for(int i=1; i<6; i+=2) {
          motorVals[i] += fv;
          if (motorVals[i] > 2000) {
            int a = motorVals[i]-2000;
            motorVals[i]=2000;
            motorVals[(i+3)%6] -= a;
          }
        }
      }
    }

    clampMotorValues(); // clamp values before turn orders

    if (change) {
      for(int i=0; i<6; i++) {
        /*if (motorVals[i]>2000) { // motor value too high
          int diff = motorVals[i]-2000;
          motorVals[i]=2000;
          for(int ii=0; ii<6; ii++) {
            if (ii!=i) {
              motorVals[ii]-=diff;
            }
          }
        } else if (motorVals[i]<1000) { // motor value too low
          int diff = 1000-motorVals[i];
          motorVals[i]=1000;
          for(int ii=0; ii<6; ii++) {
            if (ii!=i) {
              motorVals[ii]+=diff;
            }
          }
        }*/

        if (i!=0) {
          Serial.print(",");
        }
        Serial.print("Motor_");
        Serial.print(i);
        Serial.print(":");
        Serial.print(motorVals[i]);
      }
      Serial.print("\n");

      for(int i=0; i<6; i++) {
        mdv.writeMicroseconds(i, motorVals[i]);
      }
    }
  } else if ((millis() - ts) > failsafelimit) { // no data
    if (status==1) {
      status=0;
      failsafe();
    }
  }
}

void clampMotorValues() {
  for(int i=0; i<6; i++) {
        if (motorVals[i]>2000) { // motor value too high
          int diff = motorVals[i]-2000;
          motorVals[i]=2000;
          for(int ii=0; ii<6; ii++) {
            if (ii!=i) {
              motorVals[ii]-=diff;
            }
          }
        } else if (motorVals[i]<1000) { // motor value too low
          int diff = 1000-motorVals[i];
          motorVals[i]=1000;
          for(int ii=0; ii<6; ii++) {
            if (ii!=i) {
              motorVals[ii]+=diff;
            }
          }
        }
  }
}

void failsafe() {
  armed=false;

  for(int i=0; i<6; i++) {
    motorVals[i]=offVal;
    mdv.writeMicroseconds(i, offVal);
  }

  Serial.println("Failsafe"); // radio disconnected, launch gps rescue mode / shutoff / hold
}

void readGyro() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  
  int xAng = map(AcX,GminVal,GmaxVal,-90,90);
  int yAng = map(AcY,GminVal,GmaxVal,-90,90);
  int zAng = map(AcZ,GminVal,GmaxVal,-90,90);

  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  x-=cX;
  y-=cY;
  z-=cZ;

  if (x > 180) {
    x= -360+x;
  } else if (x < -180) {
    x= -360-x;
  }
  if (y > 180) {
    y= -360+y;
  } else if (y < -180) {
    y= -360-y;
  }
  y=-y;
  if (z > 180) {
    z= -360+z;
  } else if (z < -180) {
    z= -360-z;
  }
}

void Callibrate() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);

  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  cX=x;
  cY=y;
  cZ=z;
}