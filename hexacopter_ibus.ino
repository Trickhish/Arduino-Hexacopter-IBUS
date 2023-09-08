#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/FlySkyIBus/FlySkyIBus.h"
#include <Adafruit_PWMServoDriver.h>
#include <Math.h>

// Constants //
#define PWMnbr 16
const int numInputChannels = 10;
const int failsafelimit = 100;
const int offVal = 900;
const int minVal = 1000;
const int maxVal = 2000;
const int MPU_addr=0x68;
// Constants END //

// GYRO //
const float Fa = 0.98; // smoothing factor

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int GminVal=265;
int GmaxVal=402;

double x;
double y;
double z;
double Fx=0;
double Fy=0;

int cX=0;
int cY=0;
int cZ=0;
// GYRO END //

// IBUS RX //
SoftwareSerial iBusSerial(8, 9); // RX, TX
FlySkyIBus iBus;
int LastInput[numInputChannels];
int channelInput[numInputChannels];
// IBUS RX END //

// FC VARS //
int motorVals[6]; // 6 motors values
unsigned long ts;
int status = 0;
bool change=false;

bool armed = false;
bool stab = true;

int xTargetAngle=0;
int yTargetAngle=0;
const int MaxAngle = 30;

const int MaxTurningSpeed = 50;
const int MinTurningSpeed = 10;
// FC VARS END //

int ease(float v){ return(int( ((v/MaxAngle)*(v/MaxAngle)) * MaxTurningSpeed )); } // f(x)= x^2 : ease-out

Adafruit_PWMServoDriver mdv = Adafruit_PWMServoDriver(0x40);

unsigned long lrm=millis();

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
  /*if ((millis()-lrm) > 1000) {
    lrm=millis();

    Serial.print("X:");
    Serial.print(int(x));

    Serial.print(",Y:");
    Serial.print(int(y));
  }*/

  Fx = Fa*Fx + (1-Fa) * x;
  Fy = Fa*Fy + (1-Fa) * y;

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

    if (channelInput[6] != LastInput[6]) { // callibration stick
      if (channelInput[6] > 1200) {
        Serial.print("Callibration ... ");
        Callibrate();
        Serial.println("Callibrated");
        
        /*Serial.print(cX);
        Serial.print(", ");
        Serial.println(cY);*/
      } else {
        Serial.println("Callibration reseted");
        cX=0;
        cY=0;
        cZ=0; 
      }
    }

    if (channelInput[7] != LastInput[7]) { // stabilisation stick
      if (channelInput[7] > 1500) { // OFF
        stab=false;

        Serial.println("Stabilisation Desactivated");
      } else { // ON
        stab=true;

        Serial.println("Stabilisation Activated");
      }
    }

    if (armed) { // motors control
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
          if (stab) { // stab on
            yTargetAngle = (MaxAngle*fv)/500;
            //Serial.print("Ytarget:");
            //Serial.print(yTargetAngle);
            //Serial.print(", Xtarget:");
          } else {
            motorVals[0] += fv;
            motorVals[5] += fv;
          }
        } else if (channelInput[1] < 1500) { // backward
          int fv = 1500-channelInput[1];
          if (stab) { // stab on
            yTargetAngle = -((MaxAngle*fv)/500);
            //Serial.print("Ytarget:");
            //Serial.print(yTargetAngle);
            //Serial.print(", Xtarget:");
          } else {
            motorVals[0] -= fv;
            motorVals[5] -= fv;
          }
        } else {
          yTargetAngle=0;
        }
      }

      if (true || channelInput[0]!=LastInput[0]) { // left - right
        if (channelInput[0] < 1500) { // left
          int fv = 1500-channelInput[0];
          if (stab) { // stab on
            xTargetAngle = -((MaxAngle*fv)/500);
            //Serial.println(xTargetAngle);
          } else {
            fv/=2; // divide by 2
            motorVals[4] += fv;
            motorVals[1] -= fv/2;
          }
        } else if (channelInput[0] > 1500) { // right
          int fv = channelInput[0]-1500;
          if (stab) {
            xTargetAngle = ((MaxAngle*fv)/500);
            //Serial.println(xTargetAngle);
          } else {
            fv/=2; // divide by 2
            motorVals[1] += fv;
            motorVals[4] -= fv/2;
          }
        }  else {
          xTargetAngle = 0;
        }
      }

      /*Serial.print(yTargetAngle);
      Serial.print(", ");
      Serial.println(xTargetAngle);*/

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

      if (stab) { // change motor values to attain the target angle
        int xDiff = xTargetAngle - Fx;
        int yDiff = yTargetAngle - Fy;
        // yDiff pos : avant
        // xDiff pos : droite
      
        /*Serial.print("yDiff:");
        Serial.print(yDiff);
        Serial.print(", xDiff:");
        Serial.println(xDiff);*/

        Serial.print(Fy);
        Serial.print(" -> ");

        int easedVal = ease(constrain(abs(yDiff), 0, MaxAngle));
        Serial.print(easedVal);
        //Serial.print("Yeased:");
        //Serial.print(easedVal);
        if (yDiff > 0) { // avant
          motorVals[0] += easedVal;
          motorVals[5] += easedVal;
        } else if (yDiff < 0) { // arrière
          motorVals[2] += easedVal;
          motorVals[3] += easedVal;
        }

        Serial.print(", ");
        Serial.print(Fx);
        Serial.print(" -> ");

        easedVal = ease(constrain(abs(xDiff), 0, MaxAngle));
        Serial.println(easedVal);
        //Serial.print(", Xeased:");
        //Serial.println(easedVal);
        if (xDiff > 0) { // droite
          motorVals[0] += easedVal;
          motorVals[1] += easedVal;
          motorVals[2] += easedVal;
        } else if (xDiff < 0) { // gauche
          motorVals[3] += easedVal;
          motorVals[4] += easedVal;
          motorVals[5] += easedVal;
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

        /*if (i!=0) {
          Serial.print(",");
        }
        Serial.print("Motor_");
        Serial.print(i);
        Serial.print(":");
        Serial.print(motorVals[i]);*/
      }
      //Serial.print("\n");

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

  /*x-=cX;
  y-=cY;
  z-=cZ;*/

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

  x-=cX;
  y-=cY;
  z-=cZ;
}

void Callibrate() {
  /*Wire.beginTransmission(MPU_addr);
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
  cZ=z;*/

  cX=x;
  cY=y;
  cZ=z;
}