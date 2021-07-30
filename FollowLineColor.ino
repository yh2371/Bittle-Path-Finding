/*
   Choose communication mode define here:
      I2C_MODE    : I2C mode, default pin: MU_SDA <==> ARDUINO_SDA, MU_SCL <==> ARDUINO_SCL
      SERIAL_MODE : Serial mode, default pin: MU_TX <==> ARDUINO_PIN3, MU_RX <==> ARDUINO_PIN2
*/
#define I2C_MODE
//#define SERIAL_MODE
#include <I2Cdev.h>
#include "OpenCat.h"


/*
   Choose MU address here: 0x60, 0x61, 0x62, 0x63
          default address: 0x60
*/
#define MU_ADDRESS        0x50 //in later versions we set the I2C device to 0x50, 0x51, 0x52, 0x53
#define ALT_MU_ADDRESS    0x60

#include <Arduino.h>
#include <MuVisionSensor.h>

#ifdef I2C_MODE
#include <Wire.h>
#endif
#ifdef SERIAL_MODE
#include <SoftwareSerial.h>
#define TX_PIN 2
#define RX_PIN 3
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif

MuVisionSensor *Mu;
MuVisionSensor Mu0(MU_ADDRESS);
MuVisionSensor Mu1(ALT_MU_ADDRESS);

int xCoord, yCoord; //the x y returned by the sensor
int xDiff, yDiff; //the scaled distance from the center of the frame
int currentX = 0, currentY = 0; //the current x y of the camera's direction in the world coordinate
int range = 10000; //the frame size 0~100 on X and Y direction
int skip = 500, counter; //an efforts to reduce motion frequency without using delay. set skip >1 to take effect
int i2cdelay = 3;

#define SKIP 3
#ifdef SKIP
byte updateFrame = 0;
#endif
byte firstValidJoint;
byte stage = 0;
char choice;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  uint8_t err = 0;
#ifdef I2C_MODE
  Wire.begin();
  // initialized MU on the I2C port
  err = Mu0.begin(&Wire);
#elif defined SERIAL_MODE
  mySerial.begin(9600);
  // initialized MU on the soft serial port
  err = Mu0.begin(&mySerial);
#endif
  if (err == MU_OK) {
    Serial.println("MU initialized");
    Mu = &Mu0;
  } else {
    Serial.println("fail to initialize");
    err = Mu1.begin(&Wire);
    if (err == MU_OK) {
      Serial.println("MU initialized");
      Mu = &Mu1;
    }
    delay(1000);
  }

  strcpy(lastCmd, "");
  // enable vision: ball
  (*Mu).VisionBegin(VISION_COLOR_DETECT);
  (*Mu).write(VISION_COLOR_DETECT, kLabel, MU_COLOR_WHITE);            // set detect color type: black
  (*Mu).CameraSetAwb(kLockWhiteBalance); 

  pwm.begin();
  pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates

  for (byte i = 0; i < DOF; i++) {
    pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
    servoCalibs[i] = servoCalib(i);
    calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirection(i) ;
  }
  shutServos();
  counter = 0;
  //  motion.loadBySkillName("rest");
  //  transform(motion.dutyAngles);

}

void loop() {
  // put your main code here, to run repeatedly:
  long time_start = millis();
  if (!(counter++ % skip)) {
      Serial.println("entered");
      checkLine();
  }
  {
#ifndef HEAD  //skip head
      if (jointIdx == 0)
        jointIdx = 2;
#endif
#ifndef TAIL  //skip tail
      if (jointIdx == 2)
        jointIdx = 4;
#endif
      if (motion.period != 1) {//skip non-walking DOF
        if (jointIdx < 4)
          jointIdx = 4;

      }
#if WALKING_DOF==8 //skip shoulder roll 
      if (jointIdx == 4)
        jointIdx = 8;
#endif
      int dutyIdx = timer * WALKING_DOF + jointIdx - firstValidJoint;
      calibratedPWM(jointIdx, motion.dutyAngles[dutyIdx] );
      //Serial.println("cali");
      jointIdx++;

      if (jointIdx == DOF) {
        jointIdx = 0;
#ifdef SKIP
      if (updateFrame++ == SKIP) {
          updateFrame = 0;
#endif
          timer = (timer + 1) % motion.period;
          Serial.println(timer);
#ifdef SKIP
        }
#endif
      }
    }
}

void checkLine(){
  long time_start = millis();
  int x;
  int move_idx = 5;
  // read result
  if ((*Mu).GetValue(VISION_COLOR_DETECT, kStatus)) {                    // update vision result and get status, 0: undetected, other: detected
    //Serial.println("vision color detected.");
    if ((*Mu).GetValue(VISION_COLOR_DETECT, kWidthValue) > 0){
      x = (*Mu).GetValue(VISION_COLOR_DETECT, kXValue);
      if (x < 40){
        move_idx = 2;
        //Serial.println("left.");
      }
      else if (x > 60){
        move_idx = 3;
        //Serial.println("right.");
      }
      else{
        move_idx = 1;
        //Serial.println("forward.");
      }
    }
    else{
      move_idx = 4;
      //Serial.println("trot.");
    }
  } 
  else {
    //Serial.println("vision color undetected.");
    move_idx = 5;
  }
  move(move_idx);
  //Serial.print("fps = ");
  //Serial.println(1000/(millis()-time_start));
  //Serial.println();
}

void move(int a){
  char cmd[CMD_LEN] = {};
  switch (a){
    case 1: {strcpy(cmd, "wkF");break;}
    case 2: {strcpy(cmd, "wkL");break;}
    case 3: {strcpy(cmd, "wkR");break;}
    case 4: {strcpy(cmd, "vt");break;}
    case 5: {strcpy(cmd, "balance");break;}
  }
  if (strcmp(lastCmd, cmd)){
    motion.loadBySkillName(cmd);
    Serial.println(motion.period);
    #ifdef DEVELOPER
        PTF("free memory: ");
        PTL(freeMemory());
#endif
        timer = 0;
        counter = 0;
    if (strcmp(cmd, "balance") && strcmp(cmd, "lifted") && strcmp(cmd, "dropped") )
         strcpy(lastCmd, cmd);
        
        // if posture, start jointIdx from 0
        // if gait, walking DOF = 8, start jointIdx from 8
        //          walking DOF = 12, start jointIdx from 4
        firstValidJoint = (motion.period == 1) ? 0 : DOF - WALKING_DOF;
        jointIdx = firstValidJoint;
        transform(motion.dutyAngles, 1, 1, firstValidJoint);
        //Serial.println("transformed");
        if (!strcmp(cmd, "rest")) {
          shutServos();
          token = T_REST;
        }
  }
}
