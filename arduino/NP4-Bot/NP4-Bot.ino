


#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>
#include "Mpu.hpp"
#include "NP4.hpp"

NP4 np4;
MPU6050 mpu;


int DIR_PIN_L = 51;//blue
int STEP_PIN_L = 53;//purple
int ENABLE_PIN_L = 49;//green
int DIR_PIN_R = 45; //orange
int STEP_PIN_R = 43; //red
int ENABLE_PIN_R = 47;//yellow



volatile bool  pullLeft = false;
volatile bool  pullRight = false;


void setup() {
  
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(STEP_PIN_L, OUTPUT);
  pinMode(ENABLE_PIN_L, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);
  pinMode(STEP_PIN_R, OUTPUT);
  pinMode(ENABLE_PIN_R, OUTPUT);
   
  
 
  np4.setup();
}

void loop() {

  np4.loop();
}

void TC3_Handler()
{
  TC_GetStatus(TC1, 0);

  pullLeft = !pullLeft;
  digitalWrite(STEP_PIN_L, pullLeft);
  
}

 void TC4_Handler()
{

  TC_GetStatus(TC1, 1);

  pullRight = !pullRight;
  digitalWrite(STEP_PIN_R, pullRight);
}
