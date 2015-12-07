//
//  NP4.hpp
//  NP3
//
//  Created by Kris Temmerman on 15/11/15.
//
//

#ifndef NP4_hpp
#define NP4_hpp
#include "Arduino.h"

#include "Mpu.hpp"

class NP4
{


public:
    NP4(){};
    void setup();
    void loop();
    float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd);
    float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki);
    void setMotorSpeed(float speed,int motor);
    void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
    void setMotorEnabled(bool on );
    void manageCommands(String input);
    /////////////////////////////////////////////////////////
    
    float balP;
    float balD;
    float speedAdj;
    float maxAccel;
    bool turnedOn;
    float angleAdjust;
    float turningSpeed;
    float maxAccelset;
    float moveP;
    float moveI;
    float movingSpeed;
    
    float speedPID_errorSum;
    float actual_robot_speed;
    float actual_robot_speed_Old;
    float estimated_speed_filtered ;
    /////////////////////////////////////////////////////////
   
    String inputString;
    float inputAngleOld;
    
    unsigned long timerOld;
    float motorSpeed ;
    float currentSpeedL;
    float currentSpeedR;
    
    
    float PIDBal_errorOld2;
    float PIDBal_errorOld;
    float setPointOld;
    
    
    bool checkInput;   
    
    unsigned int testcount;
    
    /////////////////////////////////////////////////////////
    
    
    Mpu mpu;
 
    int DIR_PIN_L;//blue
    int STEP_PIN_L;//purple
    int ENABLE_PIN_L;//green
    int DIR_PIN_R; //orange
    int STEP_PIN_R; //red
    int ENABLE_PIN_R;//yellow

};

#endif /* NP4_hpp */
