//
//  NP4.cpp
//  NP3
//
//  Created by Kris Temmerman on 15/11/15.
//
//

#include "NP4.hpp"


void NP4::setup()
{
    balP=8.0f;
    balD =0.f;
    
    moveP=1;
    moveI=0.0;
    
    speedAdj = 2000;
    maxAccel = 200;
    maxAccelset =maxAccel;
    ///////////////////////////////////////////////////
    
    Serial2.begin(115200);
    Serial.begin(115200);
    Serial.println("setup");
    
    mpu.setup();
    
    
    DIR_PIN_L = 51;//blue
    STEP_PIN_L = 53;//purple
    ENABLE_PIN_L = 49;//green
    DIR_PIN_R = 45; //orange
    STEP_PIN_R = 43; //red
    ENABLE_PIN_R = 47;//yellow
    setMotorEnabled(false);
    
    inputAngleOld =0;
    
    timerOld =0;
    
    PIDBal_errorOld2=0;
    PIDBal_errorOld=0;
    currentSpeedL=0;
    currentSpeedR =0;
    
    angleAdjust =0;
    turningSpeed =0;
    turnedOn =false;
    movingSpeed =0;
    speedPID_errorSum=0;
    
    actual_robot_speed=0;
    actual_robot_speed_Old=0;
    estimated_speed_filtered=0 ;
    checkInput =true;
};
void NP4::loop()
{
 


    
    if(mpu.dmpReady && mpu.update() &&  turnedOn)
    {
        unsigned long timer_value = millis();
        unsigned long dt = (timer_value - timerOld);
        timerOld = timer_value;
        
        
        float inputAngle = mpu.angle +angleAdjust;
        
        
        
        
        actual_robot_speed_Old = actual_robot_speed;
        actual_robot_speed = (currentSpeedL + currentSpeedR )/2;
        actual_robot_speed/=speedAdj;
        
        
       //fix this
        float angular_velocity = (inputAngle-inputAngleOld)*90.0;
        float estimated_speed = actual_robot_speed_Old - angular_velocity;
        float estimated_speed_filtered = estimated_speed_filtered*0.95 + (float)estimated_speed*0.05;

        inputAngleOld =inputAngle;
        
        

        
        float target_angle = speedPIControl(dt,estimated_speed_filtered,movingSpeed,moveP,moveI);
        target_angle = -constrain(target_angle,-12,12);
        
        //Serial.println(target_angle);
        
        float tError = abs (inputAngle-target_angle);
        maxAccelset = maxAccel;
        if(movingSpeed==0 && tError<0.5)
        {
            maxAccelset*=0.2;
        }
        else if( tError<1.5)
        {
            maxAccelset*=0.5;
        }
        
        //float errorAcc = abs (target_angle-inputAngle)*50;
       
        motorSpeed = stabilityPDControl(dt, inputAngle, target_angle, balP, balD);
        float  motorSpeedAdj = motorSpeed * speedAdj;
        motorSpeedAdj =constrain( motorSpeedAdj,-40000,40000);
        if(inputAngle>19 || inputAngle<-19)
        {
         
           setMotorEnabled(false);
            setMotorSpeed(1 ,0 );
            setMotorSpeed(1 ,1 );
        }else
        {
            
          
            setMotorSpeed(motorSpeedAdj +turningSpeed ,0 );
            setMotorSpeed(motorSpeedAdj -turningSpeed ,1 );
            setMotorEnabled(true);
        }
       
        checkInput =true;
    }
    else if(  checkInput)
    {
        if (turnedOn &&mpu.dmpReady)
        {
            checkInput =false;
        }
    
        while (Serial2.available())
        {
            char inChar = (char)Serial2.read();
            
            
            if (inChar == '\n')
            {
                manageCommands(inputString);
                inputString ="";
                
            }else
            {
                inputString += inChar;
            }
            
        }
    
        
    
    
    }
};




void NP4::manageCommands(String inputString)
{

    String command = inputString.substring(0, 2);
    String valueS = inputString.substring(2);
    float value = valueS.toFloat();
    //turnedOn =true
    //balP=0.2f;
    //balD =26.f;
    //speedAdj = 4000;
    //maxAccel = 100;
    //turnedOn =true
    //angleAdjust =0;
    
    //turningSpeed =0;
    Serial2.print("got ");
    Serial2.print(command);
    Serial2.print("->");
    Serial2.println(value);
    //Serial2.println("ok");
    if (command == "on")
    {
        if(value==0)
        {
            turnedOn =false;
            setMotorEnabled(false );
        }else
        {
            turnedOn=true;
            setMotorEnabled(true );
        }
    }
    else if (command == "aa")
    {
        angleAdjust = value;
    }
    else if (command == "sa")
    {
        speedAdj =value;
    }
    else if (command == "ma")
    {
        maxAccel =value;
        
    } else if (command == "bp")
    {
        balP = value;
        
    } else if (command == "bd")
    {
        balD =value;
        
    }else if (command == "mp")
    {
        moveP = value;
        
    } else if (command == "mi")
    {
        moveI =value;
        
    }else if(command=="tu")
    {
    
        turningSpeed = value;
        
    }else if(command=="mo")
    {
    
        movingSpeed =value;
    }
    
    else
    {
        Serial2.print("command not found:");
        Serial2.println(inputString);
    
    }

    
}


void NP4::setMotorEnabled(bool on )
{

    checkInput =true;
    if(on)
    {
        digitalWrite(ENABLE_PIN_R,LOW);
        digitalWrite(ENABLE_PIN_L,LOW);
    }
    else
    {
    
        digitalWrite(ENABLE_PIN_R,HIGH);
        digitalWrite(ENABLE_PIN_L,HIGH);
        setMotorSpeed(1  ,0 );
        setMotorSpeed(1  ,1 );
    
    }
}

float NP4::stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
    
    float error;
    float output;
    
    error = setPoint - input;
    
 
    output = Kp * error
   
    return (output);
}

float NP4::speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
    float error;
    float output;
    
    error = setPoint-input;
    speedPID_errorSum += constrain(error,-40,40);
    speedPID_errorSum = constrain(speedPID_errorSum ,-8000,8000);
    
    output = Kp*error + Ki*speedPID_errorSum*DT*0.001;
    return(output);
}

void NP4::setMotorSpeed(float speed,int motor)
{
    float currentSpeed ;
    if(motor==0)
    {
        currentSpeed = currentSpeedL;
    }
    else
    {
        currentSpeed = currentSpeedR;
    }
    
    if ((currentSpeed - speed) > maxAccelset)
        currentSpeed -= maxAccelset;
    else if ((currentSpeed - speed) < -maxAccelset)
        currentSpeed +=  maxAccelset;
    else
        currentSpeed = speed;
    
    int absSpeed;
    
    if (currentSpeed > 0 )   // Positive speed
    {
        if(motor==0)digitalWrite(DIR_PIN_L, LOW);
        if(motor==1)digitalWrite(DIR_PIN_R, HIGH);
        absSpeed = currentSpeed;
        
    }
    else                       // Negative speed
    {
        if(motor==0)digitalWrite(DIR_PIN_L, HIGH);
        if(motor==1)digitalWrite(DIR_PIN_R, LOW);
        
        absSpeed = -currentSpeed;
        
    }
    

    if(motor==0)
    {
        startTimer(TC1, 0, TC3_IRQn, absSpeed);
        
        currentSpeedL = currentSpeed;
    }
    else
    {
       startTimer(TC1, 1, TC4_IRQn, absSpeed);
       currentSpeedR = currentSpeed;
    }
   
    
    
    
}



void NP4::startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk((uint32_t)irq);
    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
    uint32_t rc = VARIANT_MCK / 128 / frequency; //128 because we selected TIMER_CLOCK4 above
    TC_SetRA(tc, channel, rc / 2); //50% high, 50% low
    TC_SetRC(tc, channel, rc);
    TC_Start(tc, channel);
    tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ(irq);
}





