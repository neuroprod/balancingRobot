//
//  Mpu.hpp
//  NP4
//
//  Created by Kris Temmerman on 15/11/15.
//
//

#ifndef Mpu_hpp
#define Mpu_hpp

#include "Arduino.h"
#include <Wire.h>
#include <I2Cdev.h>





#include <JJ_MPU6050_DMP_6Axis.h>


#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489


class Mpu
{


public:
    Mpu(){};
    void setup();
   bool update();
    
    MPU6050 mpu;
    uint8_t devStatus;
    bool dmpReady ;
    uint8_t mpuIntStatus;
    
    Quaternion q;
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[18]; // FIFO storage buffer
    
    
    float angle;
};

#endif /* Mpu_hpp */
