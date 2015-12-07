//
//  Mpu.cpp
//  NP4
//
//  Created by Kris Temmerman on 15/11/15.
//
//

#include "Mpu.hpp"

void Mpu::setup()
{
    
    dmpReady =false;
    
   Wire.begin();
    // 4000Khz fast mode
    // TWSR = 0;
     //TWBR = ((16000000L/I2C_SPEED)-16)/2;
     //TWCR = 1<<TWEN;
    Serial.println("vInitializing I2C devices...");
    //mpu.initialize();
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
    mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
    mpu.setSleepEnabled(false);
    
    delay(2000);
    Serial.println(F("Initializing DMP..."));

   
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP Ready"));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        
    } else { // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        
    }

    mpu.resetFIFO();
}

bool Mpu::update()
{
    fifoCount = mpu.getFIFOCount();
    
    if (fifoCount >= 18)
    {
        
        if (fifoCount > 18) // If we have more than one packet we take the easy path: discard the buffer
        {
           
            mpu.resetFIFO();
            return false;
        }
        
        mpu.getFIFOBytes(fifoBuffer, 16);
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        
        mpu.resetFIFO();  // We always reset FIFO
        
    
        
        
        angle = -(atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);
        return true;
    }
    return false;
}








