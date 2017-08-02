#include <JJ_MPU6050_DMP_6Axis.h>  // 与DMP工作库的修改版本（见注释内）typedef enum 

#define I2C_SPEED 400000L //I2C速度

MPU6050 mpu;
boolean dmpReady = false;
uint16_t packetSize;

bool dmpSetup(){    
    Wire.begin();
    TWSR = 0;
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
    TWCR = 1 << TWEN;
    
    mpu.initialize();
    if(mpu.dmpInitialize() == 0) {
       mpu.setDMPEnabled(true);
       dmpReady = true;
       packetSize = mpu.dmpGetFIFOPacketSize();
    }else{
       dmpReady = false;
    }
    return dmpReady;
}

uint8_t dmpGetYPR(float *_ypr){
  Quaternion q;
  VectorFloat gravity;
  uint8_t fifoBuffer[18];
  if(!dmpReady)
     return 0;

  if(mpu.getFIFOCount() == 1024){
      mpu.resetFIFO();  
  }else{
     while(mpu.getFIFOCount() < packetSize);
     mpu.getFIFOBytes(fifoBuffer, packetSize);
     mpu.dmpGetQuaternion(&q, fifoBuffer);
     mpu.dmpGetGravity(&gravity, &q);
     mpu.dmpGetYawPitchRoll(_ypr, &q, &gravity);
     for(int i=0; i<3; i++){
       _ypr[i] *= 180/PI;
     }  
  }
  return 0;
}

