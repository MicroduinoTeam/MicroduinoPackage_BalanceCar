#include <arduino.h>

//通讯模式
//#define BLE_SoftSerial  //软串口模式
#ifndef BLE_SoftSerial  //如果没开启软串口,就开硬串口模式
#if defined(__AVR_ATmega32U4__)
#define BLE_HardSerial Serial1
#else
#define BLE_HardSerial Serial
#endif
#endif
#define BLE_SPEED 9600  //蓝牙接口速度
#define NRF_CHANNEL 70  //nRF通道
#define CHANNEL_THROTTLE  1 //油门通道
#define CHANNEL_STEERING  0 //转向通道
#define SAFE_TIME_OUT 250   //失控保护时间

//角度/速度修正参数
#define ANGLE_FIX -1.5
#define MAX_TARGET_ANGLE 12 //最大目标角度12
#define MAX_SPEED         1200
#define MAX_THROTTLE 512 //最大油门 < 512
#define MAX_STEERING 128 //最大转向 < 256

//姿态传感器安装方向
enum _DIRECTION {
  YAW = 0,
  PITCH,
  ROLL
};
#define DIRECTION PITCH

//PID整定参数
#define KP_ANG 0.15    // 0.25
#define KI_ANG 0
#define KD_ANG 35      // 30 28 26  
#define KP_SPD 0.04    //0.08//0.065
#define KI_SPD 0.02   //0.05
#define KD_SPD 0.0

#define KP_ANG_RAISUP   0.52
#define KI_ANG_RAISUP   0
#define KD_ANG_RAISUP   45
#define KP_SPD_RAISUP   0
#define KI_SPD_RAISUP   0
#define KD_SPD_RAISUP   0

//#define BAT_DETECT   5*84/33.0

