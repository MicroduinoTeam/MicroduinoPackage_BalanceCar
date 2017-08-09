#define STEPPER_SPEED 1024 //根据不同步进电机版本 设置成1024或者512
//识别步进电机版本方法:[螺丝位凸起螺杆|1024] [螺丝位正常螺丝固定|512]

//通讯模式
#if defined(__AVR_ATmega32U4__)
#define BLE_HardSerial Serial1	//CoreUSB的D0和D1是Serial1
#else
#define BLE_HardSerial Serial	//Core的D0和D1是Serial
#endif
#define BLE_SPEED 9600  //蓝牙接口速度
#define NRF_CHANNEL 70  //nRF通道
#define CHANNEL_THROTTLE  1 //油门通道
#define CHANNEL_STEERING  0 //转向通道
#define SAFE_TIME_OUT 250   //失控保护时间
#define CHANNEL_NUM 8

//姿态传感器安装方向
enum _DIRECTION {
  YAW = 0,
  PITCH,  //UPIN朝前安装
  ROLL
};
#define DIRECTION PITCH

#define ANGLE_FIX 0 //如果小车不直 校准角度

#define MAX_TARGET_ANGLE 16 //最大目标角度12

#define MAX_THROTTLE 448 //最大油门 < 512
#define MAX_STEERING 128 //最大转向 < 256


#define KP 0.22 // 0.22        
#define KD 28  // 30 28 26  
#define KP_SPD 0.065  //0.08//0.065
#define KI_SPD 0.05//0.05

// 对于raiseup控制增益
#define KP_RAISEUP 0.02 //kp兴起
#define KD_RAISEUP 40   //kd兴起
#define KP_SPD_RAISEUP 0  //没有对raiseup速度控制
#define KI_SPD_RAISEUP 0.0//油门兴起
