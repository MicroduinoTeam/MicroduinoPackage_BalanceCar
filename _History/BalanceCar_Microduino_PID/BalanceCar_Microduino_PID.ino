#include <PID_v2.h>         //PID控制库
#include <Microduino_Stepper.h>  //步进电机库
#include "userDef.h"        //用户自定义
#include "RollPitch.h"
#include "Protocol.h"       //通讯协议库

MPU6050 imu;
StepMotor stepperL(PIN_DIRB, PIN_STEPB);              //左电机，使用stepper底板A接口
StepMotor stepperR(PIN_DIRA, PIN_STEPA);              //右电机，使用stepper底板D接口
PID speedPID((double)KP_SPD, (double)KI_SPD, (double)KD_SPD, DIRECT);     //速度环控制器
PID anglePID((double)KP_ANG, (double)KI_ANG, (double)KD_ANG, DIRECT);     //角度环控制器

uint16_t channalData[CHANNEL_NUM]; //8通道数据
bool mode = 0; //nrf或者ble模式
int16_t throttle;         //油门值
int16_t steering;         //转向值
unsigned long safe_ms = millis();

float robotSpeedFilter;   //速度估计值
float robotAngle;
float targetSpeed;        //目标速度
#ifdef BAT_DETECT
float batteryValue;       //电池电压
#endif
float ypr[3];             //Yaw,Pitch,Roll三个轴的角度值

void setup() {
//   Serial.begin(115200);
  dmpSetup();
  mode = protocolSetup();  //遥控接收器初始化
  speedPID.SetMode(AUTOMATIC);    //速度还控制器初始化
  anglePID.SetMode(AUTOMATIC);    //角度环控制器初始化
  speedPID.SetITermLimits(-10, 10);
  speedPID.SetOutputLimits(-MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);
  anglePID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  stepperL.begin();       //左电机初始化
  stepperR.begin();       //右电机初始化
  for (uint8_t k = 0; k < 3; k++) {
    stepperR.setSpeed(3);
    stepperL.setSpeed(-3);
    delay(150);
    stepperR.setSpeed(-3);
    stepperL.setSpeed(3);
    delay(150);
  }
  //  Serial.println("===========start===========");
}

void loop() {
  if (protocolRead(channalData, mode)) { //判断是否接收到遥控信号
    throttle = map(channalData[CHANNEL_THROTTLE], 1000, 2000, MAX_THROTTLE, -MAX_THROTTLE);
    steering = map(channalData[CHANNEL_STEERING], 1000, 2000, -MAX_STEERING, MAX_STEERING);
    safe_ms = millis();
  }

  if (safe_ms > millis()) safe_ms = millis();
  if (millis() - safe_ms > SAFE_TIME_OUT) {
    throttle = 0;
    steering = 0;
  }

#ifdef BAT_DETECT
  batteryValue = (analogRead(A7) / 1024.0) * BAT_DETECT;   //采集电池电压
#endif
  dmpGetYPR(ypr);
  float robotAngleCache = robotAngle;
  robotAngle = ypr[DIRECTION] + ANGLE_FIX;          //取Roll轴角度
  float robotSpeed = (stepperL.getSpeed() - stepperR.getSpeed()) / 2.0;   //计算小车速度

  float angleVelocity = (robotAngle - robotAngleCache) * 90.0;
  robotSpeedFilter = robotSpeedFilter * 0.95 + (robotSpeed + angleVelocity) * 0.05;       //小车速度滤波
  float targetAngle = speedPID.Compute(robotSpeedFilter, throttle);       //速度环计算目标角度
  targetSpeed += anglePID.Compute(robotAngle, targetAngle);         //角度环计算电机转速
  targetSpeed = constrain(targetSpeed, -MAX_SPEED, MAX_SPEED);
  int16_t motorR = targetSpeed + steering;    //计算右电机转速
  int16_t motorL = -targetSpeed + steering;   //计算左电机转速

  if ((robotAngle < 65) && (robotAngle > -65)) {
    stepperAllEnable();
    stepperR.setSpeed(motorR);        //更新右电机转速
    stepperL.setSpeed(motorL);        //更新左电机转速

    if ((robotAngle < 30) && (robotAngle > -30)) {
      anglePID.SetTunings(KP_ANG, KI_ANG, KD_ANG);
      speedPID.SetTunings(KP_SPD, KI_SPD, KD_SPD);
    } else {
      anglePID.SetTunings(KP_ANG_RAISUP, KI_ANG_RAISUP, KD_ANG_RAISUP);
      speedPID.SetTunings(KP_SPD_RAISUP, KI_SPD_RAISUP, KD_SPD_RAISUP);
    }
  } else {
    stepperAllDisable();
    stepperR.setSpeed(0);        //更新右电机转速
    stepperL.setSpeed(0);        //更新左电机转速
    anglePID.SetTunings(KP_ANG_RAISUP, KI_ANG_RAISUP, KD_ANG_RAISUP);
    speedPID.SetTunings(KP_SPD_RAISUP, KI_SPD_RAISUP, KD_SPD_RAISUP);
  }
}


