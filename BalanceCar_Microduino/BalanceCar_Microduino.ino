#include "userDef.h"
#include "RollPitch.h"
#include "Protocol.h"
#include "PID.h"

#include <Microduino_Stepper.h>  //步进电机库

Stepper stepperL(PIN_DIRB, PIN_STEPB);              //左电机，使用stepper底板A接口
Stepper stepperR(PIN_DIRA, PIN_STEPA);              //右电机，使用stepper底板D接口

//----------------------------
uint16_t channalData[CHANNEL_NUM]; //8通道数据
bool mode = 1; //nrf或者ble模式
int16_t throttle = 0;       //油门值
int16_t steering = 0;       //转向值
unsigned long safe_ms = millis();

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float dmpGetPhi() {
  mpu.getYawPitchRoll(ypr);
  return ypr[DIRECTION];
}

void setup() {
  Serial.begin(115200);
  delay(200);

  mode = protocolSetup();  //遥控接收器初始化
  //-------------------------------------------
  Serial.println(F("Initializing MPU..."));
  if (!dmpSetup()) {
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("MPU Initialization failed!"));
    return;
  }

  //-------------------------------------------
  Serial.println("Initializing Stepper motors...");
  stepperAllEnable();   // 开步进驱动器
  // 小电机的振动，表明机器人已准备就绪
  stepperL.begin(STEPPER_SPEED);       //左电机初始化
  stepperR.begin(STEPPER_SPEED);       //右电机初始化
  for (uint8_t k = 0; k < 3; k++) {
    stepperL.setSpeed(-3);
    stepperR.setSpeed(3);
    delay(150);
    stepperL.setSpeed(3);
    stepperR.setSpeed(-3);
    delay(150);
  }
  stepperAllDisable();   // 关步进驱动器

  delay(2000);
  //-------------------------------------------
  Serial.println("\t start");
}

void loop() {
  if (protocolRead(channalData, mode)) { //判断是否接收到遥控信号
    throttle = map(channalData[CHANNEL_THROTTLE], 1000, 2000, MAX_THROTTLE, -MAX_THROTTLE);
    steering = map(channalData[CHANNEL_STEERING], 1000, 2000, -MAX_STEERING, MAX_STEERING);
    safe_ms = millis();
  }

  if (safe_ms > millis()) safe_ms = millis();
  if (millis() - safe_ms > 1000) {
    steering = 0;
    throttle = 0;
  }

  //-------------------------------------------
  long timer_old = timer_value;
  timer_value = millis();
  clock = timer_value - timer_old;  //估算时间

  dmpGetYPR(ypr);
  float robotAngleCache = robotAngle;
  robotAngle = ypr[DIRECTION] + ANGLE_FIX;   //实际角度

  int16_t robotSpeed_Old = robotSpeed;          // 整个机器人的速度
  robotSpeed = (stepperL.getSpeed() - stepperR.getSpeed()) / 2.0;   //计算小车速度

  float angleVelocity = (robotAngle - robotAngleCache) * 90.0; // 角速度
  robotSpeedFilter = robotSpeedFilter * 0.95 + (float)(robotSpeed_Old - angleVelocity) * 0.05; //估计速度

  //我们整合输出（角度）
  float target_angle = speedPIControl(clock, robotSpeedFilter, (float)throttle, Kp_thr, Ki_thr); //目标角速度
  target_angle = constrain(target_angle, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE); //约束目标角度

  //我们整合输出（加速度）
  targetSpeed += stabilityPDControl(clock, robotAngle, target_angle, Kp, Kd);
  targetSpeed = constrain(targetSpeed, -MAX_THROTTLE, MAX_THROTTLE); // 约束输出速度
  // 控制的转向部分的输出直接注射

  int16_t motorL = -targetSpeed + steering;   //计算左电机转速
  int16_t motorR = targetSpeed + steering;    //计算右电机转速
  // 限制最大速度
  motorL = constrain(motorL, -MAX_THROTTLE, MAX_THROTTLE);
  motorR = constrain(motorR, -MAX_THROTTLE, MAX_THROTTLE);

  robot(motorL, motorR);
}

void robot(int16_t _L, int16_t _R) {
  //  Serial.print("angle:");
  //  Serial.print(robotAngle);
  //  Serial.print(" ");
  //-------------------------------------------
  // Is robot ready (upright?)
  if ((robotAngle < 45) && (robotAngle > -45))    {
    stepperAllEnable();   // 使步进驱动器
    //    Serial.print("Motor: ");
    //    Serial.print(_R);
    //    Serial.print(",");
    //    Serial.print(_L);
    // NORMAL MODE
    stepperL.setSpeed(_L);        //更新左电机转速
    stepperR.setSpeed(_R);        //更新右电机转速

    if ((robotAngle < 25) && (robotAngle > -25)) {
      Kp = KP;  // Default or user control gains
      Kd = KD;
      Kp_thr = KP_SPD;
      Ki_thr = KI_SPD;
    }
    else {
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_SPD_RAISEUP;
      Ki_thr = KI_SPD_RAISEUP;
    }
  }
  else { // Robot not ready, angle > 60º
    stepperAllDisable();         // 使步进驱动器
    stepperL.setSpeed(0);        //更新左电机转速
    stepperR.setSpeed(0);        //更新右电机转速
    PID_errorSum = 0;  // Reset PID I term
    Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
    Kd = KD_RAISEUP;
    Kp_thr = KP_SPD_RAISEUP;
    Ki_thr = KI_SPD_RAISEUP;
  }
  //  Serial.println("");
}
