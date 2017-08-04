#define ITERM_MAX_ERROR 40   // ITERM饱和常数
#define ITERM_MAX 5000

//----------------------------
long clock;  //主机运行时间
long timer_value;//定时器的值
float robotAngle = 0; //角度调整
int16_t robotSpeed = 0;        // 整个机器人的速度（从踏步机速测）
float robotSpeedFilter = 0; //估计速度过滤
float targetSpeed = 0;

//----------------------------
float Kp_thr = KP_SPD; //油门
float Ki_thr = KI_SPD;

float Kp = KP;
float Kd = KD;

//----------------------------
float setPointOld = 0;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;

// PD的实施。 DT是毫秒
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd) {
  float error;
  float output;
  error = setPoint - input;
  // Kd的两部分实施
  //仅使用输入（传感器）的一部分而不是设定值输入输入（T-2）最大的一个
  //而第二个使用该设定值，使之更有点侵略性设定点设定点（T-1）
  output = Kp * error + (Kd * (setPoint - setPointOld) - Kd * (input - PID_errorOld2)) / DT; // + 错误 - PID_error_Old2
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // 误差为Kd值是唯一的输入组件
  setPointOld = setPoint;
  return (output);
}

//P控制实现
float speedPControl(float input, float setPoint,  float Kp) {
  float error;
  error = setPoint - input;
  return (Kp * error);
}

// PI实现。 DT是毫秒
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki) {
  float error;
  float output;
  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);
  output = Kp * error + Ki * PID_errorSum * DT * 0.001;
  return (output);
}

