int time = 30;                             // 定义统计多少时间内的脉冲数, ms
volatile int pulse = 0;                    // 用来保存脉冲的计数
int frequency = 1000;                      // pwm Hz
int PWM_cycles = 3000;                     // 一个采样周期内输出的PWM波数
float test_routation_rate = 0.0;           // 实际转速
float target_routation_rate = 20.0;        // 目标转速，圈/秒
float r1 = 0.0;
float r2 = 0.0;
float r3 = 0.0;
float r4 = 0.0;
float r5 = 0.0;
float r6 = 0.0;
float r7 = 0.0;
float r8 = 0.0;
float r9 = 0.0;
float r10 = 0.0;
float r11 = 0.0;
float r12 = 0.0;
float error = 0.0;                         // 误差
float error_last = 0.0;                    // 上一轮误差
float integeration = 0.0;                  // 积分值
float integeration_last = 0.0;             // 上轮积分值
float p = 0.0;                             // PID输出的控制变量（占空比）
float KP = 0.40;                           // PID控制参数
float KD = 0.03;
float KI = 0.80;

void setup()
{
  pinMode(2, INPUT);                        // 2管脚设置输入模式
  Serial.begin(9600);                       // 串口通讯初始化
  attachInterrupt(0, pulse_count, HIGH);    // 设置中断函数触发条件
  pinMode(13, OUTPUT);                      // 13管脚输出PWM调制信号
}

void pulse_count(){
  // 脉冲计数函数
  ++pulse;
}

float PID() {
  // PID控制器，返回占空比
  float p1 = 0.0, p2 = 0.0, p3 = 0.0, p4 = 0.0, p5 = 0.0;
  // p = KP*e + KD(e1-e2)/t + KIet（积分）
    p1 = KP*error;
    p2 = KD*1000*(error_last-error)/time;
    integeration = integeration + error*time/1000;
    p3 = KI*(integeration + error);
    p4 = p1 + p2 + p3;
  
  // sigmoid 函数防止超调后溢出
  p5 = 1 / (1+exp(-p4));
  return p5;
}

void PWM(float p) {
  // 单个周期 PWM 信号的产生
  int int_p = 0;
  int_p = int(255 * p);
  analogWrite(11, int_p);
}

void control() {
  // 采样转速并产生 j 个周期 PWM 信号
  int i, j = 50;
  test_routation_rate = 1000 * float(pulse) / (128.0 * time);      // 单位时间的脉冲数计算转速
  pulse = 0;                                                       // 脉冲清零
  error = (target_routation_rate - r12) / target_routation_rate;  // 计算误差
  p = PID();                                         // 计算占空比（PID 控制变量）
  PWM(p);                                            // 产生 PWM 信号
}

void loop() {
  interrupts();                                     // 启动中断允许
  delay(time);
  noInterrupts();                                   // 关闭所有中断
  Serial.println(r12);                              // 串口发送 PID 控制变量（调试代码）
  // Serial.println(r4);                               // 串口发送电动机转速（调试代码）
  r12 = (r10 + r11)/2;
  r11 = (r9 + r10)/2;
  r10 = (r8 + r9)/2;
  r9 = (r7 + r8)/2;
  r8 = (r6 + r7)/2;
  r7 = (r5 + r6)/2;
  r6 = (r4 + r5)/2;
  r5 = (r3 + r4)/2;
  r4 = (r2 + r3)/2;
  r3 = (r1 + r2)/2;
  r2 = (r1 + test_routation_rate)/2;
  r1 = test_routation_rate;
  control();
}
