int time = 10;                             // 定义统计多少时间内的脉冲数, ms
volatile int pulse = 0;                    // 用来保存脉冲的计数
int frequency = 1000;                      // pwm Hz
int PWM_cycles = 3000;                     // 一个采样周期内输出的PWM波数
float test_routation_rate = 0.0;           // 实际转速
float target_routation_rate = 15.0;        // 目标转速，圈/秒
float r1 = 0.0;
float r2 = 0.0;
float error = 0.0;                         // 误差
float error_last = 0.0;                    // 上一轮误差
float integeration = 0.0;                  // 积分值
float integeration_last = 0.0;             // 上轮积分值
float p = 0.0;                             // PID输出的控制变量（占空比）
float KP = 0.50;                           // PID控制参数
float KD = 0.03;
float KI = 0.50;

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
  // p = KP*e + KD(e2-e1)/t + KIet（积分）
    p1 = KP*error;
    p2 = KD*1000*(error-error_last)/time;
    integeration = integeration + error*time/1000;
    p3 = KI*(integeration + error);
    p4 = p1 + p2 + p3;
  
  // sigmoid 函数防止超调后溢出
  p5 = 1 / (1+exp(-p4));
  return p5;
}

void PWM() {
  // 单个周期 PWM 信号的产生
  int high_time = p*1000000/frequency;              // 计算高电平时间，us
  digitalWrite(13, HIGH);                           // 管脚 13 产生高电平信号
  delayMicroseconds(high_time);                     // 延迟高电平时间
  digitalWrite(13, LOW);                            // 管脚 13 产生低电平信号
  delayMicroseconds(1000000/frequency - high_time); // 延迟（周期-高电平时间）
}

void control() {
  // 采样转速并产生 j 个周期 PWM 信号
  int i, j = 50;
  test_routation_rate = 1000 * float(pulse) / (128.0 * time);      // 单位时间的脉冲数计算转速
  pulse = 0;                                                       // 脉冲清零
  error = (target_routation_rate - test_routation_rate) / target_routation_rate;  // 计算误差
  p = PID();                                         // 计算占空比（PID 控制变量）
  for(i=1;i<=j;i++) {
    // 循环 50 次
    PWM();                                           // PWM 信号调制，产生单周期脉冲
    error_last = error;                              // 上轮误差更新
  }
}

void loop() {
  interrupts();                                     // 启动中断允许
  int time1 = millis();
  control();
  int time2 = millis();
  time = time2 - time1;
  noInterrupts();                                   // 关闭所有中断
  //Serial.println(p);                              // 串口发送 PID 控制变量（调试代码）
  Serial.println(r2);       // 串口发送电动机转速（调试代码）
  r2 = (r1 + test_routation_rate)/2;
  r1 = test_routation_rate;
}
