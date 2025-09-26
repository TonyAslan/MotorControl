# 基于ESP32S3的直流电机控制
实现了直流电机的速度闭环控制，上位机使用微信小程序，通过BLE通信

# 闭环调速的实现
写在定时器中断函数，即timer0_IRQ
1. 编码器脉冲转角度
double angle = en_cnt * 360.0/3120;
en_cnt 是编码器累计脉冲数。
电机一圈会产生 3120 个脉冲（已经考虑了减速比）。
因此脉冲数 → 角度：
𝜃 = 脉冲数 / (脉冲/圈) × 360°

2. 当前速度计算
currentSpeed = (en_cnt - en_lastcnt)* 1000.0 * 60 / 3120 / 50;
定时器50ms触发一次，即 20Hz 采样。
(en_cnt - en_lastcnt) = 过去 50ms 的脉冲增量。
除以 3120 = 换算成电机转数。
除以 50ms = 换算成转/毫秒，再 ×1000 → 转/秒。
再 ×60 → 转/分钟 (RPM)。
最终得到的是电机当前的转速（RPM）。

3. 误差计算
speedError = targetSpeed - currentSpeed;
targetSpeed = 目标转速（设定值）。
currentSpeed = 实际转速（测量值）。
两者差值就是速度误差。

4. PID 三项计算
积分项
speedErrorIntegral += (speedError * 0.05); // 0.05s
每次累加误差 × 时间间隔（0.05s）。
积分能消除稳态误差，让电机最终速度更接近目标。

微分项
speedErrorDifferential = (speedError - lastSpeedError) / 0.05;
误差的变化率。
微分能预测趋势，避免超调。

PID 总和
double pidOutput = (kp*speedError) + (ki*speedErrorIntegral) + (kd*speedErrorDifferential);

kp 比例：快速响应。
ki 积分：消除偏差。
kd 微分：抑制震荡。

三项相加得到 PID 输出。

5. PWM 限幅 + 输出
pwmOutput = pidOutput;
if(pidOutput > 255) pwmOutput = 255;
if(pidOutput < -255) pwmOutput = -255;

PWM 输出范围：-255 ~ 255。
大于 255 → 饱和保护。
小于 -255 → 反方向饱和。

方向控制：
if(pwmOutput > 0)
    motor.setSpeed(1, pwmOutput); // 正转
else if(pwmOutput < 0)
    motor.setSpeed(0, -pwmOutput); // 反转
    
6. 状态更新
en_lastcnt = en_cnt;
lastSpeedError = speedError;
保存当前计数和误差，给下一次定时器中断用。

速度闭环 PID 控制器：
采集反馈：编码器脉冲 → 角度、转速。
计算误差：目标速度 − 实际速度。
PID 调节：比例 + 积分 + 微分。
输出控制：生成 PWM，驱动电机正反转。
循环执行：每 50ms 更新一次。
这样，电机就能按照目标速度运行，并且即使有负载变化，PID 也能自动调整 PWM 来保持速度。
