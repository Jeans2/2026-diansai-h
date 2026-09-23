# 双串级循迹控制

## 方向外环

八路红外和方向位置式 PD 每 10 ms 更新：

```text
placeOut = Kp * error + Kd * (error - lastError)
leftTarget  = baseSpeed + placeOut
rightTarget = baseSpeed - placeOut
```

本车没有陀螺仪，因此没有角速度反馈项。

## 速度内环

左右轮分别使用编码器增量式 PI，每 5 ms 更新：

```text
error(k) = target(k) - measured(k)
output(k) = output(k-1)
          + Ki * error(k)
          + Kp * [error(k) - error(k-1)]
```

初始速度参数：

```text
Kp = 0.20
Ki = 0.02
PWM限幅 = 0～200
```

速度环保持 5 ms计算周期，但编码器测速采用最近5次采样组成的25 ms滑动
窗口。这样一个脉冲对归一化速度的影响约为2，而不是直接按5 ms换算时的10。

每个电机仅在启动的第一次把PI内部输出初始化为PWM=80，以克服MG310静摩擦；
之后不设置最低PWM，左右轮可以分别收敛到不同输出。PI输出滤波为80%新值、
20%上次值。
