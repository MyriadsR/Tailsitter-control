# Circle Flight（论文 Fig. 8）

`plot_circle_flight_results.py` 将 coordinated flight 放在左列，将
knife-edge flight 放在右列，按论文 Fig. 8 生成 5 行 × 2 列结果图：

| 左列 | 右列 | 内容 |
|---|---|---|
| a | b | 三维位置 |
| c | d | 三轴位置跟踪误差 |
| e | f | 速度模长 |
| g | h | 比力模长 `||a-g i_z||` |
| i | j | ZXY 指令姿态与实际姿态 |

三维位置图采用 North-East-Up 显示方式：原始日志仍是 NED 坐标，但绘图时
将 `z_NED` 取负，因此竖直轴显示为正高度 `-z_NED`。其余误差和动力学量
仍按原始 NED 定义计算。

论文使用半径 `3.5 m`、参考速度 `8.1 m/s` 的圆轨迹，单圈时间约
`2.715 s`；实验实际飞行速度约为 `7.8 m/s`，所以 Fig. 8 标题使用
`7.8 m/s`。

先在项目根目录重新生成两个包含实际加速度和指令姿态的日志：

```bash
conda activate vtol_sim

python main_sim.py --trajectory circle-coordinated \
  --log validation_logs/circle-coordinated.npz --no-show

python main_sim.py --trajectory circular-knife-edge \
  --log validation_logs/circular-knife-edge.npz --no-show
```

然后绘图：

```bash
python results/CircleFlight/plot_circle_flight_results.py
```

默认输出：

```text
results/CircleFlight/circle_flight_results.png
```

为了复现论文 Fig. 8i/j 中 yaw 从 `+pi` 连续下降到 `-pi` 的显示相位，
coordinated flight 的单圈窗口相对基础圈移动 `1/4` 圈；knife-edge
flight 使用基础圈起点。默认自动选择日志中最后一个能同时容纳这两个完整
窗口的基础圈。也可以手动选择：

```bash
python results/CircleFlight/plot_circle_flight_results.py \
  --lap 2 --output results/CircleFlight/circle_flight_lap2.pdf
```

输入日志、半径、参考速度、单圈时间、DPI 和输出路径均可通过命令行覆盖，
完整参数可用 `--help` 查看。

## Fig. 9：执行器与角加速度

`plot_circle_flight_actuators.py` 继续使用相同的左右列和单圈相位，绘制：

| 左列 | 右列 | 内容 |
|---|---|---|
| a | b | 两个电机转速 `omega_1, omega_2` |
| c | d | 两个舵面偏角 `delta_1, delta_2` |
| e | f | 绕机体 `b_x` 的指令/实际角加速度 |
| g | h | 绕机体 `b_y` 的指令/实际角加速度 |
| i | j | 绕机体 `b_z` 的指令/实际角加速度 |

使用当前 `main_sim.py` 重新生成日志后运行：

```bash
python results/CircleFlight/plot_circle_flight_actuators.py
```

默认输出：

```text
results/CircleFlight/circle_flight_actuators_and_angular_acceleration.png
```

论文为提高可读性，将角加速度指令曲线按执行器响应时间移动了最多 `70 ms`。
本程序在 `0–70 ms` 范围内对三个机体轴分别估计非负时移，并只改变指令
曲线的显示时间，不改变日志数据。可关闭该处理：

```bash
python results/CircleFlight/plot_circle_flight_actuators.py \
  --max-command-shift-ms 0
```

角加速度实际值优先读取控制器反馈使用的 `angular_acceleration_lpf`；日志还
保留未滤波的 `angular_acceleration` 和指令值
`angular_acceleration_command`，三者均为机体系分量，单位 `rad/s^2`。
