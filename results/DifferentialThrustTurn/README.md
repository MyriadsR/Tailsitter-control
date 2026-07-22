# Differential Thrust Turn（论文 Fig. 11）

`plot_differential_thrust_turn_results.py` 按论文 Fig. 11 绘制五个子图：

1. a) Position：三维参考位置与实际位置；
2. b) Attitude：实际 ZXY 欧拉角 `psi、phi、theta`；
3. c) Angular velocity：机体系角速度 `Omega_x、Omega_y、Omega_z`；
4. d) Rotor speed：两个电机转速 `omega_1、omega_2`；
5. e) Flap deflection：两个舵面偏角 `delta_1、delta_2`。

三维位置图将日志中的 NED `z` 取负，显示正高度 `-z_NED`；其余数据仍
保持原始坐标和单位。姿态角单位为弧度，角速度单位为 `rad/s`，电机转速
单位为 `rad/s`，舵偏由日志中的度转换为弧度。

先在项目根目录生成包含机体角速度的新日志：

```bash
conda activate vtol_sim
python main_sim.py --trajectory differential-turn \
  --log validation_logs/differential-turn.npz --no-show
```

然后绘图：

```bash
python results/DifferentialThrustTurn/plot_differential_thrust_turn_results.py
```

默认输出：

```text
results/DifferentialThrustTurn/differential_thrust_turn_results.png
```

本项目的完整参考轨迹持续 `3.3 s`，长于论文 Fig. 11 约 `2.25 s` 的显示
窗口，因此默认绘制完整过程以避免裁掉速度反向后的退出段。若只需要相同的
横轴范围，可以指定：

```bash
python results/DifferentialThrustTurn/plot_differential_thrust_turn_results.py \
  --plot-duration 2.25 \
  --output results/DifferentialThrustTurn/differential_thrust_turn_2p25s.pdf
```

当前 `main_sim.py` 还会保存 `angular_velocity_ref`，但论文 Fig. 11c 只绘制
实测三轴角速度，因此默认结果图不显示该前馈参考。
