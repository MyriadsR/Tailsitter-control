# Lemniscate 轨迹结果图

`plot_lemniscate_results.py` 按论文 Fig. 6 的内容生成五联图：

1. a) Position：全部 8 圈的三维参考轨迹与实际轨迹；显示坐标与
   `animate_flight.py` 一致，为 North-East-Up（高度为 `-Z_NED`），三轴
   等比例且采用相同观察视角；
2. b) Position tracking error：单圈三个 NED 位置误差分量；
3. c) Speed：单圈参考速度与实际速度模长；
4. d) Acceleration：单圈参考/实际比力模长 `||a - g i_z||`；
5. e) Attitude：单圈 ZXY 指令姿态与实际姿态，单位为弧度。

先在项目根目录生成包含实际加速度和指令姿态的日志：

```bash
conda activate vtol_sim
python main_sim.py --trajectory lemniscate \
  --log validation_logs/lemniscate.npz --no-show
```

然后绘图：

```bash
python results/Lemniscate/plot_lemniscate_results.py
```

默认输出为 `results/Lemniscate/lemniscate_trajectory_results.png`，时域图
使用最后一个完整单圈。可用 `--lap 1`～`--lap 8` 选择圈数，使用
`--output path/to/figure.pdf` 可直接输出 PDF。例如：

```bash
python results/Lemniscate/plot_lemniscate_results.py \
  --lap 6 --output results/Lemniscate/lemniscate_lap6.pdf
```

程序兼容旧日志：如果缺少实际加速度，会对速度作数值微分；如果缺少指令
姿态，则姿态子图只显示实际姿态。为避免数值微分误差并得到完整曲线，建议
使用当前 `main_sim.py` 重新生成日志。
