# Knife-Edge Transitioning 轨迹结果图

`plot_knife_edge_transition_results.py` 对应论文 Fig. 7，绘制：

1. a) Position：参考和实际椭圆跑道的俯视图，包含全部 8 圈；
2. b) Position tracking error：连续两圈的 NED 三轴位置误差；
3. c) Attitude：同一两圈内的 ZXY 指令姿态与实际姿态，单位为弧度。

论文的时域数据从“knife-edge 转入倒飞 coordinated flight”的转弯起点
开始。本项目轨迹在 `t=0` 从顶部 knife-edge 直线的起点出发，绘图程序根据
`6 m/s`、`6.25 s/圈` 和 `3 m` 转弯半径自动将窗口对齐到该事件，默认窗口
为 `[7.804, 20.304] s`。

先在项目根目录生成包含指令姿态的新日志：

```bash
conda activate vtol_sim
python main_sim.py --trajectory knife-edge-transition \
  --log validation_logs/knife-edge-transition.npz --no-show
```

然后绘图：

```bash
python results/KnifeEdgeTrasition/plot_knife_edge_transition_results.py
```

默认输出：

```text
results/KnifeEdgeTrasition/knife_edge_transition_results.png
```

可以手动改变时域窗口或输出格式：

```bash
python results/KnifeEdgeTrasition/plot_knife_edge_transition_results.py \
  --window-start 14.054 \
  --output results/KnifeEdgeTrasition/knife_edge_transition_other_window.pdf
```

目录名 `KnifeEdgeTrasition` 按任务中指定的拼写保留。绘图程序中的轨迹名称
和日志名称仍使用正确的 `knife-edge-transition`。
