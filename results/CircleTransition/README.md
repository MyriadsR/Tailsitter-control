# Circle Transition（论文 Fig. 10）

`plot_circle_transition_results.py` 按论文 Fig. 10 生成 5 行 × 2 列结果图：

| 左列：静态悬停 → 协调前飞 | 右列：协调前飞 → 静态悬停 | 内容 |
|---|---|---|
| a | b | 三维位置 |
| c | d | 三轴位置跟踪误差 |
| e | f | 速度模长 |
| g | h | 比力模长 `||a-g i_z||` |
| i | j | ZXY 指令姿态与实际姿态 |

轨迹半径为 `3.5 m`，真正的匀加速/匀减速段持续 `3.0 s`，切向加速度为
`2.7 m/s^2`，目标速度为 `8.1 m/s`。为了复现论文约 `3.8 s` 的横轴和
过渡前后的稳态平台，项目轨迹现在还包含 `0.4 s` 前置稳态段和 `0.4 s`
收尾稳态段。

三维位置图延续 `CircleFlight` 的显示约定：日志仍为 NED 坐标，但竖直轴
绘制正高度 `-z_NED`；位置误差和动力学量继续按原始 NED 数据计算。

先在项目根目录生成日志：

```bash
conda activate vtol_sim

python main_sim.py --trajectory transition-to-forward \
  --log validation_logs/transition-to-forward.npz --no-show

python main_sim.py --trajectory transition-to-hover \
  --log validation_logs/transition-to-hover.npz --no-show
```

然后绘图：

```bash
python results/CircleTransition/plot_circle_transition_results.py
```

默认输出：

```text
results/CircleTransition/circle_transition_results.png
```

也可以输出 PDF 或使用其他日志：

```bash
python results/CircleTransition/plot_circle_transition_results.py \
  --output results/CircleTransition/circle_transition_results.pdf
```
