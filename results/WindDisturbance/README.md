# Wind Disturbance 风扰结果图

`plot_wind_disturbance_results.py` 读取当前 `main_sim.py` 生成的有风 NPZ 日志，
绘制以下四组时域结果：

1. 世界 NED 坐标系风速三分量及模长；
2. 风导致的世界系外力增量三分量及模长；
3. 风导致的机体系外力矩增量三分量及模长；
4. NED 位置跟踪误差三分量及模长。

检测到非零风速的时间区间会用浅橙色背景标出。

先生成指定时间区间的有风日志：

```bash
python main_sim.py --trajectory lemniscate --laps 1 \
  --wind-mode constant --wind-velocity 2.0 0.0 0.0 \
  --wind-start 2.0 --wind-end 5.0 \
  --log validation_logs/lemniscate-windowed-wind.npz --no-show
```

然后生成风扰诊断图：

```bash
python results/WindDisturbance/plot_wind_disturbance_results.py \
  validation_logs/lemniscate-windowed-wind.npz
```

默认输出为：

```text
results/WindDisturbance/wind_disturbance_results.png
```

可以限制绘图时间范围或指定输出路径：

```bash
python results/WindDisturbance/plot_wind_disturbance_results.py \
  validation_logs/lemniscate-windowed-wind.npz \
  --start 1.0 --end 6.0 \
  --output results/WindDisturbance/lemniscate_wind_response.pdf
```

原 Fig. 6～Fig. 11 绘图脚本仍保持论文版式，不额外插入风扰子图，但可以
把有风日志作为输入，以绘制有风条件下的轨迹、误差、姿态和执行器响应。
