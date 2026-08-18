# paper_planning 阶段 A

本目录实现 `study.md` 第 12、13 章建议的第一阶段：固定几何航点、固定总时间下优化分段时间比例、外层整体时间缩放，以及基于 `paper_planning` 式 (20)–(56) 的开环执行器可行性检查。

规划映射与 `controller/` 明确分离。它忽略舵面直接力这一非最小相位项，输出未限幅的单电机推力/转速和舵偏角；这些预测仅用于判断候选轨迹是否可行，不能作为闭环仿真的执行器命令。

## 构建与运行

```bash
git submodule update --init --recursive
cmake -S planning -B planning/build
cmake --build planning/build -j
(cd planning/build && ctest --output-on-failure)

# 6 m、三个固定航点，yaw 0 -> 0
./planning/build/generate_simple_trajectory \
  --output planning/output/simple_yaw0.csv

# 同一几何轨迹，yaw 0 -> pi
./planning/build/generate_simple_trajectory \
  --yaw-pi \
  --output planning/output/simple_yaw_pi.csv

# 对照组 2：既不优化时间比例，也不执行外层缩放，保持总时间 4 s
./planning/build/generate_simple_trajectory \
  --no-optimize-time-ratio \
  --no-time-scaling \
  --output planning/output/simple_yaw0_no_ratio_no_scaling.csv

# 对照组 1：优化时间比例，但不执行外层整体时间缩放，保持总时间 4 s
./planning/build/generate_simple_trajectory \
  --no-time-scaling \
  --output planning/output/simple_yaw0_no_time_scaling.csv

# 可视化速度着色三维轨迹、平滑性指标和执行器预测
python3 planning/scripts/plot_planning_metrics.py \
  planning/output/simple_yaw0.csv

# 三组对比：实线、虚线和点划线分别表示三种配置
python3 planning/scripts/plot_planning_metrics.py \
  planning/output/simple_yaw0_no_time_scaling.csv \
  --compare-csv planning/output/simple_yaw0_no_ratio_no_scaling.csv \
  --compare-csv-2 planning/output/simple_yaw0.csv \
  --primary-label "Ratio optimized, fixed 4 s" \
  --compare-label "Distance ratio, fixed 4 s" \
  --compare-label-2 "Ratio optimized + scaled" \
  --prefix yaw0_selected_three_comparison \
  --title "Yaw 0 to 0"
```

默认使用 `paper_vehicle` 参数。追加 `--simulation-vehicle` 可切换到当前 `config.py` 的质量、推力和反扭矩系数。程序的默认相对输出路径是 `output/simple_trajectory.csv`；上面的命令显式指定 `planning/output/`，从而不受当前工作目录影响。

可视化脚本默认在 CSV 同级的 `figures/` 中生成三张图：速度着色三维轨迹、Snap/yaw 加速度和原始开环执行器预测。使用 `--omega-max`、`--delta-max` 可覆盖图中的限制线，`--raw-ned-z` 可直接显示 NED z 而不转换为正高度。

`--no-optimize-time-ratio` 只跳过内层 NLopt 时间比例优化。对照组仍使用同一组航点、相同的固定总时间 `4 s`、距离比例时间初值、相同的外层最短可行时间缩放及相同的开环执行器检查，因此对比中唯一的算法变量是是否优化分段时间比例。

`--no-time-scaling` 保留内层时间比例优化，但跳过外层缩放，直接输出总时间 `4 s` 的轨迹。程序仍执行相同的开环可行性检查，并在控制台报告 `selected trajectory feasible`，只是不会自动改变时间。

## 当前范围

- 位置为 3D 七次 minimum-snap 样条，yaw 为 1D 三次 minimum-yaw-acceleration 样条，共享分段时间。
- softmax 时间映射严格保持总时间，最后一个 logit 固定为零以消除冗余自由度。
- 外层先粗扫全部缩放范围，再细化每个“不可行到可行”的切换区间，不假设全局单调。
- 稠密检查显式加入样条连接点；默认最终间隔为 2 ms，可在 `FeasibilityChecker` 构造时收紧到 0.5–1 ms。
- 当前只处理位置/yaw 航点和首末端零导数。复杂特技所需的中间速度/加速度等式约束属于后续阶段。
- 论文未公开的完整辨识参数仍采用当前工程估计，因此代码链路可复现，但数值边界不能宣称为论文实验值的严格复现。
