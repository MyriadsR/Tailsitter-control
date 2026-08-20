# paper_planning 阶段 A/B

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

## 阶段 B：幅值、机体角速度和执行器动态罚函数

阶段 B 使用新的 `generate_stage_b_trajectory`，固定总时间且完全不调用外层
`TimeScaler`。原来的三个关键航点保持不变，在后两个关键点之间插入一个可在
有界邻域内移动的辅助点，并联合优化三段时间和该辅助点：

```bash
# yaw 0 -> 0；同时导出同总时间的阶段 A 基线
./planning/build/generate_stage_b_trajectory \
  --output planning/output/stage_b_yaw0_optimized.csv \
  --baseline-output planning/output/stage_b_yaw0_baseline.csv

# yaw 0 -> pi
./planning/build/generate_stage_b_trajectory \
  --yaw-pi \
  --output planning/output/stage_b_yaw_pi_optimized.csv \
  --baseline-output planning/output/stage_b_yaw_pi_baseline.csv

# 可视化阶段 B 与阶段 A（相同固定总时间 1.8 s）的差异
python3 planning/scripts/plot_planning_metrics.py \
  planning/output/stage_b_yaw_pi_optimized.csv \
  --compare-csv planning/output/stage_b_yaw_pi_baseline.csv \
  --primary-label "Stage B: actuator penalty" \
  --compare-label "Stage A: smoothness only" \
  --prefix stage_b_vs_stage_a_yaw_pi \
  --title "Fixed total time 1.8 s, yaw 0 to pi" \
  --omega-max 1875 \
  --delta-max 0.75 \
  --body-rate-max 8.50848010347 \
  --servo-rate-max 18.75
```

默认目标在每段 `50` 个区间上用梯形积分采样原始单电机推力
`T1/T2`、舵偏角 `delta1/delta2` 和机体角速度范数。相邻节点还按与
`main_sim.py` 相同的指数一阶模型反算所需电机/舵机命令，并约束舵面有限差分
角速度。`--actuator-margin 0.75` 表示软罚函数从物理/参考包络的 `75%` 开始，
最终以完整包络进行 `2 ms` 稠密检查。常用参数：

```text
--total-time S             固定总时间，默认 1.8 s
--actuator-margin M        优化用执行器裕量，默认 0.75
--actuator-weight W        执行器积分罚权，默认 200
--geometry-weight W        辅助点偏移罚权，默认 0.05
--samples-per-segment N    每段积分区间数，默认 50
--maxeval N                最大目标函数评估数，默认 400
--body-rate-max RAD_S      机体角速度参考包络，默认 11.34464（650 deg/s）
--motor-time-constant S    电机一阶时间常数，默认 0.04 s
--servo-time-constant S    舵机一阶时间常数，默认 0.03 s
--servo-rate-max RAD_S     舵面角速度上限，默认 25 rad/s
--no-body-rate-constraint  仅用于消融，关闭机体角速度罚函数
--no-actuator-dynamics     仅用于消融，关闭电机/舵机动态和舵面速率罚函数
```

`11.34464 rad/s` 来自论文 differential-thrust turn 给出的约 `650 deg/s` 峰值，
只是当前参考包络，不是厂家认证的硬件角速度额定值。`0.04 s`、`0.03 s` 和
`25 rad/s` 沿用本项目 `config.py` 的工程估计，后续需要用闭环仿真或实验标定。

阶段 B 在普通前向映射选定连续姿态分支后，冻结该分支并用 Eigen AutoDiff
计算 `(T1,T2,delta1,delta2)` 对
`(v,a,jerk,snap,yaw,yaw_rate,yaw_acceleration)` 的 `4x15` Jacobian，以及机体
角速度对同一变量的 `3x15` Jacobian。梯度继续通过
梯形积分、离散一阶动态、位置/yaw 样条、固定总时间 softmax 和辅助点缩放映射传播，优化器使用
NLopt `LD_LBFGS`。`test_stage_b` 同时用中心有限差分检查单点执行器雅可比和完整
五维目标梯度。该梯度只在当前有效姿态分支内成立；分支切换与无效映射区域仍按
非光滑边界处理。

新版 CSV 会额外导出 `body_rate_*`、电机/舵面有限差分速率以及
`omega_cmd_req_*`、`delta_cmd_req_*`。绘图脚本检测到这些列时，会多生成一张
`*_dynamic_constraints.png`。平方罚函数是软约束；默认结果可以略越过 `75%`
规划阈值，只要最终 `FeasibilityChecker` 在完整物理/参考包络下通过。若需要
严格满足规划阈值，应提高罚权或改成显式不等式约束。

## 测试 3：自由总时间、多自由航点联合优化

`generate_joint_hover_trajectory` 实现 `study.md` 第 16.4、16.5 和第 19 章的
联合优化形式。起终 hover 点相距 `15 m`，中间三个位置航点和四个分段时间全部
参与优化。总时间不是等式约束，目标中使用 `time_weight * total_time` 防止通过
无限延长轨迹消除导数和约束罚值。当前没有障碍物或安全走廊约束。

```bash
# yaw 0 -> 0
./planning/build/generate_joint_hover_trajectory \
  --output planning/output/joint_hover_yaw0_optimized.csv \
  --baseline-output planning/output/joint_hover_yaw0_initial.csv

# yaw 0 -> pi
./planning/build/generate_joint_hover_trajectory \
  --yaw-pi \
  --output planning/output/joint_hover_yaw_pi_optimized.csv \
  --baseline-output planning/output/joint_hover_yaw_pi_initial.csv

# 画出轨迹、平滑性、高层约束、执行器幅值和动态约束对比
python3 planning/scripts/plot_planning_metrics.py \
  planning/output/joint_hover_yaw_pi_optimized.csv \
  --compare-csv planning/output/joint_hover_yaw_pi_initial.csv \
  --primary-label "Joint optimized" \
  --compare-label "Initial straight guess" \
  --prefix joint_hover_yaw_pi \
  --title "Test 3: 15 m hover-to-hover, yaw 0 to pi" \
  --velocity-max 6.0 \
  --collective-thrust-max 11.390625 \
  --body-rate-max 8.50848010347 \
  --omega-max 1875 \
  --delta-max 0.75 \
  --servo-rate-max 18.75
```

默认联合目标设置为：

```text
初始总时间                 4.0 s（每段 1.0 s，仅作为初值）
time_weight                0.5
actuator_weight            200
geometry_weight            0.02
actuator_margin            0.75
free_time_log_bound        1.5
自由航点半范围             [2, 3, 2] m
```

四段时间采用 `T_i = T_i_initial * exp(eta_i)`，所以每段严格为正，但段时间之和
自由变化。三个自由点只受有限搜索盒约束；这不是沿整条曲线施加的安全走廊。
常用命令行参数为：

```text
--distance M               hover 点距离，默认 15 m
--initial-total-time S     时间初值之和，默认 4 s
--time-weight W            总时间代价权重，默认 0.5
--free-time-log-bound B    每段 log-duration 边界，默认 1.5
--actuator-margin M        全部规划软阈值裕度，默认 0.75
--actuator-weight W        高层/执行器约束统一罚权，默认 200
--geometry-weight W        自由点归一化偏移权重，默认 0.02
--samples-per-segment N    每段约束积分区间数，默认 50
--maxeval N                最大目标评估数，默认 600
```

测试 3 同时启用速度、总推力、机体角速度高层约束，以及单电机推力、舵偏、舵速
和电机/舵机一阶动态约束。默认结果中 yaw `0 -> 0` 与 `0 -> pi` 的优化总时间
分别为约 `3.92184 s` 和 `3.92193 s`，均通过完整包络的 `2 ms` 稠密认证。
`test_joint_optimizer` 对两种 yaw 的全部 13 个变量做有限差分，当前最大梯度误差
约为 `1.73e-8`。

CSV 包含 `collective_thrust` 时，绘图脚本还会生成
`*_high_level_constraints.png`，显示速度范数、总推力和机体角速度约束。

## 测试 3 的闭环跟踪

`trajectory/planning_csv.py` 将优化 CSV 中的
`p/v/a/jerk/yaw/yaw_rate` 包装成现有控制器轨迹接口。规划器输出间隔不超过
`2 ms`，与闭环控制周期相同；接口对每个已导出的平坦输出做线性时间插值，并在
规划段结束后保持末端 hover。规划器的开环电机/舵面预测不会作为控制命令，实际
命令仍由 `paper_control` 控制链和控制分配在线计算。

```bash
# 两条轨迹各运行规划段 + 1 s 末端悬停
python main_sim.py --trajectory planning-yaw0 \
  --log planning/output/joint_hover_yaw0_tracking.npz \
  --hold-duration 1.0 --no-show

python main_sim.py --trajectory planning-yaw-pi \
  --log planning/output/joint_hover_yaw_pi_tracking.npz \
  --hold-duration 1.0 --no-show

# 生成闭环跟踪与执行器对比图
python planning/scripts/plot_tracking_results.py
```

也可以跟踪任意同格式 CSV：

```bash
python main_sim.py --trajectory planning-csv \
  --trajectory-csv planning/output/my_trajectory.csv \
  --log planning/output/my_trajectory_tracking.npz \
  --hold-duration 1.0 --no-show
```

默认日志额外记录 `rotor_speed_command`、`flap_command_deg`、
`planned_duration` 和 `trajectory_source`。当前两组结果的规划段位置 RMS/最大
误差分别为 `0.2021/0.3811 m` 和 `0.2371/0.4854 m`；两组实际执行器均未达到
`config.py` 的物理限幅。图像输出到：

```text
planning/output/figures/joint_hover_closed_loop_overview.png
planning/output/figures/joint_hover_closed_loop_actuators.png
```

## 当前范围

- 位置为 3D 七次 minimum-snap 样条，yaw 为 1D 三次 minimum-yaw-acceleration 样条，共享分段时间。
- 固定时间 Stage A/B 使用 softmax 严格保持总时间；测试 3 使用独立指数段时间并
  在目标中加入总时间代价，不固定时间之和。
- 阶段 B 使用固定姿态分支的执行器/机体角速度自动微分梯度和有界 L-BFGS；
  幅值、角速度、舵面速率和一阶动态命令限制均为采样软罚函数，最终由独立稠密
  检查认证。
- 外层先粗扫全部缩放范围，再细化每个“不可行到可行”的切换区间，不假设全局单调。
- 稠密检查显式加入样条连接点；默认最终间隔为 2 ms，可在 `FeasibilityChecker` 构造时收紧到 0.5–1 ms。
- 当前只处理位置/yaw 航点和首末端零导数。复杂特技所需的中间速度/加速度等式约束属于后续阶段。
- 当前没有障碍物和连续安全走廊；测试 3 的自由航点搜索盒只用于限制变量范围。
- 论文未公开的完整辨识参数仍采用当前工程估计，因此代码链路可复现，但数值边界不能宣称为论文实验值的严格复现。
