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

# 固定直线 Stage A 对照：同为 15 m、5 个固定等距航路点和 4 s 总时间，
# 只优化四段时间比例；不优化航路点、不加入约束、不做外层时间缩放
./planning/build/generate_simple_trajectory \
  --distance 15 --waypoint-count 5 --total-time 4 \
  --no-time-scaling \
  --output planning/output/joint_hover_yaw0_fixed_straight_ratio.csv

./planning/build/generate_simple_trajectory \
  --yaw-pi --distance 15 --waypoint-count 5 --total-time 4 \
  --no-time-scaling \
  --output planning/output/joint_hover_yaw_pi_fixed_straight_ratio.csv

# 联合优化与正确 Stage A 对照的轨迹、平滑性、高层约束、执行器和动态对比
python3 planning/scripts/plot_planning_metrics.py \
  planning/output/joint_hover_yaw_pi_optimized.csv \
  --compare-csv planning/output/joint_hover_yaw_pi_fixed_straight_ratio.csv \
  --primary-label "Joint: free time + waypoints + constraints" \
  --compare-label "Control: fixed 4 s + straight + ratio only" \
  --prefix joint_hover_yaw_pi_joint_vs_fixed_straight \
  --title "Test 3 control: yaw 0 to pi" \
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

这里的正式对照不是 `joint_hover_*_initial.csv`。后者只是联合优化器的等时间、
等距直线数值初值，没有经过任何优化。正式 Stage A 对照保持直线位置航路点
`x=[0, 3.75, 7.5, 11.25, 15] m` 不变，总时间严格固定为 `4 s`，只用 softmax
优化四段时间比例，目标只有位置 snap 能量和 yaw acceleration 能量。它不包含
时间代价、航路点变量以及任何高层/执行器罚函数；程序在优化后运行稠密检查并导出
执行器列，仅用于诊断，不会反过来影响时间比例。

对照结果为：

| 工况 | 固定直线对照分段时间 [s] | 对照峰值速度 [m/s] | 联合优化峰值速度 [m/s] |
| --- | --- | ---: | ---: |
| yaw `0 -> 0` | `[1.515394, 0.484606, 0.484606, 1.515394]` | 8.203125 | 6.049621 |
| yaw `0 -> pi` | `[1.515349, 0.484651, 0.484651, 1.515349]` | 8.202304 | 6.049637 |

两个固定直线对照都因峰值速度超过完整 `8 m/s` 参考包络而未通过稠密认证；其余
当前执行器幅值、动态和机体角速度指标均在完整包络内。这不是对照程序失败，而是
平滑性单目标在完全不知道速度/执行器约束时自然得到的结果。

### 不约束速度：内层联合约束与外层时间缩放

下面的消融实验在两种方法中都取消速度上限，但继续保留并认证总推力、机体角速度、
单电机推力、舵偏、舵速和电机/舵机一阶动态限制。

```bash
# 内层联合优化：自由总时间、三个自由位置航路点、无速度罚项
# 收紧积分与罚权，保证其余约束通过 2 ms 稠密认证
./planning/build/generate_joint_hover_trajectory \
  --no-velocity-constraint \
  --samples-per-segment 100 --actuator-weight 1000 --maxeval 500 \
  --output planning/output/joint_hover_yaw0_no_velocity_optimized.csv \
  --baseline-output planning/output/joint_hover_yaw0_no_velocity_initial.csv

./planning/build/generate_joint_hover_trajectory \
  --yaw-pi --no-velocity-constraint \
  --samples-per-segment 100 --actuator-weight 1000 --maxeval 500 \
  --output planning/output/joint_hover_yaw_pi_no_velocity_optimized.csv \
  --baseline-output planning/output/joint_hover_yaw_pi_no_velocity_initial.csv

# 对照：固定直线先优化 4 s 时间比例，再启用外层统一时间缩放；
# 外层可行性检查同样忽略速度，只检查其余约束
./planning/build/generate_simple_trajectory \
  --distance 15 --waypoint-count 5 --total-time 4 \
  --no-velocity-constraint \
  --output planning/output/joint_hover_yaw0_outer_scaled_no_velocity.csv

./planning/build/generate_simple_trajectory \
  --yaw-pi --distance 15 --waypoint-count 5 --total-time 4 \
  --no-velocity-constraint \
  --output planning/output/joint_hover_yaw_pi_outer_scaled_no_velocity.csv
```

结果如下：

| 工况 | 内层联合时间 [s] | 外层缩放时间 [s] | 内层相对缩短 | 内层/外层峰值速度 [m/s] |
| --- | ---: | ---: | ---: | ---: |
| yaw `0 -> 0` | 3.088731 | 3.839007 | 19.54% | 10.816 / 8.547 |
| yaw `0 -> pi` | 3.073283 | 3.855887 | 20.30% | 10.104 / 8.509 |

四条结果均通过“忽略速度、保留其余限制”的完整包络认证。内层联合方法明显更快，
但具有更大的 snap、速度、总推力和电机负载；外层缩放保持固定直线和固定时间比例，
轨迹更平滑保守。内层两次优化均用满约 `500` 次目标评估，因此它们是当前计算预算
下的可行最优候选，不应宣称为全局最优。

以 yaw `0 -> pi` 生成正式对比图：

```bash
python planning/scripts/plot_planning_metrics.py \
  planning/output/joint_hover_yaw_pi_no_velocity_optimized.csv \
  --compare-csv planning/output/joint_hover_yaw_pi_outer_scaled_no_velocity.csv \
  --primary-label "Inner joint constraints (no speed limit)" \
  --compare-label "Outer time scaling (no speed limit)" \
  --prefix joint_hover_yaw_pi_inner_vs_outer_no_velocity \
  --title "No velocity constraint: yaw 0 to pi" \
  --no-velocity-limit \
  --collective-thrust-max 11.390625 \
  --body-rate-max 8.50848010347 \
  --omega-max 1875 --delta-max 0.75 --servo-rate-max 18.75
```

`--no-velocity-limit` 只取消高层对比图中的速度限制线；速度曲线和峰值仍然正常输出。
对应两种生成程序中的 `--no-velocity-constraint` 才会真正从内层罚函数、外层可行性
搜索和最终认证中移除速度判据。

## 阶段 C：显式不等式约束联合优化

阶段 C 使用新的 `generate_stage_c_trajectory`，把测试 3 的十类执行器/高层
积分从加权罚项升级为 NLopt 显式不等式约束。变量仍是 13 维（4 个独立指数
分段时间 + 3 个自由航点），时间缩放彻底并入优化。约束行是各类型平方铰链
积分：行为零当且仅当优化网格上的每个采样在 `actuator_margin` 规划包络内。
无效平坦映射没有可用梯度，仍以屏障形式留在目标中。

```bash
# 默认管线：先跑软罚联合优化热启动，再用 SLSQP 硬约束精修
./planning/build/generate_stage_c_trajectory \
  --output planning/output/stage_c_yaw0_optimized.csv \
  --baseline-output planning/output/stage_c_yaw0_initial.csv

./planning/build/generate_stage_c_trajectory \
  --yaw-pi \
  --output planning/output/stage_c_yaw_pi_optimized.csv \
  --baseline-output planning/output/stage_c_yaw_pi_initial.csv

# 三方法对比图：硬约束 vs 软罚 vs 外层时间缩放
python3 planning/scripts/plot_planning_metrics.py \
  planning/output/stage_c_yaw_pi_optimized.csv \
  --compare-csv planning/output/joint_hover_yaw_pi_optimized.csv \
  --compare-csv-2 planning/output/joint_yaw_pi_outer_scaled.csv \
  --primary-label "Stage C: hard constraints" \
  --compare-label "Joint: soft penalty" \
  --compare-label-2 "Outer time scaling" \
  --prefix stage_c_vs_penalty_vs_outer_yaw_pi \
  --title "15 m hover-to-hover, yaw 0 to pi"
```

求解器默认直接使用 NLopt `LD_SLSQP`；消融实验中 `auglag-mma`、
`auglag-ccsaq`、`auglag-slsqp` 与直接 `mma` 在 240 次评估预算内均未能从深度
不可行初值到达可行（`--solver` 可切换，`--cold-start` 可跳过热启动）。
默认管线先在 `20` 采样/段的粗网格上跑软罚联合预优化作为热启动
（`--coarse-samples` 可调，`0` 退回细网格软罚热启动），SLSQP 再在 `50`
采样/段的细网格上做硬约束精修。15 m、速度约束开启时三方法对照为：

| 方法 | yaw `0->0` 总时间 [s] | yaw `0->pi` 总时间 [s] | 75% 规划包络满足情况 | 稠密认证 (margin 1.0) |
| --- | ---: | ---: | --- | --- |
| 外层时间缩放（固定直线+比例） | 4.102958 | 4.101423 | 不适用（直接用全包络） | 通过（峰值速度 7.997-7.999） |
| 软罚联合（测试 3 复跑） | 3.921840 | 3.921941 | 峰值速度超出 6 m/s 软阈值约 0.8% | 通过 |
| 阶段 C 硬约束（热启动） | 3.948456 | 3.948956 | 全部采样严格满足（残余 ≤ 4.8e-4） | 通过（峰值速度 6.002，恰好贴界） |

阶段 C 比软罚解慢约 0.7%，换来的是规划裕量下的硬保证：软罚解依赖最重的
权在 200 时仍留下 0.8% 的软越界，而阶段 C 的峰值速度精确停在 6.0 m/s 规划
边界上。热启动是必需的：yaw `0->pi` 从直线冷启动时初始点含无效映射样本
（屏障常值、零梯度），SLSQP 在 100 次评估后停在不可行点；yaw `0->0`
冷启动可用。同一台机器上的规划耗时（含最终 2 ms 稠密认证，三轮平均）：

| 方法 | 优化前 | 优化后 | 加速比 |
| --- | ---: | ---: | ---: |
| 外层时间缩放 | ~9.5 s | ~9.3 s（未改动） | 1.0x |
| 软罚联合 | ~13.1 s | ~4.2 s | 3.2x |
| 阶段 C 完整管线（yaw `0->0`） | ~22.0 s | ~3.5 s | 6.3x |
| 阶段 C 完整管线（yaw `0->pi`） | ~25.3 s | ~4.4 s | 5.7x |

加速来自两项数值等价/近似等价的重构：一，`integrateActuatorPenalty`
改为惰性 Jacobian——微基准显示单次线性化（15 维 AutoDiff，约 837 us）是
单次前向映射（Jet2，约 46 us）的 18 倍，而铰链未激活的采样点梯度贡献恰为
零，因此只有携带梯度质量的采样（幅值铰链激活或邻接激活动力学区间）才做
线性化，且重载复用一遍式预测与已选姿态分支避免重复前向；二，热启动
预优化降到粗网格（20 采样/段约为细网格单次评估的 1/3 成本，实测还减少了
迭代数），SLSQP 精修保持在细网格。最终认证始终为 2 ms 全包络稠密检查，
不参与加速折衷。

`test_stage_c` 覆盖：10×13 约束 Jacobian 中心差分校验（阈值 1e-2，见下）、
求解后全部约束行 ≤ 1e-6、端点固定、搜索盒不越界、以及 margin 1.0 的 2 ms
稠密认证。已知非光滑性：yaw 样条只有 C² 连续，yaw jerk 在分段节点跳变，
而电机转速依赖 yaw 加速度，因此动力学约束行在节点采样上的移动项只有单侧
导数，与中心差分（两侧平均）存在最高约 1% 的固有偏差；该偏差在阶段 C
之前的组合目标梯度中同样存在，只是被大数值平滑项的相对尺度掩盖。这不是
装配错误，是"罚函数采样点恰好落在节点上"这一离散化的固有性质。

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
- 阶段 C（`generate_stage_c_trajectory`）把十类采样积分改为 NLopt 显式不等式
  约束（默认 `LD_SLSQP`，软罚解热启动），在优化网格上严格满足规划裕量；
  连续时间保证仍由最终稠密检查提供。
- 外层先粗扫全部缩放范围，再细化每个“不可行到可行”的切换区间，不假设全局单调。
- 稠密检查显式加入样条连接点；默认最终间隔为 2 ms，可在 `FeasibilityChecker` 构造时收紧到 0.5–1 ms。
- 当前只处理位置/yaw 航点和首末端零导数。复杂特技所需的中间速度/加速度等式约束属于后续阶段。
- 当前没有障碍物和连续安全走廊；测试 3 的自由航点搜索盒只用于限制变量范围。
- 论文未公开的完整辨识参数仍采用当前工程估计，因此代码链路可复现，但数值边界不能宣称为论文实验值的严格复现。
