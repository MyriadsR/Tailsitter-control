# planning 阶段 A/B 项目详解

## 目录

1. [项目目标](#1-项目目标)
2. [目录结构与职责](#2-目录结构与职责)
3. [构建系统与第三方依赖](#3-构建系统与第三方依赖)
4. [公共数据类型](#4-公共数据类型)
5. [组合平坦输出轨迹](#5-组合平坦输出轨迹)
6. [固定总时间下的时间比例优化](#6-固定总时间下的时间比例优化)
7. [独立的 paper_planning 开环平坦映射](#7-独立的-paper_planning-开环平坦映射)
8. [稠密执行器可行性检查](#8-稠密执行器可行性检查)
9. [外层整体时间缩放](#9-外层整体时间缩放)
10. [简单轨迹程序的完整执行过程](#10-简单轨迹程序的完整执行过程)
11. [回归测试逐项说明](#11-回归测试逐项说明)
12. [当前限制与后续开发方向](#12-当前限制与后续开发方向)
13. [阅读代码的推荐顺序](#13-阅读代码的推荐顺序)
    - [阶段 B：执行器采样积分罚函数与辅助航点联合优化](#阶段-b执行器采样积分罚函数与辅助航点联合优化)
14. [SepticSplineND 的数学原理与逐函数说明](#14-septicsplinend-的数学原理与逐函数说明)
15. [Jet2 与 tailsitter_flatness.cpp 逐行数学说明](#15-jet2-与-tailsitter_flatnesscpp-逐行数学说明)
16. [借鉴 paper_manifold 规划并使用 paper_control 闭环跟踪](#16-借鉴-paper_manifold-规划并使用-paper_control-闭环跟踪)
17. [Stage B 执行器约束梯度与 L-BFGS](#17-stage-b-执行器约束梯度与-l-bfgs)

---

## 1. 项目目标

`planning/` 首先实现了 `paper_planning` 的阶段 A 轨迹生成流程：

1. 给定固定的位置和 yaw 航点；
2. 在固定总时间内优化各段时间比例；
3. 使用优化后的比例整体缩放轨迹时间；
4. 对每个候选轨迹执行独立的开环微分平坦前向变换；
5. 检查未经限幅的电机转速和舵面偏角是否可实现；
6. 找到扫描范围内最小的可行时间缩放系数。

整个数据流为：

```text
固定位置/yaw 航点
        │
        ▼
距离比例时间初值
        │
        ▼
固定总时间的 softmax 时间优化
        │
        ├── 3D SepticSpline：最小位置 Snap
        └── 1D CubicSpline：最小 yaw 加速度
        │
        ▼
获得最优分段时间比例 T*
        │
        ▼
粗扫描整体缩放系数 c
        │
        ▼
每个 c 都用 cT* 重新构造样条
        │
        ▼
稠密采样位置、速度、加速度、Jerk、Snap、yaw 导数
        │
        ▼
paper_planning 开环平坦变换
        │
        ▼
原始 T1、T2、omega1、omega2、delta1、delta2
        │
        ▼
执行器范围、奇异性和姿态分支连续性检查
        │
        ▼
细化所有“不可行 → 可行”切换区间
        │
        ▼
选择最小可行缩放系数
```

在此基础上，阶段 B 新增执行器采样积分罚函数：保持关键航点与总时间固定，
联合调整分段时间和一个辅助支撑点，但不再执行外层时间缩放。阶段 B 是软罚函数
优化，最终仍保留独立的稠密前向可行性检查。

---

## 2. 目录结构与职责

```text
planning/
├── CMakeLists.txt
├── README.md
├── study.md
├── config/
│   ├── paper_vehicle.yaml
│   └── simulation_vehicle.yaml
├── include/planning/
│   ├── planning_types.hpp
│   ├── flat_output_trajectory.hpp
│   ├── time_allocation.hpp
│   ├── actuator_penalty_optimizer.hpp
│   ├── tailsitter_flatness.hpp
│   ├── feasibility_checker.hpp
│   └── time_scaler.hpp
├── src/
│   ├── flat_output_trajectory.cpp
│   ├── time_allocation.cpp
│   ├── actuator_penalty_optimizer.cpp
│   ├── tailsitter_flatness.cpp
│   ├── feasibility_checker.cpp
│   └── time_scaler.cpp
├── apps/
│   ├── generate_simple_trajectory.cpp
│   └── generate_stage_b_trajectory.cpp
├── scripts/
│   └── plot_planning_metrics.py
├── tests/
│   ├── test_stage_a.cpp
│   └── test_stage_b.cpp
└── third_party/
    └── SplineTrajectory/          # Git submodule
```

各模块之间的依赖关系如下：

| 模块 | 主要输入 | 主要输出 | 作用 |
| --- | --- | --- | --- |
| `planning_types` | 无 | 公共数据类型 | 定义轨迹采样、车辆参数和可行性报告 |
| `flat_output_trajectory` | 航点、分段时间 | 平坦输出及各阶导数 | 包装位置和 yaw 两套样条 |
| `time_allocation` | 固定总时间、航点 | 最优分段时间 | 优化平滑能量且保持总时间不变 |
| `actuator_penalty_optimizer` | 固定总时间、关键/辅助点、原始执行器映射 | 分段时间和辅助点 | 对四个执行器输入的采样违反量积分并联合优化 |
| `tailsitter_flatness` | 单个平坦输出采样 | 原始执行器预测及固定分支雅可比 | 实现论文开环式 (20)～(56) |
| `feasibility_checker` | 完整组合轨迹 | 可行性报告 | 稠密调用开环映射并检查限制 |
| `time_scaler` | 基础分段时间、航点 | 最小可行缩放结果 | 粗扫描加切换区间二分 |
| `generate_simple_trajectory` | 命令行选项 | 控制台报告和 CSV | 串起阶段 A 的完整示例 |
| `generate_stage_b_trajectory` | 固定总时间和罚函数参数 | 阶段 A/B 两份 CSV | 串起阶段 B 并提供同口径基线 |
| `plot_planning_metrics.py` | 规划 CSV | 三类规划指标图 | 绘制速度着色轨迹、平滑性和执行器预测 |
| `test_stage_a` | 固定测试轨迹 | 测试通过/失败 | 验证梯度、动力学重构和边界行为 |
| `test_stage_b` | 固定关键点和一个辅助点 | 测试通过/失败 | 验证执行器雅可比、完整目标梯度和优化不变量 |

---

## 3. 构建系统与第三方依赖

### 3.1 CMake 目标

`CMakeLists.txt` 要求 C++17，并生成五个目标：

- `tailsitter_planning`：包含六个核心 `.cpp` 文件的库目标，默认构建为静态库；
- `generate_simple_trajectory`：简单轨迹生成程序；
- `generate_stage_b_trajectory`：阶段 B 执行器罚函数联合优化程序；
- `test_stage_a`：由 CTest 调用的阶段 A 回归测试。
- `test_stage_b`：由 CTest 调用的阶段 B 回归测试。

依赖包括：

- Eigen 3.3 或更高版本；
- NLopt；
- `third_party/SplineTrajectory` 头文件库。

`SPLINE_TRAJECTORY_ROOT` 是 CMake cache 变量，默认指向项目内的 submodule，因此也可以在配置时替换：

```bash
cmake -S planning -B planning/build \
  -DSPLINE_TRAJECTORY_ROOT=/another/path/SplineTrajectory
```

### 3.2 为什么使用 SplineTrajectory

本项目复用 SplineTrajectory 的样条构造、求值、能量，以及对时间、内部航点和
多项式系数偏导的解析反向传播：

```cpp
using PositionSpline = SplineTrajectory::SepticSplineND<3>;
using YawSpline = SplineTrajectory::CubicSplineND<1>;
```

- 七次样条最小化四阶导数平方积分，对应 minimum-snap / MINCO S4；
- 三次样条最小化二阶导数平方积分，对应 minimum-yaw-acceleration / MINCO S2；
- 两套样条使用完全相同的物理分段时间。

当前没有使用 SplineTrajectory 自带的完整 `SplineOptimizer`。原因是位置和 yaw
使用不同阶次样条、共享同一组时间，而且阶段 B 还要注入执行器罚函数对系数和
时间的偏导；直接组合 `getEnergyGrad()` 与 `propagateGrad()` 更清楚。

---

## 4. 公共数据类型

公共结构定义在 `planning_types.hpp`。

### 4.1 `FlatOutputSample`

它表示某个时刻的平坦输出及其导数：

```text
位置部分：p, v, a, jerk, snap, crackle
yaw 部分：psi, psi_dot, psi_ddot, psi_jerk
```

当前开环执行器映射实际需要：

- 位置到 Snap；
- yaw 到 yaw acceleration。

`crackle` 和 `yaw_jerk` 不参与当前时刻的执行器值恢复，但已用于执行器积分对
分段时间的移动采样点梯度；以后还可用于执行器速率约束。

### 4.2 `VehicleParameters`

该结构保存开环模型需要的参数：

- 质量、重力和惯量；
- 零升力迎角 `alpha_0`、电机安装角 `alpha_T`；
- 电机、舵面相对质心的几何臂长；
- 推力系数、反扭矩系数和推力俯仰力矩系数；
- 机翼、滑流和舵面气动系数；
- 电机转速和舵面范围；
- hover 速度正则量及奇异性容差。

提供两组构造方式：

- `paperVehicle()`：返回结构默认值，质量为 `0.68 kg`，`c_T=1.62e-6`，`c_mu=1.78e-8`；
- `simulationVehicle()`：以 paper 参数为基础，仅覆盖当前 Python 仿真的质量、`c_T` 和 `c_mu`。

需要特别注意：`config/*.yaml` 目前只是便于审计的参数快照，程序没有 YAML 解析器，也不会在运行时读取它们。真正生效的参数仍定义在 `VehicleParameters` 和上述两个静态构造函数中。后续如果频繁调参，应增加统一配置加载，避免 C++ 默认值与 YAML 漂移。

论文没有公布全部辨识参数。除明确区分的质量和电机系数外，两套配置仍共享若干工程估计值，因此当前结果属于算法链路复现，不能视为论文数值边界的严格复现。

### 4.3 `BranchState`

`BranchState` 保存上一个采样时刻的姿态四元数。平坦映射中的 roll 和 pitch 都有加 `k*pi` 的多分支解，下一时刻需要选择与上一姿态最接近的候选，避免仅使用 `atan2` 主值导致姿态翻转。

### 4.4 `ActuatorPrediction`

该结构记录一次开环映射的结果：

- 姿态四元数；
- 机体系角速度和角加速度；
- 期望机体系力矩；
- 总推力和两个电机的独立推力；
- 两个电机转速；
- 两个舵面偏角；
- 舵面分配矩阵最小奇异值；
- 当前选中的 roll/pitch 离散分支编号；
- 数值映射是否有效及失败原因。

`valid=true` 只表示：公式计算成功、没有奇异矩阵、没有负单电机推力、结果均为有限数。它不表示执行器没有超过上限。执行器范围由 `FeasibilityChecker` 另外判断。

### 4.5 `ActuatorLinearization`

该结构在一份普通 `ActuatorPrediction` 之外保存固定姿态分支下的
$4\times15$ 执行器雅可比、`differentiable` 标志和失败原因。15 列的顺序为
速度、加速度、Jerk、Snap 各三维，再接 yaw、yaw rate 和 yaw acceleration；
四行依次是 $T_1,T_2,\delta_1,\delta_2$。它只描述当前有效分支的局部导数，
不包含分支选择逻辑的导数。

### 4.6 `FeasibilityReport`

报告汇总：

- 整条轨迹是否可行；
- 最大归一化违反量；
- 峰值电机转速和峰值舵偏角；
- 第一次失败的时刻和原因；
- 可选的全部时序预测结果。

只有导出 CSV 或需要详细分析时才打开 `keep_predictions`，粗扫描阶段默认不保存全部采样，以减少内存占用。

---

## 5. 组合平坦输出轨迹

`FlatOutputTrajectory` 把位置和 yaw 两套不同阶次的样条组合为统一接口。

### 5.1 更新轨迹

`update()` 接收：

```cpp
durations            // M 个正分段时间
position_waypoints   // (M+1) x 3
yaw_waypoints        // (M+1) x 1
```

函数首先检查：

- 至少存在一个轨迹段；
- 航点数等于分段数加一；
- 位置与 yaw 航点数相等；
- 每段时间都是有限正数。

随后构造两套样条。默认 `BoundaryConditions` 为零，因此：

- 位置首末端速度、加速度和 Jerk 为零；
- yaw 首末端 yaw rate 为零。

中间航点只约束位置和 yaw 值，中间导数由最小能量样条自动求解。

### 5.2 yaw 展开

普通实数多项式不知道 yaw 是圆周变量。`unwrapYawWaypoints()` 将相邻 yaw 差值映射到最短的 `2*pi` 等价分支。例如：

```text
[170 deg, -170 deg]  ->  [170 deg, 190 deg]
```

内部一直保留展开后的连续角度，当前 CSV 也输出展开值，没有重新包裹到 `[-pi, pi)`。

### 5.3 轨迹采样

`sample(t)` 先将时间限制在 `[0,T]`，再分别从两套样条提取导数。这样上层模块不需要知道位置和 yaw 使用了不同次数的多项式。

### 5.4 能量与时间梯度

总目标为：

$$
J=J_{snap}+\mu_\psi J_{yaw}
$$

其中：

$$
J_{snap}=\int_0^T\left\|\mathbf p^{(4)}(t)\right\|^2dt
$$

$$
J_{yaw}=\int_0^T\ddot\psi(t)^2dt
$$

对应代码为：

```cpp
position_.getEnergy() + yaw_weight * yaw_.getEnergy()
```

时间梯度同样由两套样条的解析梯度相加。这里的 `getEnergyGradTimes()` 已包含“分段时间改变后最优多项式系数也随之改变”的隐式影响，并非只对固定系数求偏导。

---

## 6. 固定总时间下的时间比例优化

该部分由 `FixedTotalTimeAllocator` 实现。

### 6.1 距离比例初值

初始分段时间按相邻位置航点距离分配：

$$
T_i^{(0)}=\bar T\frac{\|\mathbf p_{i+1}-\mathbf p_i\|+\epsilon}
{\sum_j(\|\mathbf p_{j+1}-\mathbf p_j\|+\epsilon)}
$$

代码中的 `epsilon=1e-3`，因此即使两个相邻位置航点重合，也会得到一个正时间。当前初值没有使用 yaw 距离，yaw 对时间比例的影响由后续目标优化体现。

### 6.2 softmax 时间映射

为自动满足正时间和固定总时间，优化变量不是物理时间，而是自由变量 `tau`：

$$
T_i=\bar T\frac{e^{\tau_i}}{\sum_j e^{\tau_j}}
$$

它保证：

$$
T_i>0,\qquad\sum_iT_i=\bar T
$$

由于所有 `tau` 同时加常数不会改变结果，代码固定最后一个 logit 为零，只优化前 `M-1` 个变量。指数计算前还会减去最大 logit，避免溢出。

NLopt 变量范围设置为 `[-12,12]`。这不是物理时间约束，而是避免 L-BFGS 线搜索将 softmax 推到机器精度下的极端比例。

### 6.3 softmax 链式梯度

首先明确两层变量：

- `T_k` 是第 `k` 段的真实物理时间；
- `tau_i` 是优化器使用的无约束 logit；
- `J` 先通过所有物理时间 `T_1,...,T_M` 影响两套样条，再间接依赖 `tau`。

从 SplineTrajectory 得到物理时间梯度：

$$
g_k=\frac{\partial J}{\partial T_k}
$$

这里的 `g_k` 是当前分段时间下的完整样条能量梯度，已经包含最优多项式系数随 `T_k` 改变的影响。

#### 6.3.1 先求 softmax 的 Jacobian

记：

$$
S=\sum_j e^{\tau_j}
$$

则：

$$
T_k=\bar T\frac{e^{\tau_k}}{S}
$$

现在对任意自由变量 `tau_i` 求 `T_k` 的导数。需要区分 `k=i` 和 `k!=i`，可以用 Kronecker delta `delta_ki` 统一表示：

$$
\frac{\partial e^{\tau_k}}{\partial\tau_i}
=\delta_{ki}e^{\tau_k}
$$

同时：

$$
\frac{\partial S}{\partial\tau_i}=e^{\tau_i}
$$

使用商法则：

$$
\begin{aligned}
\frac{\partial T_k}{\partial\tau_i}
&=\bar T
\frac{\delta_{ki}e^{\tau_k}S-e^{\tau_k}e^{\tau_i}}
{S^2}\\
&=\bar T\frac{e^{\tau_k}}{S}
\left(\delta_{ki}-\frac{e^{\tau_i}}{S}\right)
\end{aligned}
$$

注意：

$$
\bar T\frac{e^{\tau_k}}{S}=T_k,
\qquad
\frac{e^{\tau_i}}{S}=\frac{T_i}{\bar T}
$$

所以 softmax 时间映射的 Jacobian 为：

$$
\boxed{
\frac{\partial T_k}{\partial\tau_i}
=T_k\left(\delta_{ki}-\frac{T_i}{\bar T}\right)
}
$$

展开来看：

$$
\frac{\partial T_i}{\partial\tau_i}
=T_i\left(1-\frac{T_i}{\bar T}\right)>0
$$

而对于 `k!=i`：

$$
\frac{\partial T_k}{\partial\tau_i}
=-\frac{T_kT_i}{\bar T}<0
$$

也就是说，增大 `tau_i` 会增加第 `i` 段的时间，但为了保持总时间不变，其他所有段的时间会按当前比例同时减少。

还可以直接验证总时间导数为零：

$$
\begin{aligned}
\sum_k\frac{\partial T_k}{\partial\tau_i}
&=\sum_kT_k\left(\delta_{ki}-\frac{T_i}{\bar T}\right)\\
&=T_i-\frac{T_i}{\bar T}\sum_kT_k\\
&=T_i-\frac{T_i}{\bar T}\bar T=0
\end{aligned}
$$

这说明 softmax 的每一次变量变化都只是在各段之间重新分配时间，不会改变总时间。

#### 6.3.2 再使用链式法则

因为 `J` 通过所有 `T_k` 依赖 `tau_i`：

$$
\frac{\partial J}{\partial\tau_i}
=\sum_k
\frac{\partial J}{\partial T_k}
\frac{\partial T_k}{\partial\tau_i}
$$

代入 `g_k` 和上面的 softmax Jacobian：

$$
\begin{aligned}
\frac{\partial J}{\partial\tau_i}
&=\sum_k g_kT_k
\left(\delta_{ki}-\frac{T_i}{\bar T}\right)\\
&=g_iT_i-\frac{T_i}{\bar T}\sum_kT_kg_k\\
&=T_i\left(g_i-\frac{1}{\bar T}\sum_kT_kg_k\right)
\end{aligned}
$$

最终得到：

$$
\boxed{
\frac{\partial J}{\partial\tau_i}
=T_i\left(g_i-\frac{1}{\bar T}\sum_kT_kg_k\right)
}
$$

其中：

$$
\bar g_T=\frac{1}{\bar T}\sum_kT_kg_k
$$

可以理解为按分段时间占比加权的平均物理时间梯度。因此公式也可以写成：

$$
\frac{\partial J}{\partial\tau_i}=T_i(g_i-\bar g_T)
$$

这正是 `objective()` 中两段代码的含义：

```cpp
weighted_mean += durations[j] * grad_t(j);
weighted_mean /= total_time;

gradient[i] = durations[i] * (grad_t(i) - weighted_mean);
```

#### 6.3.3 两段轨迹的直观例子

对于两段轨迹，代码固定 `tau_2=0`，只优化 `tau_1=tau`：

$$
T_1=\bar T\frac{e^\tau}{e^\tau+1},
\qquad
T_2=\bar T\frac{1}{e^\tau+1}
$$

可以得到：

$$
\frac{\partial T_1}{\partial\tau}
=\frac{T_1T_2}{\bar T},
\qquad
\frac{\partial T_2}{\partial\tau}
=-\frac{T_1T_2}{\bar T}
$$

所以：

$$
\boxed{
\frac{\partial J}{\partial\tau}
=\frac{T_1T_2}{\bar T}(g_1-g_2)
}
$$

该式非常直观：优化变量增加时，是把时间从第二段转移给第一段；代价如何变化只取决于两段物理时间梯度之差。

- 若 `g_1=g_2`，重新分配少量时间不会在一阶上改变代价；
- 若 `g_1>g_2`，增加第一段时间、减少第二段时间会使 `J` 增加；
- 若 `g_1<g_2`，这样重新分配会使 `J` 减小。

#### 6.3.4 为什么固定最后一个 logit 不改变公式

如果所有 `tau_i` 都是自由变量，同时给它们加同一个常数不会改变任何 `T_i`，所以优化问题存在一个冗余方向，其 Jacobian 必然是奇异的。

代码设置最后一个 logit `tau_M=0`，只把前 `M-1` 个变量交给 NLopt。虽然 `tau_M` 固定，但 `T_M` 仍通过 softmax 分母随其他 `tau_i` 改变，因此上面对任意自由 `i<M` 的推导完全不变；只是无需计算和返回固定变量 `tau_M` 的梯度。

### 6.4 优化器设置

当前使用 NLopt `LD_LBFGS`：

```text
相对目标容差：1e-10
相对变量容差：1e-9
最大评估次数：250
```

如果只有一个轨迹段，则没有时间比例自由度，直接返回总时间本身，不调用 NLopt。

优化结果同时记录初始代价、最终代价、评估次数以及是否真正运行了优化器。

---

## 7. 独立的 paper_planning 开环平坦映射

`TailsitterFlatness::forward()` 是项目最核心的部分。它从一个平坦输出采样直接计算理想模型所需的原始执行器输入。

### 7.1 与 controller 中平坦映射的根本区别

规划模块与控制模块解决的问题不同：

| planning 开环映射 | controller 闭环控制 |
| --- | --- |
| 输入是理想参考轨迹及其高阶导数 | 输入同时包含参考、当前状态和滤波测量 |
| 计算理想模型精确跟踪所需的原始执行器量 | 计算实际系统的反馈修正命令 |
| 忽略舵面直接力的非最小相位项 | 控制器会考虑当前舵面状态和 INDI 修正 |
| 不限幅电机和舵面 | 在线分配器会限制总推力、差分推力和舵偏角 |
| 不包含电机/舵机一阶动态和速率限制 | 仿真执行器包含动态、限位和速率限制 |
| 输出只用于可行性预测 | 输出会进入实际闭环仿真 |

因此 `planning/` 没有调用：

- `controller/att_ctrl.py`；
- `controller/actuator_ctrl.py`；
- 任何 `clip` 后的执行器结果。

如果规划阶段使用限幅结果，不可行轨迹会被伪装成可行轨迹。

### 7.2 为什么实现 `Jet2`

论文需要从位置到 Snap、从 yaw 到 yaw acceleration，最终得到姿态角的一阶和二阶导数。手写式 (27)～(50) 容易出现长公式和符号错误，因此代码实现了一个二阶自动微分标量：

```cpp
struct Jet2 {
    double value;
    double first;
    double second;
};
```

`Jet2` 为加、减、乘、除、`sin`、`cos`、`sqrt` 和 `atan2` 实现二阶链式法则。它不是数值时间差分，而是把解析导数随每次数学运算同步传播。

进入映射时构造：

```text
velocity Jet = (v, a, jerk)
force Jet    = (m(a-g*iz), m*jerk, m*snap)
yaw Jet      = (psi, psi_dot, psi_ddot)
```

因此从 `atan2` 得到的 roll、pitch Jet 自动包含：

```text
(phi, phi_dot, phi_ddot)
(theta, theta_dot, theta_ddot)
```

### 7.3 参考力与 hover 正则化

NED 世界系中参考力为：

$$
\mathbf f^i=m(\mathbf a-g\mathbf i_z)
$$

速度范数使用：

$$
r=\sqrt{\mathbf v^T\mathbf v+\epsilon_v^2}
$$

当前 `epsilon_v=1e-3 m/s`。因为 `r` 本身也是 Jet，代码会自动得到正则化后的 `r_dot` 和 `r_ddot`，避免静止 hover 端点的 `0/0`。

### 7.4 roll、pitch 与总推力

首先使用 yaw 将参考力变换到中间坐标系，然后计算基本 roll 分支：

$$
\phi_0=-\operatorname{atan2}(f_y^\psi,f_z^i)
$$

对每个 `phi = phi0 + k*pi`，再将速度和力变换到 phi 系。忽略舵面直接力时：

$$
\eta=\frac{\sin\bar\alpha(c_{L_T}-1)}
{\cos\bar\alpha(1-c_{D_T})}
$$

其中：

$$
\bar\alpha=\alpha_0+\alpha_T
$$

代码中的 `sigma_y` 和 `sigma_x` 分别是论文式 (24) 的 `atan2` 第一、第二参数，由参考力、速度、升力和阻力项组成：

$$
\bar\theta_0=\operatorname{atan2}(\sigma_y,\sigma_x)
$$

随后枚举 `theta_bar = theta_base + k*pi`，用式 (25) 计算总推力，只保留非负且有限的候选。

真实机体系 pitch 为：

$$
\theta=\bar\theta+\alpha_0
$$

最多会生成四组 roll/pitch 候选。若已有上一姿态，就选择四元数距离最近的候选；若没有上一姿态，则选择第一个合法候选。

### 7.5 角速度、角加速度与力矩

姿态采用 Z-X-Y 顺序。世界系到机体系的坐标变换为：

$$
R_i^b=R_\phi^b(\theta)R_\psi^\phi(\phi)R_i^\psi(\psi)
$$

角速度由 yaw、roll、pitch 三部分投影到机体系后相加，对应论文式 (38)：

$$
\Omega=
R_i^b\begin{bmatrix}0\\0\\\dot\psi\end{bmatrix}
+R_\phi^b\begin{bmatrix}\dot\phi\\0\\0\end{bmatrix}
+\begin{bmatrix}0\\\dot\theta\\0\end{bmatrix}
$$

由于所有旋转角和角速率仍是 Jet，读取 `omega.first` 就得到式 (50) 的角加速度，而不需要有限差分。

期望机体系力矩为：

$$
\mathbf m=J\dot\Omega+\Omega\times J\Omega
$$

### 7.6 差分推力和电机转速

根据论文式 (53)，代码从目标 `m_z` 解差分推力：

$$
\Delta T=T_1-T_2=\frac{m_z}{d_T}
$$

其中 `d_T` 同时包含：

- 电机横向臂产生的推力力矩；
- 两个旋翼反扭矩产生的力矩。

然后：

$$
T_1=\frac{T+\Delta T}{2},\qquad
T_2=\frac{T-\Delta T}{2}
$$

$$
\omega_i=\sqrt{\frac{T_i}{c_T}}
$$

这里不会像在线控制分配器那样截断 `T` 或 `Delta T`。任一单电机推力为负时，开环映射立即无效。

### 7.7 舵面分配

先从总期望力矩中扣除推力臂和旋翼反扭矩贡献：

$$
\mathbf m_\delta=\mathbf m-\mathbf m_T-\mathbf m_\mu
$$

舵面效率为：

$$
\nu_i=-c_{L_T}^{\delta}\cos\bar\alpha T_i
-c_{L_V}^{\delta}\|\mathbf v\|v_x^\alpha
$$

然后构造论文式 (55) 的二维线性系统：

$$
\begin{bmatrix}
-l_{\delta y}\cos\alpha_0\nu_1 & l_{\delta y}\cos\alpha_0\nu_2\\
l_{\delta x}\nu_1 & l_{\delta x}\nu_2
\end{bmatrix}
\begin{bmatrix}\delta_1\\\delta_2\end{bmatrix}
=
\begin{bmatrix}m_{\delta x}\\m_{\delta y}\end{bmatrix}
$$

代码先用 SVD 求最小奇异值。如果分配矩阵接近奇异，就返回失败；否则用 LU 直接求未经限幅的舵角。

与论文式 (52) 一致，差分推力求解忽略了舵面在 `m_z` 上乘有 `sin(alpha_0)` 的小贡献。因此这里严格重构的是 roll/pitch 分配方程，yaw 方向保留论文近似。

### 7.8 开环映射失败类型

当前可能返回：

- 总推力公式分母奇异；
- 找不到非负总推力姿态分支；
- 差分推力公式分母奇异；
- 某个单电机推力为负；
- 舵面分配矩阵奇异；
- 任意输出出现 NaN 或 Inf。

执行器超过上限不是这里的 `failure_reason`，而是在下一层可行性检查中处理。

---

## 8. 稠密执行器可行性检查

`FeasibilityChecker` 对一条已经构造好的组合轨迹执行采样检查。

### 8.1 采样点生成

默认采样目标间隔为 `2 ms`。代码用总时间除以向上取整后的区间数，因此实际间隔不超过指定值。除均匀采样外，还会显式加入所有分段连接时刻，排序并去重。

当前还没有实现峰值附近的自适应加密。最终高置信度验证可将构造参数收紧到 `0.5～1 ms`。

### 8.2 姿态分支连续性

每次调用开环映射后，检查相邻四元数的最短旋转角：

$$
\Delta\vartheta=2\arccos(|q_{k-1}^Tq_k|)
$$

若单个毫秒级采样步超过 `0.5 rad`，就视为平坦映射分支跳变，而不是正常连续姿态运动。

加入该检查的原因是：快速水平 hover-to-hover 轨迹可能经过近零总推力奇点。仅检查 NaN 和执行器上限时，姿态可能瞬间跳转约 180 度却仍被误判为可行。外层时间缩放需要避开这种分支奇点。

`0.5 rad` 当前是工程阈值且写在实现中。以后若改变采样率或加入超高速姿态轨迹，应将它改为配置参数，或结合预测角速度建立与采样间隔相关的连续性判据。

### 8.3 执行器限制

有效上限为：

```text
omega_upper = omega_max * actuator_margin
delta_upper = delta_max * actuator_margin
```

默认 `actuator_margin=1.0`，即使用原始物理边界。设为 `0.9` 可保留 10% 闭环余量。

最大归一化违反量取所有采样、所有执行器约束中的最大值：

$$
g_{max}=\max_t\left\{
\frac{\omega_1}{\omega_{upper}}-1,
\frac{\omega_2}{\omega_{upper}}-1,
\frac{|\delta_1|}{\delta_{upper}}-1,
\frac{|\delta_2|}{\delta_{upper}}-1,
\text{电机下限项}
\right\}
$$

只有 `g_max <= 0`、开环映射始终有效且姿态分支连续时，整条轨迹才可行。

---

## 9. 外层整体时间缩放

`TimeScaler` 搜索：

$$
\mathbf T(c)=c\mathbf T^*
$$

其中 `T*` 是固定总时间优化后得到的时间比例。

### 9.1 每次缩放都重新构造样条

`buildScaled()` 会将每段时间乘以 `c`，然后重新调用 `FlatOutputTrajectory::update()`。这一步不能省略，因为位置的第 `r` 阶导数按 `c^{-r}` 缩放，yaw 导数也同样改变。只修改 CSV 时间戳会得到错误的速度、Snap 和执行器预测。

### 9.2 粗扫描

在 `[minimum_scale, maximum_scale]` 中均匀检查 `coarse_samples` 个缩放系数，并保存：

- 当前缩放系数；
- 是否可行；
- 最大归一化违反量。

简单示例使用：

```text
搜索范围：[0.25, 3.0]
粗采样数：29
细化容差：5e-4
```

### 9.3 细化所有不可行到可行边界

代码不假设全局单调，只要粗扫描中出现：

```text
前一个点不可行，后一个点可行
```

就对该区间单独二分，直到区间宽度小于 `scale_tolerance`。最后从所有可行区间入口中选择最小者。

这种方法能处理粗扫描可见的多个可行区间，但仍可能漏掉宽度小于粗网格间距、且两端都不可行的狭窄可行岛。因此复杂特技需要增加粗采样数、多组扫描范围或更先进的全局搜索。

---

## 10. 简单轨迹程序的完整执行过程

`generate_simple_trajectory` 使用三个固定航点：

```text
p0 = [0.0, 0.0, -10.0]
p1 = [1.5, 0.0, -10.0]
p2 = [6.0, 0.0, -10.0]
```

中间航点使轨迹具有两个分段，从而能够真实验证固定总时间下的时间比例优化。

支持两种 yaw：

```text
默认：     [0, 0, 0]
--yaw-pi： [0, pi/2, pi]
```

固定总时间初值为 `4 s`。程序执行顺序为：

1. 构造位置和 yaw 航点矩阵；
2. 默认用 `FixedTotalTimeAllocator` 优化两段时间；若指定
   `--no-optimize-time-ratio`，则直接保留距离比例初值；
3. 根据命令行选择 paper 或 simulation 参数；
4. 以 `2 ms` 间隔创建可行性检查器；
5. 搜索整体时间缩放系数；
6. 用最终时间重新构造轨迹；
7. 保存最终轨迹和开环执行器预测 CSV；
8. 打印时间、代价和执行器峰值。

命令行参数为：

```text
--yaw-pi              使用 0 -> pi yaw 轨迹
--simulation-vehicle  使用当前仿真的质量和电机系数
--no-optimize-time-ratio
                      跳过内层时间比例优化，保留距离比例初值
--no-time-scaling     跳过外层整体时间缩放，保持固定总时间 4 s
--output FILE         指定 CSV 文件
```

当前 paper 参数运行结果约为：

| 工况 | 固定总时间优化后的分段时间 | 最小可行比例 | 最小可行总时间 |
| --- | --- | --- | --- |
| yaw `0 -> 0` | `[1.515394, 2.484606] s` | `0.441825` | `1.767299 s` |
| yaw `0 -> pi` | `[1.516318, 2.483682] s` | `0.443743` | `1.774972 s` |

`0 -> pi` 的最短时间略长于无 yaw 轨迹，符合额外 yaw 动态通常会增加执行器需求的预期。但由于参数含估计值，这些时间只能作为当前实现的回归基准。

### 10.1 不优化时间比例的对照组

`--no-optimize-time-ratio` 定义了一个单变量对照实验。它跳过内层 NLopt
优化，并使用航点距离给出的初始比例：

$$
T_i^{\mathrm{base}}
=\bar T\frac{\|\mathbf p_{i+1}-\mathbf p_i\|}
{\sum_j\|\mathbf p_{j+1}-\mathbf p_j\|}.
$$

本例的两段长度为 $1.5\ \mathrm m$ 和 $4.5\ \mathrm m$，所以在
$\bar T=4\ \mathrm s$ 时，对照组分段时间约为

$$
[T_1,T_2]=[1.000333,\ 2.999667]\ \mathrm s.
$$

这里微小的 `0.000333 s` 偏差来自 `distanceInitialGuess()` 给每段几何距离
加入的 $10^{-3}\ \mathrm m$ 正则项；它避免零长度分段得到零权重。除“是否
执行时间比例优化”外，两组的其余流程完全相同：

```text
相同固定航点和 yaw 航点
相同固定总时间 4 s
相同的七次位置样条和三次 yaw 样条
相同的外层整体时间缩放范围与精度
相同的开环平坦映射、采样间隔和执行器限制
```

因此最终总时间不是固定为 `4 s`。两组分别以自己的分段比例执行相同的外层
搜索，得到该比例下的最小可行总时间。当前 paper 参数的比较结果为：

| yaw 工况 | 方法 | 固定总时间下分段时间 `[s]` | 最小可行总时间 `[s]` | 峰值 Snap `[m/s^4]` | 峰值 yaw 加速度 `[rad/s^2]` | 峰值电机转速 `[rad/s]` | 峰值舵偏角 `[rad]` |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `0 -> 0` | 优化时间比例 | `[1.515394, 2.484606]` | 1.767299 | 516.642708 | 0 | 1920.378336 | 0.844562 |
| `0 -> 0` | 固定距离比例 | `[1.000333, 2.999667]` | 2.396484 | 983.543788 | 0 | 2012.344198 | 0.755070 |
| `0 -> pi` | 优化时间比例 | `[1.516318, 2.483682]` | 1.774972 | 509.088982 | 11.945541 | 1961.767336 | 0.622273 |
| `0 -> pi` | 固定距离比例 | `[1.000333, 2.999667]` | 2.404157 | 971.047650 | 15.208964 | 2065.017515 | 0.872620 |

在这两个算例中，优化时间比例使最小可行总时间分别降低约 `26.3%` 和
`26.2%`，峰值 Snap 降低约 `47.5%`。不过该目标优化的是固定总时间下的
Snap/yaw 加速度积分，不是逐项最小化执行器峰值，所以不能期待所有峰值都
同时下降。例如 `yaw 0 -> 0` 中优化组的峰值舵偏角反而更大，但仍低于
`1 rad` 限制。这正是必须保留开环执行器可行性检查的原因。

复现零偏航对照数据：

```bash
./planning/build/generate_simple_trajectory \
  --output planning/output/simple_yaw0.csv
./planning/build/generate_simple_trajectory \
  --no-optimize-time-ratio \
  --output planning/output/simple_yaw0_no_time_ratio_opt.csv
```

### 10.2 不进行外层时间缩放的对照组

`--no-time-scaling` 保留内层时间比例优化得到的 $T_i^*$，但固定外层缩放系数：

$$
c=1,
\qquad
T_i^{\mathrm{final}}=T_i^*,
\qquad
\sum_iT_i^{\mathrm{final}}=4\ \mathrm s.
$$

它仍调用 `FeasibilityChecker::check(..., true)`，因此 CSV 中仍然包含相同采样
间隔下的开环电机转速和舵偏角预测。区别只是程序不搜索、更不会应用新的缩放
系数。控制台的 `selected trajectory feasible` 明确报告固定时间轨迹是否满足
执行器限制。

三种配置的完整结果为：

| yaw 工况 | 配置 | 总时间 `[s]` | 峰值速度 `[m/s]` | 峰值 Snap `[m/s^4]` | 峰值 yaw 加速度 `[rad/s^2]` | 峰值电机转速 `[rad/s]` | 峰值舵偏角 `[rad]` |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `0 -> 0` | 优化比例 + 不缩放 | 4.000000 | 3.281250 | 19.687500 | 0 | 1468.781122 | 0.288180 |
| `0 -> 0` | 固定距离比例 + 不缩放 | 4.000000 | 4.502949 | 126.722036 | 0 | 1555.962199 | 0.331338 |
| `0 -> 0` | 优化比例 + 外层缩放 | 1.767299 | 7.426587 | 516.642708 | 0 | 1920.378336 | 0.844562 |
| `0 -> pi` | 优化比例 + 不缩放 | 4.000000 | 3.281740 | 19.738759 | 2.352171 | 1472.666784 | 0.300373 |
| `0 -> pi` | 固定距离比例 + 不缩放 | 4.000000 | 4.502949 | 126.722036 | 5.494212 | 1578.613681 | 0.381792 |
| `0 -> pi` | 优化比例 + 外层缩放 | 1.774972 | 7.395581 | 509.088982 | 11.945541 | 1961.767336 | 0.622273 |

前两组都固定为 `4 s`，因此它们之间的差异只来自是否优化分段时间比例，可以
公平衡量内层优化的作用。第三组沿用优化后的比例并执行外层缩放，用来回答
“在执行器限制下最多能缩短到什么程度”。时间导数随整体缩放具有不同幂次，
例如速度按 $c^{-1}$、Snap 按 $c^{-4}$ 变化，所以第三组缩短到约 `1.77 s`
后，速度、Snap 和执行器峰值都会明显增大。

生成零偏航无缩放数据：

```bash
# 优化比例、不缩放
./planning/build/generate_simple_trajectory \
  --no-time-scaling \
  --output planning/output/simple_yaw0_no_time_scaling.csv

# 不优化比例、不缩放
./planning/build/generate_simple_trajectory \
  --no-optimize-time-ratio \
  --no-time-scaling \
  --output planning/output/simple_yaw0_no_ratio_no_scaling.csv
```

### 10.3 CSV 内容

CSV 包含：

```text
t
px, py, pz
vx, vy, vz
ax, ay, az
jx, jy, jz
snap_x, snap_y, snap_z
yaw, yaw_rate, yaw_acceleration
omega1_pred, omega2_pred
delta1_pred, delta2_pred
```

`omega*_pred` 和 `delta*_pred` 是开环可行性预测，不能直接作为闭环仿真的执行器指令。

---

## 11. 回归测试逐项说明

`test_stage_a.cpp` 覆盖以下内容。

### 11.1 解析时间梯度有限差分

测试为 yaw 设置非零中间值，在 `tau=0.2` 处比较：

- SplineTrajectory 时间梯度经过 softmax 链式法则后的解析值；
- 中心有限差分数值值。

相对容差约为 `2e-5`。

### 11.2 固定总时间和能量下降

检查优化后所有分段时间之和仍为 `4 s`，并检查最终平滑能量没有高于初始距离比例能量。

### 11.3 样条端点条件

检查首末位置等于航点，并检查位置速度、加速度和 Jerk 在首末端为零。

### 11.4 hover 正则化

使用完全静止的 hover 平坦输出调用开环映射，验证：

- 不产生 NaN/Inf；
- 两个单电机推力非负。

### 11.5 力和力矩重构

测试使用预测执行器重新构造：

- hover 时的 roll/pitch 力矩；
- 轨迹中点的世界系合力。

这用于发现坐标系、旋转方向和执行器分配符号错误。

### 11.6 时间缩放和姿态连续性

测试要求：

- 外层搜索找到可行轨迹；
- 最终报告包含完整预测；
- 所有开环映射有效；
- 相邻姿态四元数连续；
- 将可行边界缩短 2% 后轨迹变为不可行。

构建和运行测试：

```bash
git submodule update --init --recursive
cmake -S planning -B planning/build
cmake --build planning/build -j
(cd planning/build && ctest --output-on-failure)
```

---

## 12. 当前限制与后续开发方向

### 12.1 中间导数约束尚未实现

当前位置七次样条只直接设置整条轨迹首末端的速度、加速度和 Jerk。loop、knife-edge、Immelmann 和 differential-thrust turn 等机动所需的中间速度方向或大小约束尚未实现。

下一步需要扩展样条等式约束系统，或实现带导数掩码的稀疏多项式求解器。高权重罚函数只能近似满足中间导数，不能视为严格等式复现。

### 12.2 阶段 B 仍不是显式约束优化

阶段 A 的 L-BFGS 只优化固定总时间内的时间比例。阶段 B 已经能够联合优化
分段时间和一个辅助位置点，并加入执行器采样积分罚函数，但仍未实现：

- 可移动的关键机动点或中间 yaw；
- 几何走廊和中间导数等式约束；
- 执行器显式不等式约束；

阶段 B 现已将开环平坦映射的固定分支自动微分雅可比，继续传播到样条系数、
航点和分段时间，并使用 L-BFGS 求解软罚函数目标。阶段 C 若要把采样罚函数升级
为显式不等式硬约束，仍需要改用 SQP、IPOPT 或增广拉格朗日等约束优化方法，
并处理姿态分支切换点与无效映射区域的非光滑性。

### 12.3 可行性仍是采样意义上的

目前不是严格连续时间硬约束。虽然包含分段连接点并使用毫秒级采样，但采样点之间仍可能隐藏很窄的峰值。后续应加入：

- 峰值附近自适应加密；
- 多项式区间界或根隔离；
- 最终 `0.5～1 ms` 高密度复核。

### 12.4 尚未检查执行器动态

当前只检查执行器幅值，没有检查：

- 电机一阶响应；
- 舵机一阶响应；
- 舵面角速度；
- 电机加速度或转速变化率；
- 闭环跟踪误差。

因此这里的“可行”表示理想确定性模型存在允许的瞬时开环输入，不保证带执行器动态和模型误差的真实系统一定能跟踪。

### 12.5 参数加载需要统一

YAML 当前不参与运行。建议后续选择一种唯一参数源：

1. C++ 读取 YAML；或
2. 从公共配置生成 C++/Python 参数；或
3. 通过 pybind11 共享同一参数对象。

否则 Python 仿真、YAML 和 C++ 默认参数可能逐渐不一致。

### 12.6 尚未接入 Python 闭环仿真

当前程序已经导出所需的高频 CSV，但还没有实现 `trajectory/planned_trajectory.py` 和 `main_sim.py --trajectory planned`。接入时只应把：

```text
position, velocity, acceleration, jerk, yaw, yaw_rate
```

提供给闭环控制器。预测的电机和舵面数据只用于对比，不能绕过控制器直接施加。

---

## 13. 阅读代码的推荐顺序

第一次阅读建议按以下顺序：

1. `planning_types.hpp`：先了解数据如何流动；
2. `flat_output_trajectory.cpp`：理解两套样条如何组合；
3. `time_allocation.cpp`：理解固定总时间优化；
4. `tailsitter_flatness.cpp`：逐段对应论文式 (20)～(56)；
5. `feasibility_checker.cpp`：理解“映射有效”和“执行器可行”的区别；
6. `time_scaler.cpp`：理解外层最短可行时间搜索；
7. `generate_simple_trajectory.cpp`：观察完整调用顺序；
8. `test_stage_a.cpp`：查看当前实现已验证和未验证的边界。

阶段 A 的核心原则可以概括为：

$$
\boxed{
\text{先在平坦输出空间获得平滑轨迹}
\;\longrightarrow\;
\text{再用独立开环模型判断执行器是否可行}
}
$$

它刻意保持样条优化和执行器可行性评估分层，为后续加入中间导数约束、闭环仿真验证及第 13 章的联合优化保留清晰接口。

---

## 14. 规划指标可视化

`scripts/plot_planning_metrics.py` 直接读取 `generate_simple_trajectory` 导出的 CSV，不重新计算样条或开环平坦映射，因此图中数据与可行性检查保存的数据一致。

基本用法：

```bash
python3 planning/scripts/plot_planning_metrics.py \
  planning/output/simple_yaw0.csv
```

默认在 CSV 同级的 `figures/` 目录生成三张图：

```text
simple_yaw0_trajectory_speed.png
simple_yaw0_smoothness.png
simple_yaw0_actuators.png
```

### 14.1 速度着色三维轨迹

脚本从位置字段构造相邻点线段，并用线段两端平均速度：

$$
v_{k+1/2}=\frac{\|\mathbf v_k\|+\|\mathbf v_{k+1}\|}{2}
$$

为每条三维线段着色。颜色条给出速度单位 `m/s`，黑色圆点和叉号分别表示轨迹起点和终点。

默认把 NED 的 `z` 取负后显示为正高度 `-z`，使用 `--raw-ned-z` 可以直接绘制原始 NED z。当前简单轨迹本身是水平直线，因此三维几何较简单；后续 loop 或 climbing turn 会更接近论文中的空间轨迹效果。

### 14.2 Snap 和 yaw 加速度

平滑性图包含两个子图：

- 上图绘制 `snap_x`、`snap_y`、`snap_z` 和 Snap 范数；
- 下图绘制 `yaw_acceleration`。

Snap 范数为：

$$
\|\mathbf s\|=\sqrt{s_x^2+s_y^2+s_z^2}
$$

这张图用于观察 minimum-snap 优化后的高阶导数分布，以及 yaw 加速度代价与机动阶段的关系。

### 14.3 电机转速和舵面偏角

执行器图绘制未经限幅的开环预测：

```text
omega1_pred, omega2_pred
delta1_pred, delta2_pred
```

默认限制线为：

```text
omega_max = 2500 rad/s
delta_max = +/-1 rad
```

可以针对其他车辆参数覆盖：

```bash
python3 planning/scripts/plot_planning_metrics.py TRAJECTORY.csv \
  --omega-max 2300 \
  --delta-max 0.85
```

限制线只是绘图参数，不会改变 CSV 数据或重新执行可行性判断。

### 14.4 其他参数

```text
--output-dir DIR   指定输出目录
--prefix NAME      指定三个文件的共同前缀
--title TEXT       指定图标题
--compare-csv CSV  读入第二份 CSV 并生成对比图
--compare-csv-2 CSV
                   可选地读入第三份 CSV
--primary-label    主 CSV 的方法名称
--compare-label    对比 CSV 的方法名称
--compare-label-2  第三份 CSV 的方法名称
--cmap NAME        指定速度颜色映射
--format FORMAT    png、pdf 或 svg
--dpi N            位图分辨率
--show             保存后显示绘图窗口
```

脚本还会在终端打印轨迹时长、最大速度、最大 Snap 范数、最大 yaw 加速度绝对值、峰值电机转速和峰值舵偏角，便于快速核对图中的量级。

### 14.5 时间比例优化与固定比例的可视化对比

向同一脚本传入 `--compare-csv` 即进入两组对比模式；再给出
`--compare-csv-2` 可以加入第三组：

```bash
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

三类输出采用一致的比较规则：

- 三维轨迹并排显示，使用所有数据的全局最小/最大速度建立共享色标，因而
  相同颜色具有相同的 `m/s` 含义；每个标题同时标出该组实际总时间；
- Snap 和 yaw 加速度画在同一坐标系，实线表示优化比例且不缩放，虚线表示
  固定距离比例且不缩放，点划线表示优化比例并缩放；颜色只区分分量；
- 两个电机转速和两个舵偏角使用同样的线型约定，并保留执行器限制线。

对比图的默认文件名会包含前两份 CSV 的 stem；三组对比建议像示例一样使用
`--prefix` 指定清晰的固定名称。终端会按所有配置分别打印六个峰值/时长指标，
方便将图片与数值回归结果相互核对。

---

## 阶段 B：执行器采样积分罚函数与辅助航点联合优化

阶段 B 对应根目录 `study.md` 第 13.6 节的第二步。新实现位于：

```text
include/planning/actuator_penalty_optimizer.hpp
src/actuator_penalty_optimizer.cpp
apps/generate_stage_b_trajectory.cpp
tests/test_stage_b.cpp
```

它复用阶段 A 的 `FlatOutputTrajectory` 和 `TailsitterFlatness`，不复制或修改
开环平坦映射。为避免符号混淆，本节用 $\Delta t_i$ 表示第 $i$ 段轨迹时间，
继续用 $T_1,T_2$ 表示两台电机的原始单电机推力。

### B.1 与阶段 A 的边界

阶段 B 的处理链为：

```text
固定总时间、固定关键位置/yaw 航点
                │
                ▼
阶段 A 平滑能量时间分配作为初值
                │
                ▼
softmax 分段时间 + 有界辅助点变量
                │
                ▼
每个候选变量重新构造位置/yaw 样条
                │
                ▼
分段采样 paper_planning 开环平坦映射
                │
                ▼
T1、T2、delta1、delta2 违反量梯形积分
                │
                ▼
固定分支解析梯度 + 有界 L-BFGS 联合优化
                │
                ▼
2 ms 稠密物理限制复核和 CSV 导出
```

该程序**没有创建或调用 `TimeScaler`**。总时间从优化开始到结束严格固定为

$$
\boxed{\bar T=1.8\ \mathrm s}
$$

（可用 `--total-time` 修改）。所以阶段 B 输出时间的变化只发生在各段之间，
不存在优化结束后的统一乘法缩放。

### B.2 固定关键点与一个自由辅助点

简单验证保留原来的三个关键点：

$$
\mathbf p_0=(0,0,-10),\qquad
\mathbf p_1=(1.5,0,-10),\qquad
\mathbf p_2=(6,0,-10).
$$

在后两个关键点之间加入名义辅助点：

$$
\mathbf q^0=(3.75,0,-10).
$$

位置航点顺序为：

$$
[\mathbf p_0,\mathbf p_1,\mathbf q,\mathbf p_2].
$$

其中 $\mathbf p_0,\mathbf p_1,\mathbf p_2$ 始终不进入几何优化变量，只有
$\mathbf q$ 可动。程序使用归一化变量 $\mathbf u\in[-1,1]^3$：

$$
\boxed{
\mathbf q=\mathbf q^0+
\operatorname{diag}(0.5,0.5,0.5)\mathbf u
}.
$$

这使辅助点只能在名义位置周围的 $1\ \mathrm m$ 立方盒内移动，不能通过任意
扭曲或缩小轨迹逃避执行器罚函数。yaw 航点同样固定：零偏航工况全部为零；
`--yaw-pi` 工况为

$$
[0,\pi/2,3\pi/4,\pi].
$$

辅助点的 yaw 只是固定插值值，本阶段没有优化 yaw 航点。

### B.3 固定总时间的分段时间变量

四个航点产生三个分段。最后一个 logit 固定为零，只优化两个自由变量
$\tau_1,\tau_2$：

$$
\begin{aligned}
\Delta t_1&=\bar T\frac{e^{\tau_1}}
{e^{\tau_1}+e^{\tau_2}+1},\\
\Delta t_2&=\bar T\frac{e^{\tau_2}}
{e^{\tau_1}+e^{\tau_2}+1},\\
\Delta t_3&=\bar T\frac{1}
{e^{\tau_1}+e^{\tau_2}+1}.
\end{aligned}
$$

因此任意优化迭代都严格满足：

$$
\Delta t_i>0,
\qquad
\sum_{i=1}^3\Delta t_i=\bar T.
$$

默认限制 $\tau_i\in[-5,5]$，防止某段时间在数值上退化到接近零。

### B.4 四个执行器输入的罚函数

规划器直接使用 `TailsitterFlatness::forward()` 的原始输出：

$$
T_1(t),\quad T_2(t),\quad\delta_1(t),\quad\delta_2(t).
$$

不对这些量做 `clip`。令规划裕量为 $m\in(0,1]$，优化阶段使用：

$$
T_{\min}=c_T\omega_{\min}^2,
\qquad
T_{\max}^{\mathrm{plan}}=c_T(m\omega_{\max})^2,
$$

$$
\delta_{\max}^{\mathrm{plan}}=m\delta_{\max}.
$$

推力范围 $\Delta T=T_{\max}^{\mathrm{plan}}-T_{\min}$。代码中的单电机
归一化平方铰链罚函数为：

$$
p_{T_i}(t)=
\left[
\frac{T_{\min}-T_i(t)}{\Delta T}
\right]_+^2
+
\left[
\frac{T_i(t)-T_{\max}^{\mathrm{plan}}}{\Delta T}
\right]_+^2,
$$

其中 $[x]_+=\max(0,x)$。单舵面罚函数为：

$$
p_{\delta_i}(t)=
\left[
\frac{|\delta_i(t)|-\delta_{\max}^{\mathrm{plan}}}
{\delta_{\max}^{\mathrm{plan}}}
\right]_+^2.
$$

平方铰链在约束边界处一阶连续。当前实现显式计算其一阶导数，并通过执行器
雅可比、样条系数和时间映射传播给优化变量。如果平坦映射无效或输出非有限值，
额外加入 `invalid_mapping_penalty`，使优化器倾向离开负推力、奇异分配或姿态
分支失败区域。

默认 $m=0.75$，对应优化阶段约：

```text
omega plan max = 1875 rad/s
delta plan max = 0.75 rad
```

这个较保守的默认值是为了让简单 `1.8 s` 验证轨迹产生可观察的阶段 A/B
差异。它不是车辆物理上限；最终验证仍使用 `2500 rad/s` 和 `1 rad`。

### B.5 分段梯形积分

每一段单独均匀划分为 $N$ 个区间，默认 $N=50$。对任一执行器罚函数
$p(t)$，第 $i$ 段的积分近似为：

$$
I_i[p]
=\frac{\Delta t_i}{N}
\left[
\frac12p(t_{i,0})
+\sum_{k=1}^{N-1}p(t_{i,k})
+\frac12p(t_{i,N})
\right].
$$

总执行器罚函数为：

$$
\boxed{
J_{\mathrm{act}}
=I[p_{T_1}]+I[p_{T_2}]
+I[p_{\delta_1}]+I[p_{\delta_2}]
+I[p_{\mathrm{invalid}}]
}.
$$

逐段积分保证所有样条连接点都被显式采样。积分阶段使用的 `50` 个区间用于
优化速度与约束分辨率之间的折中；优化完成后另外使用 `2 ms` 间隔检查，不把
积分网格本身当成连续时间证明。

### B.6 完整目标函数与默认权重

阶段 B 的目标为：

$$
\boxed{
J_B
=\rho_s\frac{J_{\mathrm{smooth}}}{J_{\mathrm{smooth}}^0}
+\rho_g\|\mathbf u\|^2
+\rho_aJ_{\mathrm{act}}
}.
$$

其中：

$$
J_{\mathrm{smooth}}
=J_{\mathrm{snap}}+\lambda_\psi J_{\mathrm{yawacc}},
\qquad \lambda_\psi=1.
$$

$J_{\mathrm{smooth}}^0$ 是阶段 A 初始轨迹的平滑能量，用它归一化可以避免
位置 Snap 能量的数值量级随总时间剧烈变化。默认配置为：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| $\rho_s$ | 1 | 归一化平滑能量权重 |
| $\rho_g$ | 0.05 | 辅助点偏移权重 |
| $\rho_a$ | 200 | 执行器积分罚权 |
| $\lambda_\psi$ | 1 | yaw 加速度能量权重 |
| $m$ | 0.75 | 优化用执行器幅值裕量 |
| $N$ | 50/段 | 罚函数积分区间数 |

因此阶段 B 并不保证 Snap 单独下降。优化器允许小幅增加平滑能量，以交换更低
的执行器罚函数；这正是和阶段 A “只最小化平滑能量”之间的本质差异。

### B.7 为什么现在使用固定分支梯度和 `LD_LBFGS`

优化变量只有五个：

```text
2 个自由时间 logits
3 个辅助点归一化坐标
```

尾座式开环映射包含姿态离散分支选择、负推力提前返回和舵面矩阵奇异性判断，
因此整个映射并非处处光滑。不过，姿态分支一旦确定，正常有效区域内从

$$
\boldsymbol\xi=(\mathbf v,\mathbf a,\mathbf j,\mathbf s,
\psi,\dot\psi,\ddot\psi)\in\mathbb R^{15}
$$

到 $(T_1,T_2,\delta_1,\delta_2)$ 的连续公式是可微的。实现先用普通
`forward()` 选择与前一样本连续的 roll/pitch 分支，再在
`linearize()` 中冻结这两个分支，用 Eigen AutoDiff 一次得到 $4\times15$
雅可比。离散 `argmin` 本身不参与求导。

执行器罚函数梯度随后传播到位置/yaw 多项式系数和显式分段时间偏导，再通过
SplineTrajectory 的 `propagateGrad()` 得到内部航点与分段时间梯度。最后继续
通过固定总时间 softmax 和辅助点缩放映射传播到五个优化变量。NLopt 因而改用
有界梯度算法 `LD_LBFGS`。

该梯度是**分段解析梯度**：在当前姿态分支与有效映射邻域内成立，不对分支切换
边界求导。无效映射罚值仍作为常数屏障处理，不能把它解释为穿越奇异区域的可靠
搜索方向。

### B.8 与阶段 A 的公平对比

`generate_stage_b_trajectory` 每次同时生成两份轨迹：

- 阶段 A 基线：同样的关键点、名义辅助点和固定总时间，只优化平滑能量的
  分段比例；
- 阶段 B：从阶段 A 时间比例开始，加入执行器罚函数并允许辅助点在有界盒内
  移动。

两者都不进行外层时间缩放。默认 paper 参数的当前结果为：

| yaw | 方法 | 分段时间 `[s]` | 辅助点 `[m]` | 峰值 Snap `[m/s^4]` | 峰值 yaw 加速度 `[rad/s^2]` | 峰值电机 `[rad/s]` | 峰值舵角 `[rad]` |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| `0 -> 0` | 阶段 A | `[0.681927, 0.322317, 0.795756]` | `[3.750000, 0, -10.000000]` | 480.109739 | 0 | 1896.658033 | 0.789456 |
| `0 -> 0` | 阶段 B | `[0.678399, 0.325646, 0.795955]` | `[3.748868, 0, -9.992095]` | 519.406884 | 0 | 1888.207614 | 0.741916 |
| `0 -> pi` | 阶段 A | `[0.681944, 0.322316, 0.795739]` | `[3.750000, 0, -10.000000]` | 480.160296 | 11.717283 | 1943.519382 | 0.608002 |
| `0 -> pi` | 阶段 B | `[0.689912, 0.325404, 0.784684]` | `[3.727321, 0.010638, -9.974084]` | 572.975170 | 11.447086 | 1903.552637 | 0.603513 |

零偏航的执行器积分从约 $2.30\times10^{-4}$ 降至
$2.8\times10^{-5}$；`0 -> pi` 工况从约 $7.27\times10^{-4}$ 降至
$8.3\times10^{-5}$。同时组合目标分别从 `1.045824` 降到 `1.011050`、从
`1.145326` 降到 `1.045637`。表中 Snap 上升是目标权衡的正常结果，不是
minimum-snap 求解错误。

运行和绘图：

```bash
./planning/build/generate_stage_b_trajectory \
  --yaw-pi \
  --output planning/output/stage_b_yaw_pi_optimized.csv \
  --baseline-output planning/output/stage_b_yaw_pi_baseline.csv

python3 planning/scripts/plot_planning_metrics.py \
  planning/output/stage_b_yaw_pi_optimized.csv \
  --compare-csv planning/output/stage_b_yaw_pi_baseline.csv \
  --primary-label "Stage B: actuator penalty" \
  --compare-label "Stage A: smoothness only" \
  --prefix stage_b_vs_stage_a_yaw_pi \
  --title "Fixed total time 1.8 s, yaw 0 to pi" \
  --omega-max 1875 \
  --delta-max 0.75
```

这里的两条绘图限制线对应阶段 B 的软罚阈值。平方积分罚函数不是硬约束，
所以最终峰值仍可能略高于线；物理极限的稠密检查结果由生成程序另外报告。

### B.9 当前限制

- 这是平方铰链软罚函数，不是阶段 C 的显式不等式硬约束；
- 罚函数只在有限采样点积分，采样间的窄峰值仍可能漏检；
- 当前只优化一个辅助位置点，不优化辅助 yaw 或中间导数；
- 固定分支梯度不包含离散姿态分支切换导数；无效映射区的常数屏障也不提供可靠
  的恢复方向，因此初值应处于有效且分支连续的邻域；
- 默认 `0.75` 裕量是验证配置，需要针对真实平台噪声、模型误差和任务速度
  重新标定；
- 最终稠密检查只认证当前开环模型，不替代 `paper_control` 闭环仿真。

### B.10 头文件中的公共类型逐项说明

`actuator_penalty_optimizer.hpp` 把优化器对外接口分成五个数据结构和一个类，
避免命令行程序直接依赖 NLopt 回调内部状态。

#### B.10.1 `ActuatorPenaltyConfig`

该结构保存构造目标函数和配置求解器所需的全部参数：

| 字段 | 默认值 | 在代码中的用途 |
| --- | ---: | --- |
| `yaw_weight` | `1.0` | 传给 `FlatOutputTrajectory::energy()`，即 $\lambda_\psi$ |
| `smoothness_weight` | `1.0` | 乘归一化后的 Snap/yaw 平滑能量 |
| `actuator_weight` | `200.0` | 乘四执行器积分与无效映射积分 |
| `geometry_weight` | `0.05` | 乘辅助点归一化偏移平方和 |
| `actuator_margin` | `0.75` | 把物理幅值转换成优化用软阈值 |
| `samples_per_segment` | `50` | 每段梯形积分的区间数，不是点数 |
| `invalid_mapping_penalty` | `10.0` | 单位时间无效平坦映射的基础罚值 |
| `auxiliary_half_range` | `[0.5,0.5,0.5]` m | 将 $[-1,1]^3$ 映射到辅助点物理偏移 |
| `tau_bound` | `5.0` | 时间 logits 的对称上下界 |
| `max_evaluations` | `400` | NLopt 最大目标函数调用次数 |
| `relative_tolerance` | `1e-7` | `ftol_rel` 和 `xtol_rel` 共用的相对容差 |

`samples_per_segment=50` 表示每段实际求值 `51` 次，因为两个端点也要参加
梯形积分。三个分段名义上会调用 `forward()` 共 `153` 次；相邻分段共享的
连接点会各作为前一段右端点和后一段左端点求值一次。两次分别乘左右分段步长
的一半；当分段步长不相等时两个权重也不相等，这正是逐段应用梯形公式应有的
结果，而不是重复积分。

`auxiliary_half_range` 是逐轴尺度，不是球形半径。如果以后设置为
`[1.0,0.2,0.5]`，含义是允许辅助点沿 x/y/z 分别移动 `±1.0/±0.2/±0.5 m`。

#### B.10.2 `ActuatorPenaltyIntegral`

该结构分别保存：

```cpp
thrust_1, thrust_2, flap_1, flap_2, invalid_mapping
```

而不是只保存一个总数。`total()` 才将五项相加。这样控制台能够指出优化器
主要在处理哪一个执行器，而不会让左右电机或左右舵面的违反量相互掩盖。

这些量都是“归一化违反量平方对时间的积分”，因此仍带有时间尺度。它们不是
推力能量、转速能量或舵机功耗，也不能直接解释为焦耳。

#### B.10.3 `StageBObjectiveTerms`

该结构保存一次候选变量评估的完整分解：

| 字段 | 含义 |
| --- | --- |
| `smoothness` | 未归一化的 $J_{\mathrm{snap}}+\lambda_\psi J_{\mathrm{yawacc}}$ |
| `normalized_smoothness` | `smoothness / smoothness_reference` |
| `geometry` | 所有辅助点归一化变量的平方和 |
| `actuator` | 五个执行器/无效映射积分分量 |
| `total` | 乘完三个权重后的最终 NLopt 标量目标 |

保留原始和归一化平滑能量非常重要：前者便于和阶段 A 的能量输出核对，后者
才是阶段 B 目标中实际参与加权的量。

#### B.10.4 `StageBOptimizationResult`

结果结构同时返回：

- `optimized`：NLopt 是否正常返回；
- `evaluations`：优化回调的实际调用次数；
- `durations`：解码后的物理分段时间；
- `position_waypoints`：只修改了指定辅助行的最终位置矩阵；
- `initial_terms`、`final_terms`：组合目标优化前后的完整分解。

`optimized=true` **不等价于** `FeasibilityReport::feasible=true`。前者只说明
数值求解器结束，后者才说明随后使用物理限制进行的稠密开环检查通过。

#### B.10.5 `StageBGradientCheckResult`

该结构保存 `checkInitialGradient()` 的验证结果：有效标志、最大缩放分量误差，
以及逐变量的解析梯度和中心有限差分梯度。它只用于测试与诊断，不参与 NLopt
迭代，也不会用有限差分结果替换优化器梯度。

#### B.10.6 `ActuatorPenaltyOptimizer`

类本身只持有两项：

```cpp
TailsitterFlatness flatness_;
ActuatorPenaltyConfig config_;
```

平坦映射按值持有，使优化器在多次目标评估期间使用完全一致的车辆参数；配置
也按值持有，避免命令行层修改临时结构后影响正在运行的优化。

公共方法 `optimize()` 接收名义航点、固定 yaw、初始分段时间和辅助点行号。
`checkInitialGradient()` 使用相同输入在阶段 A 初值处进行完整目标中心差分检查。
关键点无需另建布尔掩码：没有出现在 `auxiliary_waypoint_indices` 中的行天然固定。

### B.11 优化变量向量的精确内存布局

阶段 B 没有直接把 Eigen 航点矩阵交给 NLopt，而是构造一个低维
`std::vector<double>`。设位置航点数为 $M$、辅助点数为 $K$，则分段数为
$M-1$，自由时间 logits 数为：

$$
n_\tau=(M-1)-1=M-2.
$$

函数 `tauCount()` 因此直接返回 `positions.rows() - 2`。变量总维数为：

$$
n_x=n_\tau+3K.
$$

当前简单例中 $M=4,K=1$，所以：

$$
\boxed{
\mathbf x=[\tau_1,\tau_2,u_x,u_y,u_z]^T
}.
$$

一般情况下，第 $a$ 个辅助点、第 `axis` 个轴对应的变量下标为：

```cpp
tau_count + 3 * auxiliary + axis
```

这种布局使变量边界也很直接：前 `tau_count` 项使用
`[-tau_bound,+tau_bound]`，后面所有辅助点归一化变量统一使用 `[-1,+1]`。

### B.12 `positionsFromVariables()`：只解码辅助点

该函数先完整复制名义位置矩阵：

```cpp
auto positions = *context.nominal_positions;
```

然后只遍历 `auxiliary_indices` 指定的行。对第 $a$ 个辅助点：

$$
p_{r,k}=p^0_{r,k}+h_k u_{a,k},
$$

其中 $r$ 是该辅助点在位置矩阵中的行号，$h_k$ 是
`auxiliary_half_range[k]`。由于其他行从未被写入，关键点固定不是靠一个大罚权
近似实现，而是由变量参数化严格保证。

函数返回一个新的位置矩阵，不修改调用者传入的名义矩阵。这一点允许同一个
`ObjectiveContext` 安全评估大量不同候选变量。

### B.13 `durationsFromVariables()`：复用阶段 A 的 softmax

该函数截取变量向量开头的 $n_\tau$ 项，再调用：

```cpp
FixedTotalTimeAllocator::durationsFromTau(total_time, tau_free)
```

因此阶段 B 没有维护第二套时间参数化。数值稳定的“减最大 logit 再指数化”、
最后一个固定零 logit、严格保持总时间等性质与阶段 A 完全相同。

初始变量不是重新猜测，而是由阶段 A 分段时间反算：

$$
\tau_i^0=\log\frac{\Delta t_i^0}{\Delta t_N^0}.
$$

将该式再送入 softmax 会恢复原阶段 A 时间比例，所以
`initial_terms` 确实对应应用层导出的阶段 A baseline，而不是另一个近似初值。

### B.14 `squaredPositivePartWithDerivative()`：约束边界行为

辅助函数同时返回罚值和对输入 $g$ 的导数：

```cpp
if (!(value > 0.0))
    return {0.0, 0.0};
return {value * value, 2.0 * value};
```

其分段形式为：

$$
P(g)=
\begin{cases}
0,&g\le0,\\
g^2,&g>0.
\end{cases}
$$

所以满足软阈值时罚值和梯度都为零，越界后按违反量平方增长。在 $g=0$ 处函数值
和一阶导数连续，但二阶导数不连续。该一阶连续性使 L-BFGS 在刚接触软阈值时
不会遇到普通 $[g]_+$ 的梯度跳变，并让大越界比小越界受到更强惩罚。

### B.15 `integrateActuatorPenalty()` 逐步说明

这是阶段 B 最核心的新增函数，可以按以下顺序阅读。

#### B.15.1 从转速阈值转换为推力阈值

函数从 `flatness.parameters()` 读取和开环映射相同的 $c_T$、转速范围与舵角
范围，然后计算：

```cpp
thrust_min        = c_T * omega_min^2
planning_omega_max = margin * omega_max
thrust_max        = c_T * planning_omega_max^2
flap_max          = margin * delta_max
```

注意 margin 必须在转速平方之前使用：

$$
T_{\max}^{\mathrm{plan}}
=c_T(m\omega_{\max})^2
=m^2c_T\omega_{\max}^2.
$$

如果错误地把物理最大推力直接乘 $m$，得到的等效转速裕量会是 $\sqrt m$，与
命令行参数的含义不一致。

`thrust_scale=max(1e-9, thrust_max-thrust_min)` 防止极端错误参数导致归一化分母
为零。正常 paper 参数下该保护下界不会参与计算。

#### B.15.2 分段积分循环

外层循环遍历 `trajectory.durations()`，并维护当前段起始绝对时间
`segment_start`。内层循环从 `0` 到 `samples_per_segment`，所以同时包含左右
端点。步长和梯形权重分别为：

```cpp
step = duration / samples_per_segment
weight = step * (endpoint ? 0.5 : 1.0)
```

只求目标值时，每个样本调用：

```cpp
sample = trajectory.sample(time)
prediction = flatness.forward(sample, branch)
```

NLopt 请求梯度时则调用：

```cpp
linearization = flatness.linearize(sample, branch)
prediction = linearization.prediction
```

因此执行器罚函数始终基于重新构造的候选样条及其解析导数，而不是对阶段 A
CSV 做时间或空间插值。

#### B.15.3 姿态分支状态为何必须跨样本保留

`TailsitterFlatness::forward()` 可能存在多个等价姿态分支。`BranchState branch`
在整个积分函数开始时创建一次，而不是每个样本或每段重置。只要候选四元数
有限且范数合理，就把它写回 branch：

```cpp
branch.initialized = true;
branch.attitude = prediction.attitude;
```

这样下一个采样点会优先选择与前一点最近的姿态分支，减少目标函数因任意
$180^\circ$ 分支跳转产生的不连续。即使后续因单电机负推力等原因使
`prediction.valid=false`，只要姿态本身已经成功求出，仍可延续该姿态分支。

#### B.15.4 推力罚函数 lambda

局部 `thrust_penalty` 同时检查下界和上界。若推力不是有限数，直接返回
`invalid_mapping_penalty`；否则计算 B.4 节的两个平方铰链项。之后分别乘当前
梯形权重并累加到 `integral.thrust_1/2`。

直接约束推力而不是转速有两个好处：

1. 负单电机推力可以直接产生下界违反量；
2. 避免 $\omega=\sqrt{T/c_T}$ 在 $T\to0^+$ 附近的病态导数。

越界时同时得到 $\partial p_{T_i}/\partial T_i$，与执行器雅可比相乘后进入
15 维平坦输出导数梯度。

#### B.15.5 舵面罚函数 lambda

舵面使用 `abs(flap)`，因此一个表达式同时覆盖正负偏角限制。非有限舵角同样
返回 `invalid_mapping_penalty`，有效舵角则计算超过 `flap_max` 的归一化平方
违反量。

越界时 `sign(flap)` 给出绝对值的局部导数；因为真正参与求导的区域满足
$|\delta_i|>\delta_{\max}^{\mathrm{plan}}>0$，不会在零点使用不确定的符号导数。

#### B.15.6 `invalid_mapping` 项

只要 `prediction.valid` 为 false，还会额外累加：

```cpp
weight * invalid_mapping_penalty
```

如果无效预测同时留下 NaN 舵角，那么该采样会在 `flap_1`、`flap_2` 和
`invalid_mapping` 三项中都受到惩罚。这是有意设置的高代价屏障：普通幅值越界
只产生连续平方罚值，平坦映射根本不可求则应比小幅越界更不受欢迎。五个分量
分开保留也能让日志明确指出代价来自公式失效，而不是正常幅值超限。

### B.16 `evaluate()`：一次候选轨迹的完整求值与求导

`evaluate()` 的执行顺序与目标函数数据流一一对应：

1. 从变量向量解码分段时间；
2. 从变量向量解码辅助位置点；
3. 调用 `FlatOutputTrajectory::update()` 重建两套共享时间的样条；
4. 计算位置 Snap 与 yaw 加速度组合能量；
5. 除以固定的参考平滑能量；
6. 只对变量向量中辅助点部分求平方和；
7. 调用 `integrateActuatorPenalty()`；
8. 按三个权重组装标量 `terms.total`；
9. 如果 NLopt 请求梯度，取得平滑能量梯度并传播执行器罚函数的系数/时间偏导；
10. 通过 softmax 时间映射和辅助点缩放映射组装优化变量梯度。

几何项循环从 `tau_count` 开始，所以时间 logits 不会误计入辅助点偏移代价：

```cpp
for (i = tau_count; i < variables.size(); ++i)
    geometry += variables[i] * variables[i];
```

函数用 `try/catch` 包住整个候选构造过程。非正分段时间、样条维度错误或其他
异常不会逃出 NLopt 回调，而是统一返回 $10^{30}$。最终若加权目标成为 NaN 或
Inf，也替换成 $10^{30}$。这相当于为数值上不可构造的候选建立一个有限但极大的
外部屏障。

这里捕获异常是为了保护优化迭代，不表示错误被静默忽略：正式调用之前的配置
和维度错误由 `validateInputs()` 明确抛出；只有优化过程中个别候选失败才被转换
为大目标值。

### B.17 `ObjectiveContext` 与 NLopt 回调

NLopt C++ 接口要求 C 风格的 `void*` 用户数据。`ObjectiveContext` 保存：

```text
固定总时间
参考平滑能量
名义位置/yaw 矩阵指针
辅助点行号指针
平坦映射和配置指针
评估计数器
```

这些指针都指向 `optimize()` 调用期间仍然存活的对象，NLopt 返回前不会悬空。
使用指针而不是每次复制 Eigen 矩阵，可以减少数百次目标评估中的固定数据开销。

回调 `objective()` 先递增 `evaluations`。若 NLopt 传入非空梯度向量，就把它
交给 `evaluate()` 填写；否则只求目标值。这样同一个求值函数同时服务于
`LD_LBFGS`、初末目标报告和中心有限差分验证，不存在两套目标公式漂移的问题。

### B.18 `validateInputs()` 的所有拒绝条件

进入 NLopt 前会检查：

- `total_time > 0`；
- 至少两个位置航点；
- 位置与 yaw 航点行数相同；
- 分段时间数量恰好等于航点数减一；
- 初始分段时间之和在相对 `1e-9` 容差内等于固定总时间；
- 四类权重非负；
- `actuator_margin` 位于 `(0,1]`；
- 每段积分区间数、无效映射罚值、tau 边界、最大评估数和容差均为正；
- 辅助点逐轴半范围非负；
- 每个辅助点行号都是内部航点，不能是首末端；
- 辅助点行号不能重复。

首末端禁止成为辅助点是额外的结构保护。中间关键点是否固定则由调用者决定：
应用程序只传 `{2}`，所以第 1 行关键点不可能被优化器移动。

### B.19 `ActuatorPenaltyOptimizer::optimize()` 逐步说明

#### B.19.1 构造参考轨迹

函数先用传入的阶段 A 分段时间和名义航点构造 `reference`，并计算：

```cpp
smoothness_reference = max(1e-12, reference.energy(yaw_weight))
```

`1e-12` 保护完全静止或特殊零能量轨迹，避免后续归一化除零。参考值在整个
阶段 B 优化期间固定，不会随候选轨迹更新；否则优化器可以通过同时改变分子和
分母破坏目标的比较意义。

#### B.19.2 构造初始变量和边界

变量向量先全部置零，因此辅助点从名义位置开始。时间变量用阶段 A 分段时间
相对最后一段的对数比覆盖。边界向量先全部设为 `[-1,1]`，再把前
`tau_count` 项改写成 `[-tau_bound,+tau_bound]`。

这种先统一初始化、再覆盖时间部分的写法使多个辅助点时仍保持同一布局，不需要
为每个变量单独调用 NLopt 边界接口。

#### B.19.3 保存初值目标

调用 NLopt 前先执行一次 `evaluate()` 写入 `result.initial_terms`。这次求值不
计入 `context.evaluations`，因为计数器定义为“NLopt 实际请求了多少次目标”。
控制台因此可以同时报告清晰的初值和纯求解器调用次数。

#### B.19.4 配置 `LD_LBFGS`

求解器设置包括：

```cpp
set_min_objective(...)
set_lower_bounds(...)
set_upper_bounds(...)
set_ftol_rel(relative_tolerance)
set_xtol_rel(relative_tolerance)
set_maxeval(max_evaluations)
```

`LD_LBFGS` 使用当前回调提供的完整一阶梯度。`ftol_rel` 检查目标相对变化，
`xtol_rel` 检查变量相对变化，`maxeval` 则提供确定的最坏评估预算。它仍是局部
优化器，当前结果依赖阶段 A 初值和所选姿态分支，不宣称找到全局最优轨迹。

NLopt 正常返回后设置 `optimized=true`。异常会被包装成带
`NLopt stage B optimization failed` 前缀的 `runtime_error`，保留原始原因供
应用层统一打印。

#### B.19.5 解码最终结果

求解结束后再次调用相同的两个解码函数得到物理分段时间和位置矩阵，并重新
调用 `evaluate()` 得到 `final_terms`。没有直接相信 NLopt 返回的 `minimum`
作为最终分解，因为 `minimum` 只有总标量，无法提供五个积分分量和几何项。

### B.20 `generate_stage_b_trajectory.cpp` 应用层说明

阶段 B 应用程序负责实验定义、参数解析、基线构造、最终认证和文件输出，不把
这些职责塞进通用优化器。

#### B.20.1 命令行解析辅助函数

`parseDouble()` 使用 `std::stod` 并额外拒绝 NaN/Inf；`parseInt()` 使用
`std::stoi`。缺失参数值由局部 `next()` lambda 发现。未知选项打印完整 usage
并返回状态码 2，数值或运行异常由最外层 catch 打印并返回状态码 1。

具体范围不在解析层重复判断，而由 `ActuatorPenaltyOptimizer::validateInputs()`
集中检查。例如 `--actuator-margin 1.2` 能被解析成 double，但会在优化前得到
明确的配置错误。

#### B.20.2 航点矩阵为何有四行

应用程序按以下顺序构造位置：

```text
row 0: 固定起点       [0.00, 0, -10]
row 1: 固定关键点     [1.50, 0, -10]
row 2: 自由辅助点     [3.75, 0, -10]
row 3: 固定终点       [6.00, 0, -10]
```

调用 `optimizer.optimize(...,{2})` 是“只有第 2 行可动”的唯一授权来源。控制台
同时打印第 1 行优化前后值，便于直接确认中间关键点没有被误移动。

#### B.20.3 阶段 A baseline 的构造

应用层先调用：

```cpp
FixedTotalTimeAllocator(yaw_weight).optimize(
    total_time, nominal_positions, yaws)
```

得到只考虑平滑能量的分段时间。随后：

- baseline 用这些时间和名义辅助点构造；
- 阶段 B 把这些时间作为初值，并允许第 2 行辅助点移动。

因此两份 CSV 具有相同总时间、关键点、车辆参数和最终检查分辨率。区别只在于
阶段 B 增加了执行器罚函数和辅助点自由度。

#### B.20.4 参数模型选择

默认使用 `VehicleParameters::paperVehicle()`；指定 `--simulation-vehicle` 后，
优化积分和最终检查会同时改用 `simulationVehicle()`。不会出现“用 paper 参数
优化、用 simulation 参数检查”的混合口径。

#### B.20.5 最终稠密检查

阶段 A/B 轨迹优化完成后分别调用：

```cpp
FeasibilityChecker(TailsitterFlatness(parameters), 0.002)
checker.check(trajectory, true)
```

这里不再使用 `actuator_margin=0.75`，而是 `FeasibilityChecker` 默认的物理幅值
margin `1.0`。`keep_predictions=true` 使报告保留所有开环输出，供 CSV 写出。

程序不会因软罚函数仍有残余值而拒绝输出，也不会自动延长时间。这符合用户要求
的“不重新进行时间缩放”。物理检查是否通过由
`baseline feasible at physical limits` 和 `stage B feasible at physical limits`
明确报告。

#### B.20.6 `writeCsv()`

CSV 字段与阶段 A 完全一致，因此原绘图脚本无需阶段 B 专用解析器。函数遍历
稠密报告中的时间戳，在相同时间重新采样轨迹导数，再和报告中已经计算好的执行
器预测拼成一行。

这种写法保证位置/Snap 与执行器数据共享同一个时间索引。输出目录不存在时用
`create_directories()` 创建，文件打开失败则抛出异常，不会静默生成空文件。

#### B.20.7 控制台输出如何阅读

程序依次打印：

1. 固定总时间、yaw 工况和软罚参数；
2. NLopt 评估次数；
3. 阶段 A/B 三段时间；
4. 固定关键点和辅助点优化前后坐标；
5. 初始/最终组合目标及五个积分分量；
6. 两条轨迹在真实物理限制下的峰值与可行性；
7. 两份 CSV 的绝对路径。

如果组合目标下降但某一个峰值略升，不构成程序矛盾：优化目标是违反量的时间
积分，不是最大值；此外还同时权衡平滑与几何项。最终峰值必须结合稠密报告判断。

### B.21 `test_stage_b.cpp` 逐项保证什么

回归测试使用更低的 `20` 个积分区间/段和 `160` 次最大评估，以缩短 CTest
时间，但保留与应用相同的 yaw `0 -> pi`、关键点、辅助点和车辆模型。测试检查：

1. 求解器确实运行且至少发生一次目标评估；
2. 最终三段时间之和仍为 `1.8 s`；
3. 第 0、1、3 行三个关键点完全未移动；
4. 最终组合目标没有高于初始目标；
5. 执行器采样积分严格低于初值；
6. 最终平滑能量和执行器积分有限；
7. 重建轨迹的首末位置仍等于固定端点；
8. 固定姿态分支下 $4\times15$ 执行器雅可比通过中心有限差分检查；
9. yaw `0 -> 0` 和 `0 -> pi` 两个初始点的完整五维目标梯度均通过中心有限
   差分检查；前者还会激活舵面软约束。

当前完整目标梯度回归阈值为 $5\times10^{-4}$；两个工况中的最大缩放分量误差
为 $3.35\times10^{-7}$。这个测试同时覆盖样条系数传播、显式时间偏导、固定
总时间 softmax 和辅助点缩放链式法则。

测试没有宣称证明连续时间硬约束，也没有强制每个执行器峰值都下降。那些性质
需要 `FeasibilityChecker` 稠密检查和实验结果表来评估。

### B.22 CMake 与绘图脚本的配套改动

`CMakeLists.txt` 将 `src/actuator_penalty_optimizer.cpp` 加入
`tailsitter_planning`，因此应用和测试共享同一实现；同时新增：

```text
generate_stage_b_trajectory
test_stage_b
CTest test name: stage_b
```

绘图脚本本身不理解阶段 A/B 算法，只比较相同 CSV schema。为适配当前无界面
环境，本次还做了两个兼容修改：

- 未指定 `--show` 时默认选择 Matplotlib `Agg` 后端，避免不可用的
  `DISPLAY=:0` 触发 Tk 错误；
- 仅当 3D axis 提供 `set_box_aspect()` 时才调用，使 Ubuntu/ROS 常见的旧版
  Matplotlib 也能生成图片；
- 允许四个执行器预测列在平坦映射无效时包含 NaN，并把这些区间画成断点；
  轨迹状态列仍严格要求有限，峰值只统计有限执行器样本，同时打印无效样本数。
  这样既能画出不可行工况，又不会用跨越 NaN 的连线伪造有效执行器输出。

这两个修改只影响绘图运行环境，不改变任何轨迹或优化数值。

### B.23 推荐的调参和诊断顺序

阶段 B 出现不理想结果时，建议按以下顺序处理：

1. 先保持 `actuator_weight=200`，提高 `samples_per_segment` 检查是否是采样过粗；
2. 查看五个积分分量，确认主要违反来自电机、舵面还是无效映射；
3. 若小幅软越界不能接受，先把 `actuator_weight` 增大到 `500～1000`；
4. 若辅助点移动过多，提高 `geometry_weight` 或缩小
   `auxiliary_half_range`；
5. 若目标停在不同局部解，提高 `maxeval`，并尝试多个辅助点名义初值；
6. 若 `invalid_mapping` 非零，优先检查负推力、姿态分支和舵面分配奇异性，
   不要只继续增大普通幅值罚权；
7. 最后始终以更密的 `FeasibilityChecker` 和闭环仿真复核。

把 `actuator_margin` 从 `0.75` 改为 `1.0` 表示只在真实物理幅值之外开始惩罚，
而不是取消执行器罚函数。反过来，减小 margin 是预留模型误差裕度，但也会加大
平滑性和几何偏移的牺牲。

---

## 14. `SepticSplineND` 的数学原理与逐函数说明

本节直接对应 `third_party/SplineTrajectory/include/SplineTrajectory.hpp` 中的
`SepticSplineND<DIM>`。它解决的是：给定 $N+1$ 个 $d=\texttt{DIM}$ 维航点、
$N$ 个正分段时间，以及整条轨迹首末端的速度、加速度和 Jerk，构造一条
分段七次、全局最小 Snap 的轨迹。

### 14.1 统一记号、输入条件与优化问题

记：

- 航点为 $\mathbf P_0,\ldots,\mathbf P_N\in\mathbb R^d$；
- 第 $i$ 段持续时间为 $h_i>0$，$i=0,\ldots,N-1$；
- 全局节点时刻为 $t_0=t_s$ 和
  $t_{i+1}=t_i+h_i$；
- 第 $i$ 段的局部时间为 $\tau=t-t_i\in[0,h_i]$；
- 节点导数记为
  $\mathbf V_i=\mathbf p'(t_i)$、
  $\mathbf A_i=\mathbf p''(t_i)$、
  $\mathbf J_i=\mathbf p^{(3)}(t_i)$。

每段多项式是

$$
\mathbf p_i(\tau)=\sum_{k=0}^{7}\mathbf c_{i,k}\tau^k,
\qquad \mathbf c_{i,k}\in\mathbb R^d.
$$

类中 `coeffs_` 按

$$
\bigl[
\mathbf c_{0,0},\ldots,\mathbf c_{0,7},
\mathbf c_{1,0},\ldots,\mathbf c_{N-1,7}
\bigr]^T
$$

存成 $8N\times d$ 矩阵。

目标泛函为

$$
E[\mathbf p]
=\sum_{i=0}^{N-1}\int_0^{h_i}
\left\|\mathbf p_i^{(4)}(\tau)\right\|_2^2\,d\tau.
$$

约束为

$$
\begin{aligned}
\mathbf p_i(0)&=\mathbf P_i,
&\mathbf p_i(h_i)&=\mathbf P_{i+1},\\
(\mathbf V_0,\mathbf A_0,\mathbf J_0)&=
(\mathbf V_s,\mathbf A_s,\mathbf J_s),
&(\mathbf V_N,\mathbf A_N,\mathbf J_N)&=
(\mathbf V_e,\mathbf A_e,\mathbf J_e),
\end{aligned}
$$

并要求相邻段在节点处共享位置、速度、加速度和 Jerk。内部节点的
$(\mathbf V_i,\mathbf A_i,\mathbf J_i)$ 是优化变量。对它们取驻值后还会得到

$$
\mathbf p_{i-1}^{(r)}(h_{i-1})
=\mathbf p_i^{(r)}(0),\qquad r=4,5,6.
$$

因此最终轨迹在一般情况下是 $C^6$ 连续的。七阶导数允许在固定航点处跳变；
这个跳变正好决定能量对内部航点的梯度。

实现隐含的必要输入条件是：

$$
\#\{\mathbf P_i\}=N+1,
\qquad h_i>0,
\qquad t_0<t_1<\cdots<t_N.
$$

该类本身没有完整检查这些条件；特别是 `precomputeTimePowers()` 会计算
$1/h_i$，所以零或负持续时间会使后续公式失效。

### 14.2 类型、常量与梯度结果结构

`VectorType` 是 $d\times1$ 列向量，`RowVectorType` 是 $1\times d$ 行向量，
`MatrixType` 是动态行数、固定 $d$ 列的矩阵。所有空间维度相互独立，
但共享同一组分段时间和同一个标量块系统。

`ORDER = 7` 表示多项式次数为 7，`COEFF_NUM = 8` 表示每段有 8 个系数。
`TrajectoryType = PPolyND<DIM, 8>` 是最终用于快速求值的分段多项式。

`BoundaryStateGrads` 表示某一端状态

$$
\mathbf x_b=(\mathbf p_b,\mathbf v_b,\mathbf a_b,\mathbf j_b)
$$

对应的梯度

$$
\left(
\frac{\partial L}{\partial\mathbf p_b},
\frac{\partial L}{\partial\mathbf v_b},
\frac{\partial L}{\partial\mathbf a_b},
\frac{\partial L}{\partial\mathbf j_b}
\right).
$$

`BoundaryDualGrads` 同时保存首端和末端梯度。`Gradients` 进一步增加：

$$
\frac{dL}{d\mathbf P_1},\ldots,
\frac{dL}{d\mathbf P_{N-1}},
\qquad
\frac{dL}{dh_0},\ldots,\frac{dL}{dh_{N-1}}.
$$

注意 `BoundaryConditions<DIM>` 只显式保存首末端的
$(\mathbf v,\mathbf a,\mathbf j)$；端点位置来自 `spatial_points_` 的首尾两行。
梯度结构仍把端点位置梯度一起返回，便于外部优化所有航点。

### 14.3 构造、更新和时间预计算函数

#### 14.3.1 两组构造函数与两组 `update()`

第一种接口接收绝对节点时刻 `t_points`，执行

$$
t_s=t_0,
\qquad h_i=t_{i+1}-t_i.
$$

第二种接口直接接收 `time_segments` 和 `start_time`。两组 `update()` 与对应
构造函数数学上完全相同，只是覆盖已有对象的数据后重新求解。

默认构造函数不做计算；只有调用非默认构造函数或 `update()` 后，
`is_initialized_` 才为真。

#### 14.3.2 `convertTimePointsToSegments()`

该函数实现相邻节点时刻的一阶差分：

$$
\boxed{h_i=t_{i+1}-t_i},\qquad t_s=t_0.
$$

#### 14.3.3 `updateCumulativeTimes()`

该函数执行前缀和：

$$
t_0=t_s,
\qquad
t_{i+1}=t_i+h_i
=t_s+\sum_{k=0}^{i}h_k.
$$

所以

$$
t_e=t_N,
\qquad
T_{\mathrm{all}}=t_N-t_s=\sum_{i=0}^{N-1}h_i.
$$

这分别对应 `getEndTime()` 和 `getDuration()`。

#### 14.3.4 `precomputeTimePowers()`

为避免在块矩阵、系数和梯度公式中反复求幂，每段缓存

$$
h_i, h_i^{-1}, h_i^{-2},\ldots,h_i^{-7}.
$$

实现使用连乘，例如

$$
h^{-5}=h^{-4}h^{-1},\quad
h^{-6}=h^{-4}h^{-2},\quad
h^{-7}=h^{-4}h^{-3}.
$$

#### 14.3.5 `precomputePointDiffs()`

缓存每段位置增量

$$
\boxed{\Delta\mathbf P_i=\mathbf P_{i+1}-\mathbf P_i}.
$$

#### 14.3.6 `updateSplineInternal()`

这是总调度函数，数学数据流是

$$
\{h_i,\mathbf P_i,\text{边界导数}\}
\longrightarrow
\{t_i,h_i^{-k},\Delta\mathbf P_i\}
\longrightarrow
\{\mathbf V_i,\mathbf A_i,\mathbf J_i\}
\longrightarrow
\{\mathbf c_{i,k}\}
\longrightarrow
\texttt{PPolyND}.
$$

`is_initialized_` 在系数求解完成后置为真。

### 14.4 `computeBasisFunctions()`：位置及一至五阶导数基

定义幂基列向量的转置形式

$$
\mathbf b_0(\tau)=
[1,\tau,\tau^2,\tau^3,\tau^4,\tau^5,\tau^6,\tau^7].
$$

若
$\mathbf C_i=[\mathbf c_{i,0},\ldots,\mathbf c_{i,7}]^T$，则

$$
\mathbf p_i^{(r)}(\tau)=\mathbf b_r(\tau)\mathbf C_i,
$$

一般元素公式为

$$
(\mathbf b_r)_k=
\begin{cases}
\dfrac{k!}{(k-r)!}\tau^{k-r}, & k\ge r,\\
0, & k<r.
\end{cases}
$$

函数显式生成：

$$
\begin{aligned}
\mathbf b_1&=[0,1,2\tau,3\tau^2,4\tau^3,5\tau^4,6\tau^5,7\tau^6],\\
\mathbf b_2&=[0,0,2,6\tau,12\tau^2,20\tau^3,30\tau^4,42\tau^5],\\
\mathbf b_3&=[0,0,0,6,24\tau,60\tau^2,120\tau^3,210\tau^4],\\
\mathbf b_4&=[0,0,0,0,24,120\tau,360\tau^2,840\tau^3],\\
\mathbf b_5&=[0,0,0,0,0,120,720\tau,2520\tau^2].
\end{aligned}
$$

它们依次对应 position、velocity、acceleration、Jerk、Snap 和 Crackle。
六阶 Pop 与七阶导数虽未由该函数输出，但在其他函数中使用：

$$
\mathbf p^{(6)}=720\mathbf c_6+5040\mathbf c_7\tau,
\qquad
\mathbf p^{(7)}=5040\mathbf c_7.
$$

### 14.5 `solveSepticSpline()`：由端点状态恢复每段系数

先由 `solveInternalDerivatives()` 得到所有节点的
$(\mathbf V_i,\mathbf A_i,\mathbf J_i)$。对第 $i$ 段，四个低阶系数直接由
左端状态确定：

$$
\boxed{
\mathbf c_0=\mathbf P_i,\quad
\mathbf c_1=\mathbf V_i,\quad
\mathbf c_2=\frac{1}{2}\mathbf A_i,\quad
\mathbf c_3=\frac{1}{6}\mathbf J_i
}.
$$

以下省略段下标，记
$\Delta\mathbf P=\mathbf P_{i+1}-\mathbf P_i$、$h=h_i$，则由右端的
位置、速度、加速度、Jerk 四个 Hermite 条件解得：

$$
\begin{aligned}
\mathbf c_4={}&35\Delta\mathbf P h^{-4}
-20\mathbf V_i h^{-3}-15\mathbf V_{i+1}h^{-3}
-5\mathbf A_i h^{-2}+\frac52\mathbf A_{i+1}h^{-2}
-\frac23\mathbf J_i h^{-1}-\frac16\mathbf J_{i+1}h^{-1},\\
\mathbf c_5={}&-84\Delta\mathbf P h^{-5}
+45\mathbf V_i h^{-4}+39\mathbf V_{i+1}h^{-4}
+10\mathbf A_i h^{-3}-7\mathbf A_{i+1}h^{-3}
+\mathbf J_i h^{-2}+\frac12\mathbf J_{i+1}h^{-2},\\
\mathbf c_6={}&70\Delta\mathbf P h^{-6}
-36\mathbf V_i h^{-5}-34\mathbf V_{i+1}h^{-5}
-\frac{15}{2}\mathbf A_i h^{-4}+\frac{13}{2}\mathbf A_{i+1}h^{-4}
-\frac23\mathbf J_i h^{-3}-\frac12\mathbf J_{i+1}h^{-3},\\
\mathbf c_7={}&-20\Delta\mathbf P h^{-7}
+10\mathbf V_i h^{-6}+10\mathbf V_{i+1}h^{-6}
+2\mathbf A_i h^{-5}-2\mathbf A_{i+1}h^{-5}
+\frac16\mathbf J_i h^{-4}+\frac16\mathbf J_{i+1}h^{-4}.
\end{aligned}
$$

这些式子是一个 $8\times8$ 七次 Hermite 插值系统的闭式解。代回可验证

$$
\begin{aligned}
\mathbf p_i(0)&=\mathbf P_i,&
\mathbf p_i(h)&=\mathbf P_{i+1},\\
\mathbf p_i'(0)&=\mathbf V_i,&
\mathbf p_i'(h)&=\mathbf V_{i+1},\\
\mathbf p_i''(0)&=\mathbf A_i,&
\mathbf p_i''(h)&=\mathbf A_{i+1},\\
\mathbf p_i^{(3)}(0)&=\mathbf J_i,&
\mathbf p_i^{(3)}(h)&=\mathbf J_{i+1}.
\end{aligned}
$$

### 14.6 `solveInternalDerivatives()`：块三对角最小 Snap 方程

对内部节点 $m=1,\ldots,N-1$，定义三行块未知量

$$
\mathbf X_m=
\begin{bmatrix}
\mathbf V_m\\
\mathbf A_m\\
\mathbf J_m
\end{bmatrix}\in\mathbb R^{3\times d}.
$$

首末端块 $\mathbf X_0,\mathbf X_N$ 由边界条件给定。令
$\alpha=h_{m-1}$、$\beta=h_m$，四至六阶导数连续可写为

$$
\boxed{
L_m\mathbf X_{m-1}+D_m\mathbf X_m+U_m\mathbf X_{m+1}
=\mathbf r_m
}.
$$

左块为

$$
L_m=
\begin{bmatrix}
360\alpha^{-3}&60\alpha^{-2}&4\alpha^{-1}\\
4680\alpha^{-4}&840\alpha^{-3}&60\alpha^{-2}\\
24480\alpha^{-5}&4680\alpha^{-4}&360\alpha^{-3}
\end{bmatrix},
$$

中间块为

$$
D_m=
\begin{bmatrix}
480(\alpha^{-3}+\beta^{-3})&120(\beta^{-2}-\alpha^{-2})&16(\alpha^{-1}+\beta^{-1})\\
5400(\alpha^{-4}-\beta^{-4})&-1200(\alpha^{-3}+\beta^{-3})&120(\alpha^{-2}-\beta^{-2})\\
25920(\alpha^{-5}+\beta^{-5})&5400(\beta^{-4}-\alpha^{-4})&480(\alpha^{-3}+\beta^{-3})
\end{bmatrix},
$$

右块为

$$
U_m=
\begin{bmatrix}
360\beta^{-3}&-60\beta^{-2}&4\beta^{-1}\\
-4680\beta^{-4}&840\beta^{-3}&-60\beta^{-2}\\
24480\beta^{-5}&-4680\beta^{-4}&360\beta^{-3}
\end{bmatrix}.
$$

再记
$\Delta\mathbf P_L=\mathbf P_m-\mathbf P_{m-1}$、
$\Delta\mathbf P_R=\mathbf P_{m+1}-\mathbf P_m$，右端为

$$
\mathbf r_m=
\begin{bmatrix}
840(\Delta\mathbf P_R\beta^{-4}+\Delta\mathbf P_L\alpha^{-4})\\
10080(-\Delta\mathbf P_R\beta^{-5}+\Delta\mathbf P_L\alpha^{-5})\\
50400(\Delta\mathbf P_R\beta^{-6}+\Delta\mathbf P_L\alpha^{-6})
\end{bmatrix}.
$$

三行依次就是

$$
\mathbf p_{m-1}^{(4)}(\alpha)-\mathbf p_m^{(4)}(0)=0,
\quad
\mathbf p_{m-1}^{(5)}(\alpha)-\mathbf p_m^{(5)}(0)=0,
\quad
\mathbf p_{m-1}^{(6)}(\alpha)-\mathbf p_m^{(6)}(0)=0
$$

整理后的结果。

所有内部节点组合成块三对角线性系统 $A\mathbf X=\mathbf r$。实现使用块 Thomas
追赶法。把已知边界移到右端后，前向消元满足

$$
\begin{aligned}
\widetilde D_1&=D_1,
&\widetilde{\mathbf r}_1&=\mathbf r_1-L_1\mathbf X_0,\\
G_m&=L_m\widetilde D_{m-1}^{-1},
&\widetilde D_m&=D_m-G_mU_{m-1},\\
&&\widetilde{\mathbf r}_m&=\mathbf r_m-G_m\widetilde{\mathbf r}_{m-1}.
\end{aligned}
$$

最后一个右端还要减去 $U_{N-1}\mathbf X_N$。反向回代为

$$
\mathbf X_{N-1}=\widetilde D_{N-1}^{-1}\widetilde{\mathbf r}_{N-1},
$$

$$
\mathbf X_m=\widetilde D_m^{-1}
(\widetilde{\mathbf r}_m-U_m\mathbf X_{m+1}).
$$

由于每个块只有 $3\times3$，复杂度为 $O(Nd)$，而不是把整个
$3(N-1)d$ 维系统当作稠密矩阵求解。

### 14.7 $3\times3$ 块辅助函数

`Inverse3x3()` 使用伴随矩阵公式。若余子式矩阵为 $C=[C_{ij}]$，则

$$
\det A=a_{00}C_{00}+a_{01}C_{01}+a_{02}C_{02},
\qquad
A^{-1}=\frac{C^T}{\det A}.
$$

例如

$$
C_{00}=a_{11}a_{22}-a_{12}a_{21},\qquad
C_{01}=-(a_{10}a_{22}-a_{12}a_{20}).
$$

函数不检查 $\det A=0$，其可逆性依赖正分段时间和问题本身的非退化性。

`setBlock3x3()` 只是行优先展平：

$$
M\mapsto
[M_{00},M_{01},M_{02},M_{10},M_{11},M_{12},M_{20},M_{21},M_{22}].
$$

其余四个函数是相同线性代数的手工展开：

$$
\begin{array}{c|c}
\text{函数}&\text{数学操作}\\ \hline
\texttt{MultiplyStoredBlock3x3}&C=AB\\
\texttt{multiplyStoredBlock3x3\_3xN}&C=AB,\quad B\in\mathbb R^{3\times d}\\
\texttt{multiplyStoredBlock3x3T\_3xN}&C=A^TB\\
\texttt{subMultiplyStoredBlock3x3T\_3xN}&C\leftarrow C-A^TB\\
\texttt{subMultiplyStoredBlock3x3\_3xN}&C\leftarrow C-AB
\end{array}
$$

手工展开避免构造临时 Eigen 表达式，也让同一份块分解可同时服务正向求解和
转置伴随求解。

### 14.8 `initializePPoly()` 与只读查询函数

`initializePPoly()` 调用

```cpp
trajectory_.update(cumulative_times_, coeffs_, 8);
```

把断点 ${t_i}$、八系数矩阵和每段系数数目交给 `PPolyND`。此后在全局时刻
$t\in[t_i,t_{i+1}]$ 求值时，`PPolyND` 先取局部时间 $\tau=t-t_i$，再计算

$$
\mathbf p^{(r)}(t)=
\sum_{k=r}^{7}\frac{k!}{(k-r)!}\mathbf c_{i,k}\tau^{k-r}.
$$

`getTrajectory()` 与 `getPPoly()` 都返回同一个 `trajectory_` 的常量引用；
带 `Copy` 后缀的版本返回副本。

其余 getter 不改变数学状态：

$$
\begin{aligned}
\texttt{getDimension()}&=d,\\
\texttt{getNumPoints()}&=N+1,\\
\texttt{getNumSegments()}&=N,\\
\texttt{getStartTime()}&=t_0,\\
\texttt{getEndTime()}&=t_N,\\
\texttt{getDuration()}&=\sum_i h_i.
\end{aligned}
$$

`getSpacePoints()`、`getTimeSegments()`、`getCumulativeTimes()` 和
`getBoundaryConditions()` 分别暴露 ${\mathbf P_i}$、${h_i}$、${t_i}$
和边界导数。默认构造后、初始化前不应调用依赖 `cumulative_times_.back()` 的
时间查询函数。

### 14.9 `getEnergy()`：Snap 平方积分的闭式公式

第 $i$ 段 Snap 为

$$
\mathbf s(\tau)=\mathbf p_i^{(4)}(\tau)
=24\mathbf c_4+120\mathbf c_5\tau
+360\mathbf c_6\tau^2+840\mathbf c_7\tau^3.
$$

因此

$$
E_i=\int_0^h\|\mathbf s(\tau)\|^2d\tau
=\mathbf C_i^TQ(h)\mathbf C_i,
\qquad
Q(h)=\int_0^h\mathbf b_4(\tau)^T\mathbf b_4(\tau)d\tau,
$$

其中矩阵表达按空间各维求和。代码展开为

$$
\begin{aligned}
E_i={}&576\|\mathbf c_4\|^2h
+2880\mathbf c_4\!\cdot\!\mathbf c_5h^2\\
&+4800\|\mathbf c_5\|^2h^3
+5760\mathbf c_4\!\cdot\!\mathbf c_6h^3\\
&+21600\mathbf c_5\!\cdot\!\mathbf c_6h^4
+10080\mathbf c_4\!\cdot\!\mathbf c_7h^4\\
&+25920\|\mathbf c_6\|^2h^5
+40320\mathbf c_5\!\cdot\!\mathbf c_7h^5\\
&+100800\mathbf c_6\!\cdot\!\mathbf c_7h^6
+100800\|\mathbf c_7\|^2h^7.
\end{aligned}
$$

总能量是 $E=\sum_iE_i$。未初始化时函数返回 0；代码还跳过 $h_i\le0$
的段，但这并不意味着非正时间是合法输入，因为系数求解阶段已经使用了 $h_i^{-1}$。

### 14.10 `getEnergyPartialGradByCoeffs()`：固定时间的系数偏导

由 $E_i=\mathbf C_i^TQ\mathbf C_i$ 得

$$
\boxed{\frac{\partial E_i}{\partial\mathbf C_i}=2Q(h_i)\mathbf C_i}.
$$

由于低四阶系数不出现在 Snap 中，

$$
\frac{\partial E}{\partial\mathbf c_0}=cdots=
\frac{\partial E}{\partial\mathbf c_3}=0.
$$

其余四行是

$$
\begin{aligned}
\frac{\partial E_i}{\partial\mathbf c_4}={}&
1152\mathbf c_4h+2880\mathbf c_5h^2
+5760\mathbf c_6h^3+10080\mathbf c_7h^4,\\
\frac{\partial E_i}{\partial\mathbf c_5}={}&
2880\mathbf c_4h^2+9600\mathbf c_5h^3
+21600\mathbf c_6h^4+40320\mathbf c_7h^5,\\
\frac{\partial E_i}{\partial\mathbf c_6}={}&
5760\mathbf c_4h^3+21600\mathbf c_5h^4
+51840\mathbf c_6h^5+100800\mathbf c_7h^6,\\
\frac{\partial E_i}{\partial\mathbf c_7}={}&
10080\mathbf c_4h^4+40320\mathbf c_5h^5
+100800\mathbf c_6h^6+201600\mathbf c_7h^7.
\end{aligned}
$$

引用参数版本写入调用者给出的矩阵；无参数重载只是创建矩阵后调用同一实现。

### 14.11 `getEnergyPartialGradByTimes()`：固定系数的显式时间偏导

这里“partial”表示把系数 $\mathbf c_k$ 当常量，只改变积分上限。由 Leibniz
公式：

$$
\boxed{
\left.\frac{\partial E_i}{\partial h_i}\right|_{\mathbf C_i}
=\|\mathbf p_i^{(4)}(h_i)\|^2
}.
$$

代码先计算

$$
\mathbf p_i^{(4)}(h)=
24\mathbf c_4+120\mathbf c_5h+360\mathbf c_6h^2+840\mathbf c_7h^3,
$$

再取平方范数。它不包含 $\mathbf C_i(h_i)$ 以及内部节点导数随时间变化产生的
隐式项，适合与通用 `propagateGrad()` 配套使用。

### 14.12 `getEnergyGradTimes()`：最优样条的完整时间梯度

该函数专用于当前已经满足最小 Snap 驻值条件的样条。利用包络定理和分部积分，
系数与内部导数的隐式变化可化简掉，得到每段

$$
\boxed{
\frac{dE}{dh_i}=
-\|\mathbf p_i^{(4)}(0)\|^2
+2\mathbf p_i^{(3)}(0)\cdot\mathbf p_i^{(5)}(0)
-2\mathbf p_i^{(2)}(0)\cdot\mathbf p_i^{(6)}(0)
+2\mathbf p_i^{(1)}(0)\cdot\mathbf p_i^{(7)}(0)
}.
$$

代入左端各阶导数

$$
\mathbf p'(0)=\mathbf c_1,\quad
\mathbf p''(0)=2\mathbf c_2,\quad
\mathbf p^{(3)}(0)=6\mathbf c_3,\quad
\mathbf p^{(4)}(0)=24\mathbf c_4,
$$

$$
\mathbf p^{(5)}(0)=120\mathbf c_5,\quad
\mathbf p^{(6)}(0)=720\mathbf c_6,\quad
\mathbf p^{(7)}(0)=5040\mathbf c_7,
$$

就得到代码中的

$$
\frac{dE}{dh_i}
=-576\|\mathbf c_4\|^2
+1440\mathbf c_3\cdot\mathbf c_5
-2880\mathbf c_2\cdot\mathbf c_6
+10080\mathbf c_1\cdot\mathbf c_7.
$$

它与上一节的偏导不是同一个量：前者是重新求解最优样条后的总导数，后者是
固定系数时的积分上限偏导。

### 14.13 `getEnergyGradInnerPoints()`：内部航点梯度

对能量做四次分部积分：

$$
\delta E_i
=2\left[
\mathbf p^{(4)}\cdot\delta\mathbf p^{(3)}
-\mathbf p^{(5)}\cdot\delta\mathbf p^{(2)}
+\mathbf p^{(6)}\cdot\delta\mathbf p^{(1)}
-\mathbf p^{(7)}\cdot\delta\mathbf p
\right]_0^{h_i}.
$$

内部节点的四至六阶导数连续，使前三类节点边界项互相抵消。对内部航点
$\mathbf P_m$ 留下

$$
\boxed{
\frac{dE}{d\mathbf P_m}
=2\left[
\mathbf p_m^{(7)}(0)-
\mathbf p_{m-1}^{(7)}(h_{m-1})
\right]
}.
$$

七次多项式的七阶导数是常量 $5040\mathbf c_7$，所以

$$
\frac{dE}{d\mathbf P_m}
=10080(\mathbf c_{m,7}-\mathbf c_{m-1,7}),
$$

这就是代码的 `10080.0 * (c7_R - c7_L)`。

### 14.14 `getEnergyGradBoundary()`：首末状态梯度

使用同一个分部积分边界项，在首端得到

$$
\begin{aligned}
\frac{dE}{d\mathbf P_0}&=2\mathbf p^{(7)}(0)=10080\mathbf c_7,\\
\frac{dE}{d\mathbf V_0}&=-2\mathbf p^{(6)}(0)=-1440\mathbf c_6,\\
\frac{dE}{d\mathbf A_0}&=2\mathbf p^{(5)}(0)=240\mathbf c_5,\\
\frac{dE}{d\mathbf J_0}&=-2\mathbf p^{(4)}(0)=-48\mathbf c_4.
\end{aligned}
$$

在末端 $\tau=h_{N-1}$ 得到

$$
\begin{aligned}
\frac{dE}{d\mathbf P_N}&=-2\mathbf p^{(7)}(h),\\
\frac{dE}{d\mathbf V_N}&=2\mathbf p^{(6)}(h),\\
\frac{dE}{d\mathbf A_N}&=-2\mathbf p^{(5)}(h),\\
\frac{dE}{d\mathbf J_N}&=2\mathbf p^{(4)}(h).
\end{aligned}
$$

代码中的 `snap_end`、`crackle_end`、`pop_end` 和 `d7_end` 分别就是

$$
\begin{aligned}
\mathbf p^{(4)}(h)&=24\mathbf c_4+120\mathbf c_5h+360\mathbf c_6h^2+840\mathbf c_7h^3,\\
\mathbf p^{(5)}(h)&=120\mathbf c_5+720\mathbf c_6h+2520\mathbf c_7h^2,\\
\mathbf p^{(6)}(h)&=720\mathbf c_6+5040\mathbf c_7h,\\
\mathbf p^{(7)}(h)&=5040\mathbf c_7.
\end{aligned}
$$

### 14.15 两个 `getEnergyGrad()` 重载

这两个函数没有引入新公式，只是把三个专用解析结果组装起来：

$$
\texttt{Gradients}=
\left\{
\frac{dE}{d\mathbf P_{1:N-1}},
\frac{dE}{d\mathbf h},
\frac{dE}{d\mathbf x_s},
\frac{dE}{d\mathbf x_e}
\right\}.
$$

返回值版本内部调用引用输出版本。

### 14.16 `propagateGrad()`：任意标量损失的反向传播

`getEnergyGrad*()` 利用了最小 Snap 能量的特殊边界恒等式；`propagateGrad()`
则处理一般标量损失

$$
L=L(\mathbf C(\mathbf P,\mathbf h,\mathbf b),\mathbf h),
$$

调用者给出

$$
G_{i,k}=\frac{\partial L}{\partial\mathbf c_{i,k}},
\qquad
g_{h_i}^{\mathrm{explicit}}=
\left.\frac{\partial L}{\partial h_i}\right|_{\mathbf C}.
$$

两个公开重载都调用 `propagateGradInternal()`；区别仅是返回结构还是写入引用。

#### 14.16.1 系数映射的直接链式法则

由 14.5 节的系数公式，可直接把 $G_{i,k}$ 累加到两端位置和节点导数。
例如位置的系数为

$$
\begin{array}{c|rrrr}
&\mathbf c_4&\mathbf c_5&\mathbf c_6&\mathbf c_7\\ \hline
\partial/\partial\mathbf P_i
&-35h^{-4}&84h^{-5}&-70h^{-6}&20h^{-7}\\
\partial/\partial\mathbf P_{i+1}
&35h^{-4}&-84h^{-5}&70h^{-6}&-20h^{-7}
\end{array}
$$

低阶直接项为

$$
\frac{\partial L}{\partial\mathbf P_i}\mathrel{+}=G_{i,0},
\quad
\frac{\partial L}{\partial\mathbf V_i}\mathrel{+}=G_{i,1},
\quad
\frac{\partial L}{\partial\mathbf A_i}\mathrel{+}=\frac12G_{i,2},
\quad
\frac{\partial L}{\partial\mathbf J_i}\mathrel{+}=\frac16G_{i,3}.
$$

高阶系数对节点导数的 Jacobian 正是：

$$
\begin{array}{c|rrrr}
&\mathbf c_4&\mathbf c_5&\mathbf c_6&\mathbf c_7\\ \hline
\mathbf V_i&-20h^{-3}&45h^{-4}&-36h^{-5}&10h^{-6}\\
\mathbf V_{i+1}&-15h^{-3}&39h^{-4}&-34h^{-5}&10h^{-6}\\
\mathbf A_i&-5h^{-2}&10h^{-3}&-\frac{15}{2}h^{-4}&2h^{-5}\\
\mathbf A_{i+1}&\frac52h^{-2}&-7h^{-3}&\frac{13}{2}h^{-4}&-2h^{-5}\\
\mathbf J_i&-\frac23h^{-1}&h^{-2}&-\frac23h^{-3}&\frac16h^{-4}\\
\mathbf J_{i+1}&-\frac16h^{-1}&\frac12h^{-2}&-\frac12h^{-3}&\frac16h^{-4}
\end{array}
$$

代码中的 `ws_gd_internal_` 就是在汇总每个节点的

$$
\mathbf g_{X_i}=
\left[
\frac{\partial L}{\partial\mathbf V_i};
\frac{\partial L}{\partial\mathbf A_i};
\frac{\partial L}{\partial\mathbf J_i}
\right].
$$

固定节点状态时，系数对本段时间的偏导为

$$
\begin{aligned}
\frac{\partial\mathbf c_4}{\partial h}={}&
-140\Delta\mathbf P h^{-5}
+60\mathbf V_i h^{-4}+45\mathbf V_{i+1}h^{-4}
+10\mathbf A_i h^{-3}-5\mathbf A_{i+1}h^{-3}
+\frac23\mathbf J_i h^{-2}+\frac16\mathbf J_{i+1}h^{-2},\\
\frac{\partial\mathbf c_5}{\partial h}={}&
420\Delta\mathbf P h^{-6}
-180\mathbf V_i h^{-5}-156\mathbf V_{i+1}h^{-5}
-30\mathbf A_i h^{-4}+21\mathbf A_{i+1}h^{-4}
-2\mathbf J_i h^{-3}-\mathbf J_{i+1}h^{-3},\\
\frac{\partial\mathbf c_6}{\partial h}={}&
-420\Delta\mathbf P h^{-7}
+180\mathbf V_i h^{-6}+170\mathbf V_{i+1}h^{-6}
+30\mathbf A_i h^{-5}-26\mathbf A_{i+1}h^{-5}
+2\mathbf J_i h^{-4}+\frac32\mathbf J_{i+1}h^{-4},\\
\frac{\partial\mathbf c_7}{\partial h}={}&
140\Delta\mathbf P h^{-8}
-60\mathbf V_i h^{-7}-60\mathbf V_{i+1}h^{-7}
-10\mathbf A_i h^{-6}+10\mathbf A_{i+1}h^{-6}
-\frac23\mathbf J_i h^{-5}-\frac23\mathbf J_{i+1}h^{-5}.
\end{aligned}
$$

于是先累加直接时间项

$$
g_{h_i}\mathrel{+}=
\sum_{k=4}^{7}G_{i,k}\cdot
\frac{\partial\mathbf c_{i,k}}{\partial h_i}.
$$

代码对 `DIM <= 3` 使用标量循环，对更高维使用 Eigen 点积；两条分支的数学式
完全一致。

#### 14.16.2 内部导数隐式依赖的伴随法

内部导数满足

$$
F(\mathbf X,\theta)=A(\mathbf h)\mathbf X-\mathbf r(\mathbf P,\mathbf h,\mathbf b)=0.
$$

若逐个设计变量求 $d\mathbf X/d\theta$，代价很高。实现改为解伴随方程

$$
\boxed{A^T\boldsymbol\lambda=\mathbf g_X}.
$$

`ws_lambda_` 存储 $\boldsymbol\lambda$。代码复用正向块 Thomas 分解的缓存，
先做转置系统的前向过程，再做反向修正。

任意参数 $\theta$ 的总梯度为

$$
\boxed{
\frac{dL}{d\theta}
=\left.\frac{\partial L}{\partial\theta}\right|_{\mathbf X}
+\boldsymbol\lambda^T
\left(
\frac{\partial\mathbf r}{\partial\theta}
-\frac{\partial A}{\partial\theta}\mathbf X
\right)
}.
$$

这一个公式解释了后半个 `propagateGradInternal()` 的所有修正项。

对内部节点 $m$ 的左、右时间，分别有

$$
\begin{aligned}
\boldsymbol\phi_{h_{m-1}}={}&
\frac{\partial\mathbf r_m}{\partial\alpha}
-\frac{\partial L_m}{\partial\alpha}\mathbf X_{m-1}
-\frac{\partial D_m}{\partial\alpha}\mathbf X_m,\\
\boldsymbol\phi_{h_m}={}&
\frac{\partial\mathbf r_m}{\partial\beta}
-\frac{\partial D_m}{\partial\beta}\mathbf X_m
-\frac{\partial U_m}{\partial\beta}\mathbf X_{m+1},
\end{aligned}
$$

并累加

$$
g_{h_{m-1}}\mathrel{+}=\boldsymbol\lambda_m^T\boldsymbol\phi_{h_{m-1}},
\qquad
g_{h_m}\mathrel{+}=\boldsymbol\lambda_m^T\boldsymbol\phi_{h_m}.
$$

矩阵导数由统一规则

$$
\frac{d}{dh}h^{-k}=-kh^{-(k+1)}
$$

逐元素得到。右端导数的三行是

$$
\frac{\partial\mathbf r_m}{\partial\alpha}=
\begin{bmatrix}
-3360\Delta\mathbf P_L\alpha^{-5}\\
-50400\Delta\mathbf P_L\alpha^{-6}\\
-302400\Delta\mathbf P_L\alpha^{-7}
\end{bmatrix},
$$

$$
\frac{\partial\mathbf r_m}{\partial\beta}=
\begin{bmatrix}
-3360\Delta\mathbf P_R\beta^{-5}\\
50400\Delta\mathbf P_R\beta^{-6}\\
-302400\Delta\mathbf P_R\beta^{-7}
\end{bmatrix}.
$$

航点的伴随修正则是

$$
\frac{dL}{d\mathbf P}\mathrel{+}=
\boldsymbol\lambda_m^T\frac{\partial\mathbf r_m}{\partial\mathbf P}.
$$

具体 Jacobian 为

$$
\frac{\partial\mathbf r_m}{\partial\mathbf P_{m-1}}=
\begin{bmatrix}
-840\alpha^{-4}\\-10080\alpha^{-5}\\-50400\alpha^{-6}
\end{bmatrix},
$$

$$
\frac{\partial\mathbf r_m}{\partial\mathbf P_m}=
\begin{bmatrix}
-840(\beta^{-4}-\alpha^{-4})\\
10080(\beta^{-5}+\alpha^{-5})\\
-50400(\beta^{-6}-\alpha^{-6})
\end{bmatrix},
$$

$$
\frac{\partial\mathbf r_m}{\partial\mathbf P_{m+1}}=
\begin{bmatrix}
840\beta^{-4}\\-10080\beta^{-5}\\50400\beta^{-6}
\end{bmatrix}.
$$

每个标量系数都乘相应的 $d$ 维伴随行向量。最后，已知边界导数被移出
内部系统时出现的两项给出

$$
\mathbf g_{X_0}\leftarrow\mathbf g_{X_0}-L_1^T\boldsymbol\lambda_1,
\qquad
\mathbf g_{X_N}\leftarrow\mathbf g_{X_N}-U_{N-1}^T\boldsymbol\lambda_{N-1}.
$$

它们与前面从系数映射直接得到的端点导数梯度相加，形成最终的
`startGrads.v/a/j` 和 `endGrads.v/a/j`。

### 14.17 缓存与工作区变量的数学含义

为了避免轨迹优化中反复分配内存，类保存以下求解结果：

- `D_inv_cache_`：前向消元后的 $\widetilde D_m^{-1}$；
- `L_blocks_cache_`、`U_blocks_cache_`：原始下、上对角块；
- `D_inv_T_mul_L_next_T_cache_`：转置伴随回代需要的
  $(L_{m+1}\widetilde D_m^{-1})^T$；
- `ws_rhs_mod_`：$\widetilde{\mathbf r}_m$；
- `ws_solution_`：内部 $\mathbf X_m$；
- `ws_gd_internal_`：损失对所有节点
  $(\mathbf V,\mathbf A,\mathbf J)$ 的直接梯度；
- `ws_lambda_`：伴随变量 $\boldsymbol\lambda_m$；
- `internal_vel_`、`internal_acc_`、`internal_jerk_`：完整节点导数。

这些缓存不改变问题的数学定义，只把正向求解和反向传播都保持在
$O(Nd)$ 时间、$O(Nd)$ 空间。

### 14.18 最容易混淆的三个结论

1. `ORDER = 7` 是七次多项式，但代价惩罚的是四阶导数 Snap；七次是满足两端
   位置、速度、加速度、Jerk 共八个条件所需的最低次数。
2. `getEnergyPartialGradByTimes()` 返回固定系数的
   $\partial E/\partial h$，而 `getEnergyGradTimes()` 返回样条重新达到最优后
   的 $dE/dh$。时间分配优化应使用后者。
3. 轨迹只显式把位置约束在内部航点上；内部速度、加速度和 Jerk 由最小 Snap
   自动选择。它们的驻值条件等价于四、五、六阶导数连续，因此轨迹通常为
   $C^6$，而不是只有表面上的 $C^3$。

---

## 15. <code>Jet2</code> 与 <code>tailsitter_flatness.cpp</code> 逐行数学说明

本章对 <code>planning/src/tailsitter_flatness.cpp</code> 做代码级说明。第 7 章
给出了开环平坦映射的物理主线，本章重点回答两个更具体的问题：

1. <code>Jet2</code> 如何在每次标量和向量运算中传播时间一阶、二阶导数；
2. <code>TailsitterFlatness::forward()</code> 如何把位置和 yaw 轨迹的高阶导数
   依次转换为姿态、角速度、角加速度、机体系力矩、电机转速和舵面偏角。

### 15.1 总体输入、输出与计算链

对一个时刻的平坦输出采样，函数实际使用

$$
\mathbf v,\quad \mathbf a,\quad \mathbf j,\quad \mathbf s,
\qquad
\psi,\quad\dot\psi,\quad\ddot\psi,
$$

其中 $\mathbf j=\mathbf p^{(3)}$ 是 Jerk，
$\mathbf s=\mathbf p^{(4)}$ 是 Snap。位置 $\mathbf p$ 本身不参与当前的
平移不变开环映射，Crackle 和 yaw Jerk 也没有被使用。

完整数据流为

$$
\begin{aligned}
(\mathbf v,\mathbf a,\mathbf j,\mathbf s,\psi,\dot\psi,\ddot\psi)
&\xrightarrow{\text{Jet2}}
(\phi,\dot\phi,\ddot\phi,\theta,\dot\theta,\ddot\theta,T)\\
&\longrightarrow
(\boldsymbol\omega,\dot{\boldsymbol\omega},\mathbf M)\\
&\longrightarrow
(T_1,T_2,\omega_1,\omega_2)\\
&\longrightarrow
(\delta_1,\delta_2).
\end{aligned}
$$

返回的 <code>ActuatorPrediction</code> 是理想模型下未经限幅的开环结果。
<code>forward()</code> 只判断公式能否求解、结果是否有限以及单电机推力是否
非负，不判断电机和舵面是否超过允许幅值。

### 15.2 <code>Jet2</code> 的数学定义

<code>Jet2</code> 保存一个时间函数在当前时刻的二阶 jet：

$$
\mathcal J_2[x]
=
\begin{bmatrix}
x\\ \dot x\\ \ddot x
\end{bmatrix}.
$$

三个成员的对应关系为

$$
\texttt{value}=x,\qquad
\texttt{first}=\dot x,\qquad
\texttt{second}=\ddot x.
$$

它可以理解为截断到二阶的局部 Taylor 信息：

$$
x(t+\Delta t)
=x(t)+\dot x(t)\Delta t+\frac12\ddot x(t)\Delta t^2
+O(\Delta t^3).
$$

这里存的是实际导数 $\ddot x$，不是 Taylor 系数 $\ddot x/2$。因此乘法的
二阶项中会出现系数 2。

<code>Jet2</code> 是关于物理时间 $t$ 的前向自动微分，不是：

- 相邻采样点的有限差分；
- 对航点、分段时间或车辆参数的优化梯度；
- 通用符号代数系统。

只要输入 jet 中的导数正确，所有重载运算就按链式法则给出当前表达式的
解析时间导数。

### 15.3 加、减、乘、除运算

对

$$
\mathcal J_2[a]=(a,\dot a,\ddot a),\qquad
\mathcal J_2[b]=(b,\dot b,\ddot b),
$$

加减法逐分量执行：

$$
\mathcal J_2[a\pm b]
=
(a\pm b,\ \dot a\pm\dot b,\ \ddot a\pm\ddot b).
$$

一元负号同样逐分量取反。

乘法使用一阶和二阶乘积法则：

$$
\mathcal J_2[ab]
=
\left(
ab,\quad
\dot a\,b+a\,\dot b,\quad
\ddot a\,b+2\dot a\dot b+a\,\ddot b
\right).
$$

<code>inverse()</code> 令 $y=x^{-1}$。由

$$
\dot y=-\frac{\dot x}{x^2},
\qquad
\ddot y=
\frac{2\dot x^2}{x^3}-\frac{\ddot x}{x^2},
$$

得到代码中的

$$
\mathcal J_2[x^{-1}]
=
\left(
\frac1x,\quad
-\frac{\dot x}{x^2},\quad
\frac{2\dot x^2}{x^3}-\frac{\ddot x}{x^2}
\right).
$$

除法没有单独展开商法则，而是计算

$$
\mathcal J_2[a/b]=\mathcal J_2[a]\mathcal J_2[b^{-1}].
$$

与 <code>double</code> 的加法、左乘和除法，先把常数 $c$ 提升为

$$
\mathcal J_2[c]=(c,0,0),
$$

再调用同一套运算。这保证常数不会人为引入时间导数。

### 15.4 <code>sin</code>、<code>cos</code>、<code>sqrt</code>

对任意一元复合函数 $y=f(x)$，二阶链式法则统一为

$$
\dot y=f'(x)\dot x,
\qquad
\ddot y=f'(x)\ddot x+f''(x)\dot x^2.
$$

因此

$$
\mathcal J_2[\sin x]
=
\left(
\sin x,\quad
\cos x\,\dot x,\quad
\cos x\,\ddot x-\sin x\,\dot x^2
\right),
$$

$$
\mathcal J_2[\cos x]
=
\left(
\cos x,\quad
-\sin x\,\dot x,\quad
-\sin x\,\ddot x-\cos x\,\dot x^2
\right).
$$

对 $y=\sqrt{x}$：

$$
\dot y=\frac{\dot x}{2\sqrt{x}},
\qquad
\ddot y=
\frac{\ddot x}{2\sqrt{x}}
-\frac{\dot x^2}{4x^{3/2}}.
$$

<code>Jet2 sqrt()</code> 没有自行处理 $x\le0$。它在本文件中用于正则化速度
模，$\|\mathbf v\|^2+\epsilon_v^2>0$ 保证其定义域合法。后面的电机转速使用
普通 <code>std::sqrt(double)</code>，在调用前先检查 $T_i\ge0$：

$$
\omega_i=\sqrt{T_i/c_T}.
$$

两处都还隐含使用车辆参数 $c_T>0$。

### 15.5 二元 <code>atan2</code> 的二阶导数

令

$$
\gamma=\operatorname{atan2}(y,x),\qquad
\rho^2=x^2+y^2,
$$

并定义

$$
n=x\dot y-y\dot x.
$$

则

$$
\dot\gamma=\frac{n}{\rho^2}.
$$

对 $n$ 求导时两个交叉项抵消：

$$
\dot n
=\dot x\dot y+x\ddot y-\dot y\dot x-y\ddot x
=x\ddot y-y\ddot x.
$$

同时

$$
\frac{d\rho^2}{dt}=2(x\dot x+y\dot y).
$$

所以

$$
\boxed{
\ddot\gamma
=
\frac{
\dot n\,\rho^2
-n\,2(x\dot x+y\dot y)
}{\rho^4}
}.
$$

这分别对应代码中的 <code>numerator</code>、
<code>numerator_dot</code>、<code>radius_squared</code> 和
<code>radius_dot</code>。

当

$$
x^2+y^2<\epsilon_{\mathrm{sing}}^2
$$

时，角度本身仍用标准库 <code>atan2(y,x)</code> 计算，但一、二阶导数都被
置为 0。这是一种数值保护：原点附近方向角没有稳定定义，继续除以 $\rho^2$
会产生巨大导数或 NaN。它并不代表真实角速度为零，而是选择在奇异附近冻结
该角的局部导数。

### 15.6 <code>JetVector</code>、点积与三个坐标旋转

<code>JetVector</code> 是三个 <code>Jet2</code> 组成的向量：

$$
\mathcal J_2[\mathbf x]
=
\left(
\mathcal J_2[x_1],
\mathcal J_2[x_2],
\mathcal J_2[x_3]
\right).
$$

<code>add()</code> 逐坐标相加。<code>dot()</code> 调用 jet 乘法和加法：

$$
\mathcal J_2[\mathbf a\cdot\mathbf b]
=\sum_{k=1}^{3}\mathcal J_2[a_kb_k].
$$

因此其导数自动满足

$$
\frac{d}{dt}(\mathbf a\cdot\mathbf b)
=\dot{\mathbf a}\cdot\mathbf b+\mathbf a\cdot\dot{\mathbf b},
$$

$$
\frac{d^2}{dt^2}(\mathbf a\cdot\mathbf b)
=\ddot{\mathbf a}\cdot\mathbf b
+2\dot{\mathbf a}\cdot\dot{\mathbf b}
+\mathbf a\cdot\ddot{\mathbf b}.
$$

三个旋转函数实现的是被动坐标变换矩阵：

$$
R_z(\psi)=
\begin{bmatrix}
\cos\psi&\sin\psi&0\\
-\sin\psi&\cos\psi&0\\
0&0&1
\end{bmatrix},
$$

$$
R_x(\phi)=
\begin{bmatrix}
1&0&0\\
0&\cos\phi&\sin\phi\\
0&-\sin\phi&\cos\phi
\end{bmatrix},
$$

$$
R_y(\theta)=
\begin{bmatrix}
\cos\theta&0&-\sin\theta\\
0&1&0\\
\sin\theta&0&\cos\theta
\end{bmatrix}.
$$

<code>rotateZ</code>、<code>rotateX</code> 和 <code>rotateY</code> 分别计算
$R_z\mathbf x$、$R_x\mathbf x$、$R_y\mathbf x$。由于角度与向量分量都是
jet，例如

$$
\frac{d}{dt}\left[R_z(\psi)\mathbf x\right]
=\dot R_z(\psi,\dot\psi)\mathbf x+R_z(\psi)\dot{\mathbf x}
$$

中的两部分都会由乘法和三角函数重载自动产生，二阶导数中的所有交叉项也会
自动保留。

<code>values()</code> 读取三个坐标的 <code>value</code>，即 $\mathbf x$；
<code>firsts()</code> 读取三个坐标的 <code>first</code>，即
$\dot{\mathbf x}$。

### 15.7 非 Jet 辅助函数和候选结构

<code>worldToBodyMatrix()</code> 使用普通 <code>double</code> 计算

$$
\boxed{
R_i^b=R_y(\theta)R_x(\phi)R_z(\psi)
}.
$$

它只用于已经选定姿态后的矩阵和四元数计算，不承担求导。Eigen 四元数由

$$
R_b^i=(R_i^b)^T
$$

构造，所以返回的四元数表示机体系相对世界系的主动姿态旋转。

<code>allFinite()</code> 检查以下最终量是否都是有限数：

$$
q,\quad\boldsymbol\omega,\quad\dot{\boldsymbol\omega},\quad\mathbf M,
\quad T,T_1,T_2,\quad\omega_1,\omega_2,\quad\delta_1,\delta_2.
$$

<code>AttitudeCandidate</code> 保存一个姿态分支的

$$
(\phi,\bar\theta,T,\mathbf v^\phi,q,d_q).
$$

其中前三项和速度仍保留 jet，$q$ 是普通四元数，$d_q$ 是与上一采样姿态的
距离。

### 15.8 参数构造与 <code>TailsitterFlatness</code> 构造函数

<code>VehicleParameters::paperVehicle()</code> 直接返回结构体默认值。

<code>simulationVehicle()</code> 也从默认值开始，只覆盖

$$
m=0.7,\qquad
c_T=1.8\times10^{-6},\qquad
c_\mu=2.2\times10^{-8}.
$$

其他气动、几何、惯量和限制参数仍与 paper 配置相同。

<code>TailsitterFlatness</code> 构造函数把参数对象移动到
<code>parameters_</code>。这一步没有数学计算；它固定了后续每次
<code>forward()</code> 使用的车辆模型。

### 15.9 <code>forward()</code> 的输入 jet

函数首先保存采样时刻，然后构造速度 jet：

$$
\mathcal J_2[\mathbf v]
=
(\mathbf v,\mathbf a,\mathbf j).
$$

逐坐标看，例如

$$
\mathcal J_2[v_x]=(v_x,a_x,j_x).
$$

参考合力按代码坐标约定定义为

$$
\mathbf f^i
=m
\begin{bmatrix}
a_x\\a_y\\a_z-g
\end{bmatrix}.
$$

它的 jet 为

$$
\boxed{
\mathcal J_2[\mathbf f^i]
=
\left(
m(\mathbf a-g\mathbf e_z),
m\mathbf j,
m\mathbf s
\right)
}.
$$

这说明为什么计算角加速度必须提供 Snap：姿态由力方向决定，而姿态的二阶
导数需要力的二阶导数。

yaw jet 为

$$
\mathcal J_2[\psi]
=(\psi,\dot\psi,\ddot\psi).
$$

位置、Crackle 和 yaw Jerk 在本函数中没有进入任何 jet。

### 15.10 yaw 中间系、roll 与正则化速度

参考力先变换到去 yaw 的中间坐标系：

$$
\mathbf f^\psi=R_z(\psi)\mathbf f^i.
$$

基本 roll 分支为

$$
\boxed{
\phi_0=-\operatorname{atan2}(f_y^\psi,f_z^\psi)
}.
$$

因为两个 <code>atan2</code> 参数都是 jet，结果同时包含

$$
\mathcal J_2[\phi_0]
=(\phi_0,\dot\phi_0,\ddot\phi_0).
$$

正则化速度定义为

$$
V_\epsilon
=\sqrt{\mathbf v^T\mathbf v+\epsilon_v^2}.
$$

由 jet 运算可显式写出

$$
\dot V_\epsilon
=\frac{\mathbf v\cdot\mathbf a}{V_\epsilon},
$$

$$
\ddot V_\epsilon
=
\frac{\mathbf a\cdot\mathbf a+\mathbf v\cdot\mathbf j}{V_\epsilon}
-\frac{(\mathbf v\cdot\mathbf a)^2}{V_\epsilon^3}.
$$

所以在 $\|\mathbf v\|=0$ 时分母仍至少是 $\epsilon_v$，静止 hover 不会因
速度模求导出现 $0/0$。

### 15.11 气动常量、姿态分支与总推力

定义

$$
\bar\alpha=\alpha_0+\alpha_T,
$$

$$
d_\eta=\cos\bar\alpha(1-c_{D_T}).
$$

若

$$
|d_\eta|<\epsilon_{\mathrm{sing}},
$$

总推力公式不可逆，函数返回
<code>collective-thrust denominator is singular</code>。

否则计算

$$
\eta
=
\frac{\sin\bar\alpha(c_{L_T}-1)}
{\cos\bar\alpha(1-c_{D_T})}.
$$

roll 有两个相差 $\pi$ 的候选：

$$
\phi_k=\phi_0+k\pi,\qquad k\in\{0,1\}.
$$

加上常数 $k\pi$ 不改变 $\dot\phi$ 和 $\ddot\phi$。对每个 roll 候选，
代码计算

$$
\mathbf v^\phi=R_x(\phi_k)R_z(\psi)\mathbf v,
\qquad
\mathbf f^\phi=R_x(\phi_k)\mathbf f^\psi.
$$

接着构造

$$
\sigma_y
=
\eta(f_x^\phi+c_{D_V}V_\epsilon v_x^\phi)
-c_{L_V}V_\epsilon v_z^\phi-f_z^\phi,
$$

$$
\sigma_x
=
\eta(f_z^\phi+c_{D_V}V_\epsilon v_z^\phi)
+c_{L_V}V_\epsilon v_x^\phi+f_x^\phi.
$$

基本 pitch 变量为

$$
\bar\theta_0=\operatorname{atan2}(\sigma_y,\sigma_x).
$$

同样枚举两个相差 $\pi$ 的分支：

$$
\bar\theta_l=\bar\theta_0+l\pi,\qquad l\in\{0,1\}.
$$

总推力是

$$
\boxed{
T=
\frac{
\cos\bar\theta\, f_x^\phi
-\sin\bar\theta\, f_z^\phi
+c_{D_V}V_\epsilon
(\cos\bar\theta\,v_x^\phi-\sin\bar\theta\,v_z^\phi)
}{d_\eta}
}.
$$

这个表达式仍然是 <code>Jet2</code>，所以代码实际上也计算出了
$\dot T$ 和 $\ddot T$，尽管当前输出只保存 <code>thrust.value</code>。

四个 $(k,l)$ 组合中，只保留

$$
T\ge0,\qquad T\ \text{有限}
$$

的候选。真实机体系 pitch 为

$$
\theta=\bar\theta+\alpha_0.
$$

常数迎角偏置不改变 $\dot\theta$ 和 $\ddot\theta$。

### 15.12 四元数分支连续性

对每个合法候选，代码构造

$$
R_i^b=R_y(\theta)R_x(\phi)R_z(\psi),
\qquad
q=\operatorname{quat}\left((R_i^b)^T\right),
$$

然后归一化四元数。

若上一采样姿态可用，候选距离定义为

$$
\boxed{
d_q=1-|q_{\mathrm{prev}}^Tq|
}.
$$

绝对值消除了四元数 $q$ 和 $-q$ 表示同一旋转的二重覆盖问题。最小化
$d_q$ 等价于选择与上一姿态旋转最接近的分支。

若 <code>previous_branch.initialized == false</code>，所有候选距离都是 0，
<code>min_element</code> 会选枚举顺序中的第一个合法候选。若四个候选都被
负推力或非有限值过滤，函数返回
<code>no nonnegative-thrust attitude branch</code>。

### 15.13 从姿态角 jet 到角速度和角加速度

选定分支后已有

$$
(\psi,\dot\psi,\ddot\psi),\qquad
(\phi,\dot\phi,\ddot\phi),\qquad
(\theta,\dot\theta,\ddot\theta).
$$

代码把三个角速率重新包装为

$$
\mathcal J_2[\dot\psi]=(\dot\psi,\ddot\psi,0),\quad
\mathcal J_2[\dot\phi]=(\dot\phi,\ddot\phi,0),\quad
\mathcal J_2[\dot\theta]=(\dot\theta,\ddot\theta,0).
$$

第三个分量设为 0 是因为只读取最终角速度 jet 的一阶导数，不需要角加加速度。

角速度由三个旋转轴的贡献相加：

$$
\boldsymbol\omega
=
R_y(\theta)R_x(\phi)R_z(\psi)
\begin{bmatrix}0\\0\\\dot\psi\end{bmatrix}
+R_y(\theta)
\begin{bmatrix}\dot\phi\\0\\0\end{bmatrix}
+\begin{bmatrix}0\\\dot\theta\\0\end{bmatrix}.
$$

由于 $R_z(\psi)[0,0,\dot\psi]^T$ 不改变该向量，可以展开为

$$
\boxed{
\boldsymbol\omega=
\begin{bmatrix}
\cos\theta\,\dot\phi-\sin\theta\cos\phi\,\dot\psi\\
\dot\theta+\sin\phi\,\dot\psi\\
\sin\theta\,\dot\phi+\cos\theta\cos\phi\,\dot\psi
\end{bmatrix}
}.
$$

<code>values(omega)</code> 返回 $\boldsymbol\omega$，
<code>firsts(omega)</code> 返回

$$
\dot{\boldsymbol\omega}
=\frac{d}{dt}\boldsymbol\omega.
$$

例如第一分量自动包含

$$
\begin{aligned}
\dot\omega_x={}&
-\sin\theta\,\dot\theta\dot\phi
+\cos\theta\,\ddot\phi\\
&-\cos\theta\,\dot\theta\cos\phi\,\dot\psi
+\sin\theta\sin\phi\,\dot\phi\dot\psi
-\sin\theta\cos\phi\,\ddot\psi.
\end{aligned}
$$

其他分量由同一套 jet 运算自动得到，避免手写大量容易出错的乘积项。

### 15.14 刚体力矩

得到机体系角速度和角加速度后，代码使用欧拉刚体方程：

$$
\boxed{
\mathbf M
=I\dot{\boldsymbol\omega}
+\boldsymbol\omega\times(I\boldsymbol\omega)
}.
$$

第一项是惯性角加速度力矩，第二项是陀螺耦合项。这里的 $I$ 是
<code>VehicleParameters::inertia</code>，默认是对角矩阵，但公式本身允许
一般 $3\times3$ 惯量矩阵。

### 15.15 yaw 力矩到差分推力和电机转速

为了缩短公式，定义总推力在机体系 $x,z$ 方向的两个等效系数：

$$
k_x=
\cos\alpha_0\cos\bar\alpha(1-c_{D_T})
-\sin\alpha_0\sin\bar\alpha(c_{L_T}-1),
$$

$$
k_z=
\sin\alpha_0\cos\bar\alpha(1-c_{D_T})
+\cos\alpha_0\sin\bar\alpha(c_{L_T}-1).
$$

令

$$
\Delta T=T_1-T_2.
$$

由横向电机臂产生的 yaw 力矩和旋翼反扭矩得到

$$
M_z=
\left[
l_{T_y}k_x
-\sin\alpha_T\frac{c_\mu}{c_T}
\right]\Delta T.
$$

因此分母

$$
d_{\Delta T}
=l_{T_y}k_x-\sin\alpha_T\frac{c_\mu}{c_T}
$$

若接近 0，差分推力不能由目标 yaw 力矩唯一确定，函数返回
<code>differential-thrust denominator is singular</code>。否则

$$
\boxed{
\Delta T=\frac{M_z}{d_{\Delta T}}
},
$$

$$
T_1=\frac{T+\Delta T}{2},
\qquad
T_2=\frac{T-\Delta T}{2}.
$$

任一 $T_i<0$ 都说明该姿态和目标力矩需要电机“负推力”，函数返回
<code>negative individual motor thrust</code>。合法时由

$$
T_i=c_T\omega_i^2
$$

得到未经限幅的转速

$$
\boxed{
\omega_i=\sqrt{\frac{T_i}{c_T}}
}.
$$

这里没有检查 $\omega_i\le\omega_{\max}$；该限制由
<code>FeasibilityChecker</code> 处理。

### 15.16 推力力矩、旋翼反扭矩与剩余舵面力矩

代码把电机推力臂产生的力矩写成

$$
\mathbf M_T=
\begin{bmatrix}
l_{T_y}k_z(T_2-T_1)\\
c_{\mu_T}T\\
l_{T_y}k_x\Delta T
\end{bmatrix}.
$$

旋翼反扭矩贡献为

$$
\mathbf M_\mu=
\begin{bmatrix}
\cos\alpha_T(c_\mu/c_T)\Delta T\\
0\\
-\sin\alpha_T(c_\mu/c_T)\Delta T
\end{bmatrix}.
$$

可以直接验证其 $z$ 分量之和为

$$
M_{T,z}+M_{\mu,z}
=
\left[
l_{T_y}k_x-\sin\alpha_T(c_\mu/c_T)
\right]\Delta T=M_z.
$$

舵面需要补偿的剩余力矩为

$$
\boxed{
\mathbf M_\delta
=\mathbf M-\mathbf M_T-\mathbf M_\mu
}.
$$

### 15.17 舵面效率与二维分配

代码重新用普通矩阵计算选定姿态的世界系到机体系变换：

$$
\mathbf v^b=R_i^b\mathbf v.
$$

再把速度投影到零升力迎角对应的轴：

$$
v_x^\alpha
=\cos\alpha_0\,v_x^b+\sin\alpha_0\,v_z^b.
$$

这里使用的速度大小是未经正则化的

$$
V=\|\mathbf v\|,
$$

而不是姿态求解阶段的 $V_\epsilon$。每个舵面的局部效率系数为

$$
\boxed{
\nu_i
=
-c_{L_T}^{\delta}\cos\bar\alpha\,T_i
-c_{L_V}^{\delta}Vv_x^\alpha
}.
$$

二维分配矩阵为

$$
A_\delta=
\begin{bmatrix}
-l_{\delta y}\cos\alpha_0\,\nu_1&
l_{\delta y}\cos\alpha_0\,\nu_2\\
l_{\delta x}\nu_1&
l_{\delta x}\nu_2
\end{bmatrix}.
$$

代码只用剩余 roll、pitch 力矩：

$$
\boxed{
A_\delta
\begin{bmatrix}\delta_1\\\delta_2\end{bmatrix}
=
\begin{bmatrix}M_{\delta x}\\M_{\delta y}\end{bmatrix}
}.
$$

先通过 SVD 计算

$$
\sigma_{\min}(A_\delta).
$$

若

$$
\sigma_{\min}(A_\delta)<\epsilon_{\mathrm{sing}},
$$

分配问题病态或不可逆，函数返回
<code>flap allocation matrix is singular</code>。否则使用 full-pivot LU 求解

$$
\begin{bmatrix}\delta_1\\\delta_2\end{bmatrix}
=A_\delta^{-1}
\begin{bmatrix}M_{\delta x}\\M_{\delta y}\end{bmatrix}.
$$

求出的舵角同样未经限幅。当前模型没有用舵面补偿
$M_{\delta z}$，这是第 7.7 节所述的论文近似。

### 15.18 有效性检查与所有提前返回路径

<code>forward()</code> 默认创建 <code>valid=false</code> 的输出。只有完整走到
最后且 <code>allFinite()</code> 通过，才设置

$$
\texttt{valid}=\texttt{true}.
$$

提前返回顺序如下：

1. $|d_\eta|<\epsilon_{\mathrm{sing}}$：总推力分母奇异；
2. 四个姿态分支都没有有限非负总推力：无合法姿态分支；
3. $|d_{\Delta T}|<\epsilon_{\mathrm{sing}}$：差分推力分母奇异；
4. $T_1<0$ 或 $T_2<0$：单电机需要负推力；
5. $\sigma_{\min}(A_\delta)<\epsilon_{\mathrm{sing}}$：舵面矩阵奇异；
6. 最终任一主要输出为 NaN 或 Inf：非有限开环输出。

转速或舵角超过幅值限制不会使这里的 <code>valid</code> 变成 false。设计上将
“公式是否可求”与“执行器是否在范围内”分离，后者由下一层稠密轨迹检查完成。

### 15.19 为什么所需最高导数是 Snap 和 yaw acceleration

导数阶次可以沿调用链追踪：

$$
\mathbf p
\xrightarrow{2\text{ 阶}}
\mathbf a
\longrightarrow
\mathbf f
\longrightarrow
(\phi,\theta).
$$

要得到角速度，需要姿态角一阶导数，因此需要

$$
\dot{\mathbf f}=m\mathbf j.
$$

要得到角加速度和刚体力矩，需要姿态角二阶导数，因此需要

$$
\ddot{\mathbf f}=m\mathbf s.
$$

同理，yaw 对角速度贡献需要 $\dot\psi$，对角加速度贡献需要
$\ddot\psi$。所以当前映射的最小输入阶次是：

$$
\boxed{
\text{位置到 Snap，yaw 到 yaw acceleration}
}.
$$

Crackle 和 yaw Jerk 不参与当前时刻的执行器**值**恢复；阶段 B 对分段时间求导
时，移动积分节点需要它们组成 $\dot{\boldsymbol\xi}$。如果以后继续计算力矩
变化率、执行器速率前馈或更高阶动态，也会再次需要这两项。

### 15.20 <code>Jet2</code> 的数值边界与阅读要点

使用该实现时应特别注意：

- <code>inverse()</code> 和普通除法不检查分母，调用处必须提前做奇异性检查；
- <code>sqrt()</code> 不检查定义域，调用处必须保证输入非负；
- <code>atan2()</code> 在原点附近冻结导数，这保证数值有限，但会牺牲该处
  导数的物理解释；
- 四元数只用于选择离散姿态分支，姿态导数来自连续的 roll、pitch、yaw jet，
  不是对四元数做有限差分；
- <code>Jet2</code> 本身只传播时间导数。阶段 B 另外使用
  <code>DifferentiableJet2&lt;ADScalar&gt;</code> 和第 14.16 节的样条参数梯度，
  把固定分支执行器罚函数传播到航点与分段时间，详见第 17 章；
- 当前只保留推力 jet 的值。如果以后加入电机转速变化率约束，可以继续使用
  已经计算出的 $\dot T$，但还需谨慎处理 $T_i$ 的差分力矩导数。

从代码阅读角度，最关键的对应关系是：

| 代码对象 | 数学含义 |
| --- | --- |
| <code>velocity</code> | $(\mathbf v,\mathbf a,\mathbf j)$ |
| <code>force</code> | $(m(\mathbf a-g\mathbf e_z),m\mathbf j,m\mathbf s)$ |
| <code>yaw</code> | $(\psi,\dot\psi,\ddot\psi)$ |
| <code>phi</code> | $(\phi,\dot\phi,\ddot\phi)$ |
| <code>theta_bar</code> | $(\bar\theta,\dot{\bar\theta},\ddot{\bar\theta})$ |
| <code>theta</code> | $(\theta,\dot\theta,\ddot\theta)$ |
| <code>yaw_rate</code> | $(\dot\psi,\ddot\psi,0)$ |
| <code>phi_rate</code> | $(\dot\phi,\ddot\phi,0)$ |
| <code>theta_rate</code> | $(\dot\theta,\ddot\theta,0)$ |
| <code>omega.value</code> | $\boldsymbol\omega$ |
| <code>omega.first</code> | $\dot{\boldsymbol\omega}$ |

掌握这张对应表后，<code>forward()</code> 中看似复杂的嵌套旋转和气动表达式
可以统一理解为：先用普通物理公式写出“值”，再由 <code>Jet2</code> 的运算符
自动、同步地携带该公式的一阶和二阶时间导数。

## 16. 借鉴 `paper_manifold` 规划并使用 `paper_control` 闭环跟踪

### 16.1 总体结论

可以借鉴 `paper_manifold` 的轨迹优化框架，并使用 `paper_control` 的
PD+INDI 控制器跟踪规划结果。这也是 `paper_planning` 和 `paper_control`
原本设想的互补关系：前者生成满足标称动力学和执行器限制的参考轨迹，后者
根据实时状态和惯性测量计算闭环执行器命令。

但是，不能把 `paper_manifold` 的位置-only平坦输出、四旋翼尾座式动力学和
仅针对总推力/机体角速度的约束原样移植到当前双电机、双舵面飞翼构型。正确的
组合方式是：

1. 复用 `paper_manifold` 的 MINCO、多项式参数化、时间优化、约束罚函数和
   优化后稠密验证思想；
2. 保留 `paper_planning` 针对当前构型推导的四维平坦输出和完整执行器映射；
3. 继续由 `paper_control` 根据参考轨迹和当前状态计算实际闭环命令。

### 16.2 三个层级的变量不能混为一组

当前系统应同时保留三组变量，它们对应不同的模型边界，并不互相矛盾。

| 层级 | 状态或输出 | 输入 | 用途 |
| --- | --- | --- | --- |
| 完整飞行器 | $\mathbf x_f=(\mathbf p,\mathbf v,\mathbf R,\boldsymbol\Omega)$ | $\mathbf u_a=(T_1,T_2,\delta_1,\delta_2)$ | 真实动力学、仿真和执行器可行性 |
| 高层约化模型 | $\mathbf x_h=(\mathbf p,\mathbf v,\mathbf R)$ | $\mathbf u_h=(T,\boldsymbol\Omega)$ | 表达规划参考和高层运动能力 |
| 平坦输出空间 | $\boldsymbol\sigma=(\mathbf p,\psi)$ | 不把输入设为独立决策变量 | 实际轨迹优化 |

其中 $\boldsymbol\Omega$ 表示机体角速度，$\omega_1,\omega_2$ 表示两个电机的
转速。两者必须在变量名和日志中保持区分。

完整刚体模型中的 $\boldsymbol\Omega$ 是状态，力矩是其动力学输入；当内层角速度
控制足够快时，可以把角速度动力学划到低层闭环中，于是
$\boldsymbol\Omega_{\mathrm{cmd}}$ 可以作为高层约化模型的虚拟输入。因此可以采用

$$
\mathbf x_h=(\mathbf p,\mathbf v,\mathbf R),\qquad
\mathbf u_h=(T,\boldsymbol\Omega),
$$

但它只是一套高层变量，不能取代用于最终可行性检查的双电机和双舵面输入。

### 16.3 为什么不能照搬 `paper_manifold` 的位置-only平坦输出

`paper_manifold` 施加协调飞行条件，利用零侧滑约束消去一个姿态自由度，因此
只选择三维位置作为平坦输出。在该条件下，位置及其导数可以确定参考姿态和输入。

当前构型和 `paper_control` 则明确支持：

- 非协调和带侧滑飞行；
- knife-edge flight；
- 差分推力快速转向；
- 独立指定 yaw 的滚转、倒飞和过渡机动。

这些机动要求 yaw 保持为独立规划自由度。因此当前规划器必须继续使用

$$
\boxed{
\boldsymbol\sigma(t)=
\begin{bmatrix}
\mathbf p(t)\\
\psi(t)
\end{bmatrix}
}
$$

而不是仅优化 $\mathbf p(t)$。如果强制使用位置-only映射，就等价于重新引入某种
协调飞行姿态选择规则，会失去 `paper_control` 最有价值的非协调机动能力。

### 16.4 推荐的联合优化形式

可以采用与 `paper_manifold` 相似的时空联合优化结构：

$$
\begin{aligned}
\min_{\boldsymbol\sigma(t),T_f}\quad
&\int_0^{T_f}\left(
\|\mathbf p^{(4)}(t)\|^2
+\lambda_\psi\ddot\psi^2(t)
+\lambda_u P(\mathbf u_a(t))
\right)dt+\rho T_f,\\
\text{s.t.}\quad
&\text{航点、边界导数、安全走廊、状态和输入约束。}
\end{aligned}
$$

优化变量仍然是位置/yaw样条的内部参数、辅助航点和分段时间。状态和输入不作为
独立决策变量，而是通过当前构型的平坦映射逐级恢复：

$$
\begin{aligned}
(\mathbf R,T)
&=\mathcal F_2(\mathbf p,\dot{\mathbf p},\ddot{\mathbf p},\psi),\\
\boldsymbol\Omega
&=\mathcal F_3(\mathbf p^{(0:3)},\psi,\dot\psi),\\
(\dot{\boldsymbol\Omega},\mathbf m,T_1,T_2,\delta_1,\delta_2)
&=\mathcal F_4(\mathbf p^{(0:4)},\psi,\dot\psi,\ddot\psi).
\end{aligned}
$$

这样，平移运动学、平移动力学、姿态运动学和转动动力学都通过平坦映射按构造
满足，不需要在优化变量中再次对完整十二维动力学执行数值积分。

### 16.5 高层约束与真实执行器约束应同时保留

高层约化变量可以加入以下约束：

$$
\|\mathbf v\|\leq v_{\max},\qquad
\|\boldsymbol\Omega\|\leq\Omega_{\max},\qquad
T_{\min}\leq T\leq T_{\max}.
$$

这些约束限制参考轨迹的速度、姿态变化速度和总推力需求，适合作为快速、平滑的
中间层筛选。但它们不能保证控制分配一定可行。例如，即使
$\|\boldsymbol\Omega\|$ 很小，快速变化的角速度仍可能需要很大的
$\dot{\boldsymbol\Omega}$ 和控制力矩。

因此最终还必须使用 `paper_planning` 的完整开环映射约束：

$$
\begin{aligned}
T_{i,\min}&\leq T_i\leq T_{i,\max},\\
|\delta_i|&\leq\delta_{\max},\qquad i\in\{1,2\}.
\end{aligned}
$$

在现有幅值约束之上，建议进一步加入：

$$
|\dot T_i|\leq\dot T_{\max},\qquad
|\dot\delta_i|\leq\dot\delta_{\max},
$$

或显式加入电机和舵机的一阶动态。幅值可行并不代表执行器能够以所需速度变化，
这正是第 12.4 节所述的当前缺口。

因此，不应把当前单电机、单舵面约束降级为仅约束 $(T,\boldsymbol\Omega)$；推荐
采用

$$
\boxed{
(T,\boldsymbol\Omega)\text{ 高层约束}
+(T_1,T_2,\delta_1,\delta_2)\text{ 执行器约束}
}.
$$

### 16.6 如何理解 `paper_planning` 的开环可行性

`paper_planning` 的开环约束是标称轨迹规划的正常组成部分，并不意味着最终以
开环方式飞行。它回答的问题是：是否存在一组不越界的理想执行器输入，使标称
模型精确沿参考轨迹运动。

`paper_manifold` 的规划约束本质上同样是开环、标称和模型驱动的；它在规划阶段
检查参考总推力和机体角速度，并没有把闭环 MPC 和真实飞行器完整地嵌入轨迹
优化。对于当前构型，`paper_planning` 从 Snap 和 yaw acceleration 继续计算
$T_1,T_2,\delta_1,\delta_2$，实际上比只约束总推力和机体角速度更接近真实
执行器边界。

开环映射仍存在以下模型误差：

- 忽略舵面直接力引起的轻微非最小相位效应；
- 忽略部分自由来流和角速度相关力矩；
- 未完整包含电机、ESC 和舵机动态；
- 未包含估计延迟、传感器噪声、风和闭环控制瞬态。

这些误差不应通过删除开环执行器约束来解决，而应通过执行器裕度、闭环验证和
更高保真度评估逐步补偿。

### 16.7 与 `paper_control` 的正确数据接口

规划器预测得到的

$$
T_1^{\mathrm{ff}},T_2^{\mathrm{ff}},
\delta_1^{\mathrm{ff}},\delta_2^{\mathrm{ff}}
$$

只用于可行性检查、代价计算和结果诊断，不应直接发送到当前仿真或真实执行器。
保持 `paper_control` 原有闭环结构时，规划器应输出：

$$
\boxed{
\mathbf p_{\mathrm{ref}},
\mathbf v_{\mathrm{ref}},
\mathbf a_{\mathrm{ref}},
\mathbf j_{\mathrm{ref}},
\psi_{\mathrm{ref}},
\dot\psi_{\mathrm{ref}}
}.
$$

`paper_control` 随后完成：

1. 位置/速度 PD 产生线加速度命令；
2. 线加速度 INDI 根据当前加速度和力估计求姿态、总推力命令；
3. Jerk 和 yaw rate 通过平坦映射生成 $\boldsymbol\Omega_{\mathrm{ref}}$；
4. 姿态/角速度 PD 产生角加速度命令；
5. 角加速度 INDI 产生力矩命令；
6. 在线控制分配器计算双电机和双舵面命令并执行限幅。

规划阶段恢复出的 $\mathbf R,T,\boldsymbol\Omega$ 可以用于参考轨迹可视化、约束
检查和闭环结果对比，但不应绕过 INDI 的传感器反馈修正。

### 16.8 推荐的多保真可行性流程

如果希望约束比单纯开环检查更接近实际闭环能力，推荐使用以下两级流程：

```text
位置/yaw 航点和边界条件
        │
        ▼
MINCO/SplineTrajectory 优化 p、psi、辅助点和分段时间
        │
        ▼
当前构型平坦映射
        │
        ├── 高层参考：R、T、机体角速度
        └── 原始执行器预测：T1、T2、delta1、delta2
        │
        ▼
带裕度的约束罚函数和高密度开环认证
        │
        ▼
paper_control + 执行器动态的闭环仿真
        │
        ▼
检查跟踪误差、饱和持续时间和最小执行器裕度
        │
        ▼
失败时调整局部分段时间/辅助点并重新优化
```

第一层是可解析或可自动微分的快速内层优化；第二层是包含控制器、执行器动态、
状态估计和扰动的高保真闭环评估。若暂时无法对闭环仿真求梯度，可以把以下指标
作为低维外层黑盒目标：

$$
J_{\mathrm{closed}}=
w_p\max_t\|\mathbf p_{\mathrm{ref}}-\mathbf p\|
+w_\psi\max_t|\psi_{\mathrm{ref}}-\psi|
+w_s t_{\mathrm{saturation}}.
$$

执行器软约束建议预留约 $10\%\sim25\%$ 的闭环裕度，但最终比例必须通过当前
平台的闭环仿真或实验标定，而不能直接照搬其他论文的数值。

### 16.9 对当前 `planning/` 实现的具体判断

当前阶段 A/B 已经采用了正确的基本分层：

- 位置使用七次 minimum-snap 样条；
- yaw 使用三次 minimum-yaw-acceleration 样条；
- 二者共享分段时间；
- 通过独立 `paper_planning` 平坦映射得到未经限幅的执行器预测；
- 阶段 B 已将执行器违反量作为采样积分罚函数；
- 最后通过更密的采样执行开环可行性检查。

因此不应把现有执行器约束替换成 `paper_manifold` 的
$(T,\boldsymbol\Omega)$ 约束，而应按以下顺序增强。当前进度为：

1. **已完成**：在现有执行器约束之外加入机体角速度高层约束；
2. **已完成**：加入电机/舵机一阶执行器动态和舵面角速度约束；
3. 把规划 CSV/样条接口接入当前 `paper_control` 闭环仿真；
4. 记录闭环跟踪误差、实际饱和量、饱和持续时间和控制裕度；
5. 使用闭环结果标定 `actuator_margin`，必要时构造外层黑盒时间/航点优化；
6. 最终保留“开环平坦映射认证 + 闭环仿真认证”两道检查。

最终推荐架构可概括为：

$$
\boxed{
\text{借鉴 paper\_manifold 的优化框架}
+\text{保留 paper\_planning 的构型平坦映射与执行器约束}
+\text{使用 paper\_control 执行闭环跟踪}
}.
$$

---

## 17. Stage B 执行器约束梯度与 L-BFGS

本章记录执行器约束梯度按照“局部映射、罚函数、样条、优化变量、验证、求解器”
的顺序完成后的实现。最终结果不是对四个执行器做数值差分，而是在有效姿态分支
内得到解析/自动微分梯度，并只用有限差分做独立回归验证。

### 17.1 第一步：固定姿态分支的执行器雅可比

#### 17.1.1 单点输入、输出和 Jacobian 的列顺序

单点平坦映射的独立变量按代码中的固定顺序组成 15 维向量：

$$
\boldsymbol\xi=
\begin{bmatrix}
\mathbf v^\top&\mathbf a^\top&\mathbf j^\top&\mathbf s^\top&
\psi&\dot\psi&\ddot\psi
\end{bmatrix}^\top.
$$

其中 $\mathbf s=\mathbf p^{(4)}$ 是 Snap。位置 $\mathbf p$ 本身没有出现在
$\boldsymbol\xi$ 中，因为当前无风、空间均匀的开环模型具有平移不变性：把整条
轨迹平移一个常向量不会改变姿态、力矩或执行器输入。

执行器输出选择为

$$
\mathbf u_a=
\begin{bmatrix}T_1&T_2&\delta_1&\delta_2\end{bmatrix}^\top,
$$

因此 `ActuatorLinearization` 保存

$$
\boxed{
\mathbf J_{u\xi}=\frac{\partial\mathbf u_a}{\partial\boldsymbol\xi}
\in\mathbb R^{4\times15}
}.
$$

矩阵的第 $r$ 行是第 $r$ 个执行器对 15 个平坦变量的偏导，第 $c$ 列是四个
执行器对第 $c$ 个平坦变量的偏导。对任意小扰动
$\mathrm d\boldsymbol\xi$，一阶近似为

$$
\boxed{
\mathrm d\mathbf u_a
=\mathbf J_{u\xi}\,\mathrm d\boldsymbol\xi
}.
$$

这里对单电机推力而不是转速求导。转速满足
$\omega_i=\sqrt{T_i/c_T}$，若一定需要转速梯度，则在 $T_i>0$ 时可由

$$
\frac{\partial\omega_i}{\partial\boldsymbol\xi}
=\frac{1}{2\sqrt{c_TT_i}}
\frac{\partial T_i}{\partial\boldsymbol\xi}
$$

得到。但该系数在 $T_i\rightarrow0^+$ 时发散；直接对推力施加由转速范围换算
得到的等价边界，可以避免把这个病态因子带进优化器。

#### 17.1.2 为什么必须先固定姿态分支

令 roll/pitch 分支编号为

$$
\mathbf b=(b_\phi,b_\theta),\qquad b_\phi,b_\theta\in\{0,1\}.
$$

对固定 $\mathbf b$，连续平坦映射可以写成

$$
\mathbf u_a=\mathcal F_{\mathbf b}(\boldsymbol\xi).
$$

实际 `forward()` 还要枚举候选、删除负总推力候选，并通过四元数距离

$$
b^*(\boldsymbol\xi)
=\arg\min_{\mathbf b}\left(1-|q_{\mathrm{prev}}^Tq_{\mathbf b}|\right)
$$

选择与上一采样姿态连续的分支。完整映射
$\mathcal F_{b^*(\boldsymbol\xi)}(\boldsymbol\xi)$ 只在分支不变的邻域内光滑；
`argmin` 切换面上不存在普通意义下的唯一导数。

因此 `linearize()` 采用以下定义：先用普通 `forward()` 得到标称分支
$\mathbf b^*$，再计算

$$
\boxed{
\mathbf J_{u\xi}
=\left.
\frac{\partial\mathcal F_{\mathbf b}(\boldsymbol\xi)}
{\partial\boldsymbol\xi}
\right|_{\mathbf b=\mathbf b^*}
}.
$$

这不是忽略一个本来存在的连续导数，而是在分段光滑问题中明确选择当前光滑片
的 Jacobian。有限差分测试也固定上一姿态，并拒绝扰动后发生分支变化的样本，
保证比较的是同一个数学对象。

#### 17.1.3 外层参数微分与内层时间 Jet

原来的 `Jet2` 保存

$$
\mathcal J_2[x]=(x,\dot x,\ddot x),
$$

用于沿物理时间恢复 $\dot\phi,\ddot\phi,\boldsymbol\Omega$ 和
$\dot{\boldsymbol\Omega}$。它不是优化变量梯度。新增实现把每个 Jet 分量的标量
类型换成固定长度 AutoDiff 标量：

$$
\widehat x=(x,\nabla_{\boldsymbol\xi}x),
$$

$$
\mathcal J_2[\widehat x]
=(\widehat x,\widehat{\dot x},\widehat{\ddot x}).
$$

第 $l$ 个独立变量的种子是

$$
\widehat\xi_l=(\xi_l,\mathbf e_l),
$$

其中 $\mathbf e_l\in\mathbb R^{15}$ 是第 $l$ 个标准基向量。这样一次前向计算后，
每个输出标量已经携带完整的 15 维梯度。

外层微分使用普通全微分规则，例如

$$
\begin{aligned}
\mathrm d(xy)&=y\,\mathrm dx+x\,\mathrm dy,\\
\mathrm d(1/x)&=-\frac{\mathrm dx}{x^2},\\
\mathrm d(\sin x)&=\cos x\,\mathrm dx,\\
\mathrm d(\cos x)&=-\sin x\,\mathrm dx,\\
\mathrm d\sqrt{x}&=\frac{\mathrm dx}{2\sqrt{x}},\\
\mathrm d\operatorname{atan2}(y,x)
&=\frac{x\,\mathrm dy-y\,\mathrm dx}{x^2+y^2}.
\end{aligned}
$$

对旋转后的向量则有

$$
\mathrm d\!\left(R(\gamma)\mathbf z\right)
=R(\gamma)\,\mathrm d\mathbf z
+\frac{\partial R}{\partial\gamma}\mathbf z\,\mathrm d\gamma.
$$

`DifferentiableJet2<ADScalar>` 同时执行内层时间导数乘法规则和上述外层参数链式
法则，所以不需要手写包含二阶时间导数的巨大 $4\times15$ 展开式。

#### 17.1.4 从平坦变量到姿态和总推力的局部微分

固定分支后，输入 Jet 为

$$
\mathcal J_2[\mathbf v]=(\mathbf v,\mathbf a,\mathbf j),
$$

$$
\mathcal J_2[\mathbf f]
=\left(m(\mathbf a-g\mathbf e_z),m\mathbf j,m\mathbf s\right),
\qquad
\mathcal J_2[\psi]=(\psi,\dot\psi,\ddot\psi).
$$

对外层扰动，参考力首先满足

$$
\mathrm d\mathbf f=m\,\mathrm d\mathbf a,\qquad
\mathrm d\dot{\mathbf f}=m\,\mathrm d\mathbf j,\qquad
\mathrm d\ddot{\mathbf f}=m\,\mathrm d\mathbf s.
$$

令 $\mathbf f^\psi=R_z(\psi)\mathbf f$，则

$$
\mathrm d\mathbf f^\psi
=R_z(\psi)\mathrm d\mathbf f
+R_z'(\psi)\mathbf f\,\mathrm d\psi.
$$

基本 roll 为 $\phi_0=-\operatorname{atan2}(f_y^\psi,f_z^\psi)$，所以

$$
\boxed{
\mathrm d\phi
=-\frac{
f_z^\psi\,\mathrm df_y^\psi
-f_y^\psi\,\mathrm df_z^\psi
}{(f_y^\psi)^2+(f_z^\psi)^2}
}.
$$

分支项 $b_\phi\pi$ 是常数，因此
$\mathrm d(\phi_0+b_\phi\pi)=\mathrm d\phi_0$。正则化速度的微分为

$$
V_\epsilon=\sqrt{\mathbf v^T\mathbf v+\epsilon_v^2},\qquad
\boxed{
\mathrm dV_\epsilon=\frac{\mathbf v^T\mathrm d\mathbf v}{V_\epsilon}
}.
$$

继续按第 15.11 节定义 $\sigma_y,\sigma_x$，则固定 pitch 分支满足

$$
\bar\theta=\operatorname{atan2}(\sigma_y,\sigma_x)+b_\theta\pi,
$$

$$
\boxed{
\mathrm d\bar\theta
=\frac{
\sigma_x\,\mathrm d\sigma_y
-\sigma_y\,\mathrm d\sigma_x
}{\sigma_x^2+\sigma_y^2}
}.
$$

$\mathrm d\sigma_x$ 和 $\mathrm d\sigma_y$ 由乘积法则得到。例如

$$
\mathrm d(V_\epsilon v_x^\phi)
=v_x^\phi\,\mathrm dV_\epsilon
+V_\epsilon\,\mathrm dv_x^\phi,
$$

而 $\mathrm d\mathbf v^\phi$ 同时包含
$\mathrm d\mathbf v,\mathrm d\psi,\mathrm d\phi$ 三条路径。

令总推力公式的分子为

$$
N_T=
\cos\bar\theta f_x^\phi-
\sin\bar\theta f_z^\phi+
c_{D_V}V_\epsilon
\left(\cos\bar\theta v_x^\phi-\sin\bar\theta v_z^\phi\right),
$$

分母 $d_\eta$ 只由固定车辆参数组成，因此

$$
\boxed{
\mathrm dT=\frac{\mathrm dN_T}{d_\eta}
}.
$$

例如其中第一项的微分为

$$
\mathrm d(\cos\bar\theta f_x^\phi)
=-\sin\bar\theta f_x^\phi\,\mathrm d\bar\theta
+\cos\bar\theta\,\mathrm df_x^\phi,
$$

其余项完全由相同规则得到。实际代码没有遗漏这些交叉项，因为它们由 AutoDiff
在固定分支连续表达式上统一生成。

#### 17.1.5 从角速度、刚体力矩到双电机推力

由第 15.13 节的 Euler 角速率关系，可写成

$$
\boldsymbol\Omega
=\mathcal W(\phi,\theta,
\dot\phi,\dot\theta,\dot\psi).
$$

因此

$$
\mathrm d\boldsymbol\Omega
=\mathcal W_\phi\mathrm d\phi
+\mathcal W_\theta\mathrm d\theta
+\mathcal W_{\dot\phi}\mathrm d\dot\phi
+\mathcal W_{\dot\theta}\mathrm d\dot\theta
+\mathcal W_{\dot\psi}\mathrm d\dot\psi.
$$

$\mathrm d\dot{\boldsymbol\Omega}$ 同理由时间 Jet 的 `first` 分量自动得到。
刚体力矩

$$
\mathbf M=I\dot{\boldsymbol\Omega}
+\boldsymbol\Omega\times(I\boldsymbol\Omega)
$$

的精确一阶微分为

$$
\boxed{
\mathrm d\mathbf M
=I\,\mathrm d\dot{\boldsymbol\Omega}
+\mathrm d\boldsymbol\Omega\times(I\boldsymbol\Omega)
+\boldsymbol\Omega\times(I\,\mathrm d\boldsymbol\Omega)
}.
$$

差分推力分母 $d_{\Delta T}$ 也是固定车辆参数，所以

$$
\Delta T=\frac{M_z}{d_{\Delta T}},\qquad
\mathrm d\Delta T=\frac{\mathrm dM_z}{d_{\Delta T}}.
$$

于是双电机推力的两行 Jacobian 来自

$$
\boxed{
\mathrm dT_1=\frac12(\mathrm dT+\mathrm d\Delta T),\qquad
\mathrm dT_2=\frac12(\mathrm dT-\mathrm d\Delta T)
}.
$$

#### 17.1.6 舵面分配的矩阵微分

剩余舵面力矩为

$$
\mathbf M_\delta=\mathbf M-\mathbf M_T-\mathbf M_\mu,
$$

所以

$$
\mathrm d\mathbf M_\delta
=\mathrm d\mathbf M-\mathrm d\mathbf M_T-\mathrm d\mathbf M_\mu.
$$

对 $V=\|\mathbf v\|>0$，未正则化速度模和机体系速度的微分为

$$
\mathrm dV=\frac{\mathbf v^T\mathrm d\mathbf v}{V},
$$

$$
\mathrm d\mathbf v^b
=R_i^b\mathrm d\mathbf v+(\mathrm dR_i^b)\mathbf v.
$$

因此

$$
\mathrm dv_x^\alpha
=\cos\alpha_0\,\mathrm dv_x^b
+\sin\alpha_0\,\mathrm dv_z^b,
$$

$$
\boxed{
\mathrm d\nu_i
=-c_{L_T}^{\delta}\cos\bar\alpha\,\mathrm dT_i
-c_{L_V}^{\delta}
\left(v_x^\alpha\mathrm dV+V\mathrm dv_x^\alpha\right)
}.
$$

分配矩阵 $A_\delta$ 的每个元素都与 $\nu_1$ 或 $\nu_2$ 线性相关，因此
$\mathrm dA_\delta$ 可直接由 $\mathrm d\nu_i$ 得到。记

$$
\boldsymbol\delta=
\begin{bmatrix}\delta_1&\delta_2\end{bmatrix}^\top,\qquad
\mathbf m_\delta=
\begin{bmatrix}M_{\delta x}&M_{\delta y}\end{bmatrix}^\top,
$$

原分配方程为

$$
A_\delta\boldsymbol\delta=\mathbf m_\delta.
$$

两边微分：

$$
(\mathrm dA_\delta)\boldsymbol\delta
+A_\delta\mathrm d\boldsymbol\delta
=\mathrm d\mathbf m_\delta.
$$

因此舵面 Jacobian 最关键的公式是

$$
\boxed{
\mathrm d\boldsymbol\delta
=A_\delta^{-1}
\left[
\mathrm d\mathbf m_\delta
-(\mathrm dA_\delta)\boldsymbol\delta
\right]
}.
$$

它说明舵角梯度包含两部分：剩余目标力矩的变化，以及气动效率/分配矩阵本身的
变化。只对右端力矩求导而把 $A_\delta$ 当常数，会漏掉第二部分。当前显式
$2\times2$ 逆矩阵在 AutoDiff 标量上运算，因此两部分都会进入
$\partial\delta_i/\partial\boldsymbol\xi$。当 $V=0$ 时普通速度模不可微，代码
选择零子梯度；当分配行列式接近零或标称映射无效时，不返回可微 Jacobian。

### 17.2 第二步：从执行器雅可比得到罚函数梯度

#### 17.2.1 平方铰链的导数

定义

$$
[g]_+=\max(0,g),\qquad P(g)=[g]_+^2.
$$

则

$$
P'(g)=
\begin{cases}
0,&g\le0,\\
2g,&g>0.
\end{cases}
$$

在 $g=0$ 处左右一阶导数都为 0，所以 $P$ 是 $C^1$ 但不是 $C^2$。L-BFGS 只
使用一阶梯度，因此软阈值本身不会产生梯度跳变。

#### 17.2.2 单电机推力罚函数

为避免与差分推力 $\Delta T$ 混淆，记推力允许区间宽度为

$$
S_T=T_{\max}^{\mathrm{plan}}-T_{\min}>0.
$$

对第 $i$ 个电机定义

$$
g_{i,L}=\frac{T_{\min}-T_i}{S_T},\qquad
g_{i,U}=\frac{T_i-T_{\max}^{\mathrm{plan}}}{S_T},
$$

$$
p_{T_i}=P(g_{i,L})+P(g_{i,U}).
$$

由于

$$
\frac{\partial g_{i,L}}{\partial T_i}=-\frac1{S_T},\qquad
\frac{\partial g_{i,U}}{\partial T_i}=\frac1{S_T},
$$

得到代码中 `thrust_penalty.derivative` 对应的显式公式

$$
\boxed{
\frac{\partial p_{T_i}}{\partial T_i}
=-\frac{2[g_{i,L}]_+}{S_T}
+\frac{2[g_{i,U}]_+}{S_T}
}.
$$

因此低于下限时梯度为负，梯度下降会增大 $T_i$；高于上限时梯度为正，梯度下降
会减小 $T_i$，方向与物理直觉一致。

#### 17.2.3 单舵面罚函数

记 $\delta_p=\delta_{\max}^{\mathrm{plan}}>0$，定义

$$
g_{\delta_i}=\frac{|\delta_i|-\delta_p}{\delta_p},\qquad
p_{\delta_i}=P(g_{\delta_i}).
$$

在真正越界的区域 $|\delta_i|>\delta_p$ 内必有 $\delta_i\ne0$，所以

$$
\boxed{
\frac{\partial p_{\delta_i}}{\partial\delta_i}
=\frac{2[g_{\delta_i}]_+}{\delta_p}
\operatorname{sign}(\delta_i)
}.
$$

在允许区间内 $[g_{\delta_i}]_+=0$，导数直接取 0，因此不会在
$\delta_i=0$ 使用 $|\delta_i|$ 的不确定导数。

#### 17.2.4 从四执行器梯度回传到 15 维平坦变量

单个采样点的总幅值罚函数为

$$
p=p_{T_1}+p_{T_2}+p_{\delta_1}+p_{\delta_2}+p_{\mathrm{invalid}}.
$$

对有效映射，$p_{\mathrm{invalid}}=0$。把四个连续执行器分量的导数组成

$$
\mathbf g_u=
\frac{\partial p}{\partial\mathbf u_a}
=
\begin{bmatrix}
\partial p_{T_1}/\partial T_1\\
\partial p_{T_2}/\partial T_2\\
\partial p_{\delta_1}/\partial\delta_1\\
\partial p_{\delta_2}/\partial\delta_2
\end{bmatrix},
$$

则由

$$
\mathrm dp=\mathbf g_u^T\mathrm d\mathbf u_a
=\mathbf g_u^T\mathbf J_{u\xi}\mathrm d\boldsymbol\xi
$$

得到单点罚函数对平坦变量的列梯度：

$$
\boxed{
\mathbf g_\xi=\mathbf J_{u\xi}^\top\mathbf g_u
}.
$$

`flat_gradient.segment<3>(...)` 正是在读取 $\mathbf g_\xi$ 中对应
$\mathbf v,\mathbf a,\mathbf j,\mathbf s$ 的四个三维块，最后三项分别对应
$\psi,\dot\psi,\ddot\psi$。

若标称平坦映射无效，`linearize()` 返回 `differentiable=false`。此时该样本的
有限执行器罚值和 $p_{\mathrm{invalid}}$ 都按局部常数屏障处理，代码只保留
“积分区间长度改变”产生的时间导数，不伪造穿过负推力或分配奇异区域的局部
方向。这也是当前方法要求阶段 A 初值位于有效分支邻域的原因。

### 17.3 第三步：传播到样条系数和分段时间

#### 17.3.1 分段多项式与基函数

第 $i$ 段使用局部时间 $t\in[0,T_i]$。位置七次多项式写成

$$
\mathbf p_i(t)=C_i^T\boldsymbol\beta_0(t),\qquad
C_i\in\mathbb R^{8\times3},
$$

其中

$$
\boldsymbol\beta_0(t)=
\begin{bmatrix}1&t&t^2&t^3&t^4&t^5&t^6&t^7\end{bmatrix}^T.
$$

令

$$
\boldsymbol\beta_r(t)=\frac{\mathrm d^r\boldsymbol\beta_0}{\mathrm dt^r},
$$

则

$$
\mathbf p_i^{(r)}(t)=C_i^T\boldsymbol\beta_r(t).
$$

例如代码使用的四组基函数为

$$
\begin{aligned}
\boldsymbol\beta_1^T&=[0,1,2t,3t^2,4t^3,5t^4,6t^5,7t^6],\\
\boldsymbol\beta_2^T&=[0,0,2,6t,12t^2,20t^3,30t^4,42t^5],\\
\boldsymbol\beta_3^T&=[0,0,0,6,24t,60t^2,120t^3,210t^4],\\
\boldsymbol\beta_4^T&=[0,0,0,0,24,120t,360t^2,840t^3].
\end{aligned}
$$

yaw 三次多项式写成

$$
\psi_i(t)=D_i^T\boldsymbol\gamma_0(t),\qquad
D_i\in\mathbb R^{4\times1},\qquad
\boldsymbol\gamma_0=[1,t,t^2,t^3]^T,
$$

并使用

$$
\boldsymbol\gamma_1=[0,1,2t,3t^2]^T,\qquad
\boldsymbol\gamma_2=[0,0,2,6t]^T.
$$

#### 17.3.2 单个积分节点对多项式系数的偏导

把上一节得到的 15 维梯度分块为

$$
\mathbf g_\xi=
\begin{bmatrix}
\mathbf g_v\\\mathbf g_a\\\mathbf g_j\\\mathbf g_s\\
g_\psi\\g_{\dot\psi}\\g_{\ddot\psi}
\end{bmatrix}.
$$

固定采样局部时间时，位置导数的微分为

$$
\mathrm d\mathbf v=(\mathrm dC_i)^T\boldsymbol\beta_1,\quad
\mathrm d\mathbf a=(\mathrm dC_i)^T\boldsymbol\beta_2,\quad
\mathrm d\mathbf j=(\mathrm dC_i)^T\boldsymbol\beta_3,\quad
\mathrm d\mathbf s=(\mathrm dC_i)^T\boldsymbol\beta_4.
$$

利用标量
$\mathbf g_v^T(\mathrm dC_i)^T\boldsymbol\beta_1
=\langle\boldsymbol\beta_1\mathbf g_v^T,\mathrm dC_i\rangle_F$，得到一个权重为
$w_{ik}$ 的节点对位置系数的贡献：

$$
\boxed{
G_{C_i}\mathrel{+}=w_{ik}\left(
\boldsymbol\beta_1\mathbf g_v^T
+\boldsymbol\beta_2\mathbf g_a^T
+\boldsymbol\beta_3\mathbf g_j^T
+\boldsymbol\beta_4\mathbf g_s^T
\right)
}.
$$

这里 $\langle A,B\rangle_F=\operatorname{tr}(A^TB)$ 是 Frobenius 内积，
$G_{C_i}=\partial J_{\mathrm{act}}/\partial C_i\in\mathbb R^{8\times3}$。

同理，yaw 系数梯度为

$$
\boxed{
G_{D_i}\mathrel{+}=w_{ik}\left(
\boldsymbol\gamma_0g_\psi
+\boldsymbol\gamma_1g_{\dot\psi}
+\boldsymbol\gamma_2g_{\ddot\psi}
\right)
}.
$$

这两个公式分别对应代码中的 `position_block.noalias() += ...` 和
`yaw_block.noalias() += ...`。

#### 17.3.3 梯形积分对当前分段时间的显式偏导

每段使用 $N$ 个等长区间和 $N+1$ 个节点。定义

$$
s_k=\frac{k}{N},\qquad t_{ik}=s_kT_i,\qquad
c_k=
\begin{cases}
\frac12,&k=0\ \text{或}\ k=N,\\
1,&\text{其他节点},
\end{cases}
$$

$$
w_{ik}=\frac{T_i}{N}c_k.
$$

于是执行器积分近似为

$$
J_{\mathrm{act}}
=\sum_i\sum_{k=0}^{N}w_{ik}\,p(\boldsymbol\xi_i(t_{ik})).
$$

先固定多项式系数 $C_i,D_i$，只对当前 $T_i$ 求显式偏导。因为

$$
\frac{\partial w_{ik}}{\partial T_i}=\frac{c_k}{N},\qquad
\frac{\partial t_{ik}}{\partial T_i}=s_k,
$$

而

$$
\frac{\mathrm dp}{\mathrm dt}
=\mathbf g_\xi^T\dot{\boldsymbol\xi},
$$

所以单节点满足

$$
\boxed{
\left.
\frac{\partial(w_{ik}p_{ik})}{\partial T_i}
\right|_{C_i,D_i}
=\frac{c_k}{N}p_{ik}
+w_{ik}s_k\mathbf g_\xi^T\dot{\boldsymbol\xi}_{ik}
}.
$$

第一项来自积分步长 $T_i/N$ 的变化，第二项来自节点
$t_{ik}=s_kT_i$ 在段内移动。漏掉第一项会把常值罚函数的积分梯度错误地算成零；
漏掉第二项则会把罚函数峰值在段内移动的影响丢掉。

这里

$$
\boxed{
\dot{\boldsymbol\xi}=
\begin{bmatrix}
\mathbf a\\\mathbf j\\\mathbf s\\\mathbf p^{(5)}\\
\dot\psi\\\ddot\psi\\\psi^{(3)}
\end{bmatrix}
}.
$$

因此需要 Crackle $\mathbf p^{(5)}$ 和 yaw Jerk $\psi^{(3)}$。它们不参与当前
时刻的执行器值 $\mathbf u_a$，但参与“采样时刻随段时间移动”的导数。

前面各段时长变化会同时移动当前段的绝对起点和全局查询时刻，两者相消，当前
多项式仍在相同的局部坐标 $t_{ik}$ 上求值。因此显式项只需要当前段的
$s_k\dot{\boldsymbol\xi}$，不需要额外累加所有前序段起点的导数。

#### 17.3.4 为什么还要调用 `propagateGrad()`

上面的时间公式是在 $C_i,D_i$ 固定时得到的偏导。但 minimum-snap/minimum-yaw
样条的系数不是独立优化变量，而是航点和分段时间的函数：

$$
C=C(P,T),\qquad D=D(\Psi,T).
$$

记所有节点累加后的显式时间偏导为 $\mathbf g_T^{\mathrm{exp}}$，则

$$
\mathrm dJ_{\mathrm{act}}
=\langle G_C,\mathrm dC\rangle_F
+\langle G_D,\mathrm dD\rangle_F
+(\mathbf g_T^{\mathrm{exp}})^T\mathrm d\mathbf T.
$$

代入

$$
\mathrm dC=C_P\mathrm dP+C_T\mathrm d\mathbf T,\qquad
\mathrm dD=D_\Psi\mathrm d\Psi+D_T\mathrm d\mathbf T,
$$

可得总梯度

$$
\boxed{
\begin{aligned}
\mathbf g_{P}^{\mathrm{act}}&=C_P^*[G_C],\\
\mathbf g_{\Psi}^{\mathrm{act}}&=D_\Psi^*[G_D],\\
\mathbf g_T^{\mathrm{act}}
&=\mathbf g_T^{\mathrm{exp}}
+C_T^*[G_C]+D_T^*[G_D].
\end{aligned}
}.
$$

上标 $*$ 表示 Jacobian 的伴随作用，不要求显式构造巨大的 Jacobian 矩阵。
`position_.propagateGrad()` 和 `yaw_.propagateGrad()` 正是在执行这些伴随运算；
第 14.16 节说明了内部节点导数通过
$A^T\boldsymbol\lambda=\mathbf g_X$ 反传的具体公式。这样得到的
$\mathbf g_T^{\mathrm{act}}$ 已经同时包含：

1. 积分步长变化；
2. 采样节点在段内移动；
3. 多项式系数随段时间改变；
4. minimum-snap/minimum-yaw 内部最优导数随航点和时间改变。

只把第 17.3.3 节的显式时间项交给优化器是不完整的。

#### 17.3.5 平滑能量的梯度

平滑项为

$$
J_{\mathrm{smooth}}
=\sum_i\int_0^{T_i}\|\mathbf p_i^{(4)}(t)\|^2\,\mathrm dt
+\lambda_\psi\sum_i\int_0^{T_i}\ddot\psi_i^2(t)\,\mathrm dt.
$$

它不经过执行器映射，直接由两套样条的 `getEnergyGrad()` 得到

$$
\mathbf g_P^{\mathrm{smooth}},\qquad
\mathbf g_\Psi^{\mathrm{smooth}},\qquad
\mathbf g_T^{\mathrm{smooth}}.
$$

这些同样是包含内部最优导数隐式变化的总梯度，不是固定多项式系数下的局部
偏导。

### 17.4 第四步：传播到真正的五个优化变量

#### 17.4.1 先组合物理时间和物理航点梯度

Stage B 的目标重新写为

$$
J_B
=\rho_s\frac{J_{\mathrm{smooth}}}{J_{\mathrm{smooth}}^0}
+\rho_aJ_{\mathrm{act}}
+\rho_g\|\mathbf u\|^2.
$$

$J_{\mathrm{smooth}}^0$ 是优化开始前计算并固定的标量，因此不需要对分母求导。
对物理分段时间和内部位置航点，先得到

$$
\boxed{
\mathbf g_T
=\frac{\rho_s}{J_{\mathrm{smooth}}^0}
\mathbf g_T^{\mathrm{smooth}}
+\rho_a\mathbf g_T^{\mathrm{act}}
},
$$

$$
\boxed{
\mathbf g_P
=\frac{\rho_s}{J_{\mathrm{smooth}}^0}
\mathbf g_P^{\mathrm{smooth}}
+\rho_a\mathbf g_P^{\mathrm{act}}
}.
$$

代码也计算了 yaw 内部航点梯度

$$
\mathbf g_\Psi
=\frac{\rho_s}{J_{\mathrm{smooth}}^0}
\mathbf g_\Psi^{\mathrm{smooth}}
+\rho_a\mathbf g_\Psi^{\mathrm{act}},
$$

但当前 Stage B 的 yaw 航点固定，不进入 NLopt 变量，所以该梯度暂时不写入最终
变量向量。以后若开放辅助 yaw，可直接复用这一结果。

#### 17.4.2 固定总时间 softmax 的完整推导

设共有 $M$ 段，引入 logits $z_i$，其中最后一个固定为

$$
z_M=0,\qquad z_j=\tau_j,\quad j=1,\ldots,M-1.
$$

定义

$$
S=\sum_{r=1}^{M}e^{z_r},\qquad
T_i=\bar T\frac{e^{z_i}}{S}.
$$

对任意自由 $\tau_j$，有

$$
\begin{aligned}
\frac{\partial T_i}{\partial\tau_j}
&=\bar T
\frac{\delta_{ij}e^{z_i}S-e^{z_i}e^{z_j}}{S^2}\\
&=T_i\left(\delta_{ij}-\frac{e^{z_j}}{S}\right)\\
&=\boxed{T_i\left(\delta_{ij}-\frac{T_j}{\bar T}\right)}.
\end{aligned}
$$

这里的 $i$ 包括固定 logit 对应的最后一段，因此其他 logits 改变时 $T_M$ 仍会
通过分母变化。链式法则给出

$$
\begin{aligned}
\frac{\partial J_B}{\partial\tau_j}
&=\sum_{i=1}^{M}g_{T_i}
\frac{\partial T_i}{\partial\tau_j}\\
&=T_jg_{T_j}
-\frac{T_j}{\bar T}\sum_{i=1}^{M}T_i g_{T_i}.
\end{aligned}
$$

即代码使用的形式

$$
\boxed{
\frac{\partial J}{\partial\tau_j}
=T_j\left(
g_{T_j}-\frac{1}{\bar T}\sum_iT_i g_{T_i}
\right)
}.
$$

其中

$$
\bar g_T=\frac{1}{\bar T}\sum_iT_i g_{T_i}
$$

就是代码中的 `weighted_mean`。每个 logit 梯度只取决于本段物理时间梯度相对
加权平均值的差。所有段的共同时间伸缩方向已被固定总时间约束消除，而且任意
迭代都严格满足 $T_i>0$ 和 $\sum_iT_i=\bar T$。

#### 17.4.3 辅助点归一化变量的链式法则

辅助点使用

$$
\mathbf q=\mathbf q^0+\operatorname{diag}(\mathbf h)\mathbf u,
$$

因此

$$
\frac{\partial\mathbf q}{\partial\mathbf u}
=\operatorname{diag}(\mathbf h).
$$

几何代价定义在归一化变量上：

$$
J_{\mathrm{geometry}}=\|\mathbf u\|^2,\qquad
\nabla_\mathbf uJ_{\mathrm{geometry}}=2\mathbf u.
$$

所以第 $k$ 个轴的最终梯度为

$$
\boxed{
\frac{\partial J}{\partial u_k}
=h_k\frac{\partial J}{\partial q_k}+2\rho_g u_k.
}
$$

若辅助点是位置矩阵的第 $r$ 行，样条返回的 `inner_points` 不包含首端点，所以
代码读取第 $r-1$ 行。这是索引布局转换，不是额外数学近似。

#### 17.4.4 当前五维梯度的最终形式

当前例子有三段时间和一个三维辅助点，因此 NLopt 变量是

$$
\mathbf x=
\begin{bmatrix}\tau_1&\tau_2&u_x&u_y&u_z\end{bmatrix}^T.
$$

最终传给 L-BFGS 的梯度为

$$
\boxed{
\nabla_\mathbf xJ_B=
\begin{bmatrix}
T_1(g_{T_1}-\bar g_T)\\
T_2(g_{T_2}-\bar g_T)\\
h_xg_{q_x}+2\rho_gu_x\\
h_yg_{q_y}+2\rho_gu_y\\
h_zg_{q_z}+2\rho_gu_z
\end{bmatrix}
}.
$$

从最内层到最外层，完整计算图可以概括为

$$
\boxed{
\boldsymbol\xi
\xrightarrow{\;\mathbf J_{u\xi}\;}
\mathbf u_a
\xrightarrow{\;\mathbf g_u\;}
p
\xrightarrow{\text{quadrature}}
(G_C,G_D,\mathbf g_T^{\mathrm{exp}})
\xrightarrow{\text{spline adjoint}}
(\mathbf g_P,\mathbf g_\Psi,\mathbf g_T)
\xrightarrow{\text{softmax/auxiliary}}
\nabla_\mathbf xJ_B
}.
$$

#### 17.4.5 数学符号与代码变量的逐项对应

| 数学量 | 代码变量 | 维度/含义 |
| --- | --- | --- |
| $\mathbf J_{u\xi}$ | `linearization.jacobian` | $4\times15$ 固定分支执行器 Jacobian |
| $\mathbf g_u$ | `actuator_penalty_gradient` | $4\times1$ 罚函数对四执行器梯度 |
| $\mathbf g_\xi$ | `flat_gradient` | $15\times1$ 单点罚函数对平坦变量梯度 |
| $G_C$ | `partial_position_coefficients` | 每段 $8\times3$ 的位置系数偏导 |
| $G_D$ | `partial_yaw_coefficients` | 每段 $4\times1$ 的 yaw 系数偏导 |
| $\mathbf g_T^{\mathrm{exp}}$ | `partial_position_times` | 梯形权重和移动节点产生的显式时间偏导 |
| $\mathbf g_T^{\mathrm{act}}$ | `actuator_gradient.times` | 样条伴随传播后的执行器总时间梯度 |
| $\mathbf g_T$ | `duration_gradient` | 加权组合后的物理分段时间梯度 |
| $\bar g_T$ | `weighted_mean` | softmax 链式法则中的时间梯度加权平均 |
| $\nabla_\mathbf xJ_B$ | `objective_gradient` | 最终交给 NLopt 的变量梯度 |

`partial_yaw_times` 当前初始化为零，因为第 17.3.3 节的显式积分时间偏导只需保存
一次；`propagateCostGradient()` 最终把位置和 yaw 两套样条传播出的时间梯度
相加。把显式项放在 `partial_position_times` 而不是重复放进两者，可以避免对同一
积分权重求导两次。

### 17.5 第五步：两层有限差分验证

有限差分没有进入优化循环，只用于测试解析梯度。对待检查变量 $y_l$，步长取

$$
h_l=\epsilon\max(1,|y_l|),
$$

中心差分为

$$
\boxed{
g_l^{\mathrm{FD}}
=\frac{J(\mathbf y+h_l\mathbf e_l)-J(\mathbf y-h_l\mathbf e_l)}{2h_l}
}.
$$

在函数足够光滑且没有分支切换时，其截断误差为 $O(h_l^2)$。测试分成两层：

1. 在一条有效运动轨迹上选择采样点，冻结上一姿态和当前分支，对 15 个平坦
   输出导数逐一做中心差分，与 $4\times15$ 自动微分雅可比比较；
2. 分别在 yaw `0 -> 0` 和 `0 -> pi` 的阶段 A 初值处，对两个时间 logits 和
   三个辅助点变量逐一做中心差分，与完整 Stage B 目标梯度比较；零 yaw 工况
   会激活舵面罚函数，从而覆盖绝对值的局部导数。

单点执行器 Jacobian 使用 $\epsilon=2\times10^{-6}$，完整目标梯度使用
$\epsilon=10^{-6}$。每个分量使用缩放误差

$$
\boxed{
e_l=
\frac{|g_l^{\mathrm{analytic}}-g_l^{\mathrm{FD}}|}
{\max(1,|g_l^{\mathrm{analytic}}|,|g_l^{\mathrm{FD}}|)}
}.
$$

分母中的 1 避免接近零的梯度仅因相对除法而显示巨大误差，同时对于大梯度仍
采用相对尺度。单点测试还明确要求正负扰动后的 `roll_branch` 和
`pitch_branch` 与标称值一致；若发生分支切换，该点不适合验证固定分支
Jacobian，测试直接失败而不是把跨分支差分误认为解析梯度错误。

单点雅可比测试的最大缩放分量误差阈值为 $2\times10^{-4}$，完整目标梯度阈值
为 $5\times10^{-4}$。当前两个回归工况中的完整目标梯度最大误差为

$$
\boxed{3.35239\times10^{-7}},
$$

明显低于测试阈值。这个结果同时验证了执行器局部雅可比、积分权重、移动节点、
样条反向传播、softmax 和辅助点缩放，而不是只验证其中一个公式。

### 17.6 第六步：切换 L-BFGS 并做端到端回归

只有上述两层梯度检查通过后，阶段 B 才从 `LN_SBPLX` 切换为
`NLopt::LD_LBFGS`。当前默认应用结果为：

| 工况 | L-BFGS 目标评估数 | 组合目标 | 执行器积分 |
| --- | ---: | ---: | ---: |
| yaw `0 -> 0` | 14 | `1.045824 -> 1.011050` | 约 `2.30e-4 -> 2.8e-5` |
| yaw `0 -> pi` | 21 | `1.145326 -> 1.045637` | 约 `7.27e-4 -> 8.3e-5` |

两种工况的最终轨迹都通过真实物理幅值的 `2 ms` 稠密开环检查。完整 CTest 同时
运行阶段 A 和阶段 B，当前结果为 `2/2 passed`。

### 17.7 这一步解决了什么，仍未解决什么

已经解决：

- 不再依赖无梯度搜索估计执行器罚函数方向；
- 执行器梯度已经贯通到分段时间和辅助航点；
- 梯度由两层独立中心有限差分保护；
- 阶段 B 可以使用 L-BFGS，并保留原稠密物理认证。

仍未解决：

- 分支切换面、负单电机推力和分配奇异区并非全局光滑；
- 当前执行器限制仍是采样平方罚函数，不是连续时间硬约束；
- 尚未完成 `paper_control` 闭环跟踪认证。

第 16.9 节前两项已在第 18 章完成。下一步是接入 `paper_control` 闭环仿真，
再用实际跟踪误差、饱和持续时间和控制裕度标定规划用 `actuator_margin`。当前
约束仍是离散采样软罚函数，因此不应被表述为已经获得全局可微或连续时间严格
可行的规划器。

### 17.8 结合代码理解完整梯度执行路径

前面各节给出了数学推导，本节按照一次 NLopt 目标回调真正执行的顺序，把公式与
代码逐项对应。阅读顺序建议为：

```text
planning_types.hpp
        ↓
tailsitter_flatness.cpp: forward() / linearize()
        ↓
actuator_penalty_optimizer.cpp: integrateActuatorPenalty()
        ↓
flat_output_trajectory.cpp: propagateCostGradient()
        ↓
actuator_penalty_optimizer.cpp: evaluate() / objective()
        ↓
test_stage_b.cpp: 两层有限差分
```

#### 17.8.1 `ActuatorLinearization` 固定了 Jacobian 的接口契约

`planning/include/planning/planning_types.hpp` 中的核心定义为：

```cpp
static constexpr int flat_derivative_dimension = 15;
static constexpr int actuator_dimension = 4;
static constexpr int velocity_offset = 0;
static constexpr int acceleration_offset = 3;
static constexpr int jerk_offset = 6;
static constexpr int snap_offset = 9;
static constexpr int yaw_index = 12;
static constexpr int yaw_rate_index = 13;
static constexpr int yaw_acceleration_index = 14;

Eigen::Matrix<double, actuator_dimension,
              flat_derivative_dimension> jacobian;
Eigen::Matrix<double, 3,
              flat_derivative_dimension> angular_velocity_jacobian;
```

这些 offset 不是仅供注释使用，而是同时被以下位置引用：

- AutoDiff 独立变量的种子设置；
- `flat_gradient` 的四个三维位置导数分块；
- yaw 三个标量梯度的读取；
- 单点有限差分的变量扰动。

因此代码中的第 0～2 列始终对应 $\mathbf v$，第 3～5 列对应 $\mathbf a$，
第 6～8 列对应 $\mathbf j$，第 9～11 列对应 $\mathbf s$，最后三列依次对应
$\psi,\dot\psi,\ddot\psi$。如果以后增加风速、机体角速度约束或其他独立变量，
必须在这里统一扩展维数和 offset，不能只修改某一个局部循环。

其中 `jacobian` 是 $(T_1,T_2,\delta_1,\delta_2)$ 对 $\boldsymbol\xi$ 的
$4\times15$ Jacobian；新增的 `angular_velocity_jacobian` 是
$\boldsymbol\Omega$ 对同一个 $\boldsymbol\xi$ 的 $3\times15$ Jacobian。

`ActuatorPrediction` 额外保存 `roll_branch` 和 `pitch_branch`。普通预测负责给出
标称值和分支编号，`ActuatorLinearization` 则把这份预测与当前分支的局部 Jacobian
绑定在一起，避免调用者误把某个分支的梯度配到另一个分支的执行器值上。

#### 17.8.2 `forward()` 仍然是分支选择和标称值的唯一来源

`planning/src/tailsitter_flatness.cpp` 的 `forward()` 枚举四种组合：

```cpp
for (int roll_branch = 0; roll_branch < 2; ++roll_branch)
{
    ...
    for (int pitch_branch = 0; pitch_branch < 2; ++pitch_branch)
    {
        ...
        candidates.push_back({..., roll_branch, pitch_branch});
    }
}
```

随后使用 `std::min_element` 选择四元数距离最小的合法候选，并保存：

```cpp
output.roll_branch = selected->roll_branch;
output.pitch_branch = selected->pitch_branch;
```

这里没有把 `forward()` 整体替换成 AutoDiff 版本，原因是候选枚举、负推力过滤、
四元数距离比较和 `min_element` 都是离散逻辑。普通 `forward()` 继续承担三个
职责：

1. 决定当前标称轨迹究竟使用哪个姿态分支；
2. 返回完整的姿态、角速度、力矩、电机转速和舵面信息；
3. 给出映射是否有效以及失败原因。

AutoDiff 代码只对它选出的连续分支求局部导数，不改变原有物理判断路径。

#### 17.8.3 `ADScalar` 如何一次携带 15 个方向导数

文件中定义：

```cpp
using FlatDerivativeVector =
    Eigen::Matrix<double, ActuatorLinearization::flat_derivative_dimension, 1>;
using ADScalar = Eigen::AutoDiffScalar<FlatDerivativeVector>;
```

因此一个 `ADScalar` 实际保存

```text
value                  一个标称标量
derivatives()[0..14]   对 15 个平坦变量的梯度
```

`independentAD()` 设置单位种子：

```cpp
ADScalar result(value);
result.derivatives().setZero();
result.derivatives()[derivative_index] = 1.0;
```

数学上就是第 17.1.3 节的
$\widehat\xi_l=(\xi_l,\mathbf e_l)$。随后
`differentiableFlatDerivativeInput()` 按 offset 逐项播种：

```cpp
result.velocity[axis] = independentAD(sample.velocity[axis], 0 + axis);
result.acceleration[axis] = independentAD(sample.acceleration[axis], 3 + axis);
result.jerk[axis] = independentAD(sample.jerk[axis], 6 + axis);
result.snap[axis] = independentAD(sample.snap[axis], 9 + axis);
result.yaw = independentAD(sample.yaw, 12);
result.yaw_rate = independentAD(sample.yaw_rate, 13);
result.yaw_acceleration = independentAD(sample.yaw_acceleration, 14);
```

所以这不是对 15 个变量循环调用 15 次完整平坦映射，而是一次带 15 维导数向量
的前向传播。

#### 17.8.4 `DifferentiableJet2` 为什么是嵌套导数而不是重复求导

模板定义为：

```cpp
template <typename Scalar>
struct DifferentiableJet2
{
    Scalar value;
    Scalar first;
    Scalar second;
};
```

当 `Scalar=double` 时，它只计算固定分支的普通数值；当
`Scalar=ADScalar` 时，三个成员分别表示：

| 成员 | 数值含义 | 成员内部的 AutoDiff 梯度 |
| --- | --- | --- |
| `value` | $x$ | $\partial x/\partial\boldsymbol\xi$ |
| `first` | $\dot x$ | $\partial\dot x/\partial\boldsymbol\xi$ |
| `second` | $\ddot x$ | $\partial\ddot x/\partial\boldsymbol\xi$ |

例如构造速度和参考力时：

```cpp
const DVector velocity{{{input.velocity.x(),
                         input.acceleration.x(),
                         input.jerk.x()}, ...}};

const DVector force{{{m * input.acceleration.x(),
                      m * input.jerk.x(),
                      m * input.snap.x()}, ...}};
```

第一层三元组实现
$\mathcal J_2[\mathbf v]=(\mathbf v,\mathbf a,\mathbf j)$ 和
$\mathcal J_2[\mathbf f]=(\mathbf f,\dot{\mathbf f},\ddot{\mathbf f})$；每个三元组
成员内部的 `ADScalar` 再传播它对 15 个优化相关变量的敏感度。前者回答“沿时间
怎么变化”，后者回答“当前轨迹参数小幅改变时怎么变化”，二者不能互换。

#### 17.8.5 `evaluateFixedBranchActuators()` 如何复用一套公式

该函数是模板：

```cpp
template <typename Scalar>
FixedBranchActuators<Scalar> evaluateFixedBranchActuators(
    const FlatDerivativeInput<Scalar>& input,
    int roll_branch,
    int pitch_branch,
    const VehicleParameters& parameters);
```

同一函数会执行两次：

- `Scalar=double`：检查选定分支在连续公式中确实有效；
- `Scalar=ADScalar`：计算相同分支的执行器值及 $4\times15$ Jacobian。

roll 和 pitch 直接写成

```cpp
phi = phi_base + roll_branch * pi;
theta_bar = theta_base + pitch_branch * pi;
```

函数内部没有候选循环和 `argmin`，所以 AutoDiff 计算图只包含当前连续分支。
总推力、角速度、刚体力矩和差分推力仍严格使用第 15 章的原公式。例如：

```cpp
moment = inertia * angular_acceleration +
         angular_velocity.cross(inertia * angular_velocity);

differential_thrust = moment.z() / differential_denominator;
thrust_1 = 0.5 * (thrust + differential_thrust);
thrust_2 = 0.5 * (thrust - differential_thrust);
```

因为这些变量都是 `Scalar`，第 17.1.5 节的
$\mathrm d\mathbf M,\mathrm d\Delta T,\mathrm dT_1,\mathrm dT_2$ 会自动随每次
算术操作传播。

舵面部分没有调用只支持普通 `double` 的 SVD/LU 来求导，而是显式写出二维逆：

```cpp
determinant = a00 * a11 - a01 * a10;
flap_1 = (a11 * mx - a01 * my) / determinant;
flap_2 = (-a10 * mx + a00 * my) / determinant;
```

当矩阵元素、右端力矩和行列式都是 `ADScalar` 时，这段代码自然包含
$\mathrm dA_\delta$、$\mathrm d\mathbf m_\delta$ 和逆矩阵分母的导数，等价于
第 17.1.6 节的

$$
\mathrm d\boldsymbol\delta
=A_\delta^{-1}
(\mathrm d\mathbf m_\delta-\mathrm dA_\delta\boldsymbol\delta).
$$

所有奇异性、负推力和有限性判断只决定是否继续构造当前局部模型；代码不会对
这些 `if` 条件本身求导。

#### 17.8.6 `linearize()` 如何把 AD 输出整理成矩阵

`TailsitterFlatness::linearize()` 的调用顺序可以直接对应第 17.1.2 节：

```cpp
output.prediction = forward(sample, previous_branch);
if (!output.prediction.valid)
    return output;

nominal = evaluateFixedBranchActuators(
    flatDerivativeInput(sample), selected_roll, selected_pitch, parameters_);

differentiated = evaluateFixedBranchActuators(
    differentiableFlatDerivativeInput(sample),
    selected_roll, selected_pitch, parameters_);
```

第一次 `forward()` 决定分支并保留完整标称预测；`nominal` 调用保证新写的固定
分支公式在普通数值下有效；`differentiated` 调用才生成梯度。最后按照固定输出
顺序组成数组：

```cpp
{differentiated.thrust_1,
 differentiated.thrust_2,
 differentiated.flap_1,
 differentiated.flap_2}
```

并逐行提取：

```cpp
output.jacobian.row(actuator) =
    actuators[actuator].derivatives().transpose();
```

`derivatives()` 是 $15\times1$ 列向量，而 Jacobian 的一个执行器需要
$1\times15$ 行，所以这里必须转置。只有四行全部有限，才设置
`differentiable=true`。

#### 17.8.7 `integrateActuatorPenalty()` 中一次采样点怎样反传

该函数先按分段数分配反向传播缓存：

```cpp
partial_position_coefficients: (8 * segment_count) x 3
partial_yaw_coefficients:      (4 * segment_count) x 1
partial_position_times:        segment_count x 1
partial_yaw_times:             segment_count x 1
```

七次位置样条每段有 8 行系数，三次 yaw 样条每段有 4 行系数，因此可以用
`middleRows(segment_index * COEFF_NUM, COEFF_NUM)` 直接取得当前段对应的块。

每个节点先计算

```cpp
local_fraction = sample_index / samples_per_segment;
local_time = local_fraction * duration;
quadrature_weight = duration / samples_per_segment * trap_weight;
```

这三项分别对应 $s_k,t_{ik},w_{ik}$。`BranchState branch` 在整个双重循环外创建，
每次有效姿态写回后传给下一节点，使 `linearize()` 的标称分支沿积分时间保持连续。

NLopt 请求梯度时执行：

```cpp
linearization = flatness.linearize(sample, branch);
prediction = linearization.prediction;
```

只求初值、终值报告时则调用更便宜的 `forward()`。罚函数辅助函数同时返回
`value` 和 `derivative`，随后组成

```cpp
Eigen::Vector4d actuator_penalty_gradient(...);
flat_gradient = linearization.jacobian.transpose() *
                actuator_penalty_gradient;
```

第一行代码就是 $\mathbf g_u$，第二行就是
$\mathbf g_\xi=\mathbf J_{u\xi}^T\mathbf g_u$。如果某个执行器没有越过软阈值，
其 `derivative=0`，对应 Jacobian 行不会对当前目标梯度产生贡献；Jacobian 本身
仍然完整存在。

#### 17.8.8 系数梯度在代码中为什么是基向量外积

位置代码的核心累加为：

```cpp
position_block += quadrature_weight *
    (velocity_basis.transpose() * g_v.transpose() +
     acceleration_basis.transpose() * g_a.transpose() +
     jerk_basis.transpose() * g_j.transpose() +
     snap_basis.transpose() * g_s.transpose());
```

`velocity_basis.transpose()` 是 $8\times1$，`g_v.transpose()` 是 $1\times3$，
外积结果正好是 $8\times3$，与当前段位置系数矩阵同形。这就是第 17.3.2 节的
$\boldsymbol\beta_1\mathbf g_v^T$。yaw 对应代码为

```cpp
yaw_block += quadrature_weight *
    (yaw_basis.transpose() * g_yaw +
     yaw_rate_basis.transpose() * g_yaw_rate +
     yaw_acceleration_basis.transpose() * g_yaw_acceleration);
```

三个基向量都是 $4\times1$，结果与当前段 yaw 系数块同形。

显式时间梯度首先构造

```cpp
flat_time_derivative << acceleration, jerk, snap, crackle,
                        yaw_rate, yaw_acceleration, yaw_jerk;
```

即 $\dot{\boldsymbol\xi}$，再执行

```cpp
partial_position_times[i] +=
    trap_weight / samples_per_segment * sample_penalty +
    quadrature_weight * local_fraction *
        flat_gradient.dot(flat_time_derivative);
```

两项分别是

$$
\frac{c_k}{N}p_{ik},
\qquad
w_{ik}s_k\mathbf g_\xi^T\dot{\boldsymbol\xi}_{ik}.
$$

若 `linearization.differentiable=false`，代码跳过系数外积和移动节点项，只累加
第一项。这样常数无效屏障仍会随积分区间长度变化，但不会制造不存在的执行器
局部 Jacobian。

#### 17.8.9 `FlatOutputTrajectory` 如何调用样条伴随传播

`energyGradient()` 分别读取两套样条自身的完整梯度：

```cpp
position_gradient = position_.getEnergyGrad();
yaw_gradient = yaw_.getEnergyGrad();

result.position_inner_points = position_gradient.inner_points;
result.yaw_inner_points = yaw_weight * yaw_gradient.inner_points;
result.times = position_gradient.times +
               yaw_weight * yaw_gradient.times;
```

yaw 权重必须同时乘内部航点和时间梯度；位置部分的系数为 1。执行器罚函数不是
样条内置能量，所以 `propagateCostGradient()` 把上一节构造的偏导交给
`propagateGrad()`：

```cpp
position_gradient = position_.propagateGrad(
    partial_position_coefficients, partial_position_times);
yaw_gradient = yaw_.propagateGrad(
    partial_yaw_coefficients, partial_yaw_times);
```

位置和 yaw 的内部航点梯度分别保存，时间梯度则相加，因为两套样条共享同一组
物理分段时间。这一步执行的是第 17.3.4 节的
$C_P^*,C_T^*,D_\Psi^*,D_T^*$，而不是重新进行数值差分。

#### 17.8.10 `evaluate()` 如何组装最终 NLopt 梯度

每次回调都先从低维变量重新解码物理轨迹：

```cpp
durations = durationsFromVariables(variables, context);
positions = positionsFromVariables(variables, context);
trajectory.update(durations, positions, yaws);
```

所以目标值和梯度始终针对同一条刚重建的候选样条。随后计算三项标量：

```cpp
terms.total = smoothness_weight * normalized_smoothness +
              geometry_weight * geometry +
              actuator_weight * actuator.total();
```

当 `objective_gradient != nullptr` 时，代码取得两种物理梯度并按目标权重组合：

```cpp
duration_gradient =
    smoothness_weight / smoothness_reference * smoothness_gradient.times +
    actuator_weight * actuator_gradient.times;
```

这就是第 17.4.1 节的 $\mathbf g_T$。接着计算：

```cpp
weighted_mean += durations[i] * duration_gradient[i];
weighted_mean /= total_time;

gradient[tau_i] = durations[i] *
                  (duration_gradient[i] - weighted_mean);
```

分别对应 $\bar g_T$ 和
$\partial J/\partial\tau_i=T_i(g_{T_i}-\bar g_T)$。

辅助点代码为：

```cpp
gradient[variable] = auxiliary_half_range[axis] *
                         position_inner_gradient(waypoint - 1, axis) +
                     2.0 * geometry_weight * variables[variable];
```

第一项是 $h_kg_{q_k}$，第二项是 $2\rho_gu_k$。`waypoint - 1` 是因为样条返回的
内部航点梯度不包含首端点。

函数开始时把梯度向量清零；若候选构造抛出异常、目标非有限或梯度非有限，则
返回 $10^{30}$ 并再次把梯度置零。这避免 L-BFGS 读取上一次回调遗留的梯度，
但该零梯度屏障也说明无效区域不适合作为正常搜索空间。

#### 17.8.11 `objective()` 和 `LD_LBFGS` 的接口关系

NLopt 回调只有三行关键逻辑：

```cpp
auto& context = *static_cast<ObjectiveContext*>(raw_context);
++context.evaluations;
return evaluate(variables, context,
                gradient.empty() ? nullptr : &gradient).total;
```

当算法只请求目标值时，`gradient.empty()` 为真，积分器调用普通 `forward()`；
`LD_LBFGS` 请求一阶梯度时，传入非空向量，整条 AutoDiff 和样条伴随链被打开。
目标值与梯度共用同一个 `evaluate()`，因此不会出现“报告的目标函数”和“求导的
目标函数”使用不同公式的问题。

`optimize()` 用阶段 A 时间反算初始 logits：

```cpp
variables[i] = log(initial_durations[i] / last_duration);
```

这是 softmax 的精确逆比值，能够恢复原阶段 A 分段比例。随后创建
`nlopt::LD_LBFGS`、设置变量边界和容差。边界只限制 logits 与归一化辅助点，
物理分段时间的正值和定和性质由 softmax 本身保证。

#### 17.8.12 两层测试分别能发现哪类错误

`tests/test_stage_b.cpp::checkActuatorJacobian()` 逐一扰动 15 个平坦变量，直接比较

```text
linearize().jacobian
        与
[forward(xi + h e_l) - forward(xi - h e_l)] / (2 h)
```

它能够定位固定分支平坦映射内部的错误，例如：

- AutoDiff 列 offset 错位；
- 旋转或力矩公式漏掉一条链；
- 舵面二维逆没有传播分配矩阵梯度；
- Jacobian 行顺序与执行器顺序不一致。

`ActuatorPenaltyOptimizer::checkInitialGradient()` 则对最终变量
$[\tau_1,\tau_2,u_x,u_y,u_z]$ 做中心差分。它还覆盖：

- 平方铰链和绝对值的局部导数；
- 梯形权重与移动节点时间项；
- 位置/yaw 样条 `propagateGrad()`；
- softmax 链式法则；
- 辅助点缩放和几何项。

测试同时使用 yaw `0 -> 0` 和 `0 -> pi`。为避免新增约束在标称工况中恰好不
激活，`test_stage_b.cpp` 还只在测试内把机体角速度包络、舵面速率和舵机时间
常数收紧到 `5 rad/s`、`2 rad/s` 和 `0.2 s`，并明确断言机体角速度、电机动态、
舵面速率和舵机动态四项初值都大于零。这样完整目标有限差分实际覆盖所有新增
链路，而不是只检查一组数值为零的形式梯度。车辆默认参数并未被测试值修改。

#### 17.8.13 一次 L-BFGS 回调的最终数据流

把上述函数合并后，一次请求梯度的回调实际执行：

```text
x = [tau, auxiliary]
  ├─ durationsFromVariables() ──> T
  ├─ positionsFromVariables() ──> P
  └─ FlatOutputTrajectory::update(T, P, Psi)
          │
          ├─ energyGradient() ───────────────> g_smooth(P,T)
          │
          └─ integrateActuatorPenalty()
                ├─ sample(t)
                ├─ forward() 选择 branch
                ├─ linearize() 得到 J_u_xi
                ├─ J_u_xi^T g_u ────────────> g_xi
                ├─ basis outer products ────> G_C, G_D
                └─ quadrature time terms ───> g_T_exp
                         │
                         ▼
              propagateCostGradient()
                         │
                         ▼
                 g_actuator(P,T)
                         │
          ┌──────────────┴──────────────┐
          ▼                             ▼
    softmax chain                  auxiliary chain
          │                             │
          └──────────────┬──────────────┘
                         ▼
                  gradient dJ/dx
                         │
                         ▼
                    NLopt L-BFGS
```

这条路径也解释了为什么不能只在 `TailsitterFlatness` 中得到
$\mathbf J_{u\xi}$ 就宣称“优化梯度已经完成”：局部执行器 Jacobian 只是计算图
中间的一层，必须继续通过积分、样条内部解、时间参数化和辅助点参数化，才能形成
求解器真正需要的 $\nabla_\mathbf xJ_B$。

---

## 18. 机体角速度、执行器变化率与一阶动态约束

本章按第 16.9 节的前两步继续扩展 Stage B。原来的单电机推力和舵面偏角幅值
罚函数全部保留；新增约束不是用 $(T,\boldsymbol\Omega)$ 替换原始执行器约束，
而是在同一个目标中同时检查：

$$
T_1,T_2,\delta_1,\delta_2,\quad
\|\boldsymbol\Omega\|,\quad
\omega_{1,\mathrm{cmd}}^{\mathrm{req}},
\omega_{2,\mathrm{cmd}}^{\mathrm{req}},\quad
\dot\delta_1,\dot\delta_2,\quad
\delta_{1,\mathrm{cmd}}^{\mathrm{req}},
\delta_{2,\mathrm{cmd}}^{\mathrm{req}}.
$$

这里的 `req` 表示：如果希望一阶执行器状态严格经过相邻两个规划采样值，该区间
所需的常值命令。它不是把开环规划输出直接发送给控制器；闭环执行时仍由
`paper_control` 生成实际命令。

### 18.1 参数来源与适用边界

新增默认参数写在 `VehicleParameters`，并同步记录在两个 YAML 中：

| 参数 | 默认值 | 性质 |
| --- | ---: | --- |
| $\Omega_{\max}$ | $650\pi/180=11.34464\ \mathrm{rad/s}$ | `paper_control` differential-thrust turn 报告的约 `650 deg/s` 运动峰值，用作参考包络，并非厂家认证硬限值 |
| $\tau_m$ | $0.04\ \mathrm{s}$ | 项目 `config.py` 根据论文 Fig. 11 响应估计的电机时间常数 |
| $\tau_s$ | $0.03\ \mathrm{s}$ | 当前舵机响应量级的工程估计 |
| $\dot\delta_{\max}$ | $25\ \mathrm{rad/s}$ | 考虑气动负载后的工程速率限制 |

优化用阈值仍统一乘 `actuator_margin`。默认 $m_a=0.75$ 时：

$$
\Omega_p=0.75\Omega_{\max}=8.50848\ \mathrm{rad/s},\qquad
\dot\delta_p=0.75\dot\delta_{\max}=18.75\ \mathrm{rad/s}.
$$

电机转速命令、舵机命令的优化阈值同样分别为物理幅值的 $75\%$。最终
`FeasibilityChecker` 使用 $m_a=1$ 和约 `2 ms` 采样做完整包络认证。由于
$650\ \mathrm{deg/s}$ 只是已展示机动的参考峰值，后续仍必须用闭环仿真或实验
标定更可信的 $\Omega_{\max}$。

### 18.2 机体角速度约束及梯度

#### 18.2.1 罚函数

定义

$$
r_\Omega=\|\boldsymbol\Omega\|,
\qquad
g_\Omega=\frac{r_\Omega-\Omega_p}{\Omega_p},
\qquad
p_\Omega=[g_\Omega]_+^2.
$$

当 $r_\Omega>\Omega_p$ 时，

$$
\boxed{
\frac{\partial p_\Omega}{\partial\boldsymbol\Omega}
=\frac{2g_\Omega}{\Omega_p}
\frac{\boldsymbol\Omega}{\|\boldsymbol\Omega\|}
},
$$

否则梯度为零。真正激活时 $r_\Omega>\Omega_p>0$，所以不存在除以零问题。

#### 18.2.2 从角速度回传到平坦变量

`evaluateFixedBranchActuators()` 原本已经在求刚体力矩之前计算
$\boldsymbol\Omega$。现在把该 AutoDiff 向量保存在 `FixedBranchActuators` 中，
`linearize()` 提取

$$
J_{\Omega\xi}
=\frac{\partial\boldsymbol\Omega}{\partial\boldsymbol\xi}
\in\mathbb R^{3\times15}.
$$

于是角速度罚函数对单点平坦变量的梯度为

$$
\boxed{
\mathbf g_\xi^\Omega
=J_{\Omega\xi}^T
\frac{\partial p_\Omega}{\partial\boldsymbol\Omega}
}.
$$

代码在 `integrateActuatorPenalty()` 中把它与原执行器幅值梯度相加：

```cpp
FlatGradient flat_gradient =
    linearization.jacobian.transpose() * actuator_penalty_gradient;
flat_gradient.noalias() +=
    linearization.angular_velocity_jacobian.transpose() *
    body_rate_penalty_gradient;
```

之后仍通过第 17 章同一套样条系数、分段时间、softmax 和辅助点链路传播，不需要
为角速度约束另写一套优化器。

### 18.3 为什么执行器动态使用“相邻节点反算命令”

若直接在连续时间写

$$
u_{\mathrm{cmd}}=u+\tau\dot u,
\qquad
\dot u=J_{u\xi}\dot{\boldsymbol\xi},
$$

则对优化变量求 $\dot u$ 的梯度会出现 $u(\boldsymbol\xi)$ 的 Hessian-vector
product。它可以用二阶 AutoDiff 求，但实现复杂、计算量较大，而且当前闭环仿真
本来就以离散时间更新执行器。

当前实现采用与 `main_sim.py` 一致的一阶离散模型。对区间
$[t_k,t_{k+1}]$，令 $h=t_{k+1}-t_k$，命令在该区间保持常值：

$$
y_{k+1}=a y_k+(1-a)u_{\mathrm{cmd},k},
\qquad
a=e^{-h/\tau}.
$$

给定规划的相邻状态 $y_k,y_{k+1}$，所需命令可精确反解为

$$
\boxed{
u_{\mathrm{cmd},k}^{\mathrm{req}}
=\frac{y_{k+1}-a y_k}{1-a}
}.
$$

对电机取 $y=\omega$、$\tau=\tau_m$；对舵机取 $y=\delta$、
$\tau=\tau_s$。这相对于所采用的零阶保持离散一阶模型是精确关系，不是
$\dot u$ 的额外有限差分近似，而且只需要相邻两个节点的一阶 Jacobian。

### 18.4 从推力 Jacobian 转成电机转速 Jacobian

平坦映射的原始电机输出仍是 $T_i$，动态模型的状态则是 $\omega_i$：

$$
\omega_i=\sqrt{T_i/c_T}.
$$

在 $\omega_i>0$ 的有效工作点，

$$
\mathrm dT_i=2c_T\omega_i\,\mathrm d\omega_i,
$$

所以

$$
\boxed{
J_{\omega_i\xi}
=\frac{1}{2c_T\omega_i}J_{T_i\xi}
}.
$$

舵面状态就是 $\delta_i$，其 Jacobian 直接复用原矩阵后两行。代码
`dynamicActuatorState()` 因而构造

$$
\mathbf y=
[\omega_1,\omega_2,\delta_1,\delta_2]^T,
\qquad
J_{y\xi}\in\mathbb R^{4\times15}.
$$

若电机转速接近零，平方根映射的导数会变得病态，代码不伪造该点的动态梯度；
当前测试轨迹的有效电机工作点均远离零转速。

### 18.5 一阶动态命令罚函数的完整梯度

把单个执行器的反算命令记为

$$
c(y_0,y_1,h)=\frac{y_1-a y_0}{1-a},qquad a=e^{-h/\tau}.
$$

三个局部偏导为

$$
\boxed{
\frac{\partial c}{\partial y_0}=-\frac{a}{1-a},\qquad
\frac{\partial c}{\partial y_1}=\frac{1}{1-a}
},
$$

$$
\boxed{
\frac{\partial c}{\partial h}
=-\frac{a}{\tau}
\frac{y_1-y_0}{(1-a)^2}
}.
$$

电机命令使用上下界平方罚函数，舵机命令使用对称绝对值平方罚函数。统一把其
值记为 $p(c)$、导数记为 $p'(c)$。一个区间的积分近似为

$$
C_{\mathrm{dyn}}=h p(c).
$$

因此

$$
\boxed{
\frac{\partial C_{\mathrm{dyn}}}{\partial y_0}
=h p'(c)\left(-\frac{a}{1-a}\right),
\qquad
\frac{\partial C_{\mathrm{dyn}}}{\partial y_1}
=h p'(c)\frac{1}{1-a}
},
$$

$$
\boxed{
\left.\frac{\partial C_{\mathrm{dyn}}}{\partial h}\right|_{y_0,y_1}
=p(c)+h p'(c)\frac{\partial c}{\partial h}
}.
$$

每段有 $N$ 个等长区间，$h=T_i/N$，故最后一式对物理段时间的显式贡献还要
乘 $\partial h/\partial T_i=1/N$。代码中的 `command_first`、
`command_second`、`command_step` 和 `explicit_duration_gradient` 分别对应上述
四个量。

### 18.6 舵面角速度罚函数及梯度

相邻节点上的离散舵面角速度为

$$
r=\frac{\delta_1-\delta_0}{h}.
$$

使用规划阈值 $r_p=\dot\delta_p$，定义

$$
p_r(r)=\left[
\frac{|r|-r_p}{r_p}
\right]_+^2,
\qquad
C_r=h p_r(r).
$$

记 $g_r=p_r'(r)$，则

$$
\boxed{
\frac{\partial C_r}{\partial\delta_0}=-g_r,
\qquad
\frac{\partial C_r}{\partial\delta_1}=g_r
},
$$

$$
\boxed{
\left.\frac{\partial C_r}{\partial h}\right|_{\delta_0,\delta_1}
=p_r(r)-g_r r
}.
$$

这解释了代码里端点梯度没有额外的 $1/h$：$r$ 对端点的 $1/h$ 与外层积分
长度 $h$ 正好抵消。

### 18.7 区间端点梯度如何继续进入样条和段时间

假设前述公式得到区间代价对某个端点执行器状态的梯度
$\mathbf g_y=\partial C/\partial\mathbf y$，则

$$
\boxed{
\mathbf g_\xi=J_{y\xi}^T\mathbf g_y
}.
$$

`accumulateFlatGradient()` 用位置/yaw 基函数把该梯度加入多项式系数偏导。若
端点位于该段归一化位置 $s_k=k/N$，改变段时间还会移动该节点：

$$
\frac{\partial\boldsymbol\xi(t_k)}{\partial T_i}
=s_k\dot{\boldsymbol\xi}(t_k).
$$

因此移动节点贡献为

$$
\boxed{
\left.\frac{\partial C}{\partial T_i}\right|_{\mathrm{node}}
=s_k\mathbf g_\xi^T\dot{\boldsymbol\xi}(t_k)
}.
$$

它与第 18.5、18.6 节的显式区间长度贡献相加，再交给
`propagateCostGradient()`。这一步不能遗漏，否则时间 logits 的解析梯度会与
有限差分不一致。

### 18.8 总目标中的新增积分项

`ActuatorPenaltyIntegral` 现在包含

```text
thrust_1, thrust_2, flap_1, flap_2,
body_rate,
motor_dynamics, flap_rate, servo_dynamics,
invalid_mapping
```

所有分量都先按各自物理尺度归一化，再由同一个 `actuator_weight` 加权进入 Stage B
目标。`--no-body-rate-constraint` 和 `--no-actuator-dynamics` 只用于消融对比；
默认两类约束均开启。

需要特别区分“软规划阈值”和“最终物理认证”：平方罚函数不保证优化结果严格低于
$75\%$ 阈值。若要求规划阈值也严格满足，应提高罚权、使用增广拉格朗日/显式
不等式约束，或继续增加总时间；当前程序只把完整物理包络作为最终 `feasible`
判据。

### 18.9 稠密检查与 CSV/绘图输出

`FeasibilityChecker` 除原始幅值外，现以相邻约 `2 ms` 样本检查：

- $\|\boldsymbol\Omega\|$；
- $|\dot\delta_i|$；
- 按相同指数一阶模型反算的 $\omega_{i,\mathrm{cmd}}^{\mathrm{req}}$；
- 按相同指数一阶模型反算的 $\delta_{i,\mathrm{cmd}}^{\mathrm{req}}$。

CSV 新增机体角速度、执行器有限差分速率和反算命令列。绘图脚本在这些列存在时
额外生成 `*_dynamic_constraints.png`，依次显示机体角速度范数、电机所需命令、
舵面角速度和舵机所需命令。第一行速率/反算命令没有前一采样点，因此导出
`NaN` 并在图上自然留空。

### 18.10 默认参数优化结果

为了单独观察新增约束的作用，下面比较“原幅值约束 Stage B”和“幅值 + 机体
角速度 + 执行器动态 Stage B”。两者总时间均固定为 $1.8\ \mathrm{s}$，参数、
航点和优化自由度相同：

| 工况/指标 | 仅幅值 | 新增角速度/动态 | 变化 |
| --- | ---: | ---: | ---: |
| yaw `0 -> 0`，峰值 $\|\Omega\|$ [rad/s] | 10.111955 | 8.800369 | -12.97% |
| yaw `0 -> 0`，峰值 $|\dot\delta|$ [rad/s] | 14.924039 | 9.417600 | -36.90% |
| yaw `0 -> 0`，峰值所需电机命令 [rad/s] | 1908.321598 | 1900.212894 | -0.42% |
| yaw `0 -> 0`，峰值所需舵机命令 [rad] | 0.897998 | 0.698497 | -22.22% |
| yaw `0 -> pi`，峰值 $\|\Omega\|$ [rad/s] | 6.701638 | 6.680891 | -0.31% |
| yaw `0 -> pi`，峰值 $|\dot\delta|$ [rad/s] | 4.330420 | 4.221650 | -2.51% |
| yaw `0 -> pi`，峰值所需电机命令 [rad/s] | 1911.116071 | 1907.147248 | -0.21% |
| yaw `0 -> pi`，峰值所需舵机命令 [rad] | 0.647060 | 0.654935 | +1.22% |

yaw `0 -> 0` 中新增约束明显压低角速度、舵面速率和舵机所需命令；yaw
`0 -> pi` 原本已远低于角速度/舵面速率阈值，新增项主要对电机动态产生较小修正，
舵机所需命令略增是多目标折中，并不表示每个峰值都保证单调下降。不能比较两次
运行的“组合目标绝对值”，因为新增运行的目标函数多了四个积分分量。

默认完整约束轨迹相对纯 Stage A 的稠密结果为：

| 工况 | Stage A $\|\Omega\|$ | Stage B $\|\Omega\|$ | Stage A $|\dot\delta|$ | Stage B $|\dot\delta|$ |
| --- | ---: | ---: | ---: | ---: |
| yaw `0 -> 0` | 10.616585 | 8.800369 | 17.268658 | 9.417600 |
| yaw `0 -> pi` | 7.243117 | 6.680891 | 5.810270 | 4.221650 |

两条最终轨迹在完整物理/参考包络下均通过 `2 ms` 稠密检查。yaw `0 -> 0` 的
$8.800369\ \mathrm{rad/s}$ 仍略高于 $75\%$ 软阈值
$8.508480\ \mathrm{rad/s}$，但低于完整参考包络 $11.344640\ \mathrm{rad/s}$；
这正是“软罚函数不等于硬约束”的具体例子。

回归测试中收紧四类阈值以强制所有新增罚项非零，最终五维目标解析梯度相对中心
有限差分的最大缩放误差为

$$
\boxed{6.51575\times10^{-7}},
$$

CTest 仍为 `2/2 passed`。因此新增结果不是用无梯度方法得到的；机体角速度、
电机动态、舵面速率和舵机动态都已经贯通 L-BFGS 的完整梯度链路。

### 18.11 下一步

第 16.9 节的前两项已经完成。下一步应把这组规划 CSV/样条参考接入
`paper_control`，并同时记录：

1. 位置、速度、姿态和机体角速度跟踪误差；
2. 实际电机/舵机命令与状态，以及饱和持续时间；
3. 规划反算命令与闭环实际命令之间的差异；
4. 由闭环结果反推的最小可靠 `actuator_margin`。

只有闭环认证通过后，才能把当前“开环动态可实现性”提升为针对该控制器的轨迹
可跟踪性结论。

### 18.12 当前目标函数与约束的完整汇总

本节集中回答“当前 Stage B 的目标函数和约束到底有哪些”。必须区分四个层次：

1. 真正传给 L-BFGS 的目标函数；
2. 通过变量参数化和样条构造严格满足的等式/边界；
3. 进入目标函数的采样软约束；
4. 优化结束后执行、但不参与求解的稠密认证。

如果不做这一区分，很容易把平方罚函数误称为 NLopt 的硬不等式约束。

#### 18.12.1 优化变量

设轨迹共有 $M$ 段、$K$ 个允许移动的辅助位置航点，当前变量写为

$$
\boxed{
\mathbf x=
[\tau_1,\ldots,\tau_{M-1},
\mathbf u_1^T,\ldots,\mathbf u_K^T]^T
}.
$$

其中 $\tau_i$ 是固定总时间 softmax 的自由 logits，$\mathbf u_k\in[-1,1]^3$
是辅助点归一化偏移。实际辅助点为

$$
\mathbf q_k
=\mathbf q_k^0+\operatorname{diag}(\mathbf h)\mathbf u_k,
$$

默认

$$
\mathbf h=[0.5,0.5,0.5]^T\ \mathrm m.
$$

当前演示轨迹有三段和一个辅助点，因此

$$
\mathbf x=[\tau_1,\tau_2,u_x,u_y,u_z]^T\in\mathbb R^5.
$$

yaw 航点、起终点和关键位置航点当前固定，不是 NLopt 变量；样条多项式系数也
不是外层独立变量，而是由给定航点和分段时间的 minimum-snap/minimum-yaw
内部问题确定。

#### 18.12.2 当前总目标函数

Stage B 最小化

$$
\boxed{
J_B(\mathbf x)
=\rho_s\frac{J_{\mathrm{smooth}}}{J_{\mathrm{smooth}}^0}
+\rho_gJ_{\mathrm{geometry}}
+\rho_aJ_{\mathrm{constraint}}
}.
$$

默认权重为

$$
\rho_s=1,
\qquad
\rho_g=0.05,
\qquad
\rho_a=200.
$$

代码对应 `ActuatorPenaltyConfig` 中的 `smoothness_weight`、`geometry_weight` 和
`actuator_weight`。

##### 1. 平滑性项

$$
\boxed{
J_{\mathrm{smooth}}
=\int_0^{T_f}\|\mathbf p^{(4)}(t)\|^2\,\mathrm dt
+\lambda_\psi\int_0^{T_f}\ddot\psi^2(t)\,\mathrm dt
}.
$$

位置七次样条最小化 snap 能量，yaw 三次样条最小化 yaw acceleration 能量。
默认 $\lambda_\psi=1$。分母 $J_{\mathrm{smooth}}^0$ 是阶段 B 开始前的阶段 A
参考平滑能量，在一次优化中固定不变；归一化用于平衡平滑项和无量纲约束罚项的
数值尺度。

##### 2. 辅助点几何正则项

$$
\boxed{
J_{\mathrm{geometry}}=\sum_{k=1}^K\|\mathbf u_k\|^2
}.
$$

它抑制辅助点为了降低局部执行器峰值而无必要地移动到搜索区域边界。

##### 3. 约束罚函数项

$$
\boxed{
J_{\mathrm{constraint}}
=J_{T_1}+J_{T_2}+J_{\delta_1}+J_{\delta_2}
+J_\Omega+J_{\mathrm{motor}}
+J_{\dot\delta}+J_{\mathrm{servo}}
+J_{\mathrm{invalid}}
}.
$$

代码中的 `ActuatorPenaltyIntegral` 依次保存：

```text
thrust_1, thrust_2, flap_1, flap_2,
body_rate,
motor_dynamics, flap_rate, servo_dynamics,
invalid_mapping
```

所有连续违反量使用平方正部

$$
P(g)=[g]_+^2=[\max(0,g)]^2.
$$

因此在规划阈值以内罚值和梯度均为零，越界后罚值按归一化违反量的平方增长。

#### 18.12.3 优化内部的采样软约束

默认规划裕度为

$$
m_a=0.75.
$$

以下约束均按每段 `samples_per_segment=50` 个区间离散，并作为目标函数罚项，
不是 NLopt `add_inequality_constraint()` 注册的硬约束。

##### 1. 单电机推力幅值

平坦映射输出 $T_1(t),T_2(t)$，规划区间为

$$
\boxed{
T_{\min}\le T_i(t)\le T_{\max}^{p}
},
$$

$$
T_{\min}=c_T\omega_{\min}^2,
\qquad
T_{\max}^{p}=c_T(m_a\omega_{\max})^2.
$$

默认 paper vehicle 参数为

$$
c_T=1.62\times10^{-6},
\qquad
\omega_{\min}=0,
\qquad
\omega_{\max}=2500\ \mathrm{rad/s},
$$

所以

$$
\omega_{\max}^{p}=1875\ \mathrm{rad/s},
\qquad
T_{\max}^{p}\approx5.6953\ \mathrm N.
$$

这里的原始幅值罚函数定义在单电机推力上；电机动态罚函数则定义在转速和反算
转速命令上。

##### 2. 舵面偏角幅值

$$
\boxed{
|\delta_i(t)|\le\delta_{\max}^{p},
\qquad
\delta_{\max}^{p}=m_a\delta_{\max}
}.
$$

默认 $\delta_{\max}=1\ \mathrm{rad}$，因此优化阈值为

$$
\delta_{\max}^{p}=0.75\ \mathrm{rad}.
$$

##### 3. 机体角速度范数

$$
\boxed{
\|\boldsymbol\Omega(t)\|\le\Omega_{\max}^{p},
\qquad
\Omega_{\max}^{p}=m_a\Omega_{\max}
}.
$$

当前参考包络取论文已展示机动的约 $650\ \mathrm{deg/s}$：

$$
\Omega_{\max}=11.34464\ \mathrm{rad/s},
\qquad
\Omega_{\max}^{p}=8.50848\ \mathrm{rad/s}.
$$

该数值是参考运动包络，不是厂家认证的机体角速度硬件额定值。

##### 4. 电机一阶动态和所需命令幅值

电机状态取 $y=\omega_i$，一阶模型为

$$
\tau_m\dot\omega_i
=\omega_{i,\mathrm{cmd}}-\omega_i,
\qquad
\tau_m=0.04\ \mathrm s.
$$

对相邻规划节点，$a_m=e^{-h/\tau_m}$，反算命令为

$$
\omega_{i,\mathrm{cmd},k}^{\mathrm{req}}
=\frac{\omega_{i,k+1}-a_m\omega_{i,k}}{1-a_m}.
$$

软约束为

$$
\boxed{
\omega_{\min}
\le\omega_{i,\mathrm{cmd},k}^{\mathrm{req}}
\le m_a\omega_{\max}
}.
$$

默认即

$$
0\le\omega_{i,\mathrm{cmd},k}^{\mathrm{req}}
\le1875\ \mathrm{rad/s}.
$$

当前没有另外设置 $|\dot\omega_i|$、$|\dot T_i|$ 的独立上限；电机变化能力由
一阶时间常数和所需命令幅值共同描述。

##### 5. 舵面角速度

区间有限差分为

$$
\dot\delta_{i,k}\approx
\frac{\delta_{i,k+1}-\delta_{i,k}}{h},
$$

对应软约束

$$
\boxed{
|\dot\delta_{i,k}|
\le\dot\delta_{\max}^{p},
\qquad
\dot\delta_{\max}^{p}=m_a\dot\delta_{\max}
}.
$$

默认 $\dot\delta_{\max}=25\ \mathrm{rad/s}$，所以规划阈值为

$$
\dot\delta_{\max}^{p}=18.75\ \mathrm{rad/s}.
$$

##### 6. 舵机一阶动态和所需命令幅值

舵机状态取 $y=\delta_i$：

$$
\tau_s\dot\delta_i
=\delta_{i,\mathrm{cmd}}-\delta_i,
\qquad
\tau_s=0.03\ \mathrm s.
$$

令 $a_s=e^{-h/\tau_s}$，反算命令为

$$
\delta_{i,\mathrm{cmd},k}^{\mathrm{req}}
=\frac{\delta_{i,k+1}-a_s\delta_{i,k}}{1-a_s}.
$$

软约束为

$$
\boxed{
|\delta_{i,\mathrm{cmd},k}^{\mathrm{req}}|
\le m_a\delta_{\max}=0.75\ \mathrm{rad}
}.
$$

舵面速率和舵机命令两项同时存在：前者限制实际状态变化速度，后者限制一阶动态
为了实现该变化所需的输入幅值。

##### 7. 无效平坦映射

以下情况进入 `invalid_mapping`：

- 不存在合法的非负总推力姿态分支；
- 出现负单电机推力；
- 舵面分配矩阵接近奇异；
- 映射结果包含非有限值。

默认无效映射罚值为 `10`。该区域通常伴随分支或有效域边界，不具有可供 L-BFGS
使用的全局光滑导数；当前代码把无效罚值作为局部屏障，而不伪造穿过无效域的
梯度。

#### 18.12.4 由构造严格满足的约束

下面这些不是平方罚项，而是由变量映射、NLopt box bounds 或样条方程严格满足。

##### 1. 固定总时间和正分段时间

令最后一个固定 logit 为 $z_M=0$，其余 $z_i=\tau_i$：

$$
T_i=T_f\frac{e^{z_i}}{\sum_{j=1}^M e^{z_j}}.
$$

因此对每次迭代都有

$$
\boxed{
T_i>0,
\qquad
\sum_{i=1}^M T_i=T_f
}.
$$

当前演示总时间固定为 $T_f=1.8\ \mathrm s$。

##### 2. 时间 logits 边界

NLopt box bounds 为

$$
\boxed{-5\le\tau_i\le5}.
$$

它限制不同分段时间比例的极端程度；物理分段时间本身仍由 softmax 保证正值与
定和。

##### 3. 辅助点搜索边界

$$
\boxed{-1\le u_{k,x},u_{k,y},u_{k,z}\le1}.
$$

默认 $\mathbf h=[0.5,0.5,0.5]^T\ \mathrm m$，相当于每个可移动辅助点满足

$$
|q_{k,j}-q_{k,j}^0|\le0.5\ \mathrm m.
$$

##### 4. 航点插值

位置/yaw 样条严格经过给定的：

- 起点；
- 固定关键航点；
- 当前优化得到的辅助位置航点；
- 终点；
- 固定 yaw 航点。

##### 5. 边界导数与样条连续性

默认 `BoundaryConditions` 为零，因此位置七次样条满足

$$
\boxed{
\mathbf v(0)=\mathbf v(T_f)=\mathbf0,
\quad
\mathbf a(0)=\mathbf a(T_f)=\mathbf0,
\quad
\mathbf j(0)=\mathbf j(T_f)=\mathbf0
}.
$$

yaw 三次样条使用与其阶次相符的零首末 yaw rate：

$$
\boxed{
\dot\psi(0)=\dot\psi(T_f)=0
}.
$$

各段内部位置及所需导数连续性、yaw 及 yaw rate 连续性由样条求解器保证。

#### 18.12.5 优化后 `2 ms` 稠密认证

L-BFGS 结束后，`FeasibilityChecker` 默认按约 $2\ \mathrm{ms}$ 采样，并额外
插入所有样条连接点。默认构造使用 `actuator_margin=1`，即检查完整包络：

$$
\boxed{0\le\omega_i\le2500\ \mathrm{rad/s}},
$$

$$
\boxed{|\delta_i|\le1.0\ \mathrm{rad}},
$$

$$
\boxed{\|\boldsymbol\Omega\|\le11.34464\ \mathrm{rad/s}},
$$

$$
\boxed{|\dot\delta_i|\le25\ \mathrm{rad/s}},
$$

$$
\boxed{
0\le\omega_{i,\mathrm{cmd}}^{\mathrm{req}}
\le2500\ \mathrm{rad/s}
},
$$

$$
\boxed{
|\delta_{i,\mathrm{cmd}}^{\mathrm{req}}|
\le1.0\ \mathrm{rad}
}.
$$

稠密检查还要求：

- 每个采样点的平坦映射有效；
- 姿态分支连续；
- 毫秒级相邻姿态没有超过 $0.5\ \mathrm{rad}$ 的异常跳变。

这个 `feasible` 判定发生在优化之后，不会向 L-BFGS 返回约束 Jacobian，也不会
把违反结果自动转成 NLopt 硬约束。它是独立的开环认证层。

#### 18.12.6 当前没有包含的约束

当前 Stage B 尚未加入：

- 障碍物、碰撞和空间走廊约束；
- 速度、加速度和 jerk 的显式上限；
- 倾角约束；
- 机体角加速度和三轴力矩上限；
- 独立的电机转速变化率或单电机推力变化率上限；
- 电池电压、功率和总能量约束；
- `paper_control` 闭环跟踪误差、实际饱和量和饱和持续时间约束。

因此当前优化问题最准确的表述是：

> NLopt L-BFGS 只直接处理时间 logits 和辅助点的 box bounds；固定总时间、正段
> 时间、航点插值和边界条件由参数化/样条严格保证；执行器幅值、机体角速度、
> 舵面速率和一阶动态可实现性是可微采样软罚函数；优化结束后再用完整包络执行
> 独立的稠密开环认证。

---

## 19. 测试 3：无固定总时间的多航点时空联合优化

测试 3 把第 16.4 节推荐形式落实为一个独立的 hover-to-hover 场景。原固定总时间
Stage B 仍然保留作为对照；新增接口 `optimizeJoint()` 同时优化所有分段时间和
多个中间自由位置航点，并按第 16.5 节同时保留高层约束与当前构型的真实执行器
约束。

### 19.1 场景与未施加的约束

测试的两个 hover 点为

$$
\mathbf p_0=[0,0,-10]^T\ \mathrm m,
\qquad
\mathbf p_f=[15,0,-10]^T\ \mathrm m.
$$

相比前两项测试的 $6\ \mathrm m$ 距离，本测试延长到 $15\ \mathrm m$。在直线
初值上放置三个中间点：

$$
\mathbf q_1^0=[3.75,0,-10]^T,
\quad
\mathbf q_2^0=[7.50,0,-10]^T,
\quad
\mathbf q_3^0=[11.25,0,-10]^T.
$$

因此共有四段。测试仍包含两种固定 yaw 航点序列：

$$
\text{yaw 0 -> 0}:\quad
[0,0,0,0,0],
$$

$$
\text{yaw 0 -> pi}:\quad
[0,\pi/4,\pi/2,3\pi/4,\pi].
$$

起终端使用默认零速度、零加速度和零 jerk，因而是 hover-to-hover 边界。三个
中间 yaw 值暂时固定，测试 3 的自由航点只指位置自由度。

本测试明确没有：

- 固定总时间等式约束；
- 障碍物约束；
- 连续时间安全走廊约束。

中间点仍有有限 box bounds：默认相对标称位置允许

$$
\Delta x\in[-2,2]\ \mathrm m,
\quad
\Delta y\in[-3,3]\ \mathrm m,
\quad
\Delta z\in[-2,2]\ \mathrm m.
$$

这只是防止无界搜索的航点变量边界，不是约束整条轨迹必须位于某个安全走廊。

### 19.2 自由时间与航点的 13 维参数化

设初始每段时间为 $T_i^0=1\ \mathrm s$。测试 3 不再使用固定总时间 softmax，
而对每段引入独立 log-duration：

$$
\boxed{
T_i=T_i^0e^{\eta_i},
\qquad i=1,\ldots,4
}.
$$

这样每段始终满足 $T_i>0$，但

$$
T_f=\sum_{i=1}^4T_i
$$

是优化结果，不再固定为初始 $4\ \mathrm s$。默认 box bound 为

$$
-1.5\le\eta_i\le1.5.
$$

三个辅助点用归一化变量 $\mathbf u_k\in[-1,1]^3$ 表示，所以完整变量为

$$
\boxed{
\mathbf x=
[\eta_1,\eta_2,\eta_3,\eta_4,
\mathbf u_1^T,\mathbf u_2^T,\mathbf u_3^T]^T
\in\mathbb R^{13}
}.
$$

### 19.3 测试 3 的目标函数

联合目标为

$$
\boxed{
J_3=
\rho_s\frac{J_{\mathrm{smooth}}}{J_{\mathrm{smooth}}^0}
+\rho_T\sum_{i=1}^4T_i
+\rho_g\sum_{k=1}^3\|\mathbf u_k\|^2
+\rho_cJ_{\mathrm{constraint}}
}.
$$

默认权重为

$$
\rho_s=1,
\qquad
\rho_T=0.5,
\qquad
\rho_g=0.02,
\qquad
\rho_c=200.
$$

其中平滑项仍为位置 snap 能量加 yaw acceleration 能量。新增
$\rho_TT_f$ 很关键：取消固定总时间后，如果没有时间代价，优化器可以不断拉长
轨迹以把导数和约束罚值压向零；时间项使平滑性、可行性和完成时间形成有限折中。

### 19.4 同时保留的高层约束和执行器约束

测试 3 设置 `enable_high_level_constraints=true`。默认 $75\%$ 规划裕度下，高层
采样软约束包括：

$$
\boxed{\|\mathbf v\|\le6.0\ \mathrm{m/s}},
$$

$$
\boxed{\|\boldsymbol\Omega\|\le8.50848\ \mathrm{rad/s}},
$$

$$
\boxed{0\le T\le11.390625\ \mathrm N}.
$$

这里速度完整参考包络为 $8\ \mathrm{m/s}$；总推力阈值由两个电机各自的
$1875\ \mathrm{rad/s}$ 规划转速阈值得到：

$$
T_{\max}^{p}=2c_T(1875)^2=11.390625\ \mathrm N.
$$

高层约束之外，以下真实执行器软约束全部保留：

- $T_1,T_2$ 单电机推力幅值；
- $\delta_1,\delta_2$ 舵面偏角幅值；
- 电机一阶动态反算命令幅值；
- 舵面有限差分角速度；
- 舵机一阶动态反算命令幅值；
- 无效映射屏障。

所以测试 3 实际实现的是

$$
\boxed{
(\|\mathbf v\|,T,\|\boldsymbol\Omega\|)\text{ 高层约束}
+(T_1,T_2,\delta_1,\delta_2)\text{ 执行器幅值约束}
+\text{执行器速率/动态约束}
}.
$$

### 19.5 自由时间梯度与固定总时间版本的区别

设样条反向传播后得到物理段时间梯度

$$
g_{T_i}
=\frac{\partial}{\partial T_i}
\left[
\rho_s\frac{J_{\mathrm{smooth}}}{J_{\mathrm{smooth}}^0}
+\rho_cJ_{\mathrm{constraint}}
\right].
$$

加入时间代价后为

$$
\frac{\partial J_3}{\partial T_i}=g_{T_i}+\rho_T.
$$

由 $T_i=T_i^0e^{\eta_i}$，

$$
\frac{\partial T_i}{\partial\eta_i}=T_i,
\qquad
\frac{\partial T_i}{\partial\eta_j}=0\quad(i\ne j),
$$

所以传给 L-BFGS 的时间变量梯度为

$$
\boxed{
\frac{\partial J_3}{\partial\eta_i}
=T_i(g_{T_i}+\rho_T)
}.
$$

它与固定总时间版本的关键区别是：不再减去 softmax 的时间梯度加权平均值。每段
可以独立变长或缩短，总时间也随之改变。

### 19.6 高层速度和总推力梯度

速度罚函数使用

$$
p_v=
\left[
\frac{\|\mathbf v\|-v_p}{v_p}
\right]_+^2.
$$

激活时

$$
\boxed{
\frac{\partial p_v}{\partial\mathbf v}
=\frac{2g_v}{v_p}
\frac{\mathbf v}{\|\mathbf v\|}
}.
$$

它直接写入 15 维平坦梯度的 velocity 三维块。

总推力满足

$$
T=T_1+T_2.
$$

若 $p_T(T)$ 是总推力上下界罚函数，则

$$
\boxed{
\frac{\partial p_T}{\partial T_1}
=\frac{\partial p_T}{\partial T_2}
=\frac{\partial p_T}{\partial T}
}.
$$

因此代码把同一个总推力罚函数导数同时加到执行器 Jacobian 的前两行，不需要
额外构造一套 collective-thrust AutoDiff 映射。

### 19.7 优化结果

两种 yaw 的初始总时间均为 $4\ \mathrm s$。优化结果如下：

| 工况 | 初始总时间 [s] | 优化总时间 [s] | 目标评估数 | 完整包络认证 |
| --- | ---: | ---: | ---: | --- |
| yaw `0 -> 0` | 4.000000 | 3.921840 | 68 | 通过 |
| yaw `0 -> pi` | 4.000000 | 3.921932 | 87 | 通过 |

分段时间为：

| 工况 | $T_1$ | $T_2$ | $T_3$ | $T_4$ |
| --- | ---: | ---: | ---: | ---: |
| yaw `0 -> 0` | 1.268790 | 0.692229 | 0.690396 | 1.270425 |
| yaw `0 -> pi` | 1.271163 | 0.688954 | 0.692869 | 1.268946 |

首末段更长、中间两段更短，原因是 hover 边界需要在首段完成加速、末段完成减速，
而中间段可保持接近约束边界的巡航速度。

自由航点结果为：

| 工况 | 航点 1 [m] | 航点 2 [m] | 航点 3 [m] |
| --- | --- | --- | --- |
| yaw `0 -> 0` | `[3.373901, 0, -10.000059]` | `[7.500911, 0, -10.000113]` | `[11.617106, 0, -10.000089]` |
| yaw `0 -> pi` | `[3.386991, 0, -9.999555]` | `[7.494676, 0.000021, -9.999510]` | `[11.625403, 0.000039, -9.999842]` |

三个点确实参与优化，但 y/z 变化接近零。这不是自由度失效，而是当前无障碍、
无走廊、无风且起终点关于 x 轴对称，目标函数没有让轨迹横向或竖向绕行的收益。
优化主要重新分配 x 方向航点和段时间。

### 19.8 稠密指标对比

| 工况/指标 | 初始直线猜测 | 联合优化 |
| --- | ---: | ---: |
| yaw `0 -> 0`，峰值速度 [m/s] | 7.876371 | 6.049621 |
| yaw `0 -> 0`，峰值总推力 [N] | 11.586775 | 8.473927 |
| yaw `0 -> 0`，峰值机体角速度 [rad/s] | 10.324432 | 3.798220 |
| yaw `0 -> 0`，峰值转速 [rad/s] | 1891.075113 | 1617.222763 |
| yaw `0 -> 0`，峰值舵偏 [rad] | 0.769022 | 0.386757 |
| yaw `0 -> 0`，峰值舵速 [rad/s] | 185.369950 | 1.662690 |
| yaw `0 -> pi`，峰值速度 [m/s] | 7.876371 | 6.049637 |
| yaw `0 -> pi`，峰值总推力 [N] | 11.595620 | 8.475417 |
| yaw `0 -> pi`，峰值机体角速度 [rad/s] | 11.179657 | 2.783937 |
| yaw `0 -> pi`，峰值转速 [rad/s] | 1913.194024 | 1627.637959 |
| yaw `0 -> pi`，峰值舵偏 [rad] | 0.980739 | 0.356700 |
| yaw `0 -> pi`，峰值舵速 [rad/s] | 25.884049 | 0.538798 |

两个初始猜测均未通过完整认证；优化后两条轨迹均通过。yaw `0 -> pi` 初值还有
47 个无效执行器映射采样，优化结果为 0 个。

最终速度约 $6.05\ \mathrm{m/s}$，略高于 $75\%$ 规划软阈值
$6.0\ \mathrm{m/s}$，但低于完整参考包络 $8\ \mathrm{m/s}$。如果要求严格不超过
$6\ \mathrm{m/s}$，仍需增大罚权或把速度改成显式不等式硬约束。

### 19.9 梯度和回归验证

`test_joint_optimizer` 分别对 yaw `0 -> 0` 和 `0 -> pi` 的全部 13 个变量做中心
有限差分。检查覆盖：

- 四个独立 log-duration 及时间代价；
- 三个自由航点的九个位置变量；
- 速度和总推力高层罚函数；
- 机体角速度、原执行器幅值、速率和动态罚函数；
- 样条反向传播和自由时间指数映射。

当前最大缩放梯度误差为

$$
\boxed{1.73378\times10^{-8}},
$$

完整 CTest 为 `3/3 passed`。

### 19.10 固定直线、固定总时间、仅优化时间比例的正式对照

#### 19.10.1 对照定义

测试 3 的正式对照不是 `joint_hover_*_initial.csv`。该 initial CSV 只是联合优化
开始前的数值初值，没有经过优化，不能代表“固定时间规划方法”。正确对照沿用
Stage A 的方法，并只改变场景尺寸以与测试 3 一致：

- 起点和终点仍为 $(0,0,-10)$ m 与 $(15,0,-10)$ m；
- 五个位置航路点固定在同一直线上：

$$
\boxed{x=[0,3.75,7.5,11.25,15]\ \mathrm m,\quad y=0,\quad z=-10\ \mathrm m};
$$

- yaw `0 -> 0` 时所有 yaw 均为零；
- yaw `0 -> pi` 时固定为 $[0,\pi/4,\pi/2,3\pi/4,\pi]$；
- 总时间严格固定为 $T_f=4\ \mathrm s$；
- 中间位置航路点不参与优化；
- 不执行外层整体时间缩放；
- 不在目标中加入速度、总推力、机体角速度、执行器幅值、速率或动态约束。

唯一优化变量是四段时间比例。与 Stage A 相同，把最后一个 logit 固定为零，前三个
为自由变量 $\boldsymbol\tau\in\mathbb R^3$：

$$
\boxed{
T_i=4\frac{e^{\tau_i}}{\sum_{j=1}^{4}e^{\tau_j}},
\qquad \tau_4=0
}.
$$

因此 $T_i>0$ 且 $\sum_iT_i=4\ \mathrm s$ 由构造严格满足。对照目标只有

$$
\boxed{
\min_{\boldsymbol\tau}\quad
J_A=\int_0^4\|\mathbf p^{(4)}(t)\|^2dt
+\rho_\psi\int_0^4\ddot\psi(t)^2dt
}.
$$

这里没有 $\rho_TT_f$、几何偏移项或 $J_{\mathrm{constraint}}$。优化完成后的平坦
映射和 `FeasibilityChecker` 只负责输出诊断指标，不参与 L-BFGS 目标及梯度，
所以不会把执行器约束偷偷加入对照实验。

#### 19.10.2 运行命令

`generate_simple_trajectory` 新增了可选的 `--distance`、`--waypoint-count` 和
`--total-time`，未提供这些参数时仍保持原 6 m 三航路点实验不变。测试 3 对照为：

```bash
./planning/build/generate_simple_trajectory \
  --distance 15 --waypoint-count 5 --total-time 4 \
  --no-time-scaling \
  --output planning/output/joint_hover_yaw0_fixed_straight_ratio.csv

./planning/build/generate_simple_trajectory \
  --yaw-pi --distance 15 --waypoint-count 5 --total-time 4 \
  --no-time-scaling \
  --output planning/output/joint_hover_yaw_pi_fixed_straight_ratio.csv
```

与自由时间联合优化结果对比，以 yaw `0 -> pi` 为例：

```bash
python planning/scripts/plot_planning_metrics.py \
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

#### 19.10.3 优化结果

| 指标 | yaw `0 -> 0` | yaw `0 -> pi` |
| --- | ---: | ---: |
| 固定总时间 [s] | 4.000000 | 4.000000 |
| 优化分段时间 [s] | `[1.515394, 0.484606, 0.484606, 1.515394]` | `[1.515349, 0.484651, 0.484651, 1.515349]` |
| 初始平滑能量 | 61619.575614 | 61623.805445 |
| 最终平滑能量 | 1384.277344 | 1387.972260 |
| 峰值速度 [m/s] | 8.203125 | 8.202304 |
| 峰值总推力 [N] | 8.126835 | 8.141822 |
| 峰值机体角速度 [rad/s] | 3.403672 | 6.820617 |
| 峰值电机转速 [rad/s] | 1583.755823 | 1590.530562 |
| 峰值绝对舵偏 [rad] | 0.364346 | 0.449609 |
| 峰值绝对舵速 [rad/s] | 1.504476 | 4.593597 |
| 完整包络认证 | 不通过 | 不通过 |

两条对照均只有速度违反完整包络：

$$
8.203125>8.0\ \mathrm{m/s},
\qquad
8.202304>8.0\ \mathrm{m/s}.
$$

其余表中执行器、动态和机体角速度峰值都在完整物理/参考包络内。由于优化根本没有
看到速度约束，认证不通过正是有效的实验结果，而不是优化器或导出程序失败。

#### 19.10.4 与自由时间联合优化的比较

| 指标 | yaw `0 -> 0` 对照 | yaw `0 -> 0` 联合 | yaw `0 -> pi` 对照 | yaw `0 -> pi` 联合 |
| --- | ---: | ---: | ---: | ---: |
| 总时间 [s] | 4.000000 | 3.921840 | 4.000000 | 3.921932 |
| 峰值速度 [m/s] | 8.203125 | 6.049621 | 8.202304 | 6.049637 |
| 稠密积分平滑能量 | 1384.287 | 6988.171 | 1387.982 | 6988.293 |
| 峰值机体角速度 [rad/s] | 3.403672 | 3.798220 | 6.820617 | 2.783937 |
| 峰值舵速 [rad/s] | 1.504476 | 1.662690 | 4.593597 | 0.538798 |
| 完整包络认证 | 不通过 | 通过 | 不通过 | 通过 |

固定直线平滑性单目标把首末段拉长、中央两段压缩，使整条位置轨迹接近一次连续的
起停运动。这样能显著降低 snap 能量，却把中部速度峰值推到约
$8.20\ \mathrm{m/s}$。联合优化虽然总时间反而略短，但速度约束使它形成接近
$6\ \mathrm{m/s}$ 的速度平台；从加速段进入平台、再从平台减速需要更强的 snap，
因此它的平滑能量高于不考虑约束的对照。

这组结果不能解读为“联合优化的平滑性更好”。准确结论是：

1. 固定时间比例优化在其唯一目标——平滑能量——上更优，但产生不可接受的速度峰值；
2. 联合优化牺牲平滑能量，换取更短总时间和完整高层/执行器包络可行性；
3. yaw `0 -> pi` 中，加入约束后机体角速度从 $6.82$ 降到 $2.78\ \mathrm{rad/s}$，
   舵速从 $4.59$ 降到 $0.54\ \mathrm{rad/s}$，约束整形效果尤其明显。

#### 19.10.5 对比图输出

每种 yaw 都生成五张图：

```text
planning/output/figures/joint_hover_yaw0_joint_vs_fixed_straight_trajectory_speed.png
planning/output/figures/joint_hover_yaw0_joint_vs_fixed_straight_smoothness.png
planning/output/figures/joint_hover_yaw0_joint_vs_fixed_straight_actuators.png
planning/output/figures/joint_hover_yaw0_joint_vs_fixed_straight_dynamic_constraints.png
planning/output/figures/joint_hover_yaw0_joint_vs_fixed_straight_high_level_constraints.png

planning/output/figures/joint_hover_yaw_pi_joint_vs_fixed_straight_trajectory_speed.png
planning/output/figures/joint_hover_yaw_pi_joint_vs_fixed_straight_smoothness.png
planning/output/figures/joint_hover_yaw_pi_joint_vs_fixed_straight_actuators.png
planning/output/figures/joint_hover_yaw_pi_joint_vs_fixed_straight_dynamic_constraints.png
planning/output/figures/joint_hover_yaw_pi_joint_vs_fixed_straight_high_level_constraints.png
```

### 19.11 无速度约束：内层联合约束与外层时间缩放

#### 19.11.1 实验目的与公平约束集合

这一组对比不再限制最大平动速度，目的是观察执行器与姿态相关约束本身对最短时间
的限制。两种方法都忽略

$$
\|\mathbf v(t)\|\le v_{\max},
$$

但都继续检查：

$$
\boxed{
T,\ \|\boldsymbol\Omega\|,\ T_1,T_2,\ \delta_1,\delta_2,
\ \dot\delta_1,\dot\delta_2,
\ \omega_{cmd,1},\omega_{cmd,2},
\ \delta_{cmd,1},\delta_{cmd,2}
}.
$$

第一种方法是测试 3 的内层联合优化：四段时间和三个中间位置航路点同时优化，目标
仍含平滑性、总时间、几何偏移和上述约束罚函数。新增配置
`enable_velocity_constraint=false` 只令

$$
J_{velocity}=0,
\qquad
\frac{\partial J_{velocity}}{\partial\mathbf v}=\mathbf0,
$$

不会关闭同一高层模块中的总推力罚函数。第二种方法是 Stage A 外层策略：固定直线
位置航路点，在 $4$ s 下先优化时间比例，再对所有段施加统一尺度 $s$：

$$
T_i^{scaled}=sT_i^{ratio}.
$$

`TimeScaler` 在 $s\in[0.25,3]$ 粗扫全部范围，并细化每个不可行到可行的切换区间；
本实验的 `FeasibilityChecker` 同样关闭速度判据，所以两组最终认证使用相同的“除
速度外全部约束”集合。速度峰值仍被记录和绘图，只是不决定可行性。

#### 19.11.2 为什么收紧内层采样和罚权

关闭速度罚项后，默认 `50` 区间、罚权 `200` 的联合优化会主动把总时间压到约
$2.96$ s。该候选在目标采样点上的罚值很小，但 $2$ ms 稠密检查发现样条连接点
附近的舵面变化过快，所需舵机命令超过限制，因此不能作为有效结果。

最终实验采用

```text
samples_per_segment = 100
actuator_weight      = 1000
max_evaluations      = 500
```

更密的积分节点提高了连接点附近快速变化被目标捕捉的概率，更高罚权则使优化器优先
消除剩余约束违反。两条最终候选均通过 $2$ ms 稠密认证。这个过程也再次说明采样
平方罚函数不是数学硬约束，最终独立认证不能省略。

#### 19.11.3 运行命令

内层联合优化：

```bash
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
```

外层时间缩放对照：

```bash
./planning/build/generate_simple_trajectory \
  --distance 15 --waypoint-count 5 --total-time 4 \
  --no-velocity-constraint \
  --output planning/output/joint_hover_yaw0_outer_scaled_no_velocity.csv

./planning/build/generate_simple_trajectory \
  --yaw-pi --distance 15 --waypoint-count 5 --total-time 4 \
  --no-velocity-constraint \
  --output planning/output/joint_hover_yaw_pi_outer_scaled_no_velocity.csv
```

#### 19.11.4 时间、轨迹和计算结果

| 指标 | yaw `0 -> 0` 内层联合 | yaw `0 -> 0` 外层缩放 | yaw `0 -> pi` 内层联合 | yaw `0 -> pi` 外层缩放 |
| --- | ---: | ---: | ---: | ---: |
| 总时间 [s] | 3.088731 | 3.839007 | 3.073283 | 3.855887 |
| 相对外层缩短 | 19.54% | — | 20.30% | — |
| 峰值速度 [m/s] | 10.816327 | 8.547133 | 10.104029 | 8.508863 |
| 稠密积分平滑能量 | 21321.42 | 1845.50 | 11181.47 | 1793.81 |
| 目标评估数 | 500 | 一维粗扫+细化 | 501 | 一维粗扫+细化 |
| 除速度外完整认证 | 通过 | 通过 | 通过 | 通过 |

内层联合优化通过改变几何和独立段时间，把总时间进一步减少约 $20\%$；代价是速度
和 snap 显著上升。外层策略只能整体缩放已经确定的固定直线和时间比例，因此更
平滑、计算量也明显更小，但无法利用几何自由度绕开局部执行器困难。

内层优化的自由航路点为：

| 工况 | 航路点 1 [m] | 航路点 2 [m] | 航路点 3 [m] |
| --- | --- | --- | --- |
| yaw `0 -> 0` | `[3.287334, 0.066085, -9.804226]` | `[6.583536, 0.036108, -9.316275]` | `[11.132406, -0.026272, -9.248792]` |
| yaw `0 -> pi` | `[3.046483, 0.053454, -9.557499]` | `[6.989089, 0.123744, -9.187696]` | `[11.584073, 0.121615, -9.337027]` |

与有速度约束时几乎保持 $z=-10$ 的结果不同，取消速度限制后优化器明显抬高飞行
轨迹，即 NED 的 $z$ 变得更接近零。当前没有安全走廊或高度保持约束，所以这是
合法的搜索结果；如果希望比较时仍严格保持水平直线，应固定 $y,z$ 航路点或把
自由点搜索范围的对应分量设为零。

#### 19.11.5 其余约束指标

| 指标 | yaw `0 -> 0` 内层 | yaw `0 -> 0` 外层 | yaw `0 -> pi` 内层 | yaw `0 -> pi` 外层 |
| --- | ---: | ---: | ---: | ---: |
| 峰值总推力 [N] | 10.821399 | 8.358785 | 9.516372 | 8.350349 |
| 峰值机体角速度 [rad/s] | 4.043370 | 3.829850 | 8.398903 | 7.890257 |
| 峰值电机转速 [rad/s] | 1830.494281 | 1606.198014 | 1730.165372 | 1611.345490 |
| 峰值绝对舵偏 [rad] | 0.408827 | 0.381510 | 0.464738 | 0.523634 |
| 峰值绝对舵速 [rad/s] | 1.939349 | 1.874421 | 6.288327 | 7.337881 |
| 峰值所需电机命令 [rad/s] | 1834.439025 | 1607.556680 | 1732.760532 | 1612.555624 |
| 峰值所需舵机命令 [rad] | 0.397267 | 0.386128 | 0.541183 | 0.608719 |

两组不是每个执行器指标都由内层更低。例如 yaw `0 -> pi` 中，内层的舵偏、舵速
和所需舵机命令低于外层对照，但电机转速和总推力更高。这说明联合优化利用航路点
变化在电机负载与舵面负载之间重新分配控制需求，而外层统一缩放只能沿原曲线改变
所有导数的整体尺度。

yaw `0 -> pi` 的内层机体角速度峰值 $8.3989\ \mathrm{rad/s}$ 已接近 $75\%$
规划阈值 $8.5085\ \mathrm{rad/s}$，表明在没有速度约束时，机体角速度成为更主要的
活跃限制之一。内层两次运行都用满约 `500` 次评估，因此表中结果应称为当前预算
下通过认证的可行最优候选，而不是全局最短时间证明。

#### 19.11.6 对比图

绘图时使用 `--no-velocity-limit`，只隐藏没有启用的速度限制线，不隐藏速度曲线：

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

两种 yaw 各输出 trajectory-speed、smoothness、actuators、dynamic-constraints 和
high-level-constraints 五张图，文件前缀分别为：

```text
joint_hover_yaw0_inner_vs_outer_no_velocity_*
joint_hover_yaw_pi_inner_vs_outer_no_velocity_*
```

### 19.12 为什么 GCOPTER 含位置等约束仍能达到毫秒级

#### 19.12.1 结论与比较口径

GCOPTER 快的主要原因不是“约束数量少”，而是它通过 MINCO 稀疏参数化、几何约束
消元和反向梯度传播，使每一次目标函数及梯度计算都非常便宜。当前项目的主要开销
则不是自由航路点的位置边界，而是尾座式平坦输出经过气动力、姿态分支、力矩和
执行器分配之后的完整约束映射及其自动微分。

[GCOPTER 论文](https://arxiv.org/abs/2103.00190)的窄缝 SE(3) 规划实验在 Intel
Core i7-8700 上报告了约 $4.7\sim7.4\ \mathrm{ms}$ 的计算时间，但这个数字对应规模
较小、已经给定安全飞行走廊的具体轨迹优化算例，不能理解为任意地图规模、任意轨迹
段数和任意用户动力学模型都固定为毫秒级。[GCOPTER 官方实现](https://github.com/ZJU-FAST-Lab/GCOPTER)
也把走廊生成、MINCO、标准多旋翼平坦映射和 L-BFGS 组织成专门的高性能 C++ 实现。
因此，论文结果和当前程序的分钟级结果只有在统一模型、积分分辨率、停止条件、编译
选项及计时范围后才能直接比较。

#### 19.12.2 GCOPTER 的位置约束为何很便宜

对于凸多面体安全走廊

$$
\mathcal P_i=\{\mathbf p\mid A_i\mathbf p\leq\mathbf b_i\},
$$

GCOPTER 并不一定把所有半空间不等式直接交给通用约束 NLP 求解器，而是对位于相邻
走廊交集内的中间点使用光滑映射

$$
\boldsymbol\xi_i\in\mathbb R^{n_i}
\longmapsto
\mathbf q_i=f_{\mathcal P_i}(\boldsymbol\xi_i)\in\mathcal P_i.
$$

这样，优化器实际处理的是无约束变量：

$$
\min_{\boldsymbol\xi,\boldsymbol\tau}
J\!\left(\mathbf q(\boldsymbol\xi),\mathbf T(\boldsymbol\tau)\right).
$$

直接定义在决策变量上的位置合法性由参数化自动满足，不需要每一步求投影、KKT 系统
或大量不等式的可行方向。走廊顶点/重心坐标映射的主要计算只是小规模矩阵运算。轨迹
沿整段位于走廊内等连续时间条件仍可采用采样积分罚函数处理，但单个位置约束值

$$
g_p(t)=A_i\mathbf p(t)-\mathbf b_i
$$

只是多项式求值和矩阵乘法，远小于尾座式执行器映射的成本。

当前测试 3 没有安全走廊；三个自由航路点只采用有界归一化偏移，所以位置变量本身
甚至比典型 GCOPTER 安全走廊问题简单。它不是当前运行时间的主要来源。

#### 19.12.3 MINCO 如何减少优化变量和梯度成本

GCOPTER 不直接优化每段多项式的全部系数，而主要优化中间节点和分段时间：

$$
\mathbf q=[\mathbf q_1,\ldots,\mathbf q_{M-1}],
\qquad
\mathbf T=[T_1,\ldots,T_M].
$$

给定 $\mathbf q$ 和 $\mathbf T$ 后，多项式系数由最小控制量最优性条件恢复：

$$
A(\mathbf T)\mathbf c=\mathbf b(\mathbf q).
$$

矩阵 $A$ 具有带状结构，因此轨迹构造、控制能量计算和系数梯度向节点/时间的传播
都可做到

$$
O(M).
$$

当前项目使用的 `SplineTrajectory` 同样以节点和时间作为稀疏参数，并采用
$O(M)$ 的块三对角结构。因此当前程序与 GCOPTER 在样条后端上的数量级基本相同；
真正的差距在每个积分点上调用的尾座式模型和梯度计算。

#### 19.12.4 两类约束的单点成本不是一个数量级

GCOPTER 典型多旋翼算例的约束包括

$$
\|\mathbf v\|\leq v_{\max},\qquad
f_{\min}\leq f\leq f_{\max},\qquad
\|\boldsymbol\omega\|\leq\omega_{\max},
$$

以及位置半空间约束 $A\mathbf p-\mathbf b\preceq0$。位置、速度等约束只需评估
多项式及其导数；标准多旋翼平坦映射的主要形式也较短：

$$
\mathbf f=m(\mathbf a-\mathbf g),\qquad
f=\|\mathbf f\|,\qquad
R=R(\mathbf f,\psi),\qquad
\boldsymbol\omega=\boldsymbol\omega(\mathbf a,\mathbf j,\psi,\dot\psi).
$$

当前尾座式单个采样点则需要完成

$$
\begin{aligned}
(&\mathbf v,\mathbf a,\mathbf j,\mathbf s,
\psi,\dot\psi,\ddot\psi)\\
&\longrightarrow(R,\boldsymbol\Omega,\dot{\boldsymbol\Omega},T)\\
&\longrightarrow\mathbf M
\longrightarrow(T_1,T_2,\delta_1,\delta_2)\\
&\longrightarrow(\omega_1,\omega_2)
\longrightarrow
(u_{\omega,1},u_{\omega,2},u_{\delta,1},u_{\delta,2}).
\end{aligned}
$$

其中包含：

1. 多个姿态候选分支的枚举、可行性判断和连续分支选择；
2. `atan2`、三角函数、开方以及气动力耦合；
3. 角加速度、惯性力矩和差动推力计算；
4. 舵面 $2\times2$ 分配矩阵的奇异性检查和求解；
5. 负推力、非有限值和分支奇异性检查；
6. 相邻采样点之间的电机、舵机一阶动态、舵速和所需命令约束。

所以“一个位置约束”与“一个真实执行器及动态约束”虽然在数学表述中都算一个约束，
单次评估成本却可能相差几个数量级。

#### 19.12.5 当前一次目标评估实际完成多少计算

测试 3 的最终内层配置为四段轨迹、每段 `100` 个积分区间。代码在
`planning/src/actuator_penalty_optimizer.cpp:307-479` 中对每段端点也采样，因此
一次目标函数及梯度计算包含

$$
N_{map}=4(100+1)=404
$$

次完整平坦映射和线性化。随后
`planning/src/actuator_penalty_optimizer.cpp:481-599` 又对相邻采样点计算执行器动态，
一次评估包含

$$
N_{dyn}=4\times100=400
$$

个动态区间。

本次两个 yaw 工况分别使用约 `500` 次目标评估，所以每个工况累计约为

$$
404\times500\approx2.02\times10^5
$$

次带梯度的尾座式映射，以及

$$
400\times500=2.0\times10^5
$$

次相邻节点执行器动态计算。若每个工况顺序运行约 $2\sim3$ min，则摊到一次目标
及梯度评估约为

$$
t_{eval}\approx0.24\sim0.36\ \mathrm{s}.
$$

因此分钟级时间主要来自“大约半百万次昂贵的节点/区间运算”，不是因为优化变量只有
十几个却仍被求解器无缘无故拖慢。两个工况并行执行时观察到的约四分钟总墙钟时间还
包含了二者竞争 CPU 的影响。

#### 19.12.6 当前 15 方向前向自动微分的成本

当前每个采样点对以下 15 个平坦量求导：

$$
\underbrace{\mathbf v,\mathbf a,\mathbf j,\mathbf s}_{12},
\qquad
\underbrace{\psi,\dot\psi,\ddot\psi}_{3}.
$$

具体形成

$$
\frac{\partial(T_1,T_2,\delta_1,\delta_2)}
{\partial(\mathbf v,\mathbf a,\mathbf j,\mathbf s,
\psi,\dot\psi,\ddot\psi)}
\in\mathbb R^{4\times15}
$$

和

$$
\frac{\partial\boldsymbol\Omega}
{\partial(\mathbf v,\mathbf a,\mathbf j,\mathbf s,
\psi,\dot\psi,\ddot\psi)}
\in\mathbb R^{3\times15}.
$$

`Eigen::AutoDiffScalar` 的每个标量都携带一个 15 维导数向量，所以平坦映射内部
的大部分标量加减乘除和初等函数都同时传播 15 个方向。更重要的是，当前
`TailsitterFlatness::linearize()` 在
`planning/src/tailsitter_flatness.cpp:803-862` 内顺序执行：

1. 普通 `forward()`，用于选择连续姿态分支；
2. 固定分支的普通 `evaluateFixedBranchActuators<double>()`；
3. 固定分支的 `evaluateFixedBranchActuators<ADScalar>()`。

所以一个带梯度采样点实际重复执行了多次相近的平坦映射计算。

GCOPTER 更强调标量总代价的反向传播。对

$$
J=J(\mathbf y),\qquad
\mathbf y=(\mathbf v,\mathbf a,\mathbf j,\ldots),
$$

反向模式直接从约束罚函数的伴随量传播

$$
\bar{\mathbf y}=\frac{\partial J}{\partial\mathbf y},
$$

其复杂度通常是一次前向映射的常数倍，而不是让每个中间标量都携带全部输入方向。
当前输入维数为 15、输出最终被压缩成一个标量目标，因此手工反向梯度通常比完整
前向 Jacobian 更合适。

#### 19.12.7 当前还是未优化编译

当前 `planning/build/CMakeCache.txt` 中 `CMAKE_BUILD_TYPE` 为空，生成的
`planning/build/CMakeFiles/generate_joint_hover_trajectory.dir/flags.make` 只有

```text
-std=c++17
```

没有 `-O2` 或 `-O3`。对 Eigen 表达式模板、自定义 Jet 运算和 AutoDiff 来说，
未优化编译会产生非常明显的额外开销。GCOPTER 官方运行说明甚至建议把 CPU governor
设置为 `performance`，可见论文中的毫秒级数字本身就是面向性能的测试口径。

因此，当前分钟级程序与 GCOPTER 论文结果之间首先存在编译配置差异。单独建立
Release 目录可以避免覆盖现有调试构建：

```bash
cmake -S planning -B planning/build-release -DCMAKE_BUILD_TYPE=Release
cmake --build planning/build-release -j
```

Release 构建很可能显著缩短模板化数值代码的时间，但在完成实际计时前不应承诺具体
倍数，更不能据此保证完整尾座式执行器动态规划也会直接达到 $5\ \mathrm{ms}$。

#### 19.12.8 GCOPTER 的连续时间约束也不是无限精度硬约束

GCOPTER 将一般状态—输入限制写成积分罚函数：

$$
I_G=\int_0^T\max(G(t),0)^k\,dt,
$$

实际再用梯形积分离散：

$$
I_G\approx
\sum_{i=1}^{M}\frac{T_i}{\kappa_i}
\sum_{j=0}^{\kappa_i}\bar\omega_j
\max\!\left(
G\!\left(\frac{jT_i}{\kappa_i}\right),0
\right)^k.
$$

它同样是有限采样、有限罚权下的连续约束近似。论文也明确讨论：若要由这种方法让
违反量数学上严格为零，需要趋于无穷的罚权或积分分辨率。毫秒级速度因此并不等价
于以极密时间间隔完成了无限精度硬约束证明。

当前实现除了在优化中每段使用 100 个区间，还在优化结束后进行 $2\ \mathrm{ms}$
稠密独立认证，并检查实际执行器状态、相邻点反演的执行器命令和变化率。这个验证
口径比典型 GCOPTER 多旋翼示例更重。二者都使用积分罚函数思想，但不能只比较最终
墙钟时间而忽略约束模型与认证分辨率。

#### 19.12.9 差异汇总

| 项目 | GCOPTER 典型算例 | 当前尾座式联合优化 |
| --- | --- | --- |
| 位置约束 | 光滑映射消元或简单半空间代价 | 自由点有界偏移，不是主要开销 |
| 样条参数化 | MINCO，$O(M)$ | SplineTrajectory，基本同类 |
| 单点动力学 | 标准多旋翼平坦映射 | 尾座式气动、分支、力矩和执行器分配 |
| 梯度 | 解析/反向梯度传播 | 15 维前向 AutoDiff Jacobian |
| 每段积分区间 | 可采用较低有限分辨率 | 最终实验为 100 |
| 执行器动态 | 典型示例一般不含电机/舵机一阶动态 | 额外计算状态、命令和变化率 |
| 目标评估数 | 小规模问题可快速收敛 | 本次达到约 500 次评估上限 |
| 最终认证 | 有限积分罚函数精度 | 额外 $2\ \mathrm{ms}$ 稠密认证 |
| 编译/CPU | 面向性能的论文测试配置 | 当前构建无 `-O2/-O3` |

因此，GCOPTER 包含位置、速度、推力和角速度等约束却仍然很快并不矛盾：决定耗时的
是约束计算图的复杂度、采样数、梯度模式、优化评估次数和编译配置，而不是约束名称
或约束条数。

#### 19.12.10 推荐的加速实施顺序

在不删除真实执行器约束的前提下，后续应按以下顺序优化性能：

1. **建立 Release 基准**：在完全相同初值、参数和 `maxeval` 下分别记录 Debug/当前
   构建与 `-O3` Release 的单次目标梯度时间和总时间；
2. **分阶段计时**：分别统计样条更新、轨迹采样、普通平坦映射、AutoDiff 映射、
   罚函数累加、执行器动态和样条梯度回传，先确认实际热点；
3. **粗到细热启动**：先以每段 $30\sim50$ 个区间求得可行附近候选，再把该候选作为
   每段 100 区间的初值，避免高分辨率优化从等时间直线初值运行 500 次；
4. **消除重复前向计算**：缓存姿态分支和固定分支的公共中间量，避免
   `forward()`、普通固定分支和 AD 固定分支重复完成相同运算；
5. **改为标量目标的反向梯度**：不再构造每点完整 $4\times15$ 和 $3\times15$
   Jacobian，而从活跃罚函数直接向 15 个平坦量传播伴随梯度；
6. **约束筛选与自适应采样**：远离边界的区间采用较稀节点，仅在舵面、角速度或
   执行器命令快速变化处加密；最终仍保留独立 $2\ \mathrm{ms}$ 认证；
7. **改善变量尺度和终止条件**：时间、位置、平滑能量和高权重罚项需要量纲一致的
   尺度，减少 L-BFGS 在线搜索中因刚性罚函数造成的无效评估。

第一至第三步不改变数学模型，应该优先实施。第四、第五步最可能带来算法层面的主要
加速，但需要重新做解析梯度—有限差分一致性测试。即使这些步骤全部完成，保留完整
尾座式气动、执行器分配和执行器动态后，也不应预先把 $4.7\sim7.4\ \mathrm{ms}$
作为必须达到的指标；应先依据 Release 性能剖析设定更现实的目标。

## 20. 使用 `paper_control` 控制器跟踪测试 3 轨迹

### 20.1 闭环连接原则

测试 3 输出的是平坦输出参考，而不是应该直接施加到仿真模型的执行器命令。闭环
仿真只读取

$$
\boxed{
\mathbf p_d,\ \mathbf v_d,\ \mathbf a_d,\ \mathbf j_d,
\ \psi_d,\ \dot\psi_d
}
$$

并继续使用项目中已经复现的 `paper_control` 控制链：

$$
\begin{aligned}
(\mathbf p_d,\mathbf v_d,\mathbf a_d)
&\xrightarrow{\text{位置/速度 PD}}
\mathbf a_c \\
&\xrightarrow{\text{线加速度 INDI}}
\mathbf f_c^i \\
&\xrightarrow{\text{全局平坦变换}}
(q_c,T_c) \\
(q_c,\mathbf j_d,\psi_d,\dot\psi_d)
&\xrightarrow{\text{姿态 PD + 角速度前馈}}
\dot{\boldsymbol\omega}_c \\
&\xrightarrow{\text{角加速度 INDI}}
\mathbf m_c \\
(T_c,\mathbf m_c)
&\xrightarrow{\text{控制分配}}
(\omega_{1,c},\omega_{2,c},\delta_{1,c},\delta_{2,c}).
\end{aligned}
$$

因此规划 CSV 中的 `omega*_pred`、`delta*_pred` 仍只是规划阶段的开环可行性
预测。闭环仿真不读取这些列作为前馈执行器命令；电机和舵机命令由控制器根据
实际状态误差在线重新计算。这保持了“规划器给参考、控制器负责抗模型误差”的
正确分层。

### 20.2 CSV 连续时间接口

新增 `trajectory/planning_csv.py`，将规划器 CSV 包装为 `main_sim.py` 所需的
统一 `sample(t)` 接口。必需列为：

```text
t
px py pz
vx vy vz
ax ay az
jx jy jz
yaw yaw_rate yaw_acceleration
```

规划稠密检查以不超过 $2\ \mathrm{ms}$ 的间隔输出，而闭环控制频率为
$500\ \mathrm{Hz}$，周期同样为 $2\ \mathrm{ms}$。由于各段会显式插入真实样条
连接点，连接点和相邻规则采样有时只相差约 $0.1\ \mathrm{ms}$。如果直接用这些
相邻行的有限精度十进制数重新构造七次 Hermite 多项式，高阶系数会被极短时间
间隔放大，产生原轨迹不存在的 jerk 尖峰。

因此当前实现对 CSV 已经给出的每个平坦量分别做线性时间插值：

$$
y(t)=(1-\lambda)y_k+\lambda y_{k+1},
\qquad
\lambda=\frac{t-t_k}{t_{k+1}-t_k}.
$$

这里 $y$ 分别取 $\mathbf p,\mathbf v,\mathbf a,\mathbf j,\psi,\dot\psi$。它们在
连续时间意义下不是严格由同一个插值多项式逐级求导得到，但最大插值区间只有
$2\ \mathrm{ms}$，比在极短连接点区间重构高阶多项式更稳健。若以后需要更高
精度，推荐规划器直接导出每一段的多项式系数和持续时间，而不是从稠密 CSV
反推系数。

### 20.3 初值和末端悬停

两条规划轨迹都满足起点 hover 条件：

$$
\mathbf v_d(0)=\mathbf a_d(0)=\mathbf j_d(0)=\mathbf 0.
$$

闭环仿真使用 `initial_state_mode="hover"`，在规划起点位置和 yaw 下初始化真实
悬停姿态、电机转速、舵面配平以及 INDI 滤波器状态。因此结果不包含从地面起飞
到规划起点的额外瞬态。

默认在规划时间 $T_f$ 后增加 $1\ \mathrm s$ 末端保持：

$$
\mathbf p_d(t)=\mathbf p_d(T_f),\qquad
\mathbf v_d=\mathbf a_d=\mathbf j_d=\mathbf0,
\qquad t\ge T_f.
$$

这 $1\ \mathrm s$ 不属于优化所得总时间，只用于观察无人机到达末端 hover 后的
误差收敛。统计中“规划段 RMS/最大误差”只使用 $0\le t\le T_f$，而“最终误差”
取末端保持结束时的误差。

### 20.4 运行命令

```bash
python main_sim.py --trajectory planning-yaw0 \
  --log planning/output/joint_hover_yaw0_tracking.npz \
  --hold-duration 1.0 --no-show

python main_sim.py --trajectory planning-yaw-pi \
  --log planning/output/joint_hover_yaw_pi_tracking.npz \
  --hold-duration 1.0 --no-show

python planning/scripts/plot_tracking_results.py
```

任意同格式 CSV 可以通过通用入口运行：

```bash
python main_sim.py --trajectory planning-csv \
  --trajectory-csv planning/output/my_trajectory.csv \
  --log planning/output/my_trajectory_tracking.npz \
  --hold-duration 1.0 --no-show
```

### 20.5 闭环跟踪结果

两条轨迹均完成有限、稳定的六自由度闭环仿真。规划段结果为：

| 指标 | yaw `0 -> 0` | yaw `0 -> pi` |
| --- | ---: | ---: |
| 规划时间 [s] | 3.921840 | 3.921932 |
| 位置 RMS 误差 [m] | 0.2021 | 0.2371 |
| 位置最大误差 [m] | 0.3811 | 0.4854 |
| 末端保持结束位置误差 [m] | 0.0607 | 0.2031 |
| 速度 RMS 误差 [m/s] | 0.4022 | 0.5730 |
| 速度最大误差 [m/s] | 0.8824 | 1.4026 |
| yaw RMS 误差 [deg] | 近似 0 | 1.1715 |
| yaw 最大误差 [deg] | 近似 0 | 4.0497 |
| 末端 yaw 误差 [deg] | 0 | 0.0029 |

yaw `0 -> 0` 工况始终保持完全对称，仿真中的 yaw 数值误差在
$10^{-13}\ \mathrm{deg}$ 量级。yaw `0 -> pi` 工况需要差动推力和非对称舵面动作，
同时引入横向和高度耦合，因此位置、速度误差高于不变 yaw 工况。它仍能在规划段
内把 yaw 最大误差保持在约 $4.05^\circ$，并在末端基本准确到达
$180^\circ$。

### 20.6 执行器和机体角速度结果

| 指标 | yaw `0 -> 0` | yaw `0 -> pi` | `config.py` 物理上限/包络 |
| --- | ---: | ---: | ---: |
| 实际最大电机转速 [rad/s] | 1670.43 | 1643.74 | 2500 |
| 最大电机指令 [rad/s] | 1677.69 | 1649.02 | 2500 |
| 实际最大绝对舵偏 [deg] | 22.71 | 30.00 | 57.30 |
| 最大绝对舵面指令 [deg] | 23.32 | 30.74 | 57.30 |
| 实际最大机体角速度 [rad/s] | 3.5193 | 4.7625 | 11.3446 参考包络 |
| 最大前馈角速度 [rad/s] | 2.9340 | 3.4333 | 11.3446 参考包络 |

两组实际执行器和控制器命令均未触及 `config.py` 的物理限幅。实际机体角速度峰值
高于平坦前馈角速度峰值，是姿态反馈和 INDI 为纠正跟踪误差产生的闭环动作，但仍
显著低于当前 $650\ \mathrm{deg/s}$ 参考包络。

这里还应注意：当前规划 CSV 使用默认 `paper_vehicle` 参数，即
$m=0.68\ \mathrm{kg}$、$c_T=1.62\times10^{-6}$；闭环仿真使用 `config.py` 的
$m=0.7\ \mathrm{kg}$、$c_T=1.8\times10^{-6}$。闭环仍成功跟踪说明控制器可以
补偿这部分参数差异，但开环预测峰值与闭环真实执行器峰值不应直接逐点等同。如果
要进行严格同模型对照，应使用 `--simulation-vehicle` 重新规划后再运行闭环。

### 20.7 输出文件

闭环日志为：

```text
planning/output/joint_hover_yaw0_tracking.npz
planning/output/joint_hover_yaw_pi_tracking.npz
```

除原有状态、参考和执行器记录外，日志新增：

```text
rotor_speed_command
flap_command_deg
planned_duration
trajectory_source
```

绘图结果为：

```text
planning/output/figures/joint_hover_closed_loop_overview.png
planning/output/figures/joint_hover_closed_loop_actuators.png
```

第一张图对比空间轨迹、位置误差、速度和 yaw；第二张图对比电机实际/指令、舵面
实际/指令以及机体角速度。竖直虚线表示优化轨迹结束，虚线右侧为额外的 $1\ \mathrm s$
末端悬停，而不是规划总时间的一部分。

---

## 21. 阶段 C：显式不等式约束联合优化

对应根目录 `study.md` 第 13.6 节路线的第三步。新增实现位于：

```text
include/planning/actuator_penalty_optimizer.hpp   ConstrainedSolver 枚举与新结构
src/actuator_penalty_optimizer.cpp                分类型梯度与 optimizeJointConstrained
apps/generate_stage_c_trajectory.cpp              阶段 C 应用
tests/test_stage_c.cpp                            回归测试
include/planning/trajectory_csv.hpp               共享 CSV 写出
```

### 21.1 从软罚到硬约束的精确改动

阶段 B/测试 3 的目标把十类执行器与高层积分乘以统一罚权加进目标；阶段 C
把它们逐类注册为 NLopt 不等式行：

```text
c_k(x) = ∫ [g_k(t)]_+^2 dt ≤ 0,   k = 1..10
```

其中 $g_k$ 是与软罚完全相同的归一化铰链输入（转速/推力/舵偏/机体角速度/
速度/总推力/电机动态/舵速/舵机动态）。由于平方铰链非负，$c_k=0$ 当且仅当
该类型在优化网格的所有采样都位于 `actuator_margin` 规划包络内，与逐采样
硬约束的可行集完全一致，同时每类只需一行约束而不是数百行。无效平坦映射
没有可用梯度，保持 `invalid_mapping_penalty` 屏障在目标中；构造失败的
候选返回目标 $10^{30}$、全部约束行取正值 1、梯度置零。

目标退化为：

$$
J_C=\rho_s\frac{J_{\mathrm{smooth}}}{J_0}
+\rho_g\|\mathbf u\|^2
+\rho_T T
+\rho_a J_{\mathrm{invalid}}
$$

时间项 $\rho_T T$ 保留：没有它，任何满足约束的慢轨迹都是同等最优的。

### 21.2 分类型梯度重构

`integrateActuatorPenalty` 现在按 11 类分别积累系数/显式时间偏导，整体偏导
等于逐类之和，因此软罚路径数值不变（`test_joint_optimizer` 的梯度误差仍为
1.73e-8，与历史值一致）。阶段 C 对每类各做一次 `propagateCostGradient`，
组装 10×13 约束 Jacobian；目标只传播 invalid 屏障类。NLopt 的目标回调和
向量约束回调共享带一步缓存的单次平坦映射积分。

求解器选择经过消融：从深度不可行初值（舵机动态超限约 30 倍）出发，
`LD_SLSQP` 直接求解在 120 次评估内到达可行；`auglag-mma`、`auglag-ccsaq`、
`auglag-slsqp` 与直接 `mma` 在 240 次评估预算内均未达标。默认
`LD_SLSQP`，其余通过 `--solver` 保留。

### 21.3 热启动与冷启动限制

`optimizeJointConstrained` 接受可选热启动（对数时间比与归一化航点偏移），
应用层默认先跑软罚联合优化再做硬约束精修。原因：yaw `0 -> pi` 的直线初值
含无效映射样本，屏障区域零梯度让 SLSQP 在不可行点提前收敛（100 次评估、
最终速度 8.67 m/s）；热启动后同样问题 67 次评估收敛且全部约束行 ≤ 1e-6。
yaw `0 -> 0` 冷启动可用。热启动点会被裁剪进变量边界。

### 21.4 节点 yaw jerk 非光滑性（阶段 C 排查中发现）

约束 Jacobian 有限差分校验曾出现与步长无关的 0.2-0.7% 偏差，定位过程与
结论如下：

1. 单侧差分互相一致，说明值函数光滑、解析梯度确有偏差；
2. 用公开 API 从零重组梯度得到与库完全一致的结果，装配无误；
3. 合成实验证明样条传播机制对"移动采样点偏导"精确（7.8e-8）；
4. 逐点对比 $d\omega/dt$：机制值 $J\,\dot\xi$ 与值路径数值导数在分段节点
   处差 0.4-0.75%，且节点两侧机制值之差恰为中心差分的两倍。

根因：yaw 三次样条只有 $C^2$ 连续，yaw jerk 在节点跳变；电机转速经角加速度
链依赖 yaw 加速度，故 $d\omega/dt$ 含 $J_{\ddot\psi}\dddot\psi$ 项在节点
不连续。动力学约束的移动采样项在节点采样处因此只有单侧导数，而中心差分
给出两侧平均。该偏差在阶段 C 之前同样存在于组合目标梯度，只是被平滑项的
相对尺度掩盖。这不是错误而是"罚函数采样点恰好落在节点上"的固有性质；
若要消除，需要让样条求值显式支持按区间取单侧导数。回归测试阈值因此设为
1e-2，全零或符号错误仍会以 O(1) 误差暴露。

### 21.5 对照结果与读法

15 m、速度约束开启、margin 0.75、每段 50 采样：

| 方法 | yaw0 [s] | yawpi [s] | 规划包络 | 全包络认证 |
| --- | ---: | ---: | --- | --- |
| 外层时间缩放 | 4.102958 | 4.101423 | 直接用全包络 | 通过（速度 7.997-7.999） |
| 软罚联合 | 3.921840 | 3.921941 | 峰值速度超软阈值约 0.8% | 通过 |
| 阶段 C（热启动） | 3.949076 | 3.948889 | 严格满足（≤4.4e-4） | 通过（速度 6.002 贴界） |

阶段 C 比软罚慢 0.7%，交换到的是规划裕量下的硬保证；软罚解若要同样严格
需要更大的罚权并承担病态风险。与外层缩放相比，联合方法快约 4.4-4.7%，
因为航点几何同时参与优化。所有结果仍以 margin 1.0 的 2 ms 稠密
`FeasibilityChecker` 认证为准——优化网格上的硬约束不排除网格间峰值。

### 21.6 规划耗时优化：惰性 Jacobian 与粗到细热启动

初版阶段 C 完整管线约 22-25 s。微基准定位到单次梯度评估的耗时分布：

```text
linearize（15 维 AutoDiff 固定分支）   约 837 us/采样
forward（Jet2 前向映射）               约  46 us/采样
样条重建 + 平滑能量                     约 180 us/次
```

50 采样/段、4 段时每次梯度评估约 204 次线性化，占评估成本的九成。两项
优化（都不触碰最终 2 ms 稠密认证）：

1. 惰性 Jacobian。`integrateActuatorPenalty` 改为三遍结构：值遍对所有
   采样只调 `forward()` 并计算罚值/显式时间偏导；标记遍找出携带梯度质量的
   采样（任一幅值铰链激活，或邻接动力学区间的端点槽位非零）并缓存动力学
   端点槽位；线性化遍只对被标记采样调用新增的
   `linearize(sample, prediction, roll_branch, pitch_branch)` 重载——它
   复用值遍的预测和已选姿态分支，省去重复前向。铰链未激活的采样对每类
   平坦梯度的贡献恰为零，因此结果与全量线性化在浮点舍入意义下等价；
   全部有限差分梯度校验与端到端约束满足回归保持通过。收益随解接近可行
   而增大：深度不可行时几乎无节省，接近最优时多数采样被跳过。

2. 粗到细热启动。热启动的软罚预优化在 `20` 采样/段的粗网格上运行
   （单次评估约为细网格的 1/3 成本），SLSQP 精修保持在 `50` 采样/段。
   粗网格分辨率存在甜点：yaw `0->pi` 在 15 采样时 L-BFGS 需要 260 次评估
   才收敛，20 采样只需 64 次，25 采样 93 次；默认取 20。

同一台机器、15 m 工况、三轮平均（含最终认证与 CSV 写出）：

| 方法 | 优化前 | 优化后 |
| --- | ---: | ---: |
| 外层时间缩放 | ~9.5 s | ~9.3 s（未改动） |
| 软罚联合 | ~13.1 s | ~4.2 s |
| 阶段 C 完整管线，yaw `0->0` | ~22.0 s | ~3.5 s |
| 阶段 C 完整管线，yaw `0->pi` | ~25.3 s | ~4.4 s |

优化后的轨迹质量不变：两种 yaw 工况总时间均为 3.948-3.949 s（与优化前
差异小于 0.02%），约束行全部满足，峰值速度仍精确停在 6.0 m/s 规划边界，
2 ms 全包络稠密认证通过。`test_stage_c` 新增粗网格软罚热启动到约束求解器
的完整路径覆盖。
