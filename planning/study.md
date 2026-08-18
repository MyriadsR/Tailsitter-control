# planning 阶段 A 项目详解

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
14. [SepticSplineND 的数学原理与逐函数说明](#14-septicsplinend-的数学原理与逐函数说明)
15. [Jet2 与 tailsitter_flatness.cpp 逐行数学说明](#15-jet2-与-tailsitter_flatnesscpp-逐行数学说明)

---

## 1. 项目目标

`planning/` 实现了 `paper_planning` 的第一阶段轨迹生成流程：

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

这里实现的是阶段 A，而不是第 13 章所讨论的位置、时间和执行器约束联合优化。几何航点在整个优化过程中不会移动，执行器约束也没有被写入 L-BFGS 的目标函数或梯度中。

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
│   ├── tailsitter_flatness.hpp
│   ├── feasibility_checker.hpp
│   └── time_scaler.hpp
├── src/
│   ├── flat_output_trajectory.cpp
│   ├── time_allocation.cpp
│   ├── tailsitter_flatness.cpp
│   ├── feasibility_checker.cpp
│   └── time_scaler.cpp
├── apps/
│   └── generate_simple_trajectory.cpp
├── scripts/
│   └── plot_planning_metrics.py
├── tests/
│   └── test_stage_a.cpp
└── third_party/
    └── SplineTrajectory/          # Git submodule
```

各模块之间的依赖关系如下：

| 模块 | 主要输入 | 主要输出 | 作用 |
| --- | --- | --- | --- |
| `planning_types` | 无 | 公共数据类型 | 定义轨迹采样、车辆参数和可行性报告 |
| `flat_output_trajectory` | 航点、分段时间 | 平坦输出及各阶导数 | 包装位置和 yaw 两套样条 |
| `time_allocation` | 固定总时间、航点 | 最优分段时间 | 优化平滑能量且保持总时间不变 |
| `tailsitter_flatness` | 单个平坦输出采样 | 原始执行器预测 | 实现论文开环式 (20)～(56) |
| `feasibility_checker` | 完整组合轨迹 | 可行性报告 | 稠密调用开环映射并检查限制 |
| `time_scaler` | 基础分段时间、航点 | 最小可行缩放结果 | 粗扫描加切换区间二分 |
| `generate_simple_trajectory` | 命令行选项 | 控制台报告和 CSV | 串起阶段 A 的完整示例 |
| `plot_planning_metrics.py` | 规划 CSV | 三类规划指标图 | 绘制速度着色轨迹、平滑性和执行器预测 |
| `test_stage_a` | 固定测试轨迹 | 测试通过/失败 | 验证梯度、动力学重构和边界行为 |

---

## 3. 构建系统与第三方依赖

### 3.1 CMake 目标

`CMakeLists.txt` 要求 C++17，并生成三个目标：

- `tailsitter_planning`：包含五个核心 `.cpp` 文件的库目标，默认构建为静态库；
- `generate_simple_trajectory`：简单轨迹生成程序；
- `test_stage_a`：由 CTest 调用的阶段 A 回归测试。

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

本项目只复用 SplineTrajectory 的样条构造、求值、能量和解析时间梯度：

```cpp
using PositionSpline = SplineTrajectory::SepticSplineND<3>;
using YawSpline = SplineTrajectory::CubicSplineND<1>;
```

- 七次样条最小化四阶导数平方积分，对应 minimum-snap / MINCO S4；
- 三次样条最小化二阶导数平方积分，对应 minimum-yaw-acceleration / MINCO S2；
- 两套样条使用完全相同的物理分段时间。

当前没有使用 SplineTrajectory 自带的完整 `SplineOptimizer`。原因是本阶段只需要联合两种不同阶次样条的能量，并共享一个时间变量，直接调用 `getEnergy()` 和 `getEnergyGradTimes()` 更清楚。

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

`crackle` 和 `yaw_jerk` 已预留，便于以后实现更高阶代价、连续时间梯度或执行器速率约束。

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
- 数值映射是否有效及失败原因。

`valid=true` 只表示：公式计算成功、没有奇异矩阵、没有负单电机推力、结果均为有限数。它不表示执行器没有超过上限。执行器范围由 `FeasibilityChecker` 另外判断。

### 4.5 `FeasibilityReport`

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

### 12.2 还不是联合优化

当前 L-BFGS 只优化固定总时间内的时间比例，不优化：

- 中间位置；
- 中间 yaw；
- 几何走廊；
- 执行器约束罚函数。

第 13 章的联合优化需要把开环平坦映射对样条状态的梯度继续传播到航点和时间变量，或者使用自动微分/数值梯度构建非线性约束。

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
ab,\ 
\dot a\,b+a\,\dot b,\ 
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
\frac1x,\ 
-\frac{\dot x}{x^2},\ 
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
\sin x,\ 
\cos x\,\dot x,\ 
\cos x\,\ddot x-\sin x\,\dot x^2
\right),
$$

$$
\mathcal J_2[\cos x]
=
\left(
\cos x,\ 
-\sin x\,\dot x,\ 
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

Crackle 和 yaw Jerk 只有在继续计算力矩变化率、执行器速率前馈或更高阶
动态时才会需要。

### 15.20 <code>Jet2</code> 的数值边界与阅读要点

使用该实现时应特别注意：

- <code>inverse()</code> 和普通除法不检查分母，调用处必须提前做奇异性检查；
- <code>sqrt()</code> 不检查定义域，调用处必须保证输入非负；
- <code>atan2()</code> 在原点附近冻结导数，这保证数值有限，但会牺牲该处
  导数的物理解释；
- 四元数只用于选择离散姿态分支，姿态导数来自连续的 roll、pitch、yaw jet，
  不是对四元数做有限差分；
- <code>Jet2</code> 只传播时间导数。若要优化车辆参数、航点或执行器罚函数，
  仍需另外使用第 14.16 节的参数梯度、自动微分或伴随方法；
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
