# Tailsitter-control

基于论文全局微分平坦变换与增量非线性动态逆（INDI）的飞翼尾座无人机闭环仿真项目。项目包含六自由度刚体动力学、φ-theory 气动力模型、位置与姿态控制、控制分配、论文试验轨迹、三维姿态动画以及 Fig. 6～Fig. 11 风格的结果绘图程序。

## 1. 项目结构

```text
Tailsitter-control/
├── main_sim.py                    # 闭环仿真入口、执行器动态、INDI 滤波与日志保存
├── config.py                      # 质量、惯量、几何、气动、执行器和控制增益
├── animate_flight.py              # 位置与姿态的三维动画
├── vis_vtol.stl                   # 三维动画使用的尾座飞行器模型
├── setup_vtol_sim_env.sh          # Conda 环境创建/更新脚本
│
├── controller/
│   ├── pos_ctrl.py                # 位置/速度 PD，论文 Eq. (41)
│   ├── INDI_acc_ctrl.py           # 线加速度 INDI，论文 Eq. (46)
│   ├── att_ctrl.py                # 平坦性姿态解算、角速度前馈、姿态 PD、角 INDI
│   └── actuator_ctrl.py           # 推力/力矩到双电机和双舵面的控制分配
│
├── environment/
│   ├── aerodynamics.py            # φ-theory 合力、力矩及瞬态舵面力
│   ├── dynamics.py                # 四元数六自由度刚体动力学
│   └── wind.py                    # 无风、常值风、正弦阵风和时间窗模型
│
├── trajectory/
│   ├── __init__.py
│   └── test.py                    # 分段轨迹及论文 Sec. VI-B～VI-F 轨迹
│
├── utils/
│   ├── filters.py                 # 15 Hz/1 Hz 二阶 Butterworth 在线滤波器
│   └── math_help.py               # ZXY 欧拉角到四元数等数学工具
│
├── results/
│   ├── Lemniscate/                # Fig. 6 风格绘图
│   ├── KnifeEdgeTrasition/        # Fig. 7 风格绘图（目录名沿用现有拼写）
│   ├── CircleFlight/              # Fig. 8、Fig. 9 风格绘图
│   ├── CircleTransition/          # Fig. 10 风格绘图
│   ├── DifferentialThrustTurn/    # Fig. 11 风格绘图
│   └── WindDisturbance/           # 风速、风致外力/力矩和跟踪误差诊断图
│
├── paper/                         # 论文解析文本、页面结构和图片（Git 忽略）
├── study.md                       # 论文公式、坐标变换和控制逻辑学习笔记
├── log.md                         # 主要问题、修改内容及验证记录
├── validation_logs/               # 各轨迹 NPZ 日志（运行后生成，Git 忽略）
└── trajectory_animations/         # GIF/MP4 动画（运行后生成，Git 忽略）
```

## 2. 控制与仿真流程

```text
参考位置、速度、加速度、jerk、yaw
                  ↓
      位置/速度 PD（Eq. 41）
                  ↓
       线加速度 INDI（Eq. 46）
                  ↓
  全局微分平坦变换：期望姿态和总推力
                  ↓
 姿态 PD + jerk 角速度前馈（Eq. 35、49）
                  ↓
       角加速度 INDI（Eq. 52）
                  ↓
    双电机/双舵面控制分配（Eq. 37～40）
                  ↓
       电机与舵机执行器动态
                  ↓
      气动力模型 + 六自由度动力学
                  ↓
     加速度、角加速度和执行器滤波反馈
```

当前仿真使用 `500 Hz` 控制与滤波更新频率，以兼顾论文滤波结构和运行速度。状态使用四元数积分，积分器为固定步长 RK4。IMU、转速和舵面测量使用匹配的 `15 Hz` 二阶 Butterworth 主低通，舵面瞬态由后续 `1 Hz` 二阶 Butterworth 高通提取。论文实机更新率为 `2000 Hz`。

相较原来的 100 Hz 仿真，500 Hz 的完整多圈轨迹仍会增加一定运行时间，但明显快于严格采用 2000 Hz。

## 3. 坐标与数据约定

- 世界坐标系采用 NED：`X` 向北/前、`Y` 向东/右、`Z` 向下。
- 三维动画和结果图为了直观显示高度，将 `z_NED` 取负，显示为 North-East-Up。
- 四元数顺序为 `[qw, qx, qy, qz]`，表示机体系到世界系的旋转。
- 欧拉角采用 ZXY 顺序 `[psi, phi, theta]`，日志中的单位为度。
- 电机转速单位为 `rad/s`，日志中的舵面偏角单位为度。
- 动画默认使用 `vis_vtol.stl` 表示飞行器。模型轴映射为：STL `+x -> b_z`、
  STL `+y -> -b_y`、STL `+z -> b_x`；`--show-body-axes` 可临时叠加机体轴校验方向。

ZXY 欧拉角在奇异姿态附近可能跳变，它主要用于绘图；控制器内部姿态使用四元数。

## 4. 环境安装

推荐使用项目脚本创建 Python 3.12.12 环境：

```bash
cd /home/zr/Tailsitter-control
bash setup_vtol_sim_env.sh
conda activate vtol_sim
```

脚本安装以下依赖：

- NumPy
- SciPy
- Matplotlib
- PyVista / VTK
- ImageIO / ImageIO-FFmpeg

也可以指定环境名或重新创建环境：

```bash
bash setup_vtol_sim_env.sh --name vtol_sim --python 3.12.12
bash setup_vtol_sim_env.sh --name vtol_sim --python 3.12.12 --recreate
```

## 5. 快速运行

基础分段轨迹依次完成起飞、悬停、向前移动、悬停、向右移动和悬停：

```bash
cd /home/zr/Tailsitter-control
conda activate vtol_sim
python main_sim.py --trajectory segments
```

默认会打开位置、ZXY 欧拉角和执行器时序图，并把日志保存为 `flight_log.npz`。无图形界面运行时建议使用：

```bash
python main_sim.py --trajectory segments \
  --log flight_log_segments.npz --no-show
```

查看完整参数：

```bash
python main_sim.py --help
```

## 6. 运行论文轨迹

先创建日志目录：

```bash
mkdir -p validation_logs
```

分别运行各轨迹：

```bash
# Sec. VI-B：Bernoulli Lemniscate
python main_sim.py --trajectory lemniscate \
  --log validation_logs/lemniscate.npz --no-show

# Sec. VI-C：椭圆跑道上的 knife-edge/coordinated 转换
python main_sim.py --trajectory knife-edge-transition \
  --log validation_logs/knife-edge-transition.npz --no-show

# Sec. VI-D：协调圆周飞行
python main_sim.py --trajectory circle-coordinated \
  --log validation_logs/circle-coordinated.npz --no-show

# Sec. VI-D：定常刀锋圆周飞行
python main_sim.py --trajectory circular-knife-edge \
  --log validation_logs/circular-knife-edge.npz --no-show

# Sec. VI-E：圆周上从悬停过渡到协调前飞
python main_sim.py --trajectory transition-to-forward \
  --log validation_logs/transition-to-forward.npz --no-show

# Sec. VI-E：圆周上从协调前飞过渡到悬停
python main_sim.py --trajectory transition-to-hover \
  --log validation_logs/transition-to-hover.npz --no-show

# Sec. VI-F：差分推力快速掉头
python main_sim.py --trajectory differential-turn \
  --log validation_logs/differential-turn.npz --no-show
```

当前可选轨迹如下：

| 命令名称 | 轨迹类 | 默认参数 |
|---|---|---|
| `segments` | `TrajectoryGenerator` | 高度 10 m，前移 10 m，右移 10 m |
| `lemniscate` | `LemniscateTrajectory` | 6 m/s，7 s/圈，8 圈 |
| `knife-edge-transition` | `KnifeEdgeTransitionTrajectory` | 6 m/s，6.25 s/圈，8 圈 |
| `circle-coordinated` | `CircularTrajectory` | 半径 3.5 m，8.1 m/s，4 圈，协调飞行 |
| `circular-knife-edge` | `CircularTrajectory` | 半径 3.5 m，8.1 m/s，4 圈，刀锋飞行 |
| `transition-to-forward` | `CircularTransitionTrajectory` | 3 s 内从 0 加速至 8.1 m/s |
| `transition-to-hover` | `CircularTransitionTrajectory` | 3 s 内从 8.1 m/s 减速至 0 |
| `differential-turn` | `DifferentialThrustTurnTrajectory` | 7 m/s，0.52 s 完成 π yaw 旋转 |

`circle-knife-edge` 是兼容旧命令的别名，实际等价于 Sec. VI-C 的 `knife-edge-transition`。Sec. VI-D 的全程刀锋圆周应使用 `circular-knife-edge`。

### 覆盖轨迹参数

```bash
# 修改闭合轨迹圈数和速度
python main_sim.py --trajectory circle-coordinated \
  --speed 7.8 --laps 2 \
  --log validation_logs/circle-coordinated.npz --no-show

# 修改 Lemniscate 的速度、单圈时间和圈数
python main_sim.py --trajectory lemniscate \
  --speed 6.0 --lap-time 7.0 --laps 8 \
  --log validation_logs/lemniscate.npz --no-show

# 修改椭圆过渡轨迹
python main_sim.py --trajectory knife-edge-transition \
  --speed 6.0 --lap-time 6.25 --laps 8 \
  --log validation_logs/knife-edge-transition.npz --no-show
```

参数适用范围：

- `--laps`：Lemniscate、椭圆过渡和两种定常圆周轨迹；
- `--lap-time`：Lemniscate 与椭圆过渡轨迹；
- `--speed`：除 `segments` 外的轨迹。对圆周过渡，它表示 3 s 过渡结束时的目标速度。

### 加入风扰

风速采用世界 NED 坐标系，表示空气团相对地面的速度。常值北向风示例：

```bash
python main_sim.py --trajectory lemniscate --laps 1 \
  --wind-mode constant --wind-velocity 2.0 0.0 0.0 \
  --log validation_logs/lemniscate-constant-wind.npz --no-show
```

在平均风上叠加逐轴同频正弦阵风：

```bash
python main_sim.py --trajectory lemniscate --laps 1 \
  --wind-mode sinusoidal --wind-velocity 1.0 0.0 0.0 \
  --gust-amplitude 0.5 0.3 0.0 --gust-frequency 0.5 \
  --log validation_logs/lemniscate-gust.npz --no-show
```

只在指定时间区间加入风扰，例如在 `2.0 ≤ t < 5.0 s` 期间施加常值风：

```bash
python main_sim.py --trajectory lemniscate --laps 1 \
  --wind-mode constant --wind-velocity 2.0 0.0 0.0 \
  --wind-start 2.0 --wind-end 5.0 \
  --log validation_logs/lemniscate-windowed-wind.npz --no-show
```

`--wind-start` 默认是 `0 s`，`--wind-end` 默认持续到仿真结束。时间窗采用
左闭右开的 `[wind_start, wind_end)` 约定，即开始时刻启用、结束时刻关闭。

风扰相关参数：

| 参数 | 含义 | 默认值 |
|---|---|---|
| `--wind-mode` | `none`、`constant` 或 `sinusoidal` | `none` |
| `--wind-velocity WN WE WD` | 常值风或正弦风的平均 NED 风速，单位 m/s | `0 0 0` |
| `--gust-amplitude GN GE GD` | 正弦阵风的逐轴振幅，单位 m/s | `0 0 0` |
| `--gust-frequency` | 正弦阵风频率，单位 Hz | `0.5` |
| `--wind-start` | 风扰开始时刻，包含该时刻，单位 s | `0` |
| `--wind-end` | 风扰结束时刻，不包含该时刻，单位 s | 仿真结束 |

也可以在 Python 中组合风场，并通过 RotorPy 风格的 `update(t, position)`
接口传给仿真：

```python
from environment.wind import ConstantWind, TimeWindowWind
from main_sim import main

wind = TimeWindowWind(
    ConstantWind([2.0, 0.0, 0.0]),
    start_time=2.0,
    end_time=5.0,
)
main(
    trajectory_name="lemniscate",
    laps=1,
    log_path="validation_logs/lemniscate-windowed-wind.npz",
    show_plot=False,
    wind_profile=wind,
)
```

风只改变真实仿真对象的空速气动力，平坦性变换、控制分配和 INDI
预测模型仍使用论文的名义地速模型。这样风作为 `f_ext`、`m_ext` 进入动力学，
可用于检验控制器对未建模扰动的鲁棒性。`--wind-mode none` 为默认值，保持原仿真行为。

当前论文气动力模型的零升力坐标系横向力分量为零，因此侧风不会产生独立的
侧向阻力；它只会通过现有升力、舵面力和速度模长影响动力学。若要研究强侧风，
还需要单独辨识并加入侧向气动力系数。

## 7. NPZ 日志

`main_sim.py` 保存的主要字段包括：

| 字段 | 含义 |
|---|---|
| `time` | 时间 |
| `position`, `velocity`, `acceleration` | NED 实际平动状态 |
| `position_ref`, `velocity_ref`, `acceleration_ref` | 参考轨迹及其导数 |
| `psi_ref` | 参考偏航角 |
| `euler_deg`, `euler_command_deg` | 实际和指令 ZXY 欧拉角 |
| `angular_velocity`, `angular_velocity_ref` | 实际和参考机体角速度 |
| `angular_acceleration` | 未滤波机体角加速度 |
| `angular_acceleration_lpf` | 角 INDI 使用的滤波角加速度 |
| `angular_acceleration_command` | 指令角加速度 |
| `rotor_speed` | 两个电机的实际转速 |
| `flap_deg` | 两个舵面的实际偏角 |
| `wind_velocity` | 世界 NED 风速，单位 m/s |
| `body_airspeed` | 机体系空速，单位 m/s |
| `wind_external_force` | 风导致的世界系外力增量，单位 N |
| `wind_external_moment` | 风导致的机体系外力矩增量，单位 N·m |
| `coordinate_frame`, `euler_sequence` | 坐标系和欧拉角顺序元数据 |

每个轨迹应使用独立日志文件，避免默认 `flight_log.npz` 被下一次仿真覆盖。

## 8. 三维飞行动画

动画使用 PyVista/VTK 的 GPU 渲染管线。STL 网格只在场景创建时上传一次，
播放时仅更新飞行器 Actor 的 4×4 位姿矩阵，不会逐帧重建三角面。
如果日志包含 `wind_velocity`，动画默认还会从无人机中心绘制橙色风矢量箭头：
箭头方向表示世界系风向，箭头长度表示风速大小；风速为零时箭头自动隐藏。

在生成对应日志后，可按预设轨迹直接播放：

```bash
python animate_flight.py --trajectory lemniscate
# 0.5 倍速，即慢两倍
python animate_flight.py --trajectory lemniscate --speed 0.5
python animate_flight.py --trajectory knife-edge-transition --speed 0.5
python animate_flight.py --trajectory circle-coordinated --speed 0.5
python animate_flight.py --trajectory circular-knife-edge --speed 0.5
python animate_flight.py --trajectory transition-to-forward --speed 0.5
python animate_flight.py --trajectory transition-to-hover --speed 0.5
python animate_flight.py --trajectory differential-turn --speed 0.5
```

也可以指定任意日志：

```bash
python animate_flight.py validation_logs/lemniscate.npz
python animate_flight.py validation_logs/circle-coordinated.npz \
  --stride 10 --speed 1.5
```

可以调整风箭头长度比例，或关闭风箭头：

```bash
# 每 1 m/s 风速显示为 0.8 m 长的箭头
python animate_flight.py validation_logs/lemniscate-windowed-wind.npz \
  --wind-scale 0.8

# 保留其他动画内容，但隐藏风箭头
python animate_flight.py validation_logs/lemniscate-windowed-wind.npz \
  --no-wind-arrow
```

`--wind-scale` 的单位为 `m/(m/s)`，默认值为 `0.5`。旧日志没有
`wind_velocity` 字段时仍可正常播放，只是不显示风箭头。

默认 STL 旋转中心是模型包围盒中心。如果已知飞行器重心在 STL 坐标中的位置，
可通过 `--model-origin` 指定；也可以调整模型尺寸或临时显示机体轴：

```bash
python animate_flight.py validation_logs/lemniscate.npz \
  --model vis_vtol.stl --model-scale 1.0 \
  --model-origin 0.0 0.0 0.27

python animate_flight.py validation_logs/lemniscate.npz \
  --show-body-axes --axis-length 1.2
```

保存 GIF 或 MP4：

```bash
python animate_flight.py validation_logs/lemniscate.npz \
  --save trajectory_animations/lemniscate.gif --no-show

python animate_flight.py validation_logs/lemniscate.npz \
  --save trajectory_animations/lemniscate.mp4 --no-show
```

批量导出七种论文轨迹动画：

```bash
python animate_flight.py --all-trajectories \
  --output-dir trajectory_animations --format gif --no-show
```

MP4 编码器由 `imageio-ffmpeg` 随环境安装。GIF 需要逐帧颜色量化，批量导出时
通常 MP4 更快、文件也更小。完整参数可通过 `python animate_flight.py --help` 查看。

## 9. 生成论文风格结果图

以下程序默认读取 `validation_logs/` 中对应名称的日志，并把图片保存到各自目录。
这些默认日志名称用于保留无风论文基线；如果有风仿真使用了新的日志文件名，
需要在绘图命令中显式传入该有风日志。

例如，用指定时间风扰日志生成 Lemniscate 原论文版式的跟踪结果图：

```bash
python results/Lemniscate/plot_lemniscate_results.py \
  validation_logs/lemniscate-windowed-wind.npz \
  --output results/Lemniscate/lemniscate_windowed_wind_results.png
```

另外提供独立风扰诊断图，不改变 Fig. 6～Fig. 11 的论文子图布局：

```bash
python results/WindDisturbance/plot_wind_disturbance_results.py \
  validation_logs/lemniscate-windowed-wind.npz
```

该图显示世界 NED 风速、风致世界系外力、风致机体系力矩和位置跟踪误差，
并用浅橙色背景标出检测到的非零风扰时间区间。可使用 `--start`、`--end`
限制绘图窗口，详细说明见 `results/WindDisturbance/README.md`。

```bash
# Fig. 6：Lemniscate
python results/Lemniscate/plot_lemniscate_results.py

# Fig. 7：Knife-edge transitioning
python results/KnifeEdgeTrasition/plot_knife_edge_transition_results.py

# Fig. 8：协调/刀锋圆周的位置、误差、速度、加速度和姿态
python results/CircleFlight/plot_circle_flight_results.py

# Fig. 9：协调/刀锋圆周的电机、舵面和三轴角加速度
python results/CircleFlight/plot_circle_flight_actuators.py

# Fig. 10：悬停与协调前飞之间的双向圆周过渡
python results/CircleTransition/plot_circle_transition_results.py

# Fig. 11：差分推力掉头
python results/DifferentialThrustTurn/plot_differential_thrust_turn_results.py
```

结果脚本默认只保存图片；附加 `--show` 可同时打开窗口，使用 `--output` 可指定 PNG 或 PDF。例如：

```bash
python results/Lemniscate/plot_lemniscate_results.py \
  --lap 6 --output results/Lemniscate/lemniscate_lap6.pdf --show
```

每个结果目录中的 `README.md` 记录了对应子图、时间窗口和专用参数。

## 10. 参数与复现范围

`config.py` 中部分参数来自论文，部分参数根据论文图表、飞行器尺寸和执行器信息估算。论文没有公开所有台架数据、惯量、几何臂长及原始轨迹解析式，因此本项目不是作者飞行代码和参数的逐字复制。

其中：

- Lemniscate 根据论文给出的速度、圈时和圈数实现；
- knife-edge transitioning 椭圆依据论文 Sec. VI-C、Fig. 7 和给出的载荷信息重建；
- differential-thrust turn 根据 7 m/s、约 0.5 s 朝向反转和约 650 deg/s 最大角速度重建；
- 当前仿真使用 500 Hz 更新率，并保留论文的 15 Hz 二阶 Butterworth 主低通和 1 Hz 二阶 Butterworth 舵面高通；论文实机为 2000 Hz。此外，仿真没有模拟具体传感器噪声、异步电机编码器采样和固件浮点量化。

因此，结果适合检查论文控制逻辑、坐标变换、执行器作用和轨迹跟踪趋势，不应将数值结果视为论文实验数据的完全复现。

更详细的公式核对和问题解释见 `study.md`，代码修改与验证记录见 `log.md`。
