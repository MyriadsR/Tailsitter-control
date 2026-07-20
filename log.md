# 论文对照审查与修复日志

日期：2026-07-15

对照论文：*Global Incremental Flight Control for Agile Maneuvering of a Tailsitter Flying Wing*，Ezra Tal 与 Sertac Karaman。

## 本次工作范围

本次修改处理了首次代码审查中确认的三项主要问题：

1. 倒飞或翻面时，平坦性变换选择的 `k*pi` 欧拉角分支在四元数转回主值欧拉角时丢失。
2. 线加速度 INDI 中，去除瞬态舵面力后的加速度与仍包含瞬态舵面力的 `f_lpf` 不一致。
3. 差分推力超出可实现范围后，单电机负推力截断会改变总推力，后续力矩计算仍使用截断前的差分推力。

本次没有改变已经核对正确的 NED 坐标定义、四元数方向、ZXY 旋转顺序、零升力坐标系转换符号以及论文式 (17)、(21)–(24)、(35) 的主体推导。

## 修复 1：保留平坦性变换的全局欧拉角分支

涉及文件：

- `controller/att_ctrl.py`
- `main_sim.py`

原实现首先使用论文式 (17) 的 `k*pi` 逻辑选择连续姿态，然后只返回四元数。为了计算 jerk 和偏航率对应的 `Omega_ref`，`main_sim.py` 又将四元数转换为 ZXY 主值欧拉角。

四元数无法保留使用的是哪一组等价欧拉角。倒飞时，原始 `phi + pi` 分支会被折叠到 $-\pi/2, \pi/2$，同时等价欧拉角中的偏航和俯仰也会变化。原代码丢弃重新提取的偏航角，却继续使用原始 `psi_ref`，因此 `Omega_ref` 与实际的 `q_ref` 不一致。

修改内容：

- 为 `FlatnessAttitudeController.compute_desired_attitude_and_thrust()` 增加可选参数 `return_angles`。
- 默认调用仍返回 `(q_c, T_c)`，保持原有接口兼容。
- `return_angles=True` 时额外返回平坦性变换直接选取的 `phi` 和 `theta_bar`。
- jerk 前馈直接使用上述原始角度，不再执行 `q_ref -> Euler` 的有损往返转换。
- 日志绘图仍使用四元数转欧拉角，但对 `arcsin` 输入增加了 $-1, 1$ 数值保护。

验证结果：

- 使用倒飞当前姿态生成 300 组随机局部参考轨迹。
- 修改前，解析角速度相对姿态有限差分角速度的误差中位数约为 `1.624 rad/s`。
- 修改后，误差中位数为 `1.26e-10 rad/s`，RMS 为 `9.06e-9 rad/s`，最大值为 `1.27e-7 rad/s`。

## 修复 2：统一瞬态舵面补偿与 INDI 力模型

涉及文件：

- `main_sim.py`

论文式 (43) 从低通加速度中去除带通舵面力，以减弱舵面非最小相位加速度响应。论文随后明确要求式 (46) 中的 `f_lpf` 使用“不含瞬态分量的舵面偏转”。

原实现从 `a_lpf` 中减去了 `f_delta_hpf / m`，但 `f_alpha_lpf` 是完整真实合力的低通结果，仍包含同一个舵面瞬态分量。这会使式 (46) 额外出现约一个 `f_delta_hpf` 项。同时，姿态/推力平坦性变换使用的是原始舵面偏角之和，而不是论文要求的慢变分量。

修改内容：

- 增加 `omega_rotors_lpf`、`delta_flaps_lpf` 和 `delta_flaps_slow` 执行器滤波状态。
- 使用 `delta_flaps_lpf - delta_flaps_slow` 构造带通舵面瞬态分量。
- 使用滤波后的电机转速和慢变舵面偏转重新计算线加速度 INDI 的 `f_alpha_lpf`。
- 使用滤波后的电机转速和普通低通舵面偏转重新计算角加速度 INDI 的 `m_lpf`。
- 平坦性姿态、推力和角速度变换统一使用 `delta_flaps_slow` 的舵面偏角之和。
- 不再直接低通仿真环境给出的真实合力和真实力矩作为控制器模型输出。

验证结果：

- 对任意测试舵面低通量进行 `slow + hpf` 分解后，完整舵面力与“慢变模型力 + 瞬态舵面力”的重构残差为 `8.88e-16 N`。
- 说明式 (43) 与式 (46) 现在使用的是同一套舵面分解。

## 修复 3：使饱和后的控制分配保持自洽

涉及文件：

- `controller/actuator_ctrl.py`

原实现在计算 `T1=(T+DeltaT)/2`、`T2=(T-DeltaT)/2` 后，将负值单独截为零。这种处理会改变总推力和实际差分推力，但反扭矩和舵面力矩剥离仍使用截断前的 `DeltaT`。

修改内容：

- 首先保证总推力命令非负。
- 将差分推力约束为 `-T_c <= DeltaT <= T_c`，从而在保持 `T1+T2=T_c` 的同时保证两个电机推力均非负。
- 根据实际的 `T1-T2` 重新计算螺旋桨反扭矩和后续舵面目标力矩。
- 将原来的“伪逆后逐元素截断”改为二维有界最小二乘。在舵面达到正负 45 度边界时，重新优化另一个舵面，使滚转和俯仰力矩残差最小。

验证结果：

- 在 `T_c=5 N`、偏航力矩需求远超可实现范围的测试中，修改前实际总推力约为 `9.2 N`；修改后为 `4.999999999999999 N`。
- 两个舵面均保持在限制内。
- 1000 组随机二维有界最小二乘测试中，与 SciPy `lsq_linear` 的最大目标函数差为 `7.54e-16`，无边界越界。
- 饱和时不可实现的偏航力矩仍会留下残差，这是物理约束的正确表现，而不会再通过错误的总推力改变线加速度。

## 整体回归结果

- `python -m compileall` 通过。
- 使用无交互绘图后端运行完整 `main_sim.py` 成功。
- 修改前位置跟踪误差：RMS 约 `0.030 m`，最大约 `0.099 m`。
- 修改后位置跟踪误差：RMS `0.028 m`，最大 `0.088 m`。
- 1000 组随机 ZXY 姿态中，四元数旋转矩阵与直接执行 `Rz(psi) Rx(phi) Ry(theta)` 的最大差约为 `1.03e-15`。
- 平坦性姿态/推力反解结果重新代入气动力模型后，1000 组随机测试的最大受力残差约为 `2.67e-14 N`。

## 尚未解决的其他问题

### 参数与论文平台不一致

- `config.py` 中 `alpha_T=0 deg`，论文实验平台为 `-5 deg`。
- `c_mu_T=0`，论文最终实验估计为 `-0.025 m`。
- `alpha_0`、推力系数、反扭矩系数、惯量和部分力臂仍是估计值。

## 2026-07-16：增加论文 Bernoulli 双纽线测试轨迹

涉及文件：

- `trajectory/test.py`
- `trajectory/__init__.py`
- `main_sim.py`
- `README.md`

修改内容：

- 新增标准 Bernoulli 双纽线 `LemniscateTrajectory`，默认采用论文测试中的
  `6 m/s`、每圈 `7 s`、连续 `8` 圈和正 Y 极值起点。
- 根据一圈 `42 m` 的目标路程自动确定曲线尺度；得到尺度参数约
  `8.00898 m`。
- 使用数值弧长查找表反解曲线参数，保证沿曲线恒速，而不是曲线参数恒速。
- 解析计算速度、加速度与 jerk；取 `psi=atan2(v_y,v_x)`，使偏航中间系
  Y 轴垂直速度，并解析计算偏航角速度。
- 为 `main_sim.py` 增加 `--trajectory lemniscate` 命令行入口。双纽线测试
  使用轨迹在 `t=0` 的位置、速度、平坦性姿态、机体角速度和推力初始化，
  避免从地面静止状态不连续跳变到 6 m/s。
- 保留原 `TrajectoryGenerator` 以及默认分段轨迹行为。

验证结果：

- 一圈位置闭合误差约 `1.83e-15 m`。
- 全轨迹解析速度模长为 `6.000000000000 m/s`。
- 最大水平坐标加速度约为 `13.485 m/s^2`；与重力合成后的最大比力约为
  `1.70 g`，与论文 Table 3 的 Lemniscate 数据一致。
- 位置、速度和加速度的中心有限差分分别与解析速度、加速度和 jerk 一致，
  最大测试误差分别约为 `1.35e-4 m/s`、`2.48e-4 m/s^2` 和
  `4.96e-4 m/s^3`（有限差分步长 `1e-4 s`，同时受弧长表插值误差影响）。
- 8 圈、56 秒闭环仿真无发散，位置误差 RMS 为 `0.384 m`，最大为
  `0.566 m`；第二圈至第八圈每圈 RMS 约 `0.392 m`。
- 当前误差大于论文报告的 `0.17 m` RMS。该差异不能仅归因于轨迹生成，
  当前项目的机体参数为估计值、控制频率为 100 Hz，且多项平台参数与论文
  实机不一致，仍需辨识参数及重新整定控制增益。

## 2026-07-16：增加 Sec. VI-C～VI-F 试验轨迹及批量验证

涉及文件：

- `trajectory/test.py`
- `trajectory/__init__.py`
- `main_sim.py`
- `validate_paper_trajectories.py`
- `README.md`

新增轨迹：

1. `KnifeEdgeTransitionTrajectory`
   - 论文明确参数：`6 m/s`、`6.25 s/圈`、8 圈、顺时针、顶部直线
     knife edge、底部直线 coordinated、每个转弯 yaw 旋转 `pi/2`。
   - 根据 Fig. 7 和约 `1.6g` 载荷取转弯半径 `3 m`；由 `37.5 m`
     总周长反算每条直线长约 `9.325 m`。
   - 使用平滑曲率进入/退出代替有曲率阶跃的理想体育场曲线，使加速度和
     jerk 有限。底部 coordinated 姿态每圈在正飞/倒飞之间交替。
2. `CircularTrajectory`
   - 半径 `3.5 m`、参考速度 `8.1 m/s`。
   - 同时实现 coordinated 和 wingtip 沿切线的 knife-edge yaw 约束。
   - 最大参考比力为 `2.157g`，与论文约 `2.1g` 一致。
3. `CircularTransitionTrajectory`
   - 在 `3.5 m` 圆上用恒定切向加速度 `2.7 m/s^2` 运行 `3 s`。
   - 末速 `8.1 m/s`、路程 `12.15 m`，与论文给出的约 `8 m/s` 和
     `12 m` 一致。
   - 分别实现静止悬停到 coordinated flight，以及反向的减速过程。
   - 为忠实保留“恒定切向加速度”，轨迹在 transition 起止处存在论文所说
     可实际处理的轻微加速度不连续。
4. `DifferentialThrustTurnTrajectory`
   - 论文未给出完整时序函数，因此属于约束重建而非原始轨迹复刻。
   - 以七次 smoothstep 在 `1.6 s` 内将同一直线上的速度从 `+7 m/s`
     反向到 `-7 m/s`，峰值参考比力约 `2.19g`。
   - 同时以五次 smoothstep 在 `0.52 s` 内将 yaw 平滑旋转 `pi`，峰值
     yaw rate 约 `649 deg/s`，匹配论文给出的 `650 deg/s`。

接口与验证工具：

- `main_sim.py --trajectory` 增加六个对应选项，并根据轨迹性质选择地面、
  静止悬停或参考定常飞行初始状态。
- 日志增加实际/参考速度、参考加速度和 yaw 参考。
- 新增 `--log` 与 `--no-show`，便于批量无交互测试。
- `validate_paper_trajectories.py` 一次运行全部六种情况，将完整时序写入
  `validation_logs/`，将汇总写入 `trajectory_validation.json`，并生成
  六种参考/实际水平轨迹的 `trajectory_validation.png` 对比图。

轨迹数值检查：

- 椭圆跑道一圈的位置和速度闭合误差均为数值零，恒速模长为 `6 m/s`。
- 圆轨迹、两个 transition 和 differential turn 的位置有限差分速度、
  速度有限差分加速度、加速度有限差分 jerk 最大误差均低于
  `4e-8`（测试步长 `1e-5 s`，避开论文定义的加速度阶跃端点）。
- 椭圆跑道解析加速度/jerk 的对应有限差分误差分别约 `7.2e-8` 和
  `6.5e-6`；位置插值导致的速度有限差分误差约 `8.7e-4 m/s`。
- 椭圆顶部直线满足 `i_y^psi` 与速度平行/反平行的 knife-edge 约束，
  底部满足二者垂直的 coordinated 约束。

闭环验证结果（当前项目参数与 100 Hz 控制频率）：

| 轨迹 | 当前 RMS / 最大误差 (m) | 论文 RMS / 最大误差 (m) |
|---|---:|---:|
| Knife-edge transitioning | `0.900 / 1.714` | `0.20 / 0.48` |
| Circle coordinated | `0.404 / 0.459` | `0.15 / 0.18` |
| Circle knife edge | `2.247 / 3.164` | `0.15 / 0.17` |
| Transition to forward | `0.221 / 0.346` | `0.10 / 0.15` |
| Transition to hover | `0.259 / 0.380` | `0.15 / 0.24` |
| Differential thrust turn | `1.399 / 2.555` | `0.63 / 0.96` |

所有仿真均保持有限且运行至结束，但不能复现论文精度。主要证据与限制：

- Knife-edge transitioning 和 differential turn 均达到当前 `45 deg` 舵面
  限制，最大差分推力分别约 `6.17 N` 和 `6.49 N`；后者与论文报告的
  `6.1 N` 同量级，说明急转确实进入执行器极限区。
- Knife-edge circle 没有舵面饱和且 yaw RMS 误差约 `0.96 deg`，但实际
  速度最高达到 `10.68 m/s`，位置 RMS 达 `2.247 m`。这更符合线加速度
  回路参数/增益不匹配，而不是 yaw 轨迹或执行器饱和。
- 当前 `config.py` 使用 `alpha_T=0`、`c_mu_T=0`，论文分别为约
  `-5 deg`、`-0.025 m`。论文 knife-edge circle 需要约 `20 deg` 的
  稳态舵偏，而当前简化仿真只产生约 `2.17 deg`，两套平台的配平状态并不
  等价。
- 当前控制更新频率为 `100 Hz`，论文实机为 `2000 Hz`；滤波器结构、
  增益、惯量、电机与舵机动态也未使用论文实机辨识值。

因此，本次结果验证了轨迹可执行、平坦变换和控制链路在所有姿态模式下不会
直接数值失效；它同时否定了“当前估算参数已经足以复现论文跟踪精度”这一更强
结论。下一步若要做精度对标，应先恢复论文平台参数、舵面范围和控制频率，再
分别整定线加速度和姿态回路。

## 2026-07-17：将六种论文验证轨迹接入三维动画程序

涉及文件：

- `animate_flight.py`
- `README.md`

修改内容：

- 增加 `--trajectory` 轨迹预设，可直接选择：
  - `knife-edge-transition`
  - `circle-coordinated`
  - `circle-knife-edge`
  - `transition-to-forward`
  - `transition-to-hover`
  - `differential-turn`
- 每个预设绑定 `validation_logs/` 下对应 NPZ，并设置适合轨迹时长的默认
  抽帧间隔、播放倍率和动画标题。
- 增加 `--all-trajectories`，分别导出全部六种轨迹；支持 `--output-dir`、
  `--format gif|mp4` 和 `--dpi`。
- 保留原来的位置日志参数，因此任意 `main_sim.py` 日志和 Lemniscate 日志
  仍可直接使用。
- 抽取统一的 `animate_log()` 和 `save_animation()`，保存 MP4 时会先检查
  FFmpeg 是否可用，并在缺失时给出改用 GIF 的明确提示。

验证结果：

- 使用 `vtol_sim` Python 3.12.12 环境通过语法和命令行帮助检查。
- 使用批量入口成功生成六个 GIF，分辨率均为 `990 x 810`：
  - `trajectory_animations/knife-edge-transition.gif`：约 `1.7 MB`
  - `trajectory_animations/circle-coordinated.gif`：约 `808 KB`
  - `trajectory_animations/circle-knife-edge.gif`：约 `857 KB`
  - `trajectory_animations/transition-to-forward.gif`：约 `432 KB`
  - `trajectory_animations/transition-to-hover.gif`：约 `437 KB`
  - `trajectory_animations/differential-turn.gif`：约 `641 KB`
- 人工检查 knife-edge transitioning 动画首帧，参考轨迹、实际轨迹、时间、
  ZXY 欧拉角和红/绿/蓝机体轴均正常显示。

## 2026-07-17：纠正 VI-C 椭圆与 VI-D 标准圆的 knife-edge 命名映射

问题：

- 原 `circle-knife-edge` 命令绑定到论文 Sec. VI-D 的半径 `3.5 m` 定常圆。
- 用户所指的是 Sec. VI-C 的 oval knife-edge transitioning：顶部直线
  `b_y` 沿速度且 `b_x` 近似竖直向上，底部直线切换到 coordinated。
- 两项试验在论文中都存在，但原名称没有体现章节差异，导致动画看起来像
  “用标准圆错误替代了 VI-C 椭圆”。

论文重新核对：

- Sec. VI-C 明确写明：oval、`6 m/s`、`6.25 s/圈`、顺时针、顶部直线
  knife edge、底部直线 coordinated、每个弯道 yaw 改变 `pi/2`，并且
  coordinated 段每圈交替正飞/倒飞。
- Sec. VI-D 另行明确给出半径 `3.5 m`、参考速度 `8.1 m/s` 的标准圆，
  同时试验 coordinated 和 steady knife edge。这说明标准圆数学本身没有
  错，但不能用来表示 Sec. VI-C。

修改内容：

- `main_sim.py --trajectory circle-knife-edge` 现在指向 Sec. VI-C 的
  `KnifeEdgeTransitionTrajectory`，作为旧名称兼容入口。
- 推荐的 Sec. VI-C 名称仍为 `knife-edge-transition`。
- Sec. VI-D 定常 knife-edge 圆改用无歧义名称 `circular-knife-edge`。
- `validate_paper_trajectories.py`、动画预设、JSON 汇总和对比图统一使用
  `circular-knife-edge` 表示 Sec. VI-D。
- `animate_flight.py --trajectory circle-knife-edge` 现在读取 VI-C 椭圆日志，
  标题明确显示 `Oval Knife-Edge Transitioning Flight (Sec. VI-C)`。
- 批量动画仍输出六个不重复的论文工况；兼容别名不额外重复生成。

验证结果：

- 修正后的 `circle-knife-edge` 参考轨迹水平跨度为
  `X=5.623 m, Y=15.543 m`，是与 Fig. 7 相符的椭圆跑道而非标准圆。
- 初始 knife-edge 直线上：
  - `b_x dot (-i_z) = 0.99697`，即机体 X 轴近似竖直向上；
  - `b_y dot v_hat = 1.00000`，即翼尖 Y 轴指向速度方向。
- 8 圈各顶部直线中，实际 `b_x dot (-i_z)` 为 `0.973～0.997`，
  `abs(b_y dot v_hat)` 为 `0.954～1.000`。
- 已重新生成 `trajectory_animations/circle-knife-edge.gif` 为 VI-C 椭圆；
  原 VI-D 标准圆另存为 `trajectory_animations/circular-knife-edge.gif`。

## 2026-07-17：将 Lemniscate 加入统一三维动画预设

涉及文件：

- `animate_flight.py`
- `README.md`

修改内容：

- 新增 `--trajectory lemniscate`，读取
  `validation_logs/lemniscate.npz`。
- 动画标题明确标记为 `Lemniscate of Bernoulli (Sec. VI-B)`。
- 默认使用 `stride=20`、`speed=8`，避免 56 秒、8 圈日志产生过多帧。
- `--all-trajectories` 现在批量生成 Lemniscate 加 Sec. VI-C～VI-F 的
  其余六种轨迹，共七个不重复工况。

验证结果：

- 重新运行 8 圈 Lemniscate 仿真，位置误差 RMS 为 `0.384 m`，最大为
  `0.566 m`，日志保存为 `validation_logs/lemniscate.npz`。
- 成功生成 `trajectory_animations/lemniscate.gif`，分辨率为
  `990 x 810`，大小约 `2.0 MB`。
- 检查动画首帧，Bernoulli 双纽线参考、实际位置、ZXY 姿态文本和
  红/绿/蓝机体轴均正常显示。

## 2026-07-17：新增论文 Fig. 6 风格的 Lemniscate 五联结果图

涉及文件：

- `main_sim.py`
- `results/Lemniscate/plot_lemniscate_results.py`
- `results/Lemniscate/README.md`
- `results/Lemniscate/lemniscate_trajectory_results.png`

修改内容：

- 仿真 NPZ 日志新增世界 NED 系实际加速度 `acceleration`，避免通过速度
  数值微分构造论文 d) Acceleration 曲线。
- 日志新增 ZXY 欧拉角顺序的指令姿态 `euler_command_deg`，可在论文
  e) Attitude 子图中直接比较姿态指令和实际响应。
- 新增五联绘图程序，依次绘制全部 8 圈三维位置、单圈三轴位置误差、
  速度模长、比力模长 `||a-g i_z||` 以及 ZXY 姿态。
- 时域图默认选取最后一个完整单圈，可通过 `--lap` 指定其他圈；输入日志、
  单圈时间、输出文件、DPI 和是否显示窗口均可从命令行配置。
- 保留旧日志兼容：缺少实际加速度时用速度数值微分估算，缺少指令姿态时
  仅绘制实际姿态，并明确发出提示。

验证结果：

- `main_sim.py` 和绘图程序均通过 Python 语法检查。
- 重新完成论文默认的 `6 m/s`、`7 s/圈`、8 圈 Lemniscate 仿真，日志中
  新字段均为 `(5601, 3)` 且全部为有限值。
- 位置跟踪误差 RMS 为 `0.384 m`，最大为 `0.566 m`。
- 成功生成 `results/Lemniscate/lemniscate_trajectory_results.png`，人工检查
  五个子图、单位、图例及第 8 圈姿态曲线均正常。

后续三维视图修正：

- 删除原先交换 X/Y 并压缩 Z 轴的绘法，避免透视投影掩盖双纽线形状。
- 与 `animate_flight.py` 保持一致：采用 North X、East Y、Altitude
  `-Z_NED` 显示坐标，视角为 `elev=25 deg, azim=-55 deg`，三个坐标轴
  使用相同尺度和相同显示范围。
- 增大顶部三维子图所在行并增加子图纵向间距，避免三维坐标标签与
  b)/c) 子图标题重叠；重新生成并检查结果图，双纽线两个叶瓣清晰可见。

## 2026-07-17：新增论文 Fig. 7 风格的 Knife-Edge Transitioning 结果图

涉及文件：

- `results/KnifeEdgeTrasition/plot_knife_edge_transition_results.py`
- `results/KnifeEdgeTrasition/README.md`
- `results/KnifeEdgeTrasition/knife_edge_transition_results.png`
- `validation_logs/knife-edge-transition.npz`

论文核对：

- Fig. 7a 是全部 8 圈的水平面俯视轨迹，横轴为世界 `y`，纵轴为世界
  `x`，并非三维轨迹图。
- Fig. 7b、7c 展示连续两圈（`12.5 s`）的位置误差和 ZXY 姿态。
- 论文的时域窗口从 knife-edge 转入倒飞 coordinated flight 的转弯起点
  开始。本项目轨迹的对应时刻为 `7.804 s`，由单圈时间、速度、转弯半径
  和直线长度自动计算，默认窗口为 `[7.804, 20.304] s`。

实现内容：

- 新增 Fig. 7 三联图程序，顶部居中绘制八圈参考/实际位置，底部并列绘制
  两圈三轴位置误差和六条指令/实际姿态曲线。
- 姿态日志按 `[psi, phi, theta]` 的 ZXY 顺序从度转换成弧度，先连续化
  再消除指令/实际之间仅由欧拉角主值产生的 `2*pi` 整周差。
- 支持自定义输入日志、输出文件、窗口起点、窗口圈数、单圈时间、速度、
  转弯半径和 DPI，并保留旧日志缺失指令姿态时的明确提示。
- 目录名 `KnifeEdgeTrasition` 按用户指定拼写保留；轨迹命令和日志仍使用
  正确名称 `knife-edge-transition`。

验证结果：

- 使用当前 `main_sim.py` 重新运行 `6 m/s`、`6.25 s/圈`、8 圈仿真，日志
  已包含 `euler_command_deg`。
- 绘图程序通过 Python 语法和无窗口运行检查，成功生成 PNG；人工检查
  轨迹方向、两圈时间范围、图例、单位和姿态连续性均正常。
- 当前仿真的全程位置误差为 RMS `0.900 m`、最大 `1.714 m`；默认两圈
  窗口为 RMS `0.919 m`、最大 `1.714 m`。这些值明显大于论文实验报告的
  RMS `0.20 m`、最大 `0.48 m`，结果图忠实显示现有控制器表现，没有为
  贴合论文而缩放数据。

## 2026-07-17：新增论文 Fig. 8 风格的 Circle Flight 十联结果图

涉及文件：

- `results/CircleFlight/plot_circle_flight_results.py`
- `results/CircleFlight/README.md`
- `results/CircleFlight/circle_flight_results.png`
- `validation_logs/circle-coordinated.npz`
- `validation_logs/circular-knife-edge.npz`

论文核对：

- Fig. 8 左列 a)、c)、e)、g)、i) 为 coordinated flight，右列 b)、d)、
  f)、h)、j) 为 steady knife-edge flight。
- 五行依次为三维位置、三轴位置误差、速度模长、比力模长
  `||a-g i_z||` 和 ZXY 指令/实际姿态。
- 轨迹半径为 `3.5 m`，参考速度为 `8.1 m/s`，对应单圈时间
  `2.71496 s`；论文标题中的 `7.8 m/s` 是实验实际速度。
- 论文姿态图从 yaw `+pi` 开始并连续下降至 `-pi`。两种构型 yaw 定义
  相差 `pi/2`，因此绘图时 coordinated 窗口相对基础圈移动 `1/4` 圈，
  knife-edge 窗口从基础圈起点开始。

实现内容：

- 新增 5 行 × 2 列的 Fig. 8 绘图程序，两列采用相同的三维位置坐标范围，
  便于直接比较轨迹半径和高度偏差。
- 速度、比力、位置误差和姿态均选择一个完整圆周；默认自动选择日志中
  最后一个可以同时完成两种相位对齐的基础圈，本次为第 3 圈。
- 实际加速度直接读取仿真日志；旧日志缺少该字段时保留速度数值微分兼容。
  姿态按 ZXY 顺序连续化，并消除指令与实际之间的欧拉角整周差。
- 支持覆盖两个输入日志、圈数、半径、参考速度、单圈时间、DPI 和输出路径。

验证结果：

- 并行重新运行 `circle-coordinated` 和 `circular-knife-edge` 四圈仿真，
  两个日志均已包含 `acceleration` 和 `euler_command_deg`。
- 绘图程序通过 Python 语法、PNG 和 PDF 输出检查；人工检查 a)～j) 编号、
  坐标、图例、单位、单圈时间和 yaw 相位均正常。
- Coordinated flight：全程位置误差 RMS `0.404 m`、最大 `0.459 m`；
  选定单圈平均速度 `7.655 m/s`，平均比力 `20.254 m/s^2`。
- Knife-edge flight：全程位置误差 RMS `2.247 m`、最大 `3.164 m`；
  选定单圈平均速度 `10.191 m/s`，平均比力 `25.545 m/s^2`。
- 论文两种构型均报告约 `0.15 m` RMS、实际速度约 `7.8 m/s`。当前
  knife-edge 仿真出现轨迹外扩、加速和持续高度漂移，结果图忠实显示这一
  控制/建模问题，未通过缩放坐标或裁剪曲线掩盖。

后续三维高度显示修正：

- Fig. 8 的 a)/b) 三维位置子图将原始 NED `z` 取负，以正高度
  `Altitude -z_i` 显示。
- 共享 Z 轴范围同步在取负后的显示坐标中计算；NPZ 日志、位置误差和比力
  计算继续使用原始 NED 数据，没有改变控制或动力学逻辑。

## 2026-07-17：新增论文 Fig. 9 风格的 Circle Flight 执行器结果图

涉及文件：

- `main_sim.py`
- `results/CircleFlight/plot_circle_flight_actuators.py`
- `results/CircleFlight/README.md`
- `results/CircleFlight/circle_flight_actuators_and_angular_acceleration.png`
- `validation_logs/circle-coordinated.npz`
- `validation_logs/circular-knife-edge.npz`

论文核对：

- Fig. 9 延续 Fig. 8 的左右列定义：左列 coordinated flight，右列
  knife-edge flight。
- a)/b) 为两个电机转速，单位 `rad/s`；c)/d) 为两个舵面偏角，单位
  `rad`；e)～j) 依次为绕机体 `b_x、b_y、b_z` 的指令和实际角加速度，
  单位 `rad/s^2`。
- 论文为补偿舵机和电机响应时间，将指令角加速度曲线移动最多 `70 ms`，
  该处理仅用于图形可读性。

日志修改：

- 新增 `angular_acceleration`：动力学方程直接得到的未滤波机体系角加速度。
- 新增 `angular_acceleration_lpf`：控制器 INDI 反馈实际使用的低通角加速度。
- 新增 `angular_acceleration_command`：姿态 PD 输出的三轴角加速度指令
  `omega_dot_c`。
- 三个字段顺序均为 `[b_x, b_y, b_z]`，单位均为 `rad/s^2`。

绘图实现：

- 新增 5 行 × 2 列的 Fig. 9 绘图程序，并与 Fig. 8 使用相同的第 3 圈及
  coordinated `1/4` 圈相位偏移。
- 舵面日志由度转换成论文使用的弧度；实际角加速度优先使用
  `angular_acceleration_lpf`。
- 对每种构型、每个机体轴，在 `0～70 ms` 范围内通过归一化曲线误差估计
  非负响应时移，再将指令曲线在显示时间上向后移动；可用
  `--max-command-shift-ms 0` 完全关闭。

验证结果：

- 重新运行两种圆轨迹，新增三个角加速度字段形状均正确且全部为有限值。
- 绘图程序通过 Python 语法、PNG、PDF 和关闭指令时移的运行检查；人工
  检查 a)～j) 编号、单位、图例和单圈时间均正常。
- 自动估计的指令时移：coordinated `[30, 20, 20] ms`，knife-edge
  `[20, 30, 60] ms`，均未超过论文给出的 `70 ms`。
- 选定单圈的平均电机转速：coordinated `[614.37, 607.54] rad/s`，
  knife-edge `[847.56, 847.42] rad/s`；平均舵偏分别为
  `[-0.00573, -0.00214] rad` 和 `[-0.00431, -0.00324] rad`。
- 这些执行器工作点明显低于论文约 `1800～2000 rad/s` 和 knife-edge
  约 `20 deg` 舵偏，反映本项目气动/推进参数标定与论文实机并不一致。

## 2026-07-17：新增论文 Fig. 10 风格的 Circle Transition 结果图

涉及文件：

- `trajectory/test.py`
- `results/CircleTransition/plot_circle_transition_results.py`
- `results/CircleTransition/README.md`
- `results/CircleTransition/circle_transition_results.png`
- `validation_logs/transition-to-forward.npz`
- `validation_logs/transition-to-hover.npz`

论文核对：

- Fig. 10 左列 a)、c)、e)、g)、i) 为 static hover 到 coordinated flight，
  右列 b)、d)、f)、h)、j) 为相反方向过渡。
- 五行依次为三维位置、三轴位置误差、速度模长、比力模长
  `||a-g i_z||` 和 ZXY 指令/实际姿态。
- 圆半径为 `3.5 m`，匀加/减速段持续 `3.0 s`，切向加速度为
  `2.7 m/s^2`，目标速度约 `8.1 m/s`；论文图还显示过渡前后的短稳态段，
  总横轴约 `3.8 s`。

轨迹修改：

- `CircularTransitionTrajectory` 新增 `lead_duration` 和
  `settle_duration`，默认均为 `0.4 s`，因此总时长由 `3.0 s` 调整为
  `3.8 s`，真正的加/减速持续时间保持 `3.0 s` 不变。
- 加速轨迹在前 `0.4 s` 保持静态悬停，过渡完成后沿圆以 `8.1 m/s`
  保持 `0.4 s`；减速轨迹先沿圆定速飞行 `0.4 s`，随后减速并在末端保持
  静态参考。
- 重新推导了前后稳态段的弧长，使位置和速度在过渡起点/终点连续；稳态段
  的切向加速度为零。

绘图实现：

- 新建 `results/CircleTransition`，使用 5 行 × 2 列复现 Fig. 10 布局。
- 两个三维位置子图使用相同坐标范围，并按用户约定将 NED `z` 取负显示为
  正高度 `Altitude -z_i`；误差和动力学计算继续使用原始 NED 坐标。
- 实际加速度直接读取日志，姿态按 ZXY 顺序转换成连续弧度，并消除指令与
  实际之间仅由欧拉角主值造成的整周差。

验证结果：

- 检查两种轨迹在 `t=0、0.4、3.4、3.8 s` 的位置、速度和加速度，参考
  速度平台分别为 `0 -> 8.1 m/s` 和 `8.1 -> 0 m/s`，总时长均为
  `3.8 s`。
- 并行重新运行两种过渡仿真，新日志包含实际加速度、指令姿态和角加速度
  扩展字段。
- 绘图程序通过 Python 语法、PNG 和 PDF 输出检查；人工检查 a)～j) 编号、
  正高度显示、单位、时间平台和姿态连续性均正常。
- Hover -> coordinated：位置误差 RMS `0.214 m`、最大 `0.346 m`，实际
  末速度 `8.279 m/s`，最大比力 `21.950 m/s^2`。
- Coordinated -> hover：位置误差 RMS `0.251 m`、最大 `0.409 m`，末端
  仍有 `0.525 m/s` 实际残余速度，最大比力 `21.180 m/s^2`。
- 论文相应位置误差约为 RMS/最大 `0.10/0.15 m` 和 `0.15/0.24 m`；当前
  仿真两种过渡的误差均更大，尤其减速末端仍需更长收敛时间。

## 2026-07-17：新增论文 Fig. 11 风格的 Differential Thrust Turn 结果图

涉及文件：

- `main_sim.py`
- `results/DifferentialThrustTurn/plot_differential_thrust_turn_results.py`
- `results/DifferentialThrustTurn/README.md`
- `results/DifferentialThrustTurn/differential_thrust_turn_results.png`
- `validation_logs/differential-turn.npz`

论文核对：

- Fig. 11a 为三维位置，b) 为实际 ZXY 姿态，c) 为机体系三轴角速度，
  d) 为两个电机转速，e) 为两个舵面偏角。
- 论文图中的角速度、电机转速和舵偏单位分别为 `rad/s、rad/s、rad`；
  舵面曲线需要将项目日志的度转换成弧度。
- 论文显示窗口约 `2.25 s`，而本项目参考轨迹为保证 `2.2g` 峰值比力和
  完整的进入/速度反向/退出段，总时长为 `3.3 s`。

日志修改：

- 新增 `angular_velocity`，记录机体系 `[Omega_x, Omega_y, Omega_z]`，
  单位 `rad/s`。
- 同时新增 `angular_velocity_ref`，记录平坦性前馈角速度；论文 Fig. 11c
  只显示实际角速度，因此默认绘图不使用该参考字段。

绘图实现：

- 新建 `results/DifferentialThrustTurn`，顶部三维位置跨两列，下面按论文
  版式排列姿态/角速度和电机/舵面，共五个子图。
- 三维位置将 NED `z` 取负显示为正高度 `Altitude -z_i`；姿态 yaw 采用
  连续化 ZXY 欧拉角，避免经过 `pi` 时发生绘图跳变。
- 默认绘制完整 `3.3 s`；支持 `--plot-duration 2.25` 生成与论文相同的
  横轴窗口，也支持自定义日志、输出路径和 DPI。

验证结果：

- 使用当前 `main_sim.py` 重新运行 differential-turn，新日志的角速度字段
  形状为 `(331, 3)` 且全部为有限值。
- 绘图程序通过 Python 语法、完整 PNG、PDF 和 `2.25 s` 裁剪窗口检查；
  人工检查五个子图、单位、图例、正高度与姿态连续性均正常。
- 完整过程位置误差 RMS `1.399 m`、最大 `2.555 m`，明显大于论文报告的
  RMS `0.63 m`、最大 `0.96 m`。
- 峰值角速度为 `9.662 rad/s (553.6 deg/s)`，低于论文约 `650 deg/s`；
  最大电机转速 `994.3 rad/s`，最大两电机转速差 `735.2 rad/s`，最大舵偏
  `0.785 rad`。
- 转弯瞬态中一个电机接近零转速且两个舵面达到约 `±45 deg` 限幅，实际
  高度损失约 `2.52 m`，这是当前最大位置误差超过论文的主要直接表现。
