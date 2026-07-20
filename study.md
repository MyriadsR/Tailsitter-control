# 1. 代码中关于式 (34) 的等价写法的解释

这里的代码逻辑是正确的。它与论文式 (34) 的外观不同，是因为代码用输运定理把式中的 $\dot R_i^\phi$ 消掉了；两者数学上完全等价。

论文式 (34) 可写为：

$$
\dot{\mathbf v}^{\phi} = \dot R_i^\phi\,\mathbf v^i + R_i^\phi\,\mathbf a^i
$$

其中：

$$
\mathbf v^\phi = R_i^\phi\,\mathbf v^i
$$

## 旋转矩阵导数与角速度的关系

定义 $\omega_{\phi/i}^{\phi}$ 为 $\phi$ 坐标系相对世界坐标系的角速度，并表达在 $\phi$ 系中。对于把世界系坐标转换到旋转坐标系的矩阵 $R_i^\phi$，有：

$$
\dot R_i^\phi = -\left[\omega_{\phi/i}^{\phi}\right]_\times R_i^\phi
$$

其中：

$$
\left[\omega\right]_\times \mathbf x = \omega \times \mathbf x
$$

代入式 (34) 得到：

$$
\begin{aligned}
\dot{\mathbf v}^{\phi}
&= -\left[\omega_{\phi/i}^{\phi}\right]_\times R_i^\phi\,\mathbf v^i + R_i^\phi\,\mathbf a^i \\
&= R_i^\phi\,\mathbf a^i - \omega_{\phi/i}^{\phi} \times \mathbf v^\phi
\end{aligned}
$$

这正是代码：

```python
v_phi_dot = (
    R_i_to_phi @ a_ref
    - np.cross(omega_phi_i_in_phi, v_phi)
)
```

因此减号正确。如果 $\omega_{\phi/i}^{\phi}$ 定义的是“世界系相对 $\phi$ 系”的角速度，才应该改为加号；但当前代码构造的是“$\phi$ 系相对世界系”的角速度，所以应当使用减号。

## omega_phi_i_in_phi 的构造也正确

ZXY 旋转的前两步是：

$$
R_\phi^i = R_z(\psi)R_x(\phi)
$$

因此 $\phi$ 中间坐标系的角速度由两部分组成：

1. 绕世界 $i_z$ 的偏航角速度；
2. 绕偏航后中间 $x$ 轴的 $\dot\phi$。

表达在 $\phi$ 系中：

$$
\omega_{\phi/i}^{\phi}
= R_i^\phi
\begin{bmatrix}
0 \\
0 \\
\dot\psi
\end{bmatrix}
+
\begin{bmatrix}
\dot\phi \\
0 \\
0
\end{bmatrix}
$$

对应代码：

```python
omega_phi_i_in_phi = (
    R_i_to_phi @ np.array([0, 0, psi_dot_ref])
    + np.array([phi_dot, 0, 0])
)
```

将其展开还可以得到：

$$
\omega_{\phi/i}^{\phi}
=
\begin{bmatrix}
\dot\phi \\
\dot\psi\sin\phi \\
\dot\psi\cos\phi
\end{bmatrix}
$$

这符合 ZXY 旋转关系。

## f_phi_dot 同理

因为：

$$
f^\phi = R_i^\phi f^i
$$

所以：

$$
\dot f^\phi = R_i^\phi\dot f^i - \omega_{\phi/i}^{\phi} \times f^\phi
$$

代码：

```python
f_phi_dot = (
    R_i_to_phi @ f_dot_i
    - np.cross(omega_phi_i_in_phi, f_phi)
)
```

同样正确。

论文假设未建模外力恒定，因此：

$$
f^i = m(a - g\,i_z) - f_{\rm ext}
$$

求导得到：

$$
\dot f^i = m j
$$

代码中的：

```python
f_dot_i = self.cfg.mass * j_ref
```

也符合该假设。

需要注意的是，`R_i_to_phi @ a_ref` 是物理加速度在 $\phi$ 系中的分量，而 $v_phi_dot$ 是速度坐标值在旋转 $\phi$ 系中的时间导数，两者并不相等，相差的正是：

$$
-\omega_{\phi/i}^{\phi} \times v^\phi
$$

论文式 (31)、(32) 计算 $(\tau_x, \tau_z)$ 时需要的是后者，因此当前代码不能简化成单独的 `R_i_to_phi @ a_ref`。


这段函数返回的是“机体系 (b) 相对于世界 NED 系 (i) 的 ZXY 欧拉角”，但欧拉角本身不能说是“表达在机体系下的三个分量”。

项目采用：

$$
R_b^i = R_z(\psi) R_x(\phi) R_y(\theta)
$$

其中：

- $\psi$：绕世界系 $i_z$ 旋转；
- $\phi$：绕经过偏航后的中间 $x$ 轴旋转；
- $\theta$：绕经过前两次旋转后的中间 $y$ 轴旋转。

所以图例写成 Roll/Pitch/Yaw 便于阅读，但它们不总是分别等于绕当前机体 $(b_x,b_y,b_z)$ 的旋转。

# 2. 为什么 $\phi$ 会主要由电机差速产生

论文式 (35) 在代码中是 controller/att_ctrl.py:320：

```python
omega_ref = (
    R_i_to_b @ [0, 0, psi_dot]
    + R_phi_to_b @ [phi_dot, 0, 0]
    + [0, theta_dot, 0]
)
```

忽略偏航变化时，可写出机体系角速度 $(p,q,r)$ 与欧拉角速率的关系为：

$$
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
=
\begin{bmatrix}
\cos\theta\,\dot\phi \\
\dot\theta \\
\sin\theta\,\dot\phi
\end{bmatrix}
$$

这里 $(p,q,r)$ 才是绕当前机体 $(b_x,b_y,b_z)$ 的角速度。

对于图中的尾座悬停状态，$\theta\approx85^\circ$，因此：

$$
p\approx\cos(85^\circ)\,\dot\phi\approx0.087\,\dot\phi,\qquad
r\approx\sin(85^\circ)\,\dot\phi\approx0.996\,\dot\phi
$$

也就是说，虽然欧拉角名称叫“Roll ($\phi$)”，但在尾座悬停附近，改变 $\phi$ 几乎完全需要绕机体 $b_z$ 旋转，而不是绕 $b_x$ 旋转。

论文中的执行器对应关系正是：

- 两个舵面反向偏转主要产生 $m_x$，即绕 $b_x$ 的力矩；
- 电机差分推力主要产生 $m_z$，即绕 $b_z$ 的力矩。

代码中也明确使用 $m_z$ 计算差分推力，见 `controller/actuator_ctrl.py:63`。

所以：

- 固定翼平飞、$\theta\approx0^\circ$ 时：

$$
p\approx\dot\phi
$$

此时 $\phi$ 才是通常意义上的机体滚转，主要由差动舵面产生。

- 尾座悬停、$\theta\approx90^\circ$ 时：

$$
r\approx\dot\phi
$$

此时改变同一个欧拉角 $\phi$ 主要依靠差分推力。

这不是坐标转换错误，而是 ZXY 欧拉角在大俯仰角下的正常轴映射，也是论文采用全局姿态控制后自然产生的执行器切换效果。

## 图中现象与此一致

在约 22–28 秒向世界 (+Y) 方向移动时：

- $\theta$ 一直约为 $85^\circ$；
- $\phi$ 先变正、再变负，对应加速和减速产生的侧向力；
- 两个电机出现很小的转速差，用于产生 $m_z$；
- 两个舵面基本重合，主要是同向偏转产生 $m_y$，与图中的俯仰振荡对应，并不是主要负责这次 $\phi$ 变化。

目前图中画的是两个電机的绝对转速，约为 530 rad/s，差速量很小，所以两条线几乎重合。为了清楚判断执行器作用，建议另外画：

```python
delta_T = cfg.c_T * (
    log_rotors[:, 0]**2 - log_rotors[:, 1]**2
)

delta_flap = log_flaps[:, 1] - log_flaps[:, 0]
collective_flap = log_flaps[:, 0] + log_flaps[:, 1]
```

其中：

- `delta_T`：差分推力，主要对应 $m_z$；
- `delta_flap`：差动舵面，主要对应 $m_x$；
- `collective_flap`：同向舵面，主要对应 $m_y$。

另外，`zxy_euler_from_quaternion()` 现在只用于绘图。它在 $\phi=\\pm90^\circ$ 附近存在欧拉角奇异性，可能发生角度跳变，但已经不再参与控制器的角速度前馈计算。

根本原因是：欧拉角变化率 $(\dot\psi,\dot\phi,\dot\theta)$ 和机体系角速度 $(p,q,r)$ 不是同一个量。尾座飞机从平飞转到接近 $90^\circ$ 俯仰后，定义 $\phi$ 的中间旋转轴会从机体 $b_x$ 方向逐渐转到机体 $b_z$ 方向，因此控制器自然从差动舵面平滑过渡到差分推力。

## 1. 三套轴需要区分

论文的 ZXY 姿态分解是：

$$
R_b^i = R_z(\psi) R_x(\phi) R_y(\theta)
$$

可以理解为依次建立三个坐标系：

世界系 $i$
  │ 绕 $i_z$ 旋转 $\psi$
  ▼
偏航中间系 $\psi$
  │ 绕 $x_\psi$ 旋转 $\phi$
  ▼
滚转中间系 $\phi$
  │ 绕 $y_\phi$ 旋转 $\theta$
  ▼
机体系 $b$

因此：

- $\psi$ 的旋转轴是世界系 $i_z$；
- $\phi$ 的旋转轴是中间系 $x_\psi$；
- $\theta$ 的旋转轴是中间系 $y_\phi$。

关键在于：$x_\psi$ 不一定等于当前机体 $b_x$。最后一次俯仰旋转 $R_y(\theta)$ 会改变 $x_\psi$ 相对于机体系的方向。

## 2. $\phi$ 的旋转轴在机体系中的表示

代码中：

$$
R_\phi^b =
\begin{bmatrix}
\cos\theta & 0 & -\sin\theta \\
0 & 1 & 0 \\
\sin\theta & 0 & \cos\theta
\end{bmatrix}
$$

因此中间系 $x_\psi(=x_\phi)$ 轴表达在机体系中为：

$$
x_\phi^b = R_\phi^b \begin{bmatrix}1 \\ 0 \\ 0\end{bmatrix} =
\begin{bmatrix}
\cos\theta \\
0 \\
\sin\theta
\end{bmatrix}
$$

也就是说，$\phi$ 的实际旋转轴是：

$$
x_\phi = \cos\theta\,b_x + \sin\theta\,b_z
$$

这条公式直接解释了执行器为什么会变化。

### 固定翼平飞

当：

$$
	\theta = 0^\circ
$$

有：

$$
x_\phi = b_x
$$

所以改变 $\phi$ 就是绕机头方向 $b_x$ 滚转：

$$
\dot\phi \longrightarrow p
$$

这种机体 $x$ 轴力矩主要由两个舵面反向偏转产生。

### 尾座悬停

当：

$$
	\theta = 90^\circ
$$

有：

$$
x_\phi = b_z
$$

所以改变同一个欧拉角 $\phi$，实际是在绕当前机体 $b_z$ 旋转：

$$
\dot\phi \longrightarrow r
$$

机体 $z$ 轴力矩主要由差分推力产生。

这并不是控制器错误地把滚转分配给了电机，而是“$\phi$ 的旋转轴”已经因为最终的 $90^\circ$ 俯仰旋转，从 $b_x$ 变成了 $b_z$。

## 3. 欧拉角速率到机体角速度的完整关系

根据代码和论文式 (35)：

$$
\Omega^b = R_i^b \begin{bmatrix}0 \\ 0 \\ \dot\psi\end{bmatrix}
        + R_\phi^b \begin{bmatrix}\dot\phi \\ 0 \\ 0\end{bmatrix}
        + \begin{bmatrix}0 \\ \dot\theta \\ 0\end{bmatrix}
$$

展开后可写为矩阵形式：

$$
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
=
\begin{bmatrix}
\cos\theta & 0 & -\sin\theta\cos\phi \\
0 & 1 & \sin\phi \\
\sin\theta & 0 & \cos\theta\cos\phi
\end{bmatrix}
\begin{bmatrix} \dot\phi \\ \dot\theta \\ \dot\psi \end{bmatrix}
$$

即分量形式为：

$$
p = \cos\theta\,\dot\phi - \sin\theta\cos\phi\,\dot\psi
$$

$$
q = \dot\theta + \sin\phi\,\dot\psi
$$

$$
r = \sin\theta\,\dot\phi + \cos\theta\cos\phi\,\dot\psi
$$

当图中 $\dot\psi=0$ 时，简化为：

$$
p = \cos\theta\,\dot\phi,\qquad q = \dot\theta,\qquad r = \sin\theta\,\dot\phi
$$

在 $\theta=85^\circ$ 附近：

$$
\cos85^\circ\approx0.087,\qquad \sin85^\circ\approx0.996
$$

所以：

$$
p\approx0.087\,\dot\phi,\qquad r\approx0.996\,\dot\phi
$$

改变 $\phi$ 所需的角速度约 99.6% 落在机体 $b_z$ 轴上，只有约 8.7% 落在 $b_x$ 轴上。

严格来说这两个百分数不能直接作为功率比例，但足以说明轴向关系。

## 4. 为什么差动舵面产生 $m_x$

两个舵面位于机体 $b_y$ 轴两侧，其主要气动力沿 $b_z$ 方向。

力矩來自：

$$
m = r \times f
$$

对于左右舵面：

$$
(\pm b_y) \times (\pm b_z) \sim b_x
$$

当两个舵面反向偏转时，它们产生的 $b_x$ 力矩同向叠加，因此主要得到 $m_x$。

论文式 (13) 给出：

$$
m_{\delta,x} = l_{\delta_y} \cos\alpha_0 \left(f_{\delta_2,z}-f_{\delta_1,z}\right)
$$

而 $m_{\delta,z}$ 前面乘有很小的 $\sin\alpha_0$，所以差动舵面的主要作用确实是机体 $b_x$ 滚转。

## 5. 为什么差分推力产生 $m_z$

两个电机沿机体 $b_y$ 方向分布，推力主要沿机体 $b_x$。因此：

$$
(\pm b_y) \times b_x \sim \mp b_z
$$

当两个电机推力不相等时，左右 $b_z$ 的力矩不能完全抵消，从而得到差分推力力矩：

$$
m_z \approx l_{T_y} (T_1 - T_2)
$$

因此控制分配器使用：

$$
m_z = m_c[2],\qquad \delta_T = m_z / \mathrm{denom\_T}
$$

这符合飞行器的物理结构。

由于零升力角、推力安装角和螺旋桨反扭矩的存在，差分推力也会产生少量 $m_x$，差动舵面也会产生少量 $m_z$。所以实际上是耦合控制，只是主导方向如上。

## 6. 控制器内部是怎样一步步产生这个现象的

控制链不是”看到 $\phi$ 就直接使用电机”，而是：

世界 +Y 方向位置/速度需求
        ↓
需要 +Y 方向加速度
        ↓
INDI 得到世界系期望力 $f_c^i$
        ↓
平坦性变换计算期望姿态 $q_c$
        ↓
$q_c$ 对应一个非零的 ZXY 欧拉角 $\phi$
        ↓
四元数姿态误差转换到机体系
        ↓
$\theta\approx90^\circ$ 时，姿态误差主要落在机体 $b_z$ 轴
        ↓
姿态控制器产生主要为 $z$ 分量的角加速度命令
        ↓
角加速度 INDI 产生 $m_z$
        ↓
控制分配器用差分推力实现 $m_z$

因此电机不是直接控制”欧拉角 Roll”，而是在控制机体系 $b_z$ 的力矩；只是这个 $b_z$ 旋转在当前大俯仰姿态下表现为 ZXY 欧拉角 $\phi$ 的变化。

## 7. 静态姿态误差也有同样的映射

这不仅是 $\dot\phi$ 前馈才有的现象。

假设当前姿态与期望姿态只相差一个很小的 $\delta\phi$。该姿态误差在机体系中的表达约为：

$$
\xi_e \approx \delta\phi
\begin{bmatrix}
\cos\theta \\
0 \\
\sin\theta
\end{bmatrix}
$$

当 $\theta\approx90^\circ$：

$$
\xi_e \approx \begin{bmatrix}0 \\ 0 \\ \delta\phi \end{bmatrix}
$$

所以四元数 PD 姿态反馈同样会产生主要沿 $b_z$ 的控制命令。这说明现象并不是角速度前馈公式单独造成的，而是姿态几何本身决定的。

## 8. 为什么向 (+Y) 飞时需要改变 $\phi$

气动力模型假设零升力系中没有侧向力：

$$
i_y^\top f^\alpha = 0
$$

因此飞行器不能直接在自身侧向产生期望合力。若要获得世界 (+Y) 加速度，就必须旋转飞行器的推力/升力平面，使原本位于机体对称平面内的力获得世界 $Y$ 分量。

论文式 (17) 正是在做这件事：

$$
\phi = -\operatorname{atan2}\left(i_y^\top R_i^\psi f^i,\; i_z^\top f^i\right) + k\pi
$$

所以在图中的侧向运动阶段：

- 开始加速时需要 (+Y) 合力，$\phi$ 向一个方向变化；
- 轨迹中点加速度接近零，$\phi$ 返回附近；
- 结束前需要 (-Y) 合力进行减速，$\phi$ 向相反方向变化。

因为此时 $\theta\approx85^\circ$，这些 $\phi$ 变化主要由 $m_z$ 和差分推力实现。

## 9. 它是平滑混合，不是突然切换

执行器作用随 $\theta$ 连续变化：

$$
p_\phi = \cos\theta\,\dot\phi
$$

$$
r_\phi = \sin\theta\,\dot\phi
$$

因此：

- $\theta=0^\circ$：全部是 $p$，主要依靠差动舵面；
- $\theta=30^\circ$：$p$ 为主，开始出现 $r$；
- $\theta=45^\circ$：$p,r$ 同量级；
- $\theta=60^\circ$：$r$ 为主；
- $\theta=90^\circ$：全部是 $r$，主要依靠差分推力。

这正是论文”全局控制、无需模式切换”的一个体现。控制器没有写”悬停用差分推力、平飞用舵面”的硬切换规则，而是通过姿态几何、刚体动力学和控制分配自然形成连续过渡。

最后，项目使用 ZXY 而不是常见的 ZYX 欧拉角还有一个重要原因：传统 ZYX 在尾座飞行的 $\theta\approx\pm90^\circ$ 处会发生俯仰奇异，而 ZXY 把奇异位置转移到了 $\phi=\pm90^\circ$。当前图中 $\phi$ 只有约 $\pm10^\circ$，因此尾座悬停附近仍可正常表示。

# 3. 微分平坦条件二的含义

这里混淆了三件不同的事情：

1. 飞机沿世界 $Y$ 方向运动；
2. 世界系合力具有 $Y$ 分量；
3. 零升力坐标系中的合力具有 $\alpha_y$ 分量。

论文要求的只是第 3 项为零：

$$
i_y^\top f^\alpha = 0
$$

它并不禁止飞机侧向运动，也不要求世界系的 $Y$ 向力为零。

## 1. 论文条件真正表示什么

因为 $f^\alpha$ 已经是表达在零升力坐标系中的力：

$$
f^\alpha =
\begin{bmatrix}
f_x^\alpha \\
f_y^\alpha \\
f_z^\alpha
\end{bmatrix}
$$

所以：

$$
i_y^\top f^\alpha
= \begin{bmatrix}0 & 1 & 0\end{bmatrix} f^\alpha
= f_y^\alpha
$$

论文的条件就是：

$$
f_y^\alpha = 0
$$

也就是气动力和推力合力始终位于飞机的零升力 $x$-$z$ 对称平面内。

它不是：

$$
i_y^\top f^i = 0
$$

后者才表示世界 NED 系的 $Y$ 向力为零。

从物理向量角度，更严谨的写法应该是：

$$
(\alpha_y^i)^\top f^i = 0
$$

其中：

$$
\alpha_y^i = R_\alpha^i \begin{bmatrix}0 \\ 1 \\ 0\end{bmatrix}
$$

由于零升力系是绕机体 $y$ 轴旋转得到的，所以：

$$
\alpha_y = b_y
$$

因此该条件也可以写成：

$$
(b_y^i)^\top f^i = 0
$$

即合力垂直于当前机体展向轴 $b_y$。

## 2. 世界系侧向力可以非零

零升力系中的力经过姿态旋转后得到世界系力：

$$
f^i = R_\alpha^i \, f^\alpha
$$

即使：

$$
f^\alpha =
\begin{bmatrix}
f_x^\alpha \\
0 \\
f_z^\alpha
\end{bmatrix}
$$

经过旋转以后，世界系结果完全可以是：

$$
f^i =
\begin{bmatrix}
f_X^i \\
f_Y^i \\
f_Z^i
\end{bmatrix},
\qquad f_Y^i \neq 0
$$

这和多旋翼倾斜以后产生水平加速度完全一样：多旋翼机体系中的推力始终只有一个方向，但转到世界系以后可以同时具有水平和垂直分量。

## 3. $\phi$ 正是用来保证这个条件的

假设偏航旋转后，期望世界系合力表达在 $\psi$ 中间系中为：

$$
f^\psi =
\begin{bmatrix}
f_x^\psi \\
f_y^\psi \\
f_z^\psi
\end{bmatrix}
$$

再绕中间 $x$ 轴旋转 $\phi$，$\psi$ 系到 $\phi$ 系的转换是：

$$
R_\psi^\phi =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & \sin\phi \\
0 & -\sin\phi & \cos\phi
\end{bmatrix}
$$

转换后力的 $y$ 分量是：

$$
f_y^\phi = \cos\phi \cdot f_y^\psi + \sin\phi \cdot f_z^\psi
$$

论文选择：

$$
\phi = -\operatorname{atan2}\!\left(f_y^\psi,\, f_z^\psi\right) + k\pi
$$

忽略 $k\pi$ 分支，令：

$$
r = \sqrt{(f_y^\psi)^2 + (f_z^\psi)^2}
$$

则：

$$
\cos\phi = \frac{f_z^\psi}{r}, \qquad
\sin\phi = -\frac{f_y^\psi}{r}
$$

代入力的 $y$ 分量：

$$
\begin{aligned}
f_y^\phi
&= \frac{f_z^\psi}{r} f_y^\psi - \frac{f_y^\psi}{r} f_z^\psi \\
&= 0
\end{aligned}
$$

接下来的 $\bar\theta$ 是绕 $y$ 轴旋转，而绕 $y$ 轴不会改变力的 $y$ 分量，所以最终：

$$
f_y^\alpha = 0
$$

因此论文式 (17) 计算 $\phi$ 的目的，恰恰就是在需要世界侧向加速度时，通过旋转飞机保证合力仍然位于飞机自身的 $x$-$z$ 平面。

## 4. 侧向飞行不等于存在侧向力

即使飞机沿世界 $+Y$ 方向运动，也不代表必须始终存在 $+Y$ 向力。

牛顿第二定律约束的是加速度：

$$
f = m a
$$

不是速度：

$$
f \neq m v
$$

因此：

- 向 $+Y$ 加速时，需要世界 $+Y$ 向合力；
- 保持恒定 $+Y$ 速度时，理想情况下不需要 $Y$ 向合力；
- 向 $+Y$ 减速时，需要世界 $-Y$ 向合力。

在当前仿真中，侧向气动阻力被忽略，所以飞机一旦获得 $Y$ 向速度，即使之后：

$$
f_Y^i = 0
$$

也会继续沿 $Y$ 方向运动。

这就是图中侧向轨迹的典型过程：

- **开始侧移：** 产生 $+Y$ 加速度，$\phi$ 向一侧倾斜，世界系合力具有 $+Y$ 分量；
- **轨迹中点：** $Y$ 速度最大、$Y$ 加速度接近零，$\phi$ 回到零附近，飞机依靠惯性继续侧向运动；
- **结束侧移：** 产生 $-Y$ 加速度，$\phi$ 向相反方向倾斜，世界系合力具有 $-Y$ 分量。

在整个过程中都可以保持：

$$
f_y^\alpha = 0
$$

## 5. 侧滑速度也不违反条件

论文允许非协调飞行和刀锋飞行，此时完全可能存在：

$$
v_y^\alpha \neq 0
$$

即空气相对速度在机体展向 $b_y = \alpha_y$ 上有分量。

但论文简化模型仍然规定：

$$
f_y^\alpha = 0
$$

注意一个是速度，一个是力：

$$
v_y^\alpha \neq 0
$$

并不必然要求：

$$
f_y^\alpha \neq 0
$$

**论文之所以敢作这个简化，是因为飞翼没有机身和垂直尾翼，侧向气动力相对于升力、阻力和推力较小。**

当前气动力代码也是这样实现的：

```python
f_T_alpha     = [Fx, 0, Fz]
f_delta_alpha = [0,  0, Fz]
f_w_alpha     = [Fx, 0, Fz]
```

因此仿真环境中的 `f_alpha_real[1]` 理论上应始终为零。

## 6. 你可能检查错了变量

如果你观察到侧向飞行时”条件不成立”，很可能检查的是以下量之一：

- `f_c_i[1]` —— 这是世界系期望力的 $Y$ 分量，可以非零。
- `a_real_i[1]` —— 这是世界系 $Y$ 向加速度，可以非零。
- `vel[1]` —— 这是世界系 $Y$ 向速度，可以非零。
- `v_b[1]` —— 这是机体系展向速度，可以非零，表示侧滑或刀锋方向运动。

真正对应论文条件的是：

- `f_alpha_real[1]`

或者使用世界系力验证：

```python
alpha_y_i = R_alpha_to_i[:, 1]
condition = alpha_y_i @ f_i
```

应满足 `condition ≈ 0`。

但下面这个量不需要为零：

```python
np.array([0.0, 1.0, 0.0]) @ f_i
```

因为它检查的是世界系 $Y$ 向力。

## 7. 如果真实飞机确实存在侧向气动力

真实飞机由于翼厚、机身结构、安装误差、侧滑阻力和阵风，通常不会严格满足：

$$
f_y^\alpha = 0
$$

论文的微分平坦性证明针对的是简化模型，而不是完整真实空气动力学。

这些额外侧向力被归入 $f_{\mathrm{ext}}$。

平坦性推导中假设 $f_{\mathrm{ext}}$ 在短时间内近似恒定。INDI 根据实际加速度估计并抵消这部分未建模力：

$$
f_c^i = m\,(a_c - \tilde a_{\mathrm{lpf}}) + f_{\mathrm{lpf}}^i
$$

所以即使真实飞机存在一定侧向力，控制器仍然可以通过后续增量修正继续飞行。

但如果侧向未建模力很大、变化很快或使执行器饱和，平坦性简化模型和 INDI 的局部修正就可能不足，跟踪性能会下降。

因此，对于当前仿真：

- 如果检查 `f_alpha_real[1]`，条件实际上始终成立；
- 世界 $Y$ 向速度、加速度和力可以非零；
- 侧向飞行是通过旋转飞机的 $x$-$z$ 受力平面实现的，并没有违反微分平坦性推导的前提。


# 4. 刀锋飞行跟踪效果差
论文中的刀锋圆周跟踪好，而且两台电机存在差速，这两件事并不矛盾。关键是要把”总推力”和”差分推力”的作用分开。

## 1. 刀锋飞行不等于依靠侧向力转弯

刀锋圆周飞行时：

- 机体 $b_y$ 轴，也就是翼尖方向，基本沿轨迹切线；
- 速度主要沿 $b_y$；
- 两台电机的总推力方向则指向圆心，并略微向上抵消重力。

因此近似满足：

$$
v^b \approx
\begin{bmatrix}
0 \\ v \\ 0
\end{bmatrix},
\qquad
f^b \approx
\begin{bmatrix}
-f_r \\ 0 \\ -mg
\end{bmatrix}
$$

也就是说：

$$
i_y^\top f^\alpha \approx 0
$$

仍然成立。

“有很大的侧向速度”和”存在侧向气动力”是两件不同的事情。刀锋飞行允许：

$$
i_y^\top v^\alpha \neq 0
$$

但论文模型要求的是：

$$
i_y^\top f^\alpha = 0
$$

所以微分平坦性前提没有被破坏。

对于半径 3.5 m、速度 8.1 m/s 的圆：

$$
a_r = \frac{v^2}{r}
= \frac{8.1^2}{3.5}
\approx 18.75\ \mathrm{m/s^2}
\approx 1.91g
$$

同时还需要抵消重力，因此总比力约为：

$$
\sqrt{18.75^2 + 9.81^2}
\approx 21.16\ \mathrm{m/s^2}
\approx 2.16g
$$

刀锋飞行几乎不能利用机翼升力来提供这部分向心力，所以主要依靠两台电机的总推力：

$$
T_\Sigma = T_1 + T_2
$$

这就是论文刀锋圆周时两台电机都超过 2000 rad/s 的原因。paper/full.md:656

## 2. 电机差速主要控制姿态，而不是提供向心力

把电机推力分成：

$$
T_\Sigma = T_1 + T_2,\qquad
\Delta T = T_1 - T_2
$$

它们的作用不同：

- $T_\Sigma$：提供主要平动力，包括圆周向心力和重力补偿；
- $\Delta T$：产生姿态控制力矩，特别是 $b_z$ 方向力矩。

论文式 (37) 明确给出：

$$
\Delta T =
\frac{m_z}
{
l_{Ty}\!\left[
\cos\alpha_0 \cos\bar\alpha\,(1 - c_{D_T})
- \sin\alpha_0 \sin\bar\alpha\,(c_{L_T} - 1)
\right]
- \sin\alpha_T\,(c_\mu / c_T)
}
$$

因此只要姿态控制器要求非零 $m_z$，两台电机就会出现差速，见 paper/full.md:349。

近似来说：

$$
m_z \approx l_{Ty}\,\Delta T
$$

取 $l_{Ty} \approx 0.15$ m，即使差分推力只有 0.3 N，也能产生：

$$
m_z \approx 0.045\ \mathrm{N{\cdot}m}
$$

对于 0.7 kg 的小飞机，这已经是很有效的姿态控制力矩。

因此论文 Fig. 9 中：

- 两台电机都在约 2000 rad/s，负责提供总向心力；
- 两条曲线相差约几十到一百 rad/s，负责姿态稳定和补偿扰动力矩。

这与 Fig. 11 的”大差分推力转弯”不同。Fig. 11 中的 6.1 N 差分推力是为了在半秒内完成约 $180^\circ$ 快速掉头；Fig. 9 的刀锋圆周差速小得多。

## 3. 稳态圆周仍然可能需要持续差速

即使圆周角速度恒定，也不代表所需机体系力矩一定为零。

刚体转动方程是：

$$
m = J \dot\Omega + \Omega \times J \Omega
$$

对于圆周飞行：

$$
\dot\psi = \frac{v}{r}
\approx 2.31\ \mathrm{rad/s}
\approx 133^\circ/\mathrm{s}
$$

虽然世界系中的偏航角速度恒定，但投影到机体系后，$\Omega$ 通常同时具有 $b_x$ 和 $b_z$ 分量。

因此可能存在：

$$
\Omega \times J \Omega \neq 0
$$

此外实机还有：

- 非对角惯量 $J_{xz}$；
- 电机和机翼左右不完全对称；
- 桨流差异；
- 舵面安装误差；
- 未建模的转动气动力矩；
- 刀锋飞行本身的不稳定性；
- $14^\circ$ 的姿态偏置，用于补偿阻力；
- 动捕和姿态跟踪误差。

这些都会使姿态控制器持续要求小的 $m_z$，随后通过式 (37) 转换为电机差速。

论文并不要求稳态时：

$$
T_1 = T_2
$$

只要求总推力和力矩满足轨迹及姿态要求。

## 4. 论文为什么能够达到约 15 cm RMS

论文自己给出的一个重要原因是：定常刀锋圆周中，电机转速和舵面偏角几乎保持不变。

论文中刀锋圆周大约需要：

- 两台电机超过 2000 rad/s；
- 舵面约 $-20^\circ$；
- 机体向速度方向倾斜约 $14^\circ$；
- 机头俯转约 $152^\circ$，让推力指向圆心；
- 实际速度约 7.8 m/s，而不是参考的完整 8.1 m/s；
- 实际圆半径也略小于参考半径。

因为舵面基本是稳态偏转，所以：

$$
\dot\delta \approx 0
$$

论文位置控制器没有完整考虑的”舵面瞬态平动力”也几乎不存在。论文明确把这一点作为刀锋圆周跟踪较好的原因。paper/full.md:656

此外实机控制系统具有：

- 2000 Hz IMU 和控制更新；
- 电机光电编码器；
- 舵机电位器反馈；
- 电机和舵面测量采用匹配的 15 Hz 低通滤波；
- 360 Hz 动捕；
- IMU 前向传播补偿约 18 ms 动捕延迟；
- 台架辨识的转速—油门二次映射；
- 电机转速积分控制，补偿电池电压变化；
- 通过实飞调出的控制增益。

这些条件使 INDI 可以直接看到实际角加速度和执行器变化，及时补偿论文模型中没有描述的力矩。paper/full.md:606

## 5. 当前仿真失败并不是”电机差速导致的”

当前刀锋圆周日志中：

- 最初 0.2 s 位置误差只有约 3.9 mm；
- 初始实际加速度几乎等于参考加速度；
- 第一圈差分推力 RMS 只有约 0.029 N；
- 第一圈最大差分推力只有约 0.093 N。

所以初始轨迹、刀锋姿态和总推力反解基本正确，电机差速也非常小。

真正的问题是小误差逐渐放大以后：

- 飞机相位开始领先/滞后参考；
- 推力方向产生误差；
- 轨迹逐渐向外扩张；
- 位置控制器继续提高向心力；
- 电机最终达到 2500 rad/s；
- 后期差分推力最大增至约 2.07 N。

也就是说，后期的大差速是控制器试图挽救已经发散的姿态和轨迹的结果，不是最初导致发散的原因。

## 6. 当前仿真缺少完整的周期配平

我加入的初始配平目前主要满足：

$$
c_{\mu T}\,(T_1 + T_2) + m_{\delta y} \approx 0
$$

即用同向舵面抵消 $c_{\mu T}$ 的俯仰力矩。

但定常圆周的完整转动平衡应该满足：

$$
m_\mathrm{body} = J \dot\Omega + \Omega \times J \Omega
$$

圆周稳态时通常 $\dot\Omega \approx 0$，仍需满足：

$$
m_\mathrm{body} = \Omega \times J \Omega
$$

当前刀锋圆周初始角速度约为：

$$
\Omega \approx
\begin{bmatrix}
1.36 \\ 0 \\ -1.87
\end{bmatrix}
\mathrm{rad/s}
$$

使用当前估算惯量：

$$
J = \operatorname{diag}(0.0095,\, 0.0030,\, 0.0115)
$$

得到所需陀螺耦合力矩约：

$$
\Omega \times J \Omega
\approx
\begin{bmatrix}
0 \\ 0.005 \\ 0
\end{bmatrix}
\mathrm{N{\cdot}m}
$$

当前初始化没有预先建立这部分力矩，所以仿真开始时出现了约：

$$
\dot\Omega_y \approx -1.22\ \mathrm{rad/s^2}
$$

的俯仰角加速度。

对于协调飞行，这个小扰动容易恢复；但刀锋飞行本身是不稳定状态，又几乎没有被动方向稳定性，因此小姿态误差会逐渐变成推力方向误差和圆周相位误差。

论文实验不是在数学上”瞬间生成一个圆周状态然后开始积分”，而是飞机已经进入并稳定在周期轨道后记录多圈数据。此时：

- 同向舵面配平已经建立；
- 差分推力配平已经建立；
- 陀螺耦合力矩已经由 INDI 补偿；
- 滤波器和电机积分器都处于稳态。

这也是当前仿真与论文实验的重要区别。

## 结论

论文中的电机差速是正常且必要的：

$$
\boxed{\text{总推力负责圆周向心力，差分推力负责姿态稳定}}
$$

论文刀锋圆周跟踪好，主要因为它是一个接近定常的周期飞行状态，舵面和电机变化很小，再结合高带宽实机 INDI 和完整执行器反馈，能够持续稳定本来不稳定的刀锋姿态。

当前仿真需要进一步加入”完整周期配平”，同时求解：

$$
R_\alpha^i \, f^\alpha = m\,(a - g\,i_z)
$$

$$
m_\mathrm{body} = J \dot\Omega + \Omega \times J \Omega
$$

并联合求解：

$$
q,\quad T_1,\quad T_2,\quad \delta_1,\quad \delta_2
$$

而不能只配平总推力和 $c_{\mu T}$ 俯仰力矩。把 $c_{\mu T}$ 临时改成零虽然会让仿真变好，但只是删除了真实的舵面—俯仰—平动力耦合，不是论文控制算法的正确复现。


# 5. 电机和舵机一阶动态代码解释

对应代码如下：

```python
# 模拟电机和舵机的不同一阶动态。舵面另施加带载角速率
# 限制，避免单个仿真步内产生不可实现的大幅跳变。
motor_alpha = 1.0 - np.exp(-dt / cfg.motor_time_constant)
servo_alpha = 1.0 - np.exp(-dt / cfg.servo_time_constant)

omega_rotors += motor_alpha * (omega_cmd - omega_rotors)
omega_rotors = np.clip(omega_rotors, cfg.omega_min, cfg.omega_max)

delta_step = servo_alpha * (delta_cmd - delta_flaps)
max_delta_step = cfg.servo_rate_max * dt
delta_flaps += np.clip(delta_step, -max_delta_step, max_delta_step)
delta_flaps = np.clip(delta_flaps, -cfg.delta_max, cfg.delta_max)
```

这段代码的作用是：控制器给出的 `omega_cmd` 和 `delta_cmd` 不是立即成为真实执行器状态，而是先经过电机、舵机的动态响应和物理限幅。这样得到的 `omega_rotors` 与 `delta_flaps` 才用于下一步动力学计算。

## 5.1 电机一阶动态

电机被近似为一阶惯性环节：

$$
\tau_m\dot{\omega}=\omega_{\mathrm{cmd}}-\omega
$$

其中：

- $\omega_{\mathrm{cmd}}$ 是控制器期望的电机转速；
- $\omega$ 是当前实际电机转速；
- $\tau_m$ 是 `motor_time_constant`，表示电机响应快慢。

在一个仿真步长 $\Delta t$ 内，假设命令保持不变，其精确离散形式为：

$$
\omega_{k+1}
=
\omega_k+
\left(1-e^{-\Delta t/\tau_m}\right)
(\omega_{\mathrm{cmd},k}-\omega_k)
$$

因此：

```python
motor_alpha = 1.0 - np.exp(-dt / cfg.motor_time_constant)
omega_rotors += motor_alpha * (omega_cmd - omega_rotors)
```

`motor_alpha` 是当前仿真步向目标值靠近的比例。若项目使用：

$$
\Delta t=0.01\ \mathrm{s},\qquad \tau_m=0.04\ \mathrm{s}
$$

则：

$$
\alpha_m=1-e^{-0.01/0.04}\approx0.221
$$

也就是每个仿真步消除约 $22.1\%$ 的剩余转速误差。例如转速从 $0$ 跳变到 $2000\ \mathrm{rad/s}$ 时，第一步约为 $442\ \mathrm{rad/s}$，第二步约为 $787\ \mathrm{rad/s}$。经过一个时间常数后达到最终变化量的约 $63.2\%$，约三个时间常数后达到 $95\%$。

随后：

```python
omega_rotors = np.clip(
    omega_rotors, cfg.omega_min, cfg.omega_max
)
```

把转速限制在电机允许范围内。若配置为 $0$ 到 $2500\ \mathrm{rad/s}$，则实际转速不会越过该范围。

这里需要注意：代码只给电机设置了一阶惯性和转速上下限，没有像舵机一样单独设置硬性的转速变化率限制。因此电机的变化速度由时间常数和当前误差共同决定；误差越大，瞬时变化率也越大。

## 5.2 舵机一阶动态

舵机同样先经过一阶惯性：

$$
\tau_\delta\dot{\delta}
=
\delta_{\mathrm{cmd}}-\delta
$$

其未经速率限制的离散增量为：

$$
\Delta\delta_{\mathrm{raw}}
=
\left(1-e^{-\Delta t/\tau_\delta}\right)
(\delta_{\mathrm{cmd}}-\delta)
$$

对应：

```python
servo_alpha = 1.0 - np.exp(-dt / cfg.servo_time_constant)
delta_step = servo_alpha * (delta_cmd - delta_flaps)
```

若 $\tau_\delta=0.03\ \mathrm{s}$ 且 $\Delta t=0.01\ \mathrm{s}$，则：

$$
\alpha_\delta=1-e^{-0.01/0.03}\approx0.283
$$

即在没有其他约束时，每一步消除约 $28.3\%$ 的剩余舵角误差。

## 5.3 舵面角速度限制

真实舵机不能在一个仿真步内瞬间转过任意角度，因此代码又计算：

```python
max_delta_step = cfg.servo_rate_max * dt
```

若最大舵面角速度为：

$$
\dot{\delta}_{\max}=25\ \mathrm{rad/s}
$$

且 $\Delta t=0.01\ \mathrm{s}$，则单步最大变化量为：

$$
\Delta\delta_{\max}
=
\dot{\delta}_{\max}\Delta t
=
0.25\ \mathrm{rad}
\approx14.3^\circ
$$

更新时使用：

```python
delta_flaps += np.clip(
    delta_step, -max_delta_step, max_delta_step
)
```

因此一阶模型给出的单步变化若超过允许值，就会被截断。例如舵面从 $-1$ rad 接收到 $+1$ rad 命令时，一阶模型原始增量约为：

$$
0.283\times(1-(-1))=0.566\ \mathrm{rad}
$$

但速率限制只允许本步变化 $0.25$ rad。

## 5.4 舵角机械限位

最后：

```python
delta_flaps = np.clip(
    delta_flaps, -cfg.delta_max, cfg.delta_max
)
```

限制舵面的绝对偏转角。若 `delta_max = 1.0 rad`，则舵角范围为：

$$
-1.0\leq\delta\leq1.0\ \mathrm{rad}
\approx\pm57.3^\circ
$$

因此舵机同时受到三类约束：

1. 一阶响应延迟；
2. 最大舵面角速度；
3. 最大机械偏转角。

## 5.5 完整数学表达

电机更新可以写为：

$$
\omega_{k+1}
=
\operatorname{clip}
\left[
\omega_k+\alpha_m(\omega_{\mathrm{cmd},k}-\omega_k),
\omega_{\min},
\omega_{\max}
\right]
$$

舵机更新可以写为：

$$
\Delta\delta_k
=
\operatorname{clip}
\left[
\alpha_\delta(\delta_{\mathrm{cmd},k}-\delta_k),
-\dot{\delta}_{\max}\Delta t,
+\dot{\delta}_{\max}\Delta t
\right]
$$

$$
\delta_{k+1}
=
\operatorname{clip}
\left[
\delta_k+\Delta\delta_k,
-\delta_{\max},
+\delta_{\max}
\right]
$$

相应的信号流程为：

```text
电机命令
omega_cmd
    ↓
电机一阶动态
    ↓
转速上下限
    ↓
omega_rotors
    ↓
推力和反扭矩

舵面命令
delta_cmd
    ↓
舵机一阶动态
    ↓
舵面角速度限制
    ↓
舵角机械限位
    ↓
delta_flaps
    ↓
气动力和气动力矩
```

## 5.6 为什么不能直接令实际值等于命令值

若直接写成：

```python
omega_rotors = omega_cmd
delta_flaps = delta_cmd
```

就相当于假设电机和舵机能够瞬时响应。这样会高估执行器能力，使控制器在仿真中可以依靠不可实现的高频、大幅控制动作稳定飞机，通常会得到过于理想的跟踪结果，也无法暴露执行器饱和、相位滞后和带宽不足等问题。

## 5.7 与测量低通滤波的区别

这里的一阶模型描述的是执行器本身的物理动态，作用在控制命令到真实执行器状态之间。项目中的测量低通滤波则描述传感器或估计信号的带宽，作用在真实状态到控制器测量值之间。两者都可能带来滞后，但物理含义和信号位置不同，不能互相替代。

## 5.8 日志记录的注意事项

`omega_rotors += ...` 和 `delta_flaps += ...` 都是原地更新。如果日志直接保存数组引用，例如：

```python
log_rotors.append(omega_rotors)
```

历史记录可能全部指向同一个数组，之后更新会覆盖先前数据。更安全的写法是：

```python
log_rotors.append(omega_rotors.copy())
log_flaps.append(delta_flaps.copy())
```

这样每个时刻保存的都是独立快照。


# 6. INDI 中加速度滤波与执行器滤波的关系

这部分滤波不能简单理解为“让曲线更平滑”。它是 INDI 控制律的一部分，主要完成三件事：

1. 抑制 IMU 振动和数值微分带来的高频噪声；
2. 使加速度测量与执行器模型输出具有近似相同的相位延迟；
3. 从舵面运动中分离出快速瞬态分量，并从线加速度反馈中扣除它，以减弱舵面非最小相位响应引起的俯仰振荡。

代码中存在两组需要成对匹配的信号：

| INDI 通道 | 惯性测量侧 | 执行器模型侧 |
|---|---|---|
| 线加速度 INDI | `a_lpf_tilde` | `f_alpha_lpf` |
| 角加速度 INDI | `omega_dot_lpf` | `m_lpf` |

其整体关系可以概括为：

```text
真实线加速度/IMU
        ↓ 主低通
      a_lpf
        ↓ 减去舵面瞬态加速度
  a_lpf_tilde ───────────────┐
                             │ Eq. (46)
电机转速 ──主低通───────────┤
舵面偏角 ──主低通──慢低通───┤
                             ↓
                    f_alpha_lpf
                             ↓
                       期望合力 f_c

真实角加速度
        ↓ 主低通
 omega_dot_lpf ──────────────┐
                             │ Eq. (52)
电机转速 ──主低通───────────┤
舵面偏角 ──主低通───────────┤
                             ↓
                          m_lpf
                             ↓
                       期望力矩 m_c
```

## 6.1 二阶 Butterworth 滤波器是什么意思

当前代码按照论文 Sec. VI.A 使用二阶数字 Butterworth 滤波器。归一化后的二阶离散滤波器可写为：

$$
y_k=b_0x_k+b_1x_{k-1}+b_2x_{k-2}-a_1y_{k-1}-a_2y_{k-2}
$$

与原来只保存一个历史状态的一阶指数滤波相比，二阶滤波器具有两个极点，截止频率之后的滚降更快。Butterworth 的特点是通带内幅值响应最大平坦，二阶低通的连续原型幅值满足：

$$
|H_{\mathrm{LP}}(j2\pi f)|
=
\frac{1}{\sqrt{1+(f/f_c)^4}}
$$

因此在 $f=f_c$ 时：

$$
20\log_{10}|H|=-3.0103\ \mathrm{dB}
$$

论文公开的实机频率与当前仿真配置分别为：

$$
f_{s,\mathrm{paper}}=2000\ \mathrm{Hz},\qquad
f_{s,\mathrm{sim}}=500\ \mathrm{Hz},\qquad
f_{c,\mathrm{LP}}=15\ \mathrm{Hz},\qquad
f_{c,\mathrm{HP}}=1\ \mathrm{Hz}
$$

为缩短仿真时间，控制与滤波更新率暂时使用 $500\ \mathrm{Hz}$。数字滤波器系数通过 `scipy.signal.butter(..., fs=500, output="sos")` 重新设计，并以二阶节的转置直接二型结构逐采样更新。`utils/filters.py` 对所有通道复用同一组系数，从而保证 IMU、电机转速和舵面测量具有一致的幅频与相频响应。

## 6.2 加速度低通 `a_lpf`

代码为：

```python
a_real_i = state_dot[3:6]
a_lpf = acceleration_lpf_filter.update(a_real_i)
```

`state_dot[3:6]` 是世界 NED 坐标系中的速度导数：

$$
\pmb a^i=\dot{\pmb v}^i
=g\pmb i_z+\frac{1}{m}
\left(\pmb R_\alpha^i\pmb f^\alpha+\pmb f_{\mathrm{ext}}^i\right)
$$

因此这里的 `a_real_i` 已经包含重力加速度。在 NED 坐标系中，世界 $+Z$ 轴向下，所以重力项为 $+g\pmb i_z$。

论文 Eq. (42) 写成：

$$
\pmb a_{\mathrm{lpf}}^i
=
\left(
\pmb R_b^i\hat{\pmb a}^{\,b}
+g\pmb i_z
\right)_{\mathrm{lpf}}
$$

实际飞机的加速度计首先给出机体系中的比力或去重力惯性测量，需要转换到世界系并补回重力；当前仿真直接读取动力学模型计算的 $\dot{\pmb v}^i$，所以省略了传感器坐标转换过程，但物理结果对应论文中的世界系重力修正加速度。

低通的主要目的包括：

- 抑制电机和螺旋桨振动造成的加速度计高频噪声；
- 避免高频噪声直接进入位置控制器 Eq. (41)；
- 避免 INDI 在 Eq. (46) 中把测量噪声误认为真实加速度误差，并产生高频力指令。

滤波的代价是延迟。若只滤波加速度而不滤波执行器测量，INDI 会把这种延迟误认为未建模外力，因此必须进行通道匹配。

## 6.3 为什么电机和舵面测量也要使用同一个主低通

代码为：

```python
omega_rotors_lpf = rotor_lpf_filter.update(omega_rotors)
delta_flaps_lpf = flap_lpf_filter.update(delta_flaps)
```

电机转速和舵面角本身通常比 IMU 加速度干净，但论文仍要求对它们使用与 IMU 相同的低通滤波器。原因是 INDI 不是单独观察某一个信号，而是在同一时刻比较：

$$
\text{惯性测量得到的实际加速度}
\quad\text{与}\quad
\text{执行器模型预测的加速度}
$$

例如电机转速突然增加后，真实推力和加速度几乎立即变化。若：

- `a_lpf` 经过低通，因此变化较慢；
- `omega_rotors` 不滤波，因此模型力立即增大；

那么 Eq. (44) 会暂时看到“模型力已经增加，但测量加速度还没有增加”。控制器会把两者的瞬态差值错误地估计成外力或模型误差，进而给出多余的修正指令。

反过来，如果执行器测量比加速度通道延迟更多，也会产生符号相反的伪误差。故二者的重点不是噪声幅值相同，而是频率响应、相位延迟和时间戳尽量匹配。

可将这个原则写成：

$$
\pmb a_{\mathrm{lpf}}
\longleftrightarrow
\pmb f_{\mathrm{lpf}},
\qquad
\dot{\pmb\Omega}_{\mathrm{lpf}}
\longleftrightarrow
\pmb m_{\mathrm{lpf}}
$$

等号两边的量必须代表近似相同时间尺度上的飞行器响应。

## 6.4 舵面为什么还要分成去瞬态分量和瞬态分量

主低通后的舵面量继续经过论文规定的 $1\ \mathrm{Hz}$ 二阶 Butterworth 高通：

```python
delta_flaps_hpf = flap_hpf_filter.update(delta_flaps_lpf)
delta_flaps_without_transient = delta_flaps_lpf - delta_flaps_hpf
```

定义：

$$
\delta_{\mathrm{lpf}}
=
H_{\mathrm{main}}\delta
$$

$$
\delta_{\mathrm{hpf}}
=
H_{\mathrm{HP},1\mathrm{Hz}}\delta_{\mathrm{lpf}}
$$

$$
\delta_{\mathrm{without\ transient}}
=
\delta_{\mathrm{lpf}}-\delta_{\mathrm{hpf}}
$$

所以 `delta_flaps_hpf` 相对于原始舵面信号经历了两级滤波：

1. 先由 $15\ \mathrm{Hz}$ 二阶主低通去掉高频测量噪声并匹配 IMU 相位；
2. 再由 $1\ \mathrm{Hz}$ 二阶高通去掉配平和缓慢变化部分。

因此从原始舵面输入看，它实际是大约 $1$ 到 $15\ \mathrm{Hz}$ 的带通瞬态分量。这里不能再用一阶滤波中的 `lpf - slow_lpf` 代替，因为二阶高通具有不同的相位和幅值响应。

两部分满足严格的代数分解：

$$
\delta_{\mathrm{lpf}}
=
\delta_{\mathrm{without\ transient}}
+
\delta_{\mathrm{hpf}}
$$

其物理含义为：

- $\delta_{\mathrm{without\ transient}}$：从主低通舵角中扣除高通瞬态后的分量；
- $\delta_{\mathrm{hpf}}$：快速打舵、回舵及舵面振荡等瞬态部分。

## 6.5 舵面的非最小相位加速度响应

舵面动作同时产生两个效应：

1. 舵面直接改变气动力，几乎立即造成线加速度；
2. 舵面产生俯仰或滚转力矩，使飞机姿态变化，随后改变推力和机翼力方向。

对于某些轴向，这两个效应在短时间内可能方向相反。控制器为了获得期望的长期加速度而打舵时，最先测到的直接舵面加速度可能反而朝错误方向，这就是这里所说的非最小相位响应。

如果位置/线加速度环把这段短暂反向加速度当作飞机没有服从命令，就会进一步增大姿态或推力指令；内环随之打出更大舵量，之后姿态效应建立时又产生过量加速度，容易形成俯仰振荡。

论文 Eq. (43) 的处理不是删除舵面全部气动力，而是只从加速度反馈中减去快速舵面运动对应的可预测瞬态力。慢变舵面力仍保留在控制模型中。

## 6.6 三次气动力计算分别表示什么

### 6.6.1 线加速度 INDI 的低频模型力

```python
f_alpha_lpf, _, _ = aero.compute_forces_and_moments(
    v_b,
    omega_rotors_lpf[0],
    omega_rotors_lpf[1],
    delta_flaps_without_transient[0],
    delta_flaps_without_transient[1],
)
```

这里使用：

- 主低通后的电机转速；
- 去除高通瞬态后的舵面偏角；
- 当前机体系速度 `v_b`。

得到的 `f_alpha_lpf` 是零升力坐标系 $\alpha$ 下的低频模型合力：

$$
\pmb f_{\mathrm{lpf}}^\alpha
=
\pmb f^\alpha
\left(
\omega_{\mathrm{lpf}},
\delta_{\mathrm{without\ transient}},
\pmb v
\right)
$$

它包含低频推力、机翼力和去除瞬态后的舵面力，但不包含 `delta_flaps_hpf` 对应的快速舵面力。

### 6.6.2 角加速度 INDI 的模型力矩

```python
_, m_lpf, _ = aero.compute_forces_and_moments(
    v_b,
    omega_rotors_lpf[0],
    omega_rotors_lpf[1],
    delta_flaps_lpf[0],
    delta_flaps_lpf[1],
)
```

角加速度 INDI 没有采用 Eq. (43) 那样的舵面瞬态扣除，所以 `m_lpf` 使用完整的主低通舵面偏角 `delta_flaps_lpf`。快速舵面动作产生的力矩正是姿态控制所需要的控制作用，不能从角加速度模型中删除。

### 6.6.3 Eq. (43) 所需的瞬态舵面力

```python
_, _, f_delta_hpf = aero.compute_forces_and_moments(
    v_b,
    omega_rotors_lpf[0],
    omega_rotors_lpf[1],
    delta_flaps_hpf[0],
    delta_flaps_hpf[1],
)
```

`compute_forces_and_moments()` 的第三个返回值只包含舵面力，因此：

$$
\pmb f_{\delta_{\mathrm{hpf}}}^\alpha
=
\pmb f_\delta^\alpha
\left(
\omega_{\mathrm{lpf}},
\delta_{\mathrm{hpf}},
\pmb v
\right)
$$

当前气动力模型中的舵面力对舵角是线性的，所以：

$$
\pmb f_\delta^\alpha(\delta_{\mathrm{lpf}})
=
\pmb f_\delta^\alpha(\delta_{\mathrm{without\ transient}})
+
\pmb f_\delta^\alpha(\delta_{\mathrm{hpf}})
$$

这使去瞬态力与瞬态力的分离在当前模型内保持自洽。

## 6.7 Eq. (43) 如何修正加速度

代码为：

```python
R_alpha_to_i = R_b_to_i @ R_alpha_to_b
a_lpf_tilde = a_lpf - (
    1.0 / cfg.mass
) * (R_alpha_to_i @ f_delta_hpf)
```

`f_delta_hpf` 位于零升力坐标系 $\alpha$，而 `a_lpf` 位于世界坐标系 $i$，因此必须先通过：

$$
\pmb R_\alpha^i
=
\pmb R_b^i\pmb R_\alpha^b
$$

把瞬态舵面力旋转到世界系，再除以质量变成加速度：

$$
\pmb a_{\delta_{\mathrm{hpf}}}^i
=
\frac{1}{m}
\pmb R_\alpha^i
\pmb f_{\delta_{\mathrm{hpf}}}^\alpha
$$

最后得到论文 Eq. (43)：

$$
\tilde{\pmb a}_{\mathrm{lpf}}
=
\pmb a_{\mathrm{lpf}}
-
\frac{1}{m}
\pmb R_\alpha^i
\pmb f_{\delta_{\mathrm{hpf}}}^\alpha
$$

$\tilde{\pmb a}_{\mathrm{lpf}}$ 可以理解为：

> 如果忽略快速打舵直接造成的短时线加速度，飞机当前的低频加速度是多少。

它不是新的传感器测量，而是“滤波后的 IMU 加速度减去模型预测的舵面瞬态加速度”。

## 6.8 为什么 `a_lpf_tilde` 必须与使用去瞬态舵面的 `f_alpha_lpf` 配对

设滤波后的实际加速度近似为：

$$
\pmb a_{\mathrm{lpf}}
\approx
g\pmb i_z
+
\frac{1}{m}
\left[
\pmb R_\alpha^i
\left(
\pmb f_{\mathrm{without\ transient}}^\alpha
+
\pmb f_{\delta_{\mathrm{hpf}}}^\alpha
\right)
+
\pmb f_{\mathrm{ext}}^i
\right]
$$

减去瞬态舵面加速度后：

$$
\tilde{\pmb a}_{\mathrm{lpf}}
\approx
g\pmb i_z
+
\frac{1}{m}
\left(
\pmb R_\alpha^i
\pmb f_{\mathrm{without\ transient}}^\alpha
+
\pmb f_{\mathrm{ext}}^i
\right)
$$

此时 Eq. (44) 用去瞬态模型力估计外力：

$$
\hat{\pmb f}_{\mathrm{ext}}^i
=
m\left(
\tilde{\pmb a}_{\mathrm{lpf}}-g\pmb i_z
\right)
-
\pmb R_\alpha^i
\pmb f_{\mathrm{lpf}}^\alpha
$$

只要：

$$
\pmb f_{\mathrm{lpf}}^\alpha
\approx
\pmb f_{\mathrm{without\ transient}}^\alpha
$$

就有：

$$
\hat{\pmb f}_{\mathrm{ext}}^i
\approx
\pmb f_{\mathrm{ext}}^i
$$

进而得到 Eq. (46)：

$$
\pmb f_c^i
=
m\left(
\pmb a_c^i-\tilde{\pmb a}_{\mathrm{lpf}}^i
\right)
+
\pmb f_{\mathrm{lpf}}^i
$$

这里第一项是需要增加的力，第二项是当前低频执行器已经产生的力。

如果 `a_lpf_tilde` 已经减去了瞬态舵面力，但 `f_alpha_lpf` 仍使用完整的 `delta_flaps_lpf`，那么同一个瞬态分量会在 Eq. (44) 和 Eq. (46) 中出现不一致，控制器会把它误认为外力或额外所需控制力。因此代码必须采用：

```text
a_lpf_tilde  ↔  omega_rotors_lpf + delta_flaps_without_transient
```

而不能采用：

```text
a_lpf_tilde  ↔  omega_rotors_lpf + delta_flaps_lpf
```

## 6.9 角加速度滤波与执行器力矩滤波

代码按照论文先对陀螺仪角速度使用同一个 $15\ \mathrm{Hz}$ 主低通，再进行数值微分：

```python
omega_lpf_previous = omega_lpf.copy()
omega_lpf = angular_velocity_lpf_filter.update(omega_curr)
omega_dot_lpf = (omega_lpf - omega_lpf_previous) / dt
```

这样得到论文 Eq. (50) 使用的 $\dot{\pmb\Omega}_{\mathrm{lpf}}$，并在角加速度 INDI 中与 `m_lpf` 配对：

$$
\pmb m_c
=
\pmb J
\left(
\dot{\pmb\Omega}_c
-
\dot{\pmb\Omega}_{\mathrm{lpf}}
\right)
+
\pmb m_{\mathrm{lpf}}
$$

其逻辑与线加速度 INDI 相同：

- $\dot{\pmb\Omega}_{\mathrm{lpf}}$ 表示经过主低通的实际角响应；
- $\pmb m_{\mathrm{lpf}}$ 表示由同样经过主低通的电机和舵面测量预测出的当前力矩；
- 两者之差隐式包含未建模气动力矩、参数误差和外部扰动力矩。

角加速度通道不需要扣除 `f_delta_hpf`，因为舵面快速力矩是姿态内环希望利用的主要控制作用。Eq. (43) 只针对舵面直接线加速度造成的非最小相位问题。

仿真虽然可以直接取得 `state_dot[10:13]`，但控制反馈不再使用这个理想角加速度；它只用于记录未滤波真值。实际角 INDI 反馈严格采用“角速度低通后数值微分”的论文链路。

## 6.10 滤波更新在控制循环中的时序

当前滤波状态在完成物理动力学计算后更新，并在下一次控制循环中使用：

```text
第 k 次控制计算
    ↓
执行器动态更新
    ↓
计算真实力、力矩和 state_dot
    ↓
积分物理状态
    ↓
用第 k 次响应更新各滤波器
    ↓
第 k+1 次控制计算使用这些滤波状态
```

因此控制器看到的是上一采样周期结束时形成的滤波状态，天然包含至少一个离散控制周期的因果延迟。这是合理的实时控制结构，但分析相位裕度时必须把采样延迟、传感器滤波延迟和执行器动态延迟一起考虑。

## 6.11 与电机、舵机一阶动态的区别

第 5 节中的：

```python
omega_rotors += motor_alpha * (omega_cmd - omega_rotors)
delta_flaps += ...
```

表示真实执行器从命令到物理状态的动态，属于被控对象：

$$
\text{控制命令}
\rightarrow
\text{执行器动态}
\rightarrow
\text{真实转速/舵角}
$$

本节的：

```python
omega_rotors_lpf = ...
delta_flaps_lpf = ...
```

表示控制器对执行器测量进行信号处理，属于测量链路：

$$
\text{真实转速/舵角}
\rightarrow
\text{传感器测量与滤波}
\rightarrow
\text{INDI 使用的反馈量}
$$

两者会叠加，但不能互相替代。执行器时间常数描述电机或舵机确实需要多久才能运动到目标值；测量低通描述控制器多久、以多大带宽观察到该运动。

## 6.12 当前仿真与论文滤波实现的对应关系

论文实验使用：

- $2000\ \mathrm{Hz}$ 的 IMU 与控制更新频率；
- 截止频率 $15\ \mathrm{Hz}$ 的二阶 Butterworth 主低通；
- 对 IMU、电机转速和舵面偏角应用相同主低通；
- 截止频率 $1\ \mathrm{Hz}$ 的二阶 Butterworth 高通提取瞬态舵面分量。

当前仿真保留论文的滤波器结构，但降低更新率：

- $500\ \mathrm{Hz}$ 的控制与滤波更新频率，论文实机为 $2000\ \mathrm{Hz}$；
- 截止频率 $15\ \mathrm{Hz}$ 的二阶 Butterworth 主低通；
- IMU 角速度、世界系加速度、电机转速和舵面偏角共用相同主低通设计；
- 主低通舵面量经过截止频率 $1\ \mathrm{Hz}$ 的二阶 Butterworth 高通；
- 用 `delta_flaps_lpf - delta_flaps_hpf` 得到 Eq. (46) 使用的不含瞬态舵面分量；
- 用低通角速度的数值微分获得角加速度 INDI 反馈。

因此当前代码与论文的滤波器阶数和截止频率一致，但控制/滤波更新率不再完全一致。500 Hz 相对于 15 Hz 主截止频率仍有约 33 倍采样裕量，足以实现稳定的数字 Butterworth 滤波；其离散相位和采样延迟仍会与 2000 Hz 实机存在差异。其他实现边界还包括：仿真直接提供无噪声动力学状态，没有进一步模拟具体 IMU 噪声、编码器约 $200\ \mathrm{Hz}$ 的异步采样、模拟舵机电位器噪声和论文固件的 32 位浮点量化。

## 6.13 最重要的理解

这组滤波器的核心不是分别把加速度、电机和舵面数据“滤得好看”，而是保持以下两种一致性：

1. **时间一致性**：加速度测量与根据执行器测量计算的模型力必须具有相近延迟；
2. **物理分量一致性**：从加速度中减去舵面瞬态力后，Eq. (46) 使用的模型力也必须排除同一瞬态分量。

满足这两点时，INDI 中“测得的响应减去模型预测的响应”才主要代表真实的模型误差和外部扰动，而不会被滤波器自身的相位差或重复计算的舵面瞬态污染。


# 7. 双电机总推力与差分推力的可行分配

对应代码为：

```python
# 在保持总推力的前提下同时约束单电机的上、下限。
# T1=(T_c+delta_T)/2, T2=(T_c-delta_T)/2，因此差分推力
# 的可行区间会随总推力收缩。
thrust_min = self.cfg.c_T * self.cfg.omega_min**2
thrust_max = self.cfg.c_T * self.cfg.omega_max**2
T_c = np.clip(float(T_c), 2.0 * thrust_min, 2.0 * thrust_max)
delta_lower = max(2.0 * thrust_min - T_c, T_c - 2.0 * thrust_max)
delta_upper = min(T_c - 2.0 * thrust_min, 2.0 * thrust_max - T_c)
delta_T = np.clip(delta_T, delta_lower, delta_upper)
T1 = (T_c + delta_T) / 2.0
T2 = (T_c - delta_T) / 2.0
realized_delta_T = T1 - T2
```

这段代码的作用是：将控制器要求的总推力 `T_c` 和差分推力 `delta_T` 分配给两个电机，同时保证每个电机都不超过允许的转速和推力范围。

定义：

$$
T_c=T_1+T_2
$$

$$
\Delta T=T_1-T_2
$$

其中代码中的 `delta_T` 就是 $\Delta T$。联立两式可得论文 Eq. (38)：

$$
T_1=\frac{T_c+\Delta T}{2},\qquad
T_2=\frac{T_c-\Delta T}{2}
$$

## 7.1 将电机转速限制转换为推力限制

代码首先计算：

```python
thrust_min = self.cfg.c_T * self.cfg.omega_min**2
thrust_max = self.cfg.c_T * self.cfg.omega_max**2
```

根据论文 Eq. (6)：

$$
T_i=c_T\omega_i^2
$$

因此电机转速范围：

$$
\omega_{\min}\leq\omega_i\leq\omega_{\max}
$$

对应单个电机的推力范围：

$$
T_{\min}=c_T\omega_{\min}^2,\qquad
T_{\max}=c_T\omega_{\max}^2
$$

当前配置为：

$$
c_T=1.8\times10^{-6}\ \mathrm{N/(rad/s)^2}
$$

$$
\omega_{\min}=0,\qquad
\omega_{\max}=2500\ \mathrm{rad/s}
$$

所以：

$$
T_{\min}=0
$$

$$
T_{\max}
=
1.8\times10^{-6}\times2500^2
=
11.25\ \mathrm N
$$

即单个电机最多产生约 $11.25\ \mathrm N$ 推力。

这里默认 $c_T>0$ 且电机转速非负，因此转速区间可直接通过平方转换为推力区间。

## 7.2 限制总推力

代码为：

```python
T_c = np.clip(
    float(T_c),
    2.0 * thrust_min,
    2.0 * thrust_max,
)
```

两个电机都处于允许范围时，总推力必须满足：

$$
2T_{\min}\leq T_c\leq2T_{\max}
$$

当前参数下：

$$
0\leq T_c\leq22.5\ \mathrm N
$$

如果控制器要求：

$$
T_c=25\ \mathrm N
$$

两个电机最多只能实现：

$$
T_c=22.5\ \mathrm N
$$

因此这里的“保持总推力”是指保持经过物理限幅后的 `T_c`。如果原始总推力命令超出了电机能力，则不可能继续保持原始命令，只能将其截断到可实现范围。

`float(T_c)` 用于把 NumPy 标量或其他标量类型转换为普通 Python 浮点数，避免后续 `max`、`min` 和 `np.clip` 出现不必要的数组类型传播。

## 7.3 从第一个电机约束推导差分推力范围

第一个电机必须满足：

$$
T_{\min}\leq T_1\leq T_{\max}
$$

代入：

$$
T_1=\frac{T_c+\Delta T}{2}
$$

得到：

$$
T_{\min}
\leq
\frac{T_c+\Delta T}{2}
\leq
T_{\max}
$$

两边乘以 2 并移项：

$$
2T_{\min}-T_c
\leq
\Delta T
\leq
2T_{\max}-T_c
$$

因此第一个电机给出的差分推力区间为：

$$
\Delta T
\in
\left[
2T_{\min}-T_c,\,
2T_{\max}-T_c
\right]
$$

## 7.4 从第二个电机约束推导差分推力范围

第二个电机必须满足：

$$
T_{\min}\leq T_2\leq T_{\max}
$$

代入：

$$
T_2=\frac{T_c-\Delta T}{2}
$$

得到：

$$
T_{\min}
\leq
\frac{T_c-\Delta T}{2}
\leq
T_{\max}
$$

整理后：

$$
T_c-2T_{\max}
\leq
\Delta T
\leq
T_c-2T_{\min}
$$

因此第二个电机给出的差分推力区间为：

$$
\Delta T
\in
\left[
T_c-2T_{\max},\,
T_c-2T_{\min}
\right]
$$

## 7.5 取两个约束区间的交集

要让两个电机同时满足限制，$\Delta T$ 必须位于两个区间的交集。因此下界取两个下界中较大的一个：

$$
\Delta T_{\min}
=
\max
\left(
2T_{\min}-T_c,\,
T_c-2T_{\max}
\right)
$$

上界取两个上界中较小的一个：

$$
\Delta T_{\max}
=
\min
\left(
2T_{\max}-T_c,\,
T_c-2T_{\min}
\right)
$$

这正是代码：

```python
delta_lower = max(
    2.0 * thrust_min - T_c,
    T_c - 2.0 * thrust_max,
)
delta_upper = min(
    2.0 * thrust_max - T_c,
    T_c - 2.0 * thrust_min,
)
```

然后将控制器要求的差分推力限制在可行区间内：

```python
delta_T = np.clip(
    delta_T,
    delta_lower,
    delta_upper,
)
```

如果控制器要求的 $\Delta T$ 可以实现，则保持原值；如果不能实现，则取最接近要求的可行边界值。

## 7.6 当前零最小推力配置下的简化形式

当前：

$$
T_{\min}=0
$$

所以差分推力范围可简化为：

$$
|\Delta T|
\leq
\min
\left(
T_c,\,
2T_{\max}-T_c
\right)
$$

这个关系说明差分推力能力会随总推力变化：

- 总推力接近零时，没有足够的基础推力进行差速；
- 总推力位于中间范围时，差分推力能力最大；
- 总推力接近最大值时，其中一个电机容易达到上限，差分推力能力再次减小。

其可行范围在 $T_c=T_{\max}$ 时最大：

$$
|\Delta T|_{\max}=T_{\max}
$$

此时理论上一个电机可达到 $T_{\max}$，另一个电机可降到零。

## 7.7 低总推力示例

假设：

$$
T_c=4\ \mathrm N
$$

当前单电机范围为：

$$
0\leq T_i\leq11.25\ \mathrm N
$$

则差分推力范围为：

$$
-4\leq\Delta T\leq4\ \mathrm N
$$

取最大正差分推力 $\Delta T=4\ \mathrm N$：

$$
T_1=\frac{4+4}{2}=4\ \mathrm N
$$

$$
T_2=\frac{4-4}{2}=0
$$

此时第二个电机已经达到最小推力，不能继续增加正差分推力，否则会要求：

$$
T_2<0
$$

但普通固定旋翼不能产生负推力。

## 7.8 接近最大总推力时的示例

假设：

$$
T_c=20\ \mathrm N
$$

因为两个电机各自最大只能产生 $11.25\ \mathrm N$，差分推力范围为：

$$
-2.5\leq\Delta T\leq2.5\ \mathrm N
$$

取最大正差分推力 $\Delta T=2.5\ \mathrm N$：

$$
T_1=\frac{20+2.5}{2}=11.25\ \mathrm N
$$

$$
T_2=\frac{20-2.5}{2}=8.75\ \mathrm N
$$

此时第一个电机已经达到最大推力，不能继续增大差分推力。

这也解释了为什么飞行器在高总推力状态下不一定具有更强的差分推力控制能力：两个电机虽然都很有力，但留给“一增一减”的余量反而变小。

## 7.9 计算两个电机的实际推力

完成总推力和差分推力限幅后：

```python
T1 = (T_c + delta_T) / 2.0
T2 = (T_c - delta_T) / 2.0
```

由构造可知：

$$
T_1+T_2=T_c
$$

$$
T_1-T_2=\Delta T
$$

并且由于 $\Delta T$ 已经被限制在可行区间中：

$$
T_{\min}\leq T_1,T_2\leq T_{\max}
$$

随后可以由 Eq. (6) 反解电机转速：

$$
\omega_1=\sqrt{\frac{T_1}{c_T}},\qquad
\omega_2=\sqrt{\frac{T_2}{c_T}}
$$

## 7.10 `realized_delta_T` 的含义

代码最后计算：

```python
realized_delta_T = T1 - T2
```

它表示经过总推力限幅和差分推力限幅后，两个电机真正能够实现的差分推力：

$$
\Delta T_{\mathrm{realized}}
=
T_1-T_2
$$

在当前实现中，代数上有：

$$
\Delta T_{\mathrm{realized}}
=
\Delta T_{\mathrm{clipped}}
$$

但单独保存这个变量仍然很重要，因为控制器最初请求的差分推力可能已经被截断。后续计算电机反扭矩、偏心推力力矩以及需要由舵面补偿的剩余力矩时，都应该使用实现后的 `realized_delta_T`，而不能继续使用饱和前的差分推力命令。

例如电机反扭矩差与差分推力的关系为：

$$
\mu_1+\mu_2
=
\frac{c_\mu}{c_T}
\Delta T_{\mathrm{realized}}
$$

如果后续仍使用限幅前的 $\Delta T$，控制分配器就会错误地认为电机产生了一个实际不存在的力矩，导致剩余舵面力矩计算不自洽。

## 7.11 为什么不能分别截断 `T1` 和 `T2`

一种看似简单的写法是先计算两个电机推力，再分别截断：

```python
T1 = np.clip(
    (T_c + delta_T) / 2.0,
    thrust_min,
    thrust_max,
)
T2 = np.clip(
    (T_c - delta_T) / 2.0,
    thrust_min,
    thrust_max,
)
```

但独立截断后，通常会同时改变总推力和差分推力：

$$
T_{1,\mathrm{clipped}}
+
T_{2,\mathrm{clipped}}
\neq
T_c
$$

$$
T_{1,\mathrm{clipped}}
-
T_{2,\mathrm{clipped}}
\neq
\Delta T
$$

例如要求：

$$
T_c=20\ \mathrm N,\qquad
\Delta T=10\ \mathrm N
$$

未经限制时：

$$
T_1=15\ \mathrm N,\qquad
T_2=5\ \mathrm N
$$

如果只把第一个电机截断到 $11.25\ \mathrm N$，则变成：

$$
T_1=11.25\ \mathrm N,\qquad
T_2=5\ \mathrm N
$$

实际总推力为：

$$
T_{\mathrm{realized}}=16.25\ \mathrm N
$$

实际差分推力为：

$$
\Delta T_{\mathrm{realized}}=6.25\ \mathrm N
$$

总推力和差分推力都偏离了原来的目标。

当前实现先限制 $T_c$，再求固定该 $T_c$ 时 $\Delta T$ 的可行区间。因此它采用的优先级是：

1. 先让总推力落入双电机的整体能力范围；
2. 在保持该可行总推力不变的条件下，尽可能实现所需差分推力；
3. 根据最终 $T_1$、$T_2$ 重新计算真实差分推力，供后续力矩分配使用。

这保证了饱和后的控制分配仍然保持代数与物理上的自洽。


# 8. Knife-Edge Transition 跟踪变差的原因与调参验证

在增加执行器动态并把飞行器参数改为论文平台参数之后，knife-edge-transition 跟踪变差并不是主要由 `alpha_T=-5 deg` 单独造成的。受控消融表明，主要因素是 `c_mu_T=-0.025 m` 引入的动态俯仰配平负担，以及电机和舵机延迟；两者影响相当且相互耦合。

## 8.1 受控消融结果

使用当前 $500\ \mathrm{Hz}$ 控制与滤波结构，对 knife-edge-transition 运行两圈，得到：

| 配置 | RMS / 最大位置误差 | 相对改善 |
|---|---:|---:|
| 当前完整模型 | $0.816/2.422\ \mathrm m$ | — |
| 仅令 `alpha_T=0` | $0.796/2.347\ \mathrm m$ | RMS 改善约 $2.6\%$ |
| 仅令 `c_mu_T=0` | $0.613/1.344\ \mathrm m$ | RMS 改善约 $25\%$ |
| 仅移除执行器动态 | $0.652/1.698\ \mathrm m$ | RMS 改善约 $20\%$ |
| `alpha_T=c_mu_T=0` 且使用理想执行器 | $0.575/1.232\ \mathrm m$ | RMS 改善约 $30\%$ |

完整八圈的关键结果为：

| 配置 | RMS / 最大位置误差 |
|---|---:|
| 当前完整模型 | $0.838/2.520\ \mathrm m$ |
| `c_mu_T=0` | $0.623/1.344\ \mathrm m$ |
| 理想执行器 | $0.691/1.863\ \mathrm m$ |
| 旧安装参数与理想执行器 | $0.589/1.232\ \mathrm m$ |

因此在当前仿真中的影响排序大致是：

1. `c_mu_T` 造成的动态俯仰配平；
2. 电机和舵机动态延迟；
3. `alpha_T=-5 deg` 本身。

这些影响并非完全线性叠加，因为执行器延迟、俯仰配平、姿态误差和线加速度误差之间存在闭环耦合。

## 8.2 `alpha_T` 不是动态配平参数

`config.py` 中：

```python
self.alpha_T = np.deg2rad(-5.0)
```

表示电机推力轴相对机体系的固定安装角。它会改变：

- 推力在零升力坐标系中的方向；
- 差分推力产生的力矩映射；
- 螺旋桨反扭矩的轴向分解；
- 平坦性变换中的 $\bar\alpha=\alpha_0+\alpha_T$。

但是控制器和仿真气动力模型使用的是同一个 `alpha_T`，所以这里没有产生控制器与被控对象之间的参数不一致。仅将它恢复为零后，位置 RMS 只改善约 $2.6\%$，说明它不是跟踪显著变差的主要原因。

如果实际飞机的安装角确实为 $-5^\circ$，不应该为了得到更好的仿真曲线而将它改成零。否则只是把真实的推力方向耦合从模型中删除。

## 8.3 真正要求配平的是 `c_mu_T`

`config.py` 中：

```python
self.c_mu_T = -0.025
```

它产生与总推力成正比的俯仰矩：

$$
m_{T,y}
=
c_{\mu T}(T_1+T_2)
=
c_{\mu T}T_c
$$

该力矩需要两个舵面同向偏转产生相反的俯仰矩。若两个舵面使用相同配平角：

$$
\delta_1=\delta_2=\delta_{\mathrm{trim}}
$$

根据当前舵面力模型，近似配平条件为：

$$
c_{\mu T}T_c
-
l_{\delta x}
\left[
c_{L_T}^{\delta}\cos\bar\alpha\,T_c
+
2c_{L_V}^{\delta}\lVert v\rVert v_x^\alpha
\right]
\delta_{\mathrm{trim}}
=
0
$$

因此：

$$
\delta_{\mathrm{trim}}
=
\frac{c_{\mu T}T_c}
{
l_{\delta x}
\left[
c_{L_T}^{\delta}\cos\bar\alpha\,T_c
+
2c_{L_V}^{\delta}\lVert v\rVert v_x^\alpha
\right]
}
$$

悬停时 $v=0$，代入当前参数：

$$
\delta_{\mathrm{trim}}
\approx
\frac{-0.025}
{
0.075\times1.25\times\cos(-7^\circ)
}
\approx
-0.269\ \mathrm{rad}
$$

这正好约为代码中的 $-0.27\ \mathrm{rad}$ 初始配平舵角，也与论文根据静态悬停配平得到的参数一致。

## 8.4 为什么需要动态配平

飞行过程中：

- 总推力 $T_c$ 不断变化；
- 空速 $\lVert v\rVert$ 和 $v_x^\alpha$ 不断变化；
- 轨迹在正飞和倒飞之间交替；
- 舵面在螺旋桨滑流和自由来流中的效率不断变化。

因此：

$$
\delta_{\mathrm{trim}}
=
\delta_{\mathrm{trim}}
\left(
T_c,\lVert v\rVert,v_x^\alpha
\right)
$$

配平角不是一个始终固定的 $-0.27\ \mathrm{rad}$。

当前代码并非完全缺少动态配平：

1. 仿真初始化时利用 $c_{\mu T}$、总推力和舵面效率迭代求解初始配平角；
2. 控制分配器每个控制周期都计算：

   $$
   \pmb m_{\delta,c}
   =
   \pmb m_c
   -
   \pmb m_T
   -
   \pmb m_\mu
   $$

3. 其中 $\pmb m_T$ 已包含 $c_{\mu T}T_c$；
4. 随后控制分配器求出两个舵面应提供的补偿力矩。

所以跟踪变差不是因为代码完全没有考虑 $c_{\mu T}$。真正的问题是：动态配平命令已经计算出来，但电机、舵机和测量滤波器不能瞬时实现该命令。

## 8.5 执行器延迟为什么影响明显

当前模型使用：

- 电机时间常数：$0.04\ \mathrm{s}$；
- 舵机时间常数：$0.03\ \mathrm{s}$；
- 舵面最大角速度：$25\ \mathrm{rad/s}$；
- $15\ \mathrm{Hz}$ 二阶 Butterworth 测量低通。

轨迹的曲率过渡长度为：

$$
L_{\mathrm{blend}}=0.6\ \mathrm m
$$

在 $6\ \mathrm{m/s}$ 下，曲率主要变化时间约为：

$$
t_{\mathrm{blend}}
=
\frac{0.6}{6}
=
0.1\ \mathrm{s}
$$

电机和舵机的 $30$～$40\ \mathrm{ms}$ 时间常数已经占这段过程的 $30\%$～$40\%$，再叠加主低通的相位延迟后，控制作用相对参考变化出现明显滞后。

此外，电机和舵面的响应速度不同：

- 总推力变化会改变 $c_{\mu T}T_c$；
- 舵面必须同步改变才能抵消该俯仰矩；
- 电机与舵机具有不同时间常数；
- 舵面快速运动还会产生论文 Eq. (43) 所处理的非最小相位线加速度。

因此即使静态控制分配完全正确，瞬态过程中也可能暂时存在未抵消俯仰矩。

移除执行器动态后，八圈位置 RMS 从 $0.838\ \mathrm m$ 降到 $0.691\ \mathrm m$。两圈数据中：

- 姿态误差 RMS 从约 $19.65^\circ$ 降到 $15.66^\circ$；
- 三轴角加速度跟踪误差明显降低；
- 位置 RMS 从 $0.816\ \mathrm m$ 降到 $0.652\ \mathrm m$。

这说明执行器延迟是第二个主要因素。

## 8.6 为什么正飞和倒飞圈误差交替

当前完整模型的八圈位置 RMS 呈现明显交替：

```text
0.976, 0.616, 1.023, 0.617,
1.023, 0.617, 1.023, 0.617 m
```

这是因为 coordinated 段每一圈在正飞和倒飞之间切换。加入 `c_mu_T` 后，俯仰矩配平与当前姿态、舵面力方向和推力方向耦合，导致其中一类姿态分支的跟踪明显更差。

令 `c_mu_T=0` 后，各圈位置 RMS 基本统一为：

```text
0.604～0.631 m
```

这进一步说明主要参数因素是推力俯仰矩，而不是单独的 `alpha_T=-5 deg`。

`c_mu_T=0` 后的姿态误差改善相对有限，但速度误差和位置误差明显下降。这说明其主要影响不仅是瞬时姿态误差，还包括舵面配平力对平动力的持续耦合。

## 8.7 当前工况不是首先受到硬饱和限制

当前完整模型在八圈 knife-edge-transition 中达到：

- 最大电机转速约 $2011\ \mathrm{rad/s}$，低于 $2500\ \mathrm{rad/s}$ 上限；
- 最大舵面偏角约 $45.5^\circ$，低于 $57.3^\circ$ 上限；
- 最大差分推力约 $4.93\ \mathrm N$。

因此默认增益下跟踪变差并不是首先由硬饱和造成的，主要是动态配平、执行器相位延迟和控制带宽不足。

但提高姿态环增益后，舵面会迅速接近限位，所以硬饱和会成为进一步调参的约束。

## 8.8 姿态环增益扫描

为了判断能否通过调参改善，按照保持近似阻尼比例的方式缩放姿态环：

$$
K_\xi'=s^2K_\xi,\qquad
K_\Omega'=sK_\Omega
$$

两圈扫描结果为：

| 带宽因子 $s$ | RMS / 最大位置误差 | 最大舵偏 | RMS 偏航误差 |
|---:|---:|---:|---:|
| $0.6$ | $4.006/8.059\ \mathrm m$ | $30.7^\circ$ | $18.23^\circ$ |
| $0.8$ | $1.337/3.778\ \mathrm m$ | $36.3^\circ$ | $5.73^\circ$ |
| $1.0$ | $0.816/2.422\ \mathrm m$ | $45.5^\circ$ | $4.53^\circ$ |
| $1.2$ | $0.592/1.672\ \mathrm m$ | $49.8^\circ$ | $3.72^\circ$ |
| $1.4$ | $0.494/1.222\ \mathrm m$ | $51.3^\circ$ | $4.17^\circ$ |
| $1.6$ | $0.467/0.967\ \mathrm m$ | $54.0^\circ$ | $4.82^\circ$ |
| $1.8$ | $0.466/0.837\ \mathrm m$ | $56.1^\circ$ | $5.18^\circ$ |
| $2.0$ | $0.498/0.790\ \mathrm m$ | $56.8^\circ$ | $5.49^\circ$ |

结果表明：

- 默认姿态环带宽偏低；
- $s=1.2$～$1.4$ 可以明显改善位置和偏航跟踪；
- $s=1.6$ 以后位置 RMS 改善很小；
- 更高带宽会增加偏航误差、差分推力和舵面占用；
- $s=1.8$～$2.0$ 时舵面已经非常接近 $57.3^\circ$ 限位。

完整八圈使用 $s=1.4$ 时：

| 指标 | 当前增益 | $1.4$ 倍带宽 |
|---|---:|---:|
| RMS 位置误差 | $0.838\ \mathrm m$ | $0.506\ \mathrm m$ |
| 最大位置误差 | $2.520\ \mathrm m$ | $1.258\ \mathrm m$ |
| 最大舵偏 | $45.5^\circ$ | $53.8^\circ$ |
| 舵面极限 | $57.3^\circ$ | $57.3^\circ$ |

所以调参确实可以显著改善跟踪，但舵面裕量会明显减小。

## 8.9 已验证的候选姿态增益

$s=1.4$ 对应：

```python
k_xi_roll = 78.4
k_omega_roll = 14.0

k_xi_pitch = 54.88
k_omega_pitch = 9.8

k_xi_yaw = 54.88
k_omega_yaw = 9.8
```

这是根据当前轨迹验证得到的候选值，并不代表可以直接作为所有轨迹的最终参数。因为最大舵面偏角已达到约 $53.8^\circ$，只剩约 $3.5^\circ$ 机械裕量。

若希望更保守，可以先从 $s=1.2$～$1.3$ 开始，再逐步增加。每次调整都应同时验证：

- knife-edge-transition；
- coordinated circle；
- knife-edge circle；
- 两个圆周过渡；
- differential-thrust turn；
- 电机和舵面饱和时间比例；
- 姿态与角加速度相位裕量。

## 8.10 推荐的改善顺序

### 1. 保留真实的 `alpha_T`

如果目标是复现论文飞行器，应保留：

$$
\alpha_T=-5^\circ
$$

把它改为零只能删除真实安装角耦合，消融结果也表明收益很小。

### 2. 重新确认 `c_mu_T`

论文中的：

$$
c_{\mu T}=-0.025\ \mathrm m
$$

是根据论文实机悬停时每个舵面约 $-0.27\ \mathrm{rad}$ 的配平角辨识得到的。如果当前仿真中的飞行器几何、舵面效率、推力系数或真实配平角与论文平台不同，就不能直接照搬这个值。

应使用当前平台实际或仿真稳态配平数据重新计算：

$$
c_{\mu T}
=
\delta_{\mathrm{trim}}
l_{\delta x}
c_{L_T}^{\delta}
\cos\bar\alpha
$$

悬停数据确定后，还应检查前飞状态下同一参数能否给出合理舵面配平。

### 3. 先调姿态内环

建议从 $s=1.2$ 开始，逐步测试到 $s=1.4$，而不是直接使用 $1.8$ 或 $2.0$。内环稳定、快速且仍有执行器裕量后，再调整位置和速度外环。

### 4. 加入执行器动态预测

当前控制分配使用当前时刻的静态逆模型。进一步改善可以采用：

- 根据电机和舵机一阶模型预测下一时刻实际转速与舵角；
- 根据预测值计算将要实现的力和力矩；
- 在控制分配中补偿电机与舵机的不同相位；
- 对动态配平舵角加入适量相位超前或参考预瞄。

这比简单地把时间常数改小更符合真实物理。

### 5. 使用显式动态配平前馈

可以把 $\delta_{\mathrm{trim}}(T_c,v)$ 作为舵面公共偏转的前馈基值，再由角 INDI 只计算增量控制量：

$$
\delta_i
=
\delta_{\mathrm{trim}}(T_c,v)
+
\Delta\delta_i
$$

这样姿态反馈不必完全依靠误差出现后再建立配平舵角。

当前控制分配虽然已经隐式包含该关系，但显式分离配平前馈有利于加入执行器预测、速率限制和优先级管理。

### 6. 必要时平滑参考轨迹

若执行器带宽确实不足，可以把曲率融合长度从：

$$
0.6\ \mathrm m
$$

增加到约：

$$
1.0\text{～}1.2\ \mathrm m
$$

这样可降低曲率和 jerk 的变化速度，但这会改变参考轨迹，只应在真实执行器无法满足原轨迹带宽时采用。

## 8.11 最终判断

`alpha_T=-5 deg` 不是主要问题。它是固定推力安装角，且控制器与动力学模型使用同一数值。

主要参数因素是：

$$
c_{\mu T}T_c
$$

产生的推力俯仰矩。它要求舵面进行随总推力和空速变化的动态配平，并进一步产生线加速度耦合。

执行器的 $30$～$40\ \mathrm{ms}$ 一阶延迟以及测量低通相位延迟，是另一个同量级的重要因素。它们使电机推力矩和舵面补偿力矩不能同步建立。

通过姿态环调参可以将八圈位置 RMS 从约 $0.838\ \mathrm m$ 改善到约 $0.506\ \mathrm m$，但同时会明显压缩舵面裕量。若希望进一步接近论文结果，需要同时完成：

1. 基于当前平台重新辨识 $c_{\mu T}$ 和舵面效率；
2. 调整姿态内环带宽；
3. 加入执行器动态预测或显式动态配平前馈；
4. 在所有轨迹上检查执行器饱和与鲁棒性。

为了获得更好曲线而直接令 `alpha_T` 或 `c_mu_T` 为零，并不是论文飞行器的正确复现方式。


# 9. 飞行器三轴控制效率比较

不能直接通过 PD 增益大小判断三轴控制效率。PD 增益表示姿态误差要转换成多大的期望角加速度，而真正的控制效率取决于：

$$
\text{控制效率}
\sim
\pmb J^{-1}
\frac{\partial\pmb m}{\partial\pmb u}
$$

即执行器单位变化产生的力矩，再除以对应机体轴的转动惯量。

按照当前飞行器参数，主要结论是：

- 机体俯仰轴 $b_y$ 的角加速度效率最高；
- 机体滚转轴 $b_x$ 次之；
- 机体偏航轴 $b_z$ 的单位输入效率相对较低，但差分推力的可用范围大，而且不依赖空速；
- 悬停时若考虑完整执行器范围，俯仰最强，滚转和偏航接近；
- 前飞速度增加后，舵面效率随空速显著增大，通常变为“俯仰 $>$ 滚转 $>$ 偏航”。

## 9.1 三个机体轴的主要执行器

| 机体轴 | 主要执行器 | 主要控制方式 |
|---|---|---|
| $b_x$ 滚转 | 两个舵面 | 差动舵面 |
| $b_y$ 俯仰 | 两个舵面 | 同向舵面 |
| $b_z$ 偏航 | 两个电机 | 差分推力 |

这里讨论的是机体系三轴，不是简单按照名称解释 ZXY 欧拉角。

尾座悬停时，ZXY 欧拉角轴与当前机体轴不重合。例如在 $\theta\approx90^\circ$ 附近，欧拉角 $\phi$ 的变化可能主要对应机体 $b_z$ 角速度，因此会主要由电机差速完成。这并不改变机体系中“差动舵面主要产生 $m_x$、同向舵面主要产生 $m_y$、差分推力主要产生 $m_z$”的执行器映射。

## 9.2 当前飞行器惯量

当前惯量为：

$$
J_x=0.0095,\qquad
J_y=0.0030,\qquad
J_z=0.0115
\quad\mathrm{kg\,m^2}
$$

俯仰惯量 $J_y$ 明显最小：

$$
J_y
\approx0.316J_x
\approx0.261J_z
$$

所以即使同向舵面产生的俯仰力矩小于差动舵面产生的滚转力矩，除以较小的 $J_y$ 后，俯仰角加速度仍然可能最大。

## 9.3 悬停总推力工作点

使用当前参数估算悬停总推力：

$$
T_c\approx6.92\ \mathrm N
$$

两个电机近似平均分配：

$$
T_1\approx T_2\approx3.46\ \mathrm N
$$

下面的局部控制效率均在该工作点附近计算。它表示小范围控制输入变化产生的角加速度变化，不等于任意大输入下的全局响应。

## 9.4 滚转轴：差动舵面

定义差动舵面输入：

$$
\delta_1=-\delta_d,\qquad
\delta_2=+\delta_d
$$

悬停附近的滚转力矩效率约为：

$$
\frac{\partial m_x}{\partial\delta_d}
\approx
1.029\ \mathrm{N\,m/rad}
$$

除以滚转惯量：

$$
\frac{\partial\dot p}{\partial\delta_d}
=
\frac{1}{J_x}
\frac{\partial m_x}{\partial\delta_d}
\approx
108.4\
\frac{\mathrm{rad/s^2}}{\mathrm{rad}}
$$

这里的差动舵面输入约定为一个舵面增加 $\delta_d$、另一个减少 $\delta_d$。如果改用 $\delta_2-\delta_1$ 作为输入变量，数值会相差一个 2 倍比例，因此比较控制效率时必须先统一输入定义。

由于 $\alpha_0$ 不完全为零，差动舵面还会产生一个较小的 $m_z$ 分量，其相对大小约为：

$$
\frac{m_z}{m_x}
\approx
\tan\alpha_0
$$

当前 $\alpha_0=-2^\circ$，所以该交叉耦合只有约 $-3.5\%$，滚转仍然是主要作用。

## 9.5 俯仰轴：同向舵面

定义同向舵面输入：

$$
\delta_1=\delta_2=\delta_c
$$

悬停附近的俯仰力矩效率约为：

$$
\frac{\partial m_y}{\partial\delta_c}
\approx
0.644\ \mathrm{N\,m/rad}
$$

它小于滚转力矩效率，但俯仰惯量 $J_y$ 很小，因此：

$$
\frac{\partial\dot q}{\partial\delta_c}
=
\frac{1}{J_y}
\frac{\partial m_y}{\partial\delta_c}
\approx
214.6\
\frac{\mathrm{rad/s^2}}{\mathrm{rad}}
$$

所以同样改变一弧度舵角时，俯仰角加速度约为滚转角加速度的两倍：

$$
\frac{
\partial\dot q/\partial\delta_c
}{
\partial\dot p/\partial\delta_d
}
\approx1.98
$$

这说明“俯仰舵面力臂比较短”并不等于“俯仰控制效率最低”。必须同时考虑惯量。

## 9.6 偏航轴：差分推力

定义差分推力：

$$
\Delta T=T_1-T_2
$$

当前模型中，差分推力对偏航力矩的效率约为：

$$
\frac{\partial m_z}{\partial\Delta T}
\approx
0.149\ \mathrm m
$$

因此：

$$
\frac{\partial\dot r}{\partial\Delta T}
=
\frac{1}{J_z}
\frac{\partial m_z}{\partial\Delta T}
\approx
12.96\
\frac{\mathrm{rad/s^2}}{\mathrm N}
$$

该数值的单位与舵面效率不同，不能直接把 $12.96$ 与 $108.4$ 或 $214.6$ 比较。必须进一步考虑差分推力的实际可用范围。

悬停总推力约为 $6.92\ \mathrm N$，且当前电机最小推力为零，所以最大差分推力约为：

$$
|\Delta T|_{\max}
\approx6.92\ \mathrm N
$$

对应最大偏航角加速度量级：

$$
|\dot r|_{\max}
\approx
12.96\times6.92
\approx
89.7\ \mathrm{rad/s^2}
$$

差分推力还会产生一定的 $m_x$ 交叉耦合，因为推力安装角、零升力角和螺旋桨反扭矩都不为零。当前控制分配器使用完整力矩模型处理该耦合，而不是假设差分推力只产生纯 $m_z$。

## 9.7 悬停附近的执行器能力比较

若暂时按单位最大舵角和悬停最大差分推力作量级比较：

| 机体轴 | 最大角加速度量级 |
|---|---:|
| 俯仰 $b_y$ | 约 $215\ \mathrm{rad/s^2}$ |
| 滚转 $b_x$ | 约 $108\ \mathrm{rad/s^2}$ |
| 偏航 $b_z$ | 约 $90\ \mathrm{rad/s^2}$ |

因此不考虑配平占用时：

$$
\text{俯仰}
>
\text{滚转}
>
\text{偏航}
$$

但是悬停时两个舵面还需要约：

$$
\delta_{\mathrm{trim}}
\approx-0.27\ \mathrm{rad}
$$

的公共配平偏角。若保持两个舵面围绕该公共偏角作对称差动，则差动输入的对称余量约为：

$$
|\delta_d|_{\max}
\approx
1.0-0.27
=
0.73\ \mathrm{rad}
$$

对应滚转角加速度量级下降到：

$$
108.4\times0.73
\approx79.1\ \mathrm{rad/s^2}
$$

因此考虑悬停配平余量后，更实际的结论是：

$$
\text{俯仰最强，滚转与偏航接近}
$$

俯仰本身也存在正负方向不对称的舵角余量：从 $-0.27\ \mathrm{rad}$ 配平点向正方向约有 $1.27\ \mathrm{rad}$ 理论行程，向负方向只有约 $0.73\ \mathrm{rad}$。所以实际俯仰控制能力也具有方向性。

## 9.8 前飞时滚转和俯仰效率增加

舵面力的有效系数包含：

$$
c_{L_T}^{\delta}\cos\bar\alpha\,T_i
+
c_{L_V}^{\delta}\lVert v\rVert v_x^\alpha
$$

第一项来自螺旋桨滑流，第二项来自自由来流。

当速度主要沿零升力轴方向时：

$$
v_x^\alpha\approx\lVert v\rVert\approx V
$$

所以自由来流舵效近似按：

$$
c_{L_V}^{\delta}V^2
$$

增加。

在约 $6\ \mathrm{m/s}$、$v_x^\alpha\approx6\ \mathrm{m/s}$，并保留悬停量级螺旋桨滑流作为粗略工作点时：

$$
\frac{\partial\dot p}{\partial\delta_d}
\approx
272\
\frac{\mathrm{rad/s^2}}{\mathrm{rad}}
$$

$$
\frac{\partial\dot q}{\partial\delta_c}
\approx
539\
\frac{\mathrm{rad/s^2}}{\mathrm{rad}}
$$

这些只是局部量级，实际值还取决于当时总推力、姿态、$v_x^\alpha$ 和舵面偏角。但趋势非常明确：

- 前飞越快，滚转和俯仰舵面通常越有效；
- 偏航主要依赖差分推力，不会像舵面一样按 $V^2$ 显著增强；
- 高速前飞时，俯仰和滚转通常比偏航具有更大的局部角加速度效率。

## 9.9 差分推力偏航为什么仍然很重要

虽然按单位输入或最大角加速度量级，偏航不一定最高，但差分推力具有其他关键优势：

- 静态悬停时仍然有效；
- 不依赖机翼空速；
- 电机横向力臂 `l_Ty = 0.15 m` 较大；
- 可以产生论文中约 $6.1\ \mathrm N$ 的大差分推力；
- 飞翼没有垂尾，因此差分推力承担主动方向稳定任务；
- 在 knife-edge 和大侧滑工况中仍可直接控制 $m_z$。

所以“差分推力偏航权威强”并不等价于“单位输入产生的角加速度一定是三轴最高”。

## 9.10 当前动态模型中偏航不一定最快

当前执行器动态为：

```python
motor_time_constant = 0.04
servo_time_constant = 0.03
```

电机时间常数反而比舵机稍大。因此在当前仿真里，偏航差分推力的物理建立速度并不一定比舵面控制的滚转和俯仰更快。

“偏航响应极快”更准确的含义应该是：

- 无刷电机差分推力具有较强控制权威；
- 全飞行包线内都可使用；
- 不会在静态悬停时因空速为零而失效。

它不表示当前一阶执行器模型中的偏航通道时间常数一定最小。

## 9.11 为什么滚转 PD 增益最大

当前姿态增益为：

```python
k_xi_roll = 78.4
k_omega_roll = 14.0

k_xi_pitch = 54.88
k_omega_pitch = 9.8

k_xi_yaw = 54.88
k_omega_yaw = 9.8
```

这不表示滚转执行器效率最高。

姿态 PD 输出的是期望角加速度：

$$
\dot{\pmb\Omega}_c
=
\pmb K_\xi\pmb\xi_e
+
\pmb K_\Omega
\left(
\pmb\Omega_{\mathrm{ref}}
-
\pmb\Omega_{\mathrm{lpf}}
\right)
$$

后续角 INDI 才将角加速度增量转换为力矩：

$$
\pmb m_c
=
\pmb J
\left(
\dot{\pmb\Omega}_c
-
\dot{\pmb\Omega}_{\mathrm{lpf}}
\right)
+
\pmb m_{\mathrm{lpf}}
$$

控制分配器再把力矩转换成电机和舵面命令。

因此：

$$
K_\xi\text{ 较大}
\not\Rightarrow
\text{执行器效率较高}
$$

滚转增益较大表示当前调参希望滚转轴具有更高的指令闭环带宽，部分用于补偿：

- $J_x$ 大于 $J_y$；
- 滚转局部角加速度效率低于俯仰；
- knife-edge 转换需要快速建立较大姿态变化；
- 差动舵面还受到公共配平舵角的剩余行程约束。

如果把角加速度控制视为理想，单轴近似闭环形式为：

$$
\ddot e
+
K_\Omega\dot e
+
K_\xi e
\approx0
$$

其自然频率和阻尼近似为：

$$
\omega_n\approx\sqrt{K_\xi},\qquad
\zeta\approx\frac{K_\Omega}{2\sqrt{K_\xi}}
$$

所以这些增益首先描述期望闭环动态，而不是气动或执行器的物理效率。

## 9.12 最终结论

按照当前参数的局部机体角加速度效率，在不考虑悬停配平占用时：

$$
\boxed{
\text{俯仰 }b_y
>
\text{滚转 }b_x
>
\text{偏航 }b_z
}
$$

考虑悬停公共舵面配平占用和完整执行器范围后，更实际的描述是：

$$
\boxed{
\text{俯仰最强，滚转与偏航接近}
}
$$

在约 $6\ \mathrm{m/s}$ 前飞时，自由来流使舵面控制效率显著增加，所以通常表现为：

$$
\boxed{
\text{俯仰}
>
\text{滚转}
>
\text{偏航}
}
$$

但这三个结论都针对机体系角速度分量 $[p,q,r]$。在尾座悬停和 knife-edge 姿态下，ZXY 欧拉角 Roll/Pitch/Yaw 不能与 $b_x,b_y,b_z$ 简单一一对应。判断某个欧拉角由哪个执行器主导时，还必须结合当前姿态下的欧拉角速度到机体角速度映射。
