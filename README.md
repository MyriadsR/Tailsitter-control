<!--
 * @Author: xzr && zhenrxu@qq.com
 * @Date: 2026-03-19 19:50:07
 * @LastEditors: xzr && zhenrxu@qq.com
 * @LastEditTime: 2026-04-08 09:59:00
 * @FilePath: /Tailsitter_ws/README.md
 * @Description: 尾座无人机控制
 * 
 * Copyright (c) 2026 by zhenrxu@qq.com, All Rights Reserved. 
-->
1. 代码结构组成：
```
Tailsitter_ws/
│
├── main_sim.py           # 仿真主程序：设置初始状态，运行仿真循环
├── config.py             # 参数配置：质量、转动惯量、气动导数、电机拟合系数等
│
├── environment/          # 物理世界（受力与运动学）
│   ├── dynamics.py       # 6自由度刚体动力学 (6-DOF Kinematics & Dynamics)
│   └── aerodynamics.py   # 气动力模型：计算阻力、升力、襟翼带来的弯度变化效应等
│
├── controller/           # 飞控算法
│   ├── position_ctrl.py  # 外环控制器：计算期望受力向量
│   ├── attitude_ctrl.py  # 内环姿态控制器：包含解算俯仰角和推力的公式 (19)-(23)
│   └── motor_ctrl.py     # 底层电机控制：包含前馈+积分的转速控制律公式
│
└── utils/                # 工具函数
    ├── math_utils.py     # 数学工具：限制幅值(saturation)、带有保护的 atan2 等
    └── plotter.py        # 绘图脚本：可视化结果
```

2. 需要用到的 python = 3.12.12 包：
   - numpy
   - scipy
   - matplotlib