# 概述

**为应对频繁工作环境被损坏导致的数据、代码及工具丢失风险（即便采用移动硬盘备份仍存在意外删除的可能），特将部分技术资料、算法实现及学习心得集中归档于此文档。**

文档主体规划为六个章节：
1.  **前四章：核心算法文档** - 系统阐述组合导航相关理论、模型与算法实现。
2.  **第五章：V模型左侧 - 开发实现** - 聚焦工程化实现，包括实时导航软件的设计与开发。
3.  **第六章：V模型右侧 - 测试验证与分析** - 涵盖测试程序开发、数据处理脚本编写及事后数据分析方法。

**说明：** 除直接服务于客户端的实时导航软件外，一套完备的测试验证工具链（用于数据回放、结果分析、性能评估）及事后数据处理脚本对于保障系统质量和问题定位至关重要。此类工具链在逻辑上归属于第六章（V模型右侧）。

## 1.  惯性导航基础与建模
**本节内容主要参考并基于[PSINS](https://psins.org.cn/)开源惯性导航算法库的框架与思想构建，并针对当前版本程序给出关键建模说明。**

### 1.1  惯性导航基本原理
惯性导航（Inertial Navigation, IN）的核心原理在于利用惯性测量单元（IMU）提供的原始数据：
1.  **角速度测量**：IMU中的陀螺仪测量载体坐标系（b系）相对于惯性坐标系（i系）的角速度 $\boldsymbol{\omega}_{ib}^{b}$。
2.  **比力测量**：IMU中的加速度计测量载体在b系中感受到的比力（Specific Force） $\boldsymbol{f}_{ib}^{b}$。 *(注：比力包含了载体运动加速度和重力加速度的复合效应)*

基于 $\boldsymbol{\omega}_{ib}^{b}$ 和 $\boldsymbol{f}_{ib}^{b}$ 这两组核心测量值，通过惯性导航算法（主要包括姿态更新、速度更新和位置更新）进行递推解算：
1.  **姿态解算**：利用 $\boldsymbol{\omega}_{ib}^{b}$ 更新载体坐标系 (b系) 相对于惯性坐标系 (i系) 的姿态。
2.  **速度与位置解算**：在惯性坐标系 (i系) 下，利用解算得到的姿态将 $\boldsymbol{f}_{ib}^{b}$ 转换到 i系 ($\boldsymbol{f}_{ib}^{i}$)，结合地球重力场模型，积分求解得到载体在 i系 下的速度 $\boldsymbol{v}_{ib}^{i}$ 和位置 $\boldsymbol{p}_{ib}^{i}$。

**导航参数转换**：为了获得载体相对于地球（通常指当地地理导航坐标系，即 n 系）的运动信息（姿态、速度、位置），需要引入地球模型参数（如地球自转角速度 $\boldsymbol{\omega}_{ie}$、地球半径）和当前位置信息：
1.  利用地球自转角速度 $\boldsymbol{\omega}_{ie}$ 和载体在地球上的运动速度 $\boldsymbol{v}_{eb}^{n}$（待求量），计算 i系 到 n系 的旋转角速度 $\boldsymbol{\omega}_{in}^{n}$。
2.  结合载体在 i系 下的运动参数 ($\boldsymbol{v}_{ib}^{i}$, $\boldsymbol{p}_{ib}^{i}$) 和 $\boldsymbol{\omega}_{in}^{n}$，通过坐标系转换关系（例如方向余弦矩阵或四元数），最终解算出载体在 n系 下的姿态（b系 相对于 n系）、速度 $\boldsymbol{v}_{eb}^{n}$ 和位置（经纬度、高度）等导航参数。

### 1.2 IMU器件误差建模
#### 1.2.1 确定性误差
##### 1.2.1.1 空间安装误差与时间同步误差
**a. 陀螺杆臂效应**  
当陀螺仪与IMU参考点存在空间偏移时，载体旋转运动导致陀螺敏感到附加角速度：
$$\Delta\boldsymbol{\omega}_{arm} = \frac{1}{2} \left( 
\underbrace{\boldsymbol{\omega}_{ib}^b \times (\boldsymbol{r}_g \times \boldsymbol{\omega}_{ib}^b)}_{\text{离心项（向心加速度引起）}} 
+ \underbrace{\boldsymbol{r}_g \times \dot{\boldsymbol{\omega}}_{ib}^b}_{\text{欧拉项（切向加速度引起）}} 
\right)$$
其中 $\boldsymbol{r}_g \in \mathbb{R}^3$ 为陀螺仪到IMU参考点的杆臂矢量（单位：m）。  
*工程补偿*：精密机械安装 + 标定后软件补偿（需已知 $\boldsymbol{r}_g$ 和角加速度）

**b. 加速度计杆臂效应**  
加速度计空间偏移导致比力测量误差：
$$
\Delta\boldsymbol{f}_{arm} =  
\underbrace{\boldsymbol{r}_a \times \dot{\boldsymbol{\omega}}_{ib}^b}_{\text{切向加速度项}} 
+ \underbrace{\boldsymbol{\omega}_{ib}^b \times (\boldsymbol{r}_a \times \boldsymbol{\omega}_{ib}^b)}_{\text{向心加速度项}} 
+ \underbrace{2\boldsymbol{\omega}_{ib}^b \times \boldsymbol{v}_{rel}^b}_{\text{科氏力项}}
$$
其中 $\boldsymbol{r}_a \in \mathbb{R}^3$ 为加速度计杆臂矢量，$\boldsymbol{v}_{rel}^b$ 为杆臂引起的相对速度。  
*注*：当 $\boldsymbol{r}_a = \boldsymbol{r}_g$ 时，杆臂效应模型可统一表述。

**c. 时间同步误差**  
传感器采样时刻偏差导致相位延迟：
$$
\begin{aligned}
&\text{陀螺仪：} & \boldsymbol{\tilde{\omega}}(t_k) &= \boldsymbol{\omega}(t_k - \tau_g) + \boldsymbol{n}_g(t_k) \\
&\text{加速度计：} & \boldsymbol{\tilde{f}}(t_k) &= \boldsymbol{f}(t_k - \tau_a) + \boldsymbol{n}_a(t_k)
\end{aligned}
$$
小延迟近似补偿公式（$\tau_g, \tau_a < 5\text{ms}$）：
$$
\Delta\boldsymbol{\omega} \approx \dot{\boldsymbol{\omega}} \cdot \tau_g, \quad 
\Delta\boldsymbol{f} \approx \dot{\boldsymbol{f}} \cdot \tau_a
$$
*工程实现*：硬件同步信号 + 软件插值补偿

##### 1.2.1.2 传感器固有误差
**a. 零偏建模 (Bias)**  
温度相关的非线性模型：
$$
\boldsymbol{b}_g(t) = \boldsymbol{b}_{g0} + \boldsymbol{b}_{gT}(T) + \boldsymbol{w}_g(t)
$$
- $\boldsymbol{b}_{g0}$：常温零偏（标定可补偿）
- $\boldsymbol{b}_{gT}$：温度相关项（需建立温度-零偏查找表）
- $\boldsymbol{w}_g$：随机波动（纳入随机误差处理）

**b. 尺度因子误差 (Scale Factor)**  
非线性标度模型：
$$\begin{aligned}
&\text{陀螺仪：} & \boldsymbol{\omega}_{meas} &= (\boldsymbol{I} + \boldsymbol{K}_g + \boldsymbol{K}_{g2}\|\boldsymbol{\omega}\|) \boldsymbol{\omega}_{true} \\
&\text{加速度计：} & \boldsymbol{f}_{meas} &= (\boldsymbol{I} + \boldsymbol{K}_a) \boldsymbol{f}_{true}
\end{aligned}$$
其中 $\boldsymbol{K}g = \mathrm{diag}([k_{gx}, k_{gy}, k_{gz}])$，高阶项 $\boldsymbol{K}_{g2}$ 针对大动态场景（如战术级IMU）。

**c. 非正交误差与安装偏差**  
轴系失准变换矩阵：
$$
\boldsymbol{T} = 
\begin{bmatrix}
1 & -\beta_{yz} & \beta_{zy} \\
\beta_{xz} & 1 & -\beta_{zx} \\
-\beta_{xy} & \beta_{yx} & 1
\end{bmatrix}
+ \mathrm{diag}([\delta_{xx}, \delta_{yy}, \delta_{zz}])
$$
- 非对角元素 $\beta$：轴间非正交角（单位：rad）
- 对角元素 $\delta$：安装位置偏差（单位：m）

**d. 综合误差模型**  
$$
\begin{bmatrix}
\boldsymbol{\tilde{\omega}}_{ib}^b \\ 
\boldsymbol{\tilde{f}}_{ib}^b
\end{bmatrix} = 
\boldsymbol{T} 
\begin{bmatrix}
(\boldsymbol{I} + \boldsymbol{K}_g) \boldsymbol{\omega}_{true}(t - \tau_g) \\ 
(\boldsymbol{I} + \boldsymbol{K}_a) \boldsymbol{f}_{true}(t - \tau_a)
\end{bmatrix}
+ 
\begin{bmatrix}
\Delta\boldsymbol{\omega}_{arm} \\ 
\Delta\boldsymbol{f}_{arm}
\end{bmatrix}
+ 
\begin{bmatrix}
\boldsymbol{b}_g \\ 
\boldsymbol{b}_a
\end{bmatrix}
$$
*标定要求*：需通过6位置法、温度试验等多工况标定

#### 1.2.2 随机误差模型
| 误差类型              | 随机过程模型                          | Allan方差参数        | 物理意义                  |
|-----------------------|---------------------------------------|----------------------|--------------------------|
| 角度随机游走 (ARW)    | $\sigma_g = \frac{N_g}{\sqrt{\Delta t}}$ | $N_g$ [rad/s/√Hz]   | 陀螺白噪声积分           |
| 零偏不稳定性 (BI)     | $\dot{\boldsymbol{b}}_g = -\frac{1}{\tau}\boldsymbol{b}_g + \boldsymbol{w}$ | $B$ [rad/s]         | 零偏慢变分量             |
| 速度随机游走 (VRW)    | $\sigma_a = \frac{N_a}{\sqrt{\Delta t}}$ | $N_a$ [m/s/√h]      | 加速度计白噪声积分       |
| 加速度零偏不稳定性    | $\dot{\boldsymbol{b}}_a = -\frac{1}{\tau}\boldsymbol{b}_a + \boldsymbol{v}$ | $B_a$ [μg]          | 加速度计零偏慢变分量     |

**一阶马尔可夫模型推导**  
连续时间微分方程：
$$\dot{b}(t) = -\frac{1}{\tau} b(t) + w(t)$$
- $b(t)$：随机误差状态
- $\tau$：相关时间常数
- $w(t)$：驱动白噪声 $\sim \mathcal{N}(0, \sigma_w^2)$

离散化过程（采样间隔 $\Delta t$）：
$$
\boxed{
\begin{aligned}
b_k &= F b_{k-1} + w_k \\
F &= e^{-\Delta t / \tau} \\
\sigma_d^2 &= \frac{\tau \sigma_w^2}{2} (1 - e^{-2\Delta t/\tau})
\end{aligned}
}
$$
*工程应用*：用于Kalman滤波器状态噪声协方差设计

### 1.3 圆锥误差与划桨误差补偿
#### 圆锥误差（Coning Error）
**产生机理**：当存在同频异相角振动时，陀螺输出常值漂移：
$$\boldsymbol{\omega}(t) = \begin{bmatrix} \Omega \sin\phi \cos\omega t \\ \Omega \sin\phi \sin\omega t \\ \Omega \cos\phi \end{bmatrix}$$

**补偿算法**：多子样叉积补偿
$$\delta \hat{\boldsymbol{\phi}}(T) = \sum_{i=1}^{N-1} k_{N-i} (\Delta \boldsymbol{\theta}_{m i} \times \Delta \boldsymbol{\theta}_{m N})$$
补偿系数表：
| 子样数 | $k_1$       | $k_2$       | $k_3$       | $k_4$       | $k_5$       | 剩余误差 $\rho_N$ |
|--------|-------------|-------------|-------------|-------------|-------------|-------------------|
| 2      | 2/3         | -           | -           | -           | -           | $1/960$          |
| 3      | 27/20       | 9/20        | -           | -           | -           | $1/204120$       |
| 4      | 214/105     | 92/105      | 54/105      | -           | -           | $1/82575360$     |

*工程建议*：
1. 战术级应用：二子样算法（计算量/精度平衡）
2. 航空级应用：三至四子样算法
3. 避免超过六子样（噪声放大效应）

#### 划桨误差（Sculling Error）
**产生机理**：角振动与线振动耦合导致速度误差：
$$\boldsymbol{\omega}(t) = \begin{bmatrix} 0 \\ \phi\Omega\cos\Omega t \\ 0 \end{bmatrix}, \quad \boldsymbol{f}(t) = \begin{bmatrix} a\Omega\cos\Omega t \\ 0 \\ 0 \end{bmatrix}$$

**补偿方法**：速度旋转算法 + 划桨补偿项
$$\Delta\boldsymbol{v}_{comp} = \frac{1}{2} \Delta\boldsymbol{\theta} \times \Delta\boldsymbol{v} + \frac{1}{12} (\Delta\boldsymbol{\theta} \times \Delta\boldsymbol{v} + \Delta\boldsymbol{v} \times \Delta\boldsymbol{\theta})$$

### 1.4 姿态更新算法
四元数微分方程：
$$\dot{\boldsymbol{q}}_b^i = \frac{1}{2} \boldsymbol{q}_b^i \otimes \boldsymbol{\omega}_{ib}^b$$

**离散化实现（基于等效旋转矢量）**：
$$
\boldsymbol{q}_{b(m)}^i = \boldsymbol{q}_{b(m-1)}^i \otimes \boldsymbol{q}_{b(m)}^{b(m-1)}
$$
其中：
$$
\boldsymbol{q}_{b(m)}^{b(m-1)} = \begin{bmatrix}
\cos(\|\Delta\boldsymbol{\phi}\|/2) \\
\frac{\Delta\boldsymbol{\phi}}{\|\Delta\boldsymbol{\phi}\|} \sin(\|\Delta\boldsymbol{\phi}\|/2)
\end{bmatrix}, \quad \Delta\boldsymbol{\phi} = \Delta\boldsymbol{\theta} + \delta\hat{\boldsymbol{\phi}}_{coning}
$$

*优势*：计算量比方向余弦法减少40%，无奇点问题

### 1.5 速度与位置更新
#### 速度更新
导航系下速度微分方程：
$$\dot{\boldsymbol{v}}_{en}^n = \boldsymbol{C}_b^n \boldsymbol{f}_{sf}^b - (2\boldsymbol{\omega}_{ie}^n + \boldsymbol{\omega}_{en}^n) \times \boldsymbol{v}_{en}^n + \boldsymbol{g}^n$$

**有害加速度项**：
- $2\boldsymbol{\omega}_{ie}^n \times \boldsymbol{v}_{en}^n$：科氏加速度
- $\boldsymbol{\omega}_{en}^n \times \boldsymbol{v}_{en}^n$：向心加速度
- $\boldsymbol{g}^n$：重力加速度

#### 位置更新
位置微分方程：
$$\dot{\boldsymbol{p}} = \boldsymbol{M}_{pv} \boldsymbol{v}^n, \quad \boldsymbol{p} = [L, \lambda, h]^T$$
离散化（梯形积分法）：
$$
\boldsymbol{p}_m = \boldsymbol{p}_{m-1} + \boldsymbol{M}_{pv}(t_{m-1/2}) \cdot (\boldsymbol{v}_{m-1}^n + \boldsymbol{v}_m^n) \frac{T}{2}
$$
外推公式（提高精度）：
$$x_{m-1/2} = \frac{3x_{m-1} - x_{m-2}}{2} \quad (x = L, h, R_{Mh}, R_{Nh})$$

*注*：位置更新周期通常为速度更新的2-5倍

## 1.6 误差分析，即对导航参数进行时间微分





## 4.  融合技术
### 卡尔曼滤波技术




###  MEMS 车载 GNSS/INS 松组合导航方案

MEMS 惯性传感器的误差特性复杂，在车载应用环境下，可利用的观测量（如 GNSS 位置/速度）相对有限，导致部分误差参数（如高阶随机过程参数）的可观测性较低，难以在线精确估计。因此，在状态量建模时，应优先关注对导航解算影响显著且相对可观测的误差项。

针对典型车载应用场景，推荐采用以下核心状态向量进行估计：
$$ X_k = [\delta \mathbf{\phi}^T, \delta \mathbf{v}^T, \delta \mathbf{p}^T, \mathbf{e}_b^T, \mathbf{d}_b^T]^T $$
**(状态量定义见下文)**。其中，部分难以在线稳定估计的参数（如陀螺仪角随机游走、加速度计相关时间较长的马尔可夫过程参数等）可考虑通过离线标定或后处理方式确定。

**状态量说明：**
*   $\delta \mathbf{\phi}$: 姿态误差角向量 (3x1)
*   $\delta \mathbf{v}$: 速度误差向量 (3x1)
*   $\delta \mathbf{p}$: 位置误差向量 (3x1)
*   $\mathbf{e}_b$: 陀螺仪零偏向量 (3x1)
*   $\mathbf{f}_b$: 加速度计零偏向量 (3x1)


**该模型的核心思想是：** 在保证主要误差项得到有效估计和补偿的前提下，通过精简状态维度和聚焦可观测参数，提升滤波器的稳定性和实时性，更适用于资源受限的MEMS车载平台。





