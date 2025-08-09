# 概述

**为应对频繁工作环境被损坏导致的数据、代码及工具丢失风险（即便采用移动硬盘备份仍存在意外删除的可能），特将部分技术资料、算法实现及学习心得集中归档于此文档。**  \

文档主体规划为六个章节：
1.  **第一章节** - 简单阐述惯性导航的实现。
2.  **第二章节** - 简单阐述卫星导航中SPP的实现。
3.  **第三章节** - 简单阐述里程计的递推实现。
4.  **第四章节** - 简单阐述视觉的实现。
5.  **第五章：kalman滤波的融合** -穷哥们资源受限的嵌入式平台
6.  **第六章：基于优化的融合**   -富少爷使用的域控制器
7.  **第七章：V模型左侧 - 开发实现** - 聚焦工程化实现，包括实时导航软件的设计与开发。
8.  **第八章：V模型右侧 - 测试验证与分析** - 涵盖测试程序开发、数据处理脚本编写及事后数据分析方法。

**说明：** 除直接服务于客户端的实时导航软件外，一套完备的测试验证工具链（用于数据回放、结果分析、性能评估）及事后数据处理脚本对于保障系统质量和问题定位至关重要。此类工具链在逻辑上归属于第六章（V模型右侧）。

## 0. 一些当时很难理解的术语的解释
**可观测性** 可观测性（Observability） 是一个核心概念，用于衡量系统状态（如位置、速度、姿态、传感器误差等）能否通过已有的传感器测量值在有限时间内被唯一确定的程度。它直接反映了导航系统的信息获取能力和对状态误差的估计潜力。
组合导航系统通常使用状态估计器（如卡尔曼滤波）融合来自不同传感器（如IMU、GNSS、里程计、视觉等）的数据。 \
系统存在许多需要估计的状态量（位置、速度、姿态、陀螺仪零偏、加速度计零偏等），但并非所有状态都能被传感器直接测量到。 \
可观测性分析就是要回答： 在给定的传感器配置和运动状态下，我们能否利用所有可用的测量信息（包括直接测量和间接推导）来唯一地、可靠地推断出系统所有（或部分）未知状态的值？ \
数学定义（简化）： \
一个动态系统（如导航系统）的状态空间模型通常表示为： \
ẋ(t) = f(x(t), u(t), w(t)) (状态方程 - 描述状态随时间如何演化) \
z(t) = h(x(t), v(t)) (观测方程 - 描述测量如何与状态关联) \
其中 x(t) 是状态向量，u(t) 是控制输入（可选），w(t) 是过程噪声，z(t) 是观测向量，v(t) 是观测噪声。 \
系统在时间区间 [t₀, t₁] 内是可观测的，如果对于该区间内的任意两个不同的初始状态 x₁(t₀) 和 x₂(t₀)，它们产生的观测序列 {z(t) for t in [t₀, t₁]} 也是不同的。换句话说，不同的初始状态会导致不同的测量输出，因此我们可以通过观测序列来区分不同的初始状态（从而估计出它们）。 \
在组合导航中的具体含义： \
状态可分离： 可观测性高意味着不同的状态组合（例如，姿态角误差和加速度计零偏）对观测结果（例如，GNSS位置或速度）的影响模式是显著不同且可区分的。卡尔曼滤波器能够“看清”这些状态之间的区别。 \
误差可估计： 可观测性高的状态（或状态组合），其误差（如IMU的零偏）能够被传感器测量有效地约束和修正。滤波器的估计会快速收敛到真实值附近。 \
信息冗余与互补： 可观测性源于不同传感器提供互补信息以及系统动态运动带来的信息变化。例如： \
静止时，仅凭IMU无法区分重力加速度和载体加速度，导致姿态和加速度计零偏不可观测（或弱可观测）。 \
一旦载体开始加速运动，重力分量和运动加速度分量在IMU测量中的表现就不同了，结合GNSS提供的速度/位置信息，姿态和加速度计零偏就变得（部分）可观测了。 \
转弯运动对于分离方位角误差和陀螺仪零偏特别有效。\
高度变化对于分离垂直通道误差和气压计/高度计零偏很重要。 

**优化问题**
SLAM的核心优化问题（如Bundle Adjustment, 基于图优化）通常是一个非线性最小二乘问题：目标是找到最优的机器人位姿（可能还有地图点坐标），使得预测的观测值（如图像特征点位置、激光雷达点云到面的距离）与实际观测值之间的误差平方和最小。 \
雅可比矩阵 J 本质上就是误差函数 e 关于优化变量 ξ（这里主要指位姿，通常用李代数 se(3) 或 so(3) 表示）的一阶导数（梯度）矩阵： J = ∂e/∂ξ \
非线性优化通过迭代线性化来逼近最优解。在当前位姿估计点 ξ_k 附近，复杂的非线性误差函数 e(ξ) 被近似为一个线性函数：e(ξ_k + Δξ) ≈ e(ξ_k) + J(ξ_k) * Δξ \
雅可比矩阵 J 精确地描述了在当前位置 ξ_k 附近，位姿的微小变化 Δξ 会对误差 e 产生怎样的线性影响。优化器利用这个线性模型来决定如何走这一步 Δξ \
它提供了关键的梯度信息 (J^T * e)，告诉优化器朝哪个方向调整位姿能降低误差。 \
它用于构建近似的Hessian矩阵 (J^T * J)，这是计算有效优化步长 (Δξ) 的基础。 \
它量化了位姿变化对误差的局部影响，并在一定程度上揭示了系统的可观测性。 




## 1.  惯性导航基础与建模
**本节内容主要参考并基于[PSINS](https://psins.org.cn/)开源惯性导航算法库的框架与思想构建，并针对当前版本程序给出关键建模说明。**
### 1.1  惯性导航基本原理
惯性导航（Inertial Navigation, IN）的核心原理在于利用惯性测量单元（IMU）提供的原始数据：
1.  **角速度测量**：IMU中的陀螺仪测量载体坐标系（b系）相对于惯性坐标系（i系）的角速度$\boldsymbol{\omega}_{ib}^{b}$。
2.  **比力测量**：IMU中的加速度计测量载体在b系中感受到的比力（Specific Force）$\boldsymbol{f}_{ib}^{b}$。 *(注：比力包含了载体运动加速度和重力加速度的复合效应)*

基于$\boldsymbol{\omega}_{ib}^{b}$和$\boldsymbol{f}_{ib}^{b}$这两组核心测量值，通过惯性导航算法（主要包括姿态更新、速度更新和位置更新）进行递推解算：
1.  **姿态解算**：利用$\boldsymbol{\omega}_{ib}^{b}$更新载体坐标系 (b系) 相对于惯性坐标系 (i系) 的姿态。
2.  **速度与位置解算**：在惯性坐标系 (i系) 下，利用解算得到的姿态将$\boldsymbol{f}_{ib}^{b}$转换到 i系 ($\boldsymbol{f}_{ib}^{i}$)，结合地球重力场模型，积分求解得到载体在 i系 下的速度$\boldsymbol{v}_{ib}^{i}$和位置$\boldsymbol{p}_{ib}^{i}$。

**导航参数转换**：为了获得载体相对于地球（通常指当地地理导航坐标系，即 n 系）的运动信息（姿态、速度、位置），需要引入地球模型参数（如地球自转角速度$\boldsymbol{\omega}_{ie}$、地球半径）和当前位置信息：
1.  利用地球自转角速度$\boldsymbol{\omega}_{ie}$和载体在地球上的运动速度$\boldsymbol{v}_{eb}^{n}$（待求量），计算 i系 到 n系 的旋转角速度$\boldsymbol{\omega}_{in}^{n}$。
2.  结合载体在 i系 下的运动参数 ($\boldsymbol{v}_{ib}^{i}$,$\boldsymbol{p}_{ib}^{i}$) 和$\boldsymbol{\omega}_{in}^{n}$，通过坐标系转换关系（例如方向余弦矩阵或四元数），最终解算出载体在 n系 下的姿态（b系 相对于 n系）、速度$\boldsymbol{v}_{eb}^{n}$和位置（经纬度、高度）等导航参数。

### 1.2 IMU器件误差建模
#### 1.2.1 确定性误差
##### 1.2.1.1 空间安装误差与时间同步误差
**a. 陀螺杆臂效应**  
当陀螺仪与IMU参考点存在空间偏移时，载体旋转运动导致陀螺敏感到附加角速度：
$$
\Delta\boldsymbol{\omega}_{lever} = 0.5*\boldsymbol{\omega}_{ib}^b \times (\boldsymbol{r}_g \times \boldsymbol{\omega}_{ib}^b) + 0.5*\boldsymbol{r}_g \times \dot{\boldsymbol{\omega}}_{ib}^b
$$
- **离心项**：$\boldsymbol{\omega}_{ib}^b \times (\boldsymbol{r}_g \times \boldsymbol{\omega}_{ib}^b)$（向心加速度引起）
- **欧拉项**：$\boldsymbol{r}_g \times \dot{\boldsymbol{\omega}}_{ib}^b$（切向加速度引起）

其中$\boldsymbol{r}_g \in \mathbb{R}^3$为陀螺仪到IMU参考点的杆臂矢量（单位：m）。  
*工程补偿*：精密机械安装 + 标定后软件补偿（需已知$\boldsymbol{r}_g$和角加速度）

**b. 加速度计杆臂效应**  
加速度计空间偏移导致比力测量误差：
$$
\Delta\boldsymbol{f}_{lever} = \boldsymbol{r}_a \times \dot{\boldsymbol{\omega}}_{ib}^b + \boldsymbol{\omega}_{ib}^b \times (\boldsymbol{r}_a \times \boldsymbol{\omega}_{ib}^b) +  2\boldsymbol{\omega}_{ib}^b \times \boldsymbol{v}_{\text{rel}}^b
$$
- **切向加速度项**：$\boldsymbol{r}_a \times \dot{\boldsymbol{\omega}}_{ib}^b$
- **向心加速度项**：$\boldsymbol{\omega}_{ib}^b \times (\boldsymbol{r}_a \times \boldsymbol{\omega}_{ib}^b)$
- **科氏力项**：$2\boldsymbol{\omega}_{ib}^b \times \boldsymbol{v}_{\text{rel}}^b$

其中：
- $\boldsymbol{r}_a \in \mathbb{R}^3$：加速度计杆臂矢量
- $\boldsymbol{v}_{\text{rel}}^b$：杆臂引起的相对速度

*注*：当$\boldsymbol{r}_a = \boldsymbol{r}_g$时，杆臂效应模型可统一表述。
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
-$\boldsymbol{b}_{g0}$：常温零偏（标定可补偿）
-$\boldsymbol{b}_{gT}$：温度相关项（需建立温度-零偏查找表）
-$\boldsymbol{w}_g$：随机波动（纳入随机误差处理）

**b. 尺度因子误差 (Scale Factor)**  
非线性标度模型：
$$\begin{aligned}
&\text{陀螺仪：} & \boldsymbol{\omega}_{meas} &= (\boldsymbol{I} + \boldsymbol{K}_g + \boldsymbol{K}_{g2}\|\boldsymbol{\omega}\|) \boldsymbol{\omega}_{true} \\
&\text{加速度计：} & \boldsymbol{f}_{meas} &= (\boldsymbol{I} + \boldsymbol{K}_a) \boldsymbol{f}_{true}
\end{aligned}$$
其中$\boldsymbol{K}g = \mathrm{diag}([k_{gx}, k_{gy}, k_{gz}])$，高阶项$\boldsymbol{K}_{g2}$针对大动态场景（如战术级IMU）。

**c. 非正交误差与安装偏差**  
轴系失准变换矩阵：
$$
\boldsymbol{T}=\begin{bmatrix}1&-\beta_{yz}&\beta_{zy}\\\beta_{xz}&1&-\beta_{zx}\\-\beta_{xy}&\beta_{yx}&1\end{bmatrix}+\mathrm{diag}([\delta_{xx},\delta_{yy},\delta_{zz}])
$$
- 非对角元素$\beta$：轴间非正交角（单位：rad）
- 对角元素$\delta$：安装位置偏差（单位：m）

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
| 角度随机游走 (ARW)    |$\sigma_g = \frac{N_g}{\sqrt{\Delta t}}$|$N_g$[rad/s/√Hz]   | 陀螺白噪声积分           |
| 零偏不稳定性 (BI)     |$\dot{\boldsymbol{b}}_g = -\frac{1}{\tau}\boldsymbol{b}_g + \boldsymbol{w}$|$B$[rad/s]         | 零偏慢变分量             |
| 速度随机游走 (VRW)    |$\sigma_a = \frac{N_a}{\sqrt{\Delta t}}$|$N_a$[m/s/√h]      | 加速度计白噪声积分       |
| 加速度零偏不稳定性    |$\dot{\boldsymbol{b}}_a = -\frac{1}{\tau}\boldsymbol{b}_a + \boldsymbol{v}$|$B_a$[μg]          | 加速度计零偏慢变分量     |

**一阶马尔可夫模型推导**  
连续时间微分方程：
$$\dot{b}(t) = -\frac{1}{\tau} b(t) + w(t)$$
-$b(t)$：随机误差状态
-$\tau$：相关时间常数
-$w(t)$：驱动白噪声$\sim \mathcal{N}(0, \sigma_w^2)$

离散化过程（采样间隔$\Delta t$）：
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
| 子样数 |$k_1$      |$k_2$      |$k_3$      |$k_4$      |$k_5$      | 剩余误差$\rho_N$|
|--------|-------------|-------------|-------------|-------------|-------------|-------------------|
| 2      | 2/3         | -           | -           | -           | -           |$1/960$         |
| 3      | 27/20       | 9/20        | -           | -           | -           |$1/204120$      |
| 4      | 214/105     | 92/105      | 54/105      | -           | -           |$1/82575360$    |

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
-$2\boldsymbol{\omega}_{ie}^n \times \boldsymbol{v}_{en}^n$：科氏加速度
-$\boldsymbol{\omega}_{en}^n \times \boldsymbol{v}_{en}^n$：向心加速度
-$\boldsymbol{g}^n$：重力加速度

#### 位置更新
位置微分方程：
$$\dot{\boldsymbol{p}} = \boldsymbol{M}_{pv} \boldsymbol{v}^n, \quad \boldsymbol{p} = [L, \lambda, h]^T$$
离散化（梯形积分法）：
$$
\boldsymbol{p}_m = \boldsymbol{p}_{m-1} + \boldsymbol{M}_{pv}(t_{m-1/2}) \cdot (\boldsymbol{v}_{m-1}^n + \boldsymbol{v}_m^n) \frac{T}{2}
$$
外推公式（提高精度）：
$$x_{m-1/2} = \frac{3x_{m-1} - x_{m-2}}{2} \quad (x = L, h, R_{Mh}, R_{Nh})$$


## 1.6 误差分析，对导航参数进行微分
在控制理论中，若需要对一个系统进行调整，一般调整其微分量，达到间接控制的目的，遂给出微分方程，通过控制误差量以及反馈来控制导航参数。

首先，将地球自转角速度$\omega_{i\varepsilon}^{n}$以及导航系转动角速度$\omega_{en}^{n}$的表达式分别重写如下：
$$\omega_{i\varepsilon}^{n}=\begin{bmatrix}0\\\omega_{i\varepsilon}\cos L\\\omega_{i\varepsilon}\sin L\end{bmatrix}$$
$$\omega_{en}^{n}=\begin{bmatrix}-\nu_{\mathrm{N}}/(R_{M}+h)\\v_{\mathrm{E}}/(R_{N}+h)\\v_{\mathrm{E}}\tan L/(R_{N}+h)\end{bmatrix}$$
对$\omega_{i\varepsilon}^{n}$进行微分，得
$$\delta\omega_{ie}^n=\begin{bmatrix}0\\-\omega_{ie}\sin L\cdot\delta L\\\omega_{ie}\cos L\cdot\delta L\end{bmatrix}=\boldsymbol{M}_1\delta\boldsymbol{p}$$
对$\omega_{en}^{n}$进行微分，得
$$\begin{gathered}\delta\omega_{en}^{n}=\begin{bmatrix}-\delta v_{\mathrm{N}}/\left(R_{M}+h\right)+v_{\mathrm{N}}\delta h/\left(R_{M}+h\right)^{2}\\\delta v_{\mathrm{E}}/\left(R_{N}+h\right)-v_{\mathrm{E}}\delta h/\left(R_{N}+h\right)^{2}\\\tan L\cdot\delta v_{\mathrm{E}}/\left(R_{\mathrm{N}}+h\right)+v_{\mathrm{E}}\sec^{2}L\cdot\delta L/\left(R_{N}+h\right)-v_{\mathrm{E}}\tan L\cdot\delta h/\left(R_{N}+h\right)^{2}\end{bmatrix}\\\left.=\left[\begin{array}{c}{-\delta\nu_{\mathrm{N}}/R_{\mathrm{M}h}+\nu_{\mathrm{N}}\delta h/R_{\mathrm{M}h}^{2}}\\{\delta\nu_{\mathrm{E}}/R_{\mathrm{N}h}-\nu_{\mathrm{E}}\delta h/R_{Nh}^{2}}\\{\tan L\cdot\delta\nu_{\mathrm{E}}/R_{\mathrm{N}h}+\nu_{\mathrm{E}}\sec^{2}L\cdot\delta L/R_{\mathrm{N}h}-\nu_{\mathrm{E}}\tan L\cdot\delta h/R_{\mathrm{N}h}^{2}}\end{array}\right.\right]=\boldsymbol{M}_{a\nu}\delta\boldsymbol{\nu}^{n}+\boldsymbol{M}_{2}\delta\boldsymbol{p}\end{gathered}$$
简记$R_{Mn}=R_{M}+h$,$R_{Nh}=R_{N}+h$,$\delta\boldsymbol{p}=\begin{bmatrix}\delta L&\delta\lambda&\delta h\end{bmatrix}^{\mathrm{T}}$
定义
$$
\begin{gathered}\boldsymbol{M}_{1}=\begin{bmatrix}0&0&0\\-\omega_{\mathrm{g}}\sin L&0&0\\\omega_{\mathrm{g}}\cos L&0&0\end{bmatrix}
\\
\boldsymbol{M}_{an}=\begin{bmatrix}0&-1/R_{\mathrm{M}n}&0\\1/R_{\mathrm{N}n}&0&0\\\tan L/R_{Nn}&0&0\end{bmatrix}
\\
\boldsymbol{M}_{2}=\begin{bmatrix}0&0&\nu_{\mathrm{N}}/R_{\mathrm{M}n}^{2}\\0&0&-\nu_{\mathrm{E}}/R_{\mathrm{N}n}^{2}\\v_{\mathrm{E}}\sec^{2}L/R_{\mathrm{N}n}&0&-\nu_{\mathrm{E}}\tan L/R_{\mathrm{N}n}^{2}\end{bmatrix}\end{gathered}
$$

那么姿态误差方程为，
$$
\begin{aligned}
\mathrm{\dot{\phi}}
&=\phi\times\omega_{in}^{n}+\delta\omega_{in}^{n}-\delta\omega_{ib}^{n}\\&=\phi\times\omega_{in}^{n}+(\delta\omega_{ie}^{n}+\delta\omega_{en}^{n})-C_{b}^{n}\delta\omega_{ib}^{b}\\
&=\boldsymbol{\phi}\times\boldsymbol{\omega}_{in}^{n}+(\boldsymbol{M}_{1}\delta\boldsymbol{p}+\boldsymbol{M}_{an}\delta\boldsymbol{v}^{n}+\boldsymbol{M}_{2}\delta\boldsymbol{p})-\boldsymbol{C}_{b}^{n}(\omega_{ibx}^{b}\delta\boldsymbol{K}_{\mathbf{G}x}+\omega_{iby}^{b}\delta\boldsymbol{K}_{\mathbf{G}y}+\omega_{ibz}^{b}\delta\boldsymbol{K}_{\mathbf{G}z}+\boldsymbol{\varepsilon}^{b})\\
&=-\omega_{in}^{n}\times\boldsymbol{\phi}+\boldsymbol{M}_{an}\delta\boldsymbol{\nu}^{n}+(\boldsymbol{M}_{1}+\boldsymbol{M}_{2})\delta\boldsymbol{p}-\omega_{ibx}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}x}-\omega_{iby}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}y}-\omega_{ibz}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}z}-\boldsymbol{C}_{iby}\\
&=\boldsymbol{M}_{aa}\boldsymbol{\phi}+\boldsymbol{M}_{a\nu}\delta\boldsymbol{\nu}^{n}+\boldsymbol{M}_{ap}\delta\boldsymbol{p}-\omega_{ibx}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}x}-\omega_{iby}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}y}-\omega_{ibz}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}z}-\boldsymbol{C}_{b}^{n}\boldsymbol{\varepsilon}^{b}\end{aligned}
$$
速度误差方程为，
$$
\begin{gathered}
\delta\dot{\nu}^{n}=\boldsymbol{f}_{\mathrm{sf}}^{n}\times\boldsymbol{\phi}+\left[(\boldsymbol{\nu}^{n}\times)\boldsymbol{M}_{cn}-(2\boldsymbol{\omega}_{i\boldsymbol{e}}^{n}+\boldsymbol{\omega}_{en}^{n})\times\right]\mathbf{\delta}\boldsymbol{\nu}^{n}+\left[(\boldsymbol{\nu}^{n}\times)(2\boldsymbol{M}_{1}+\boldsymbol{M}_{2})+\boldsymbol{M}_{3}\right]\mathbf{\delta}\boldsymbol{p}\\+C_{b}^{n}(f_{sfx}^{b}\delta K_{Ax}+f_{sfy}^{b}\delta K_{Ay}+f_{sfz}^{b}\delta K_{Az}+\nabla^{b})
\end{gathered}
$$
其中
$$
\begin{aligned}
&M_{_{va}}=(f_{\mathrm{sf}}^{n}\times)\\&\boldsymbol{M}_{_{w}}=(v^{n}\times)M_{av}-\left[(2\omega_{ie}^{n}+\omega_{en}^{n})\times\right]\\
&\boldsymbol{M}_{\nu p}=(v^{n}\times)(2M_{1}+M_{2})+M_{3}
\end{aligned}
$$

位置误差为
$$\delta\dot{p}=M_{pv}\delta v^{n}+M_{pp}\delta p$$
其中，
$$\boldsymbol{M}_{pv}=\begin{bmatrix}0&1/R_{Mh}&0\\\sec L/R_{Nh}&0&0\\0&0&1\end{bmatrix}$$
$$\left.\boldsymbol{M}_{pp}=\left\lfloor\begin{array}{ccc}{0}&{0}&{-\nu_{_\mathrm{N}}/R_{_\mathrm{Mh}}^{2}}\\{v_{_\mathrm{E}}\sec L\tan L/R_{_\mathrm{Nh}}}&{0}&{-\nu_{_\mathrm{E}}\sec L/R_{_\mathrm{Nh}}^{2}}\\{0}&{0}&{0}\end{array}\right.\right\rfloor$$

## 1.7 IMU系统级标定（误差方程在器件级标定中的应用）
**参考文献**：谢波, 秦永元, 万彦辉. 激光陀螺捷联惯导系统多位置标定方法[J]. 中国惯性技术学报, 2011, 19(2): 157-162, 169

### 1.7.1 误差模型建立
在载体坐标系（b系）建立IMU误差模型：

**加速度计误差模型**：
$$ 
\delta \boldsymbol{f}^{b} = 
\begin{bmatrix}
\delta f_x^{b} \\ 
\delta f_y^{b} \\ 
\delta f_z^{b}
\end{bmatrix} = 
\begin{bmatrix}
K_{ax} \\ 
K_{ay} \\ 
K_{az}
\end{bmatrix} + 
\begin{bmatrix}
K_{axx} & 0 & 0 \\ 
K_{ayx} & K_{ayy} & 0 \\ 
K_{azx} & K_{azy} & K_{azz}
\end{bmatrix}
\begin{bmatrix}
f_x^{b} \\ 
f_y^{b} \\ 
f_z^{b}
\end{bmatrix}
$$
其中：
- $\delta f_I^{b} (I=x,y,z)$：加速度计测量误差
- $f_I^{b}$：加速度计理想输出值
- $K_{aI}$：加速度计零偏（bias）
- $K_{aII}$：加速度计标度因数误差
- 非对角元素：加速度计安装误差系数

**陀螺仪误差模型**：
$$ 
\delta \boldsymbol{\omega}_{ib}^{b} = 
\begin{bmatrix}
\delta \omega_{ibx}^{b} \\ 
\delta \omega_{iby}^{b} \\ 
\delta \omega_{ibz}^{b}
\end{bmatrix} = 
\begin{bmatrix}
D_{gx} \\ 
D_{gy} \\ 
D_{gz}
\end{bmatrix} + 
\begin{bmatrix}
E_{gxx} & E_{gxy} & E_{gxz} \\ 
E_{gyx} & E_{gyy} & E_{gyz} \\ 
E_{gzx} & E_{gzy} & E_{gzz}
\end{bmatrix}
\begin{bmatrix}
\omega_{ibx}^{b} \\ 
\omega_{iby}^{b} \\ 
\omega_{ibz}^{b}
\end{bmatrix}
$$
其中：
- $\delta \omega_{ibI}^{b} (I=x,y,z)$：陀螺仪测量误差
- $\omega_{ibI}^{b}$：陀螺仪理想输出值
- $D_{gI}$：陀螺仪常值漂移
- $E_{gII}$：陀螺仪标度因数误差
- 非对角元素：陀螺仪安装误差系数

### 1.7.2 导航误差方程
捷联惯导系统误差方程为：
$$
\begin{aligned}
\delta \dot{\boldsymbol{\nu}}^{n} &= \boldsymbol{f}^{n} \times \boldsymbol{\phi} - (2\boldsymbol{\omega}_{ie}^{n} + \boldsymbol{\omega}_{en}^{n}) \times \delta \boldsymbol{\nu}^{n} \\
&- (2\delta \boldsymbol{\omega}_{ie}^{n} + \delta \boldsymbol{\omega}_{en}^{n}) \times \boldsymbol{\nu}^{n} + \delta \boldsymbol{f}^{n} \\
\dot{\boldsymbol{\phi}} &= \delta \boldsymbol{\omega}_{ie}^{n} + \delta \boldsymbol{\omega}_{en}^{n} - (\boldsymbol{\omega}_{ie}^{n} + \boldsymbol{\omega}_{en}^{n}) \times \boldsymbol{\phi} - \delta \boldsymbol{\omega}_{ib}^{n}
\end{aligned}
$$
其中：
- $\delta \dot{\boldsymbol{\nu}}^{n}$：速度误差导数
- $\dot{\boldsymbol{\phi}}$：姿态误差导数
- $\delta \boldsymbol{f}^{n}$：n系加速度计测量误差
- $\delta \boldsymbol{\omega}_{ib}^{n}$：n系陀螺仪测量误差

### 1.7.3 静态标定简化模型
在静态标定条件下：
$$
\begin{gathered}
\boldsymbol{\nu}^{n} = 0, \quad 
\delta \boldsymbol{\omega}_{ie}^{n} = 0, \quad 
\delta \boldsymbol{\omega}_{en}^{n} \approx 0 \\
\boldsymbol{f}^{n} = \begin{bmatrix} 0 & 0 & -g \end{bmatrix}^{T}, \quad 
\boldsymbol{\omega}_{ie}^{n} = \begin{bmatrix} \omega_{ie} \cos L & 0 & \omega_{ie} \sin L \end{bmatrix}^{T}
\end{gathered}
$$
其中：
- $g$：当地重力加速度
- $\omega_{ie}$：地球自转角速率
- $L$：标定位置纬度

取n系为东北天（ENU）地理坐标系，忽略牵连加速度项$-2\boldsymbol{\omega}_{ie}^{n} \times \delta \boldsymbol{\nu}^{n}$，得到简化误差方程：
$$
\begin{cases}
\delta \dot{\nu}_E^{n} = \delta f_E^{n} \\
\delta \dot{\nu}_N^{n} = \delta f_N^{n} \\
\delta \dot{\nu}_U^{n} = g \phi_N + \delta f_U^{n}
\end{cases}
$$

### 1.7.4 多位置标定方法
#### 标定原理
在标定过程中，设备静止状态下的速度误差由各项误差参数产生。当系统从一个位置翻转到另一个位置时，短时间内静态导航速度误差由惯性器件误差激励产生。通过设计一系列不同方位的机动，可激励出不同的误差参数，进而通过参数辨识实现全参数标定。

#### 标定流程设计
1. **转动方案**：
   - 设备绕三轴（X,Y,Z）每个轴正转3次，反转3次，共18次转动
   - 每次转动时间：<10s
   - 每次静止时间：>300s
   - 总标定时间：≈1.5小时

2. **位置序列**：
   | 位置序号 | 初始方位 | 转动操作 |
   |----------|----------|----------|
   | 1        | 东北天   | $+ox_b$  |
   | 2        | 东天南   | $+ox_b$  |
   | 3        | 东南地   | $+ox_b$  |
   | 4        | 东地北   | $-ox_b$  |
   | 5        | 东南地   | $-ox_b$  |
   | 6        | 东天南   | $-ox_b$  |
   | 7        | 东北天   | $+oy_b$  |
   | 8        | 地北东   | $+oy_b$  |
   | 9        | 西北地   | $+oy_b$  |
   | 10       | 天北西   | $-oy_b$  |
   | 11       | 西北地   | $-oy_b$  |
   | 12       | 地北东   | $-oy_b$  |
   | 13       | 东北天   | $+oz_b$  |
   | 14       | 北西天   | $+oz_b$  |
   | 15       | 西南天   | $+oz_b$  |
   | 16       | 南东天   | $-oz_b$  |
   | 17       | 西南天   | $-oz_b$  |
   | 18       | 北西天   | $-oz_b$  |
   | 19       | 东北天   | 停止     |

### 1.7.5 MEMS器件标定适配
#### 特殊考虑
上述流程适用于可静态初始化的光纤IMU系统。对于无法精确初始化的MEMS系统，需采用以下改进方案：

1. **初始对准**：
   - 采用双矢量定姿方法
   - 使用陀螺输出进行粗对准

2. **联合滤波**：
   - 使用滤波方法将各阶段数据联合处理（非独立辨识）
   - 建立全流程状态空间模型

#### MEMS误差建模
考虑MEMS器件特性，建立简化误差模型： \
**陀螺误差**：
$$ \delta \boldsymbol{\omega}_{ib}^{b} = \boldsymbol{K}_g \cdot \boldsymbol{\omega}_{ib}^{b} - (\boldsymbol{e}_b + \boldsymbol{g}_{sens} \cdot \boldsymbol{f}_{ib}^{b}) \cdot t_s $$
**加速度计误差**：
$$ \boldsymbol{f}_{ib}^{b} = \boldsymbol{K}_a \cdot \boldsymbol{f}_{ib}^{b} - (\boldsymbol{d}_b + \boldsymbol{K}_{ap} \cdot |\boldsymbol{f}_{ib}^{b}|) \cdot t_s $$

#### 状态空间模型
**状态向量**（42维）：
$$ \boldsymbol{X}_k = 
\begin{bmatrix} 
\boldsymbol{\phi} & \delta \boldsymbol{v}^n & \boldsymbol{e}_b & \boldsymbol{d}_b & \delta \boldsymbol{K}_g & \delta \boldsymbol{K}_a & \delta \boldsymbol{K}_{ap} & \boldsymbol{g}_{sens} 
\end{bmatrix}
 $$


**状态转移矩阵**：
$$
\boldsymbol{F} = \begin{bmatrix}
-\boldsymbol{\omega}_{\times} & \boldsymbol{0}_{3\times3} & -\boldsymbol{C}_b^n & \boldsymbol{0}_{3\times3} & -\boldsymbol{\omega}_x \boldsymbol{C}_b^n & -\boldsymbol{\omega}_y \boldsymbol{C}_b^n & -\boldsymbol{\omega}_z \boldsymbol{C}_b^n & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & -\boldsymbol{f}_x \boldsymbol{C}_b^n & -\boldsymbol{f}_y \boldsymbol{C}_b^n & -\boldsymbol{f}_z \boldsymbol{C}_b^n \\
\boldsymbol{f}_{\times} & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{C}_b^n & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{f}_x \boldsymbol{C}_b^n & \boldsymbol{f}_y \boldsymbol{C}_b^n & \boldsymbol{f}_z \boldsymbol{C}_b^n & \boldsymbol{C}_b^n \cdot \text{diag}(|\boldsymbol{f}_b|) & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} \\
\boldsymbol{0}_{36\times42}
\end{bmatrix}
$$

**状态转移矩阵**：
| $\boldsymbol{\phi}$  | $\delta vn$ | $eb$ | $db$ | $\delta Kg(:,1)$ | $\delta Kg(:,2)$ | $\delta Kg(:,3)$ | $\delta Ka(:,1)$ | $\delta Ka(:,2)$ | $\delta Ka(:,3)$ | $\delta Kap$ | $gSens(:,1)$ | $gSens(:,2)$ | $gSens(:,3)$ |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | 
| $-w_x$ | $0_{33}$ | $-C_n^b$ | $0_{33}$ | $-w_xC_n^b$ | $-w_yC_n^b$ | $-w_zC_n^b$ | $0_{33}$ |  $0_{33}$ | $0_{33}$ | $0_{33}$ | $-f_xC_n^b$ | $-f_yC_n^b$ | $-f_zC_n^b$ |
| $f_X$ | $0_{33}$ | $0_{33}$| $C_n^b$ | $0_{33}$ | $0_{33}$ | $0_{33}$ | $f_xC_n^b$ | $f_yC_n^b$ | $f_zC_n^b$ | $C_n^b*diag(abs(fb))$ |  $0_{33}$ | $0_{33}$ | $0_{33}$ |
| $0_{36*42}$ |


**量测方程**：
量测量为零速与零角速度条件：
$$ \boldsymbol{z}_k =
\begin{bmatrix} 
\delta \boldsymbol{v}^n \\ 
\delta \boldsymbol{\phi} 
\end{bmatrix}
 $$
**量测矩阵：**
$$
\boldsymbol{H} = 
\begin{bmatrix}
\boldsymbol{0}_{3\times3} & \boldsymbol{I}_{3\times3} & \boldsymbol{0}_{3\times36} \\
0,0,1 & \boldsymbol{0}_{1\times39}
\end{bmatrix}
$$

| $\boldsymbol{\phi}$  | $\delta vn$ | $eb$ | $db$ | $\delta Kg(:,1)$ | $\delta Kg(:,2)$ | $\delta Kg(:,3)$ | $\delta Ka(:,1)$ | $\delta Ka(:,2)$ | $\delta Ka(:,3)$ | $\delta Kap$ | $gSens(:,1)$ | $gSens(:,2)$ | $gSens(:,3)$ |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | 
| $0_{33}$ | $I_{33}$ | $0_{3*36}$  |
| $0,0,1$ | $0_{1*39} $ |

#### 迭代优化
1. 初始化：双矢量定姿
2. 迭代次数：>=5次
3. 每次迭代后更新参数并重新处理

**matlab实现见 sysclbtMEMS 函数。**

### 1.8 吊舱转动导致的算法计算异常
**参考文献**：翁浚,刘健宁,寇科,等.吊舱SINS/GNSS组合导航多杆臂效应在线估计算法[J].中国惯性技术学报,2021,29(02):184-190.DOI:10.13695/j.cnki.12-1222/o3.2021.02.007.

### 1.9 长直线工况下航向漂移
**参考文献**： 传递对准

## 2. GNSS定位基础与定位原理
**本节内容主要参考并基于[RTKlib](https://www.rtklib.com/)开源GNSS算法库的框架构建（v2.4.3），重点说明关键模型实现细节。**
**参考 GPS原理以及接收机设计**

### 2.1 无线电定位基本原理
GNSS定位通过测量到多颗已知位置卫星的距离，采用空间后方交会确定接收机位置。实现高精度定位需建立四大基础系统：

**时间系统**  
- GPS时（GPST）：以1980年1月6日UTC 00:00为起点，由原子钟维持
- 接收机本地时与GPST的钟差 $\delta t_r$ 作为待估参数
- RTKlib实现：函数处理周数/周秒转换

**空间系统**  
- WGS84坐标系（EPSG:4979）：
  $$ \begin{cases} 
  a = 6378137.0 \, \text{m} \\ 
  1/f = 298.257223563 
  \end{cases} $$
- 卫星位置计算：星历参数 → 轨道位置（`eph2pos()`函数）
- RTKlib使用`ecef2pos()`函数转换地心直角坐标到大地坐标

**测距系统**  
| 测距信号   | 精度    | 特性                     | RTKlib观测值       |
|------------|---------|--------------------------|-------------------|
| C/A码      | 1-3 m   | 捕获快，抗干扰弱         | `obsd_t.P[0]`     |
| P码        | 0.3-1 m | 军用，抗干扰强           | `obsd_t.P[1]`     |
| 载波相位   | 2-5 mm  | 含整周模糊度，需固定     | `obsd_t.L[0]/[1]` |

**质量系统**  
- 导航电文中的URA（用户测距精度）
- 接收机信号质量指标：载噪比(CN0)、锁定时间(locktime)

**定位流程**：
解析到信号后，根据卫星prn解出C/A码，载波相位以及多普勒测速，并得到解析信号过程中的信号质量消息，存储在文件中。 \
通过后方交会的方法得到载体的位置，其中需要考虑接收机的晶振钟差导致的测距损失。  \
在信号传播过程中，需要考虑电离层延时与对流层延时，需要考虑信号传播延时。 \
信号发送的时候需要考虑卫星的钟差？

在获得了超过参数的，准确的卫星(已知点) <---> 载体(未知点) 的距离(>=4)后，使用最小加权二乘方法获得方差最小的估计结果。 \
或使用运动学模型的方法约束，假设载体的运动符合某个运动模型，将该运动模型设为状态方程，测量值作为观测量，进行滤波计算，得到最优估计。

### 2.2 时间系统与空间系统定义与转换
#### 2.2.1 时间系统
**时间基准 = 时间原点 + 时间尺度**

| 时间系统               | 基准类型             | 起点                             | 特性                  | GNSS应用         |
|------------------------|----------------------|----------------------------------|-----------------------|------------------|
| 原子时(AT)             | 原子跃迁频率         | -                                | 超高稳定(10⁻¹⁶)      | 基础时间尺度     |
| 国际原子时(TAI)        | 原子钟集合           | 1958-01-01 UT 00:00:00           | 连续无跳秒           | 精密时间参考     |
| 世界时(UT)             | 地球自转周期         | 格林尼治平午                     | 受地球自转影响       | 天文导航         |
| 协调世界时(UTC)        | TAI秒长+UT时刻       | 同TAI                            | 闰秒调整             | 民用时间         |
| GPS时(GPST)            | 原子时               | 1980-01-06 UTC 00:00:00          | 连续无闰秒           | GPS/伽利略系统  |
| 北斗时(BDST)           | 原子时               | 2006-01-01 UTC 00:00:00          | 连续无闰秒           | 北斗系统        |
| GLONASS时              | UTC(莫斯科)          | -                                | 有闰秒               | GLONASS系统     |

**关键关系**：
- GPST = TAI - 19s + 常数偏移
- BDST = GPST - 14s + 常数偏移
- UTC = TAI - 当前闰秒数（2023年为37s）

#### 2.2.2 空间系统
**坐标系统 = 原点 + 轴向 + 尺度**
**1. 地心地固坐标系(ECEF)**  
- 原点：地球质心
- Z轴：指向国际地球自转服务(IERS)参考极
- X轴：指向格林尼治子午面与赤道交点
- Y轴：构成右手坐标系
- 主要实现：
  - WGS-84(GPS)：`[X, Y, Z]`
  - CGCS2000(北斗)：与WGS-84差异<2cm

**2. 站心坐标系(ENU)**  
- 原点：接收机天线相位中心
- 轴向：
  - E(East)：东向
  - N(North)：北向
  - U(Up)：天向(椭球法线方向)

**3. 坐标转换**  
**ECEF → 大地坐标(L,λ,h)**：
$$
\begin{cases}
\lambda = atan2(Y, X) \\
L = atan\left( \frac{Z}{\sqrt{X^2+Y^2}} \cdot \frac{1}{1-e^2} \right) \quad \text{(迭代求解)} \\
h = \frac{\sqrt{X^2+Y^2}}{\cos L} - N
\end{cases}
$$
其中 $N = \frac{a}{\sqrt{1-e^2 \sin^2 L}}$，$e^2 = 2f - f^2$

**大地坐标(L,λ,h) -> ECEF**：

$$
\begin{bmatrix} E \\ N \\ U \end{bmatrix} = 
\begin{bmatrix}
-\sin\lambda & \cos\lambda & 0 \\
-\sin L \cos\lambda & -\sin L \sin\lambda & \cos L \\
\cos L \cos\lambda & \cos L \sin\lambda & \sin L
\end{bmatrix}
\begin{bmatrix} X - X_0 \\ Y - Y_0 \\ Z - Z_0 \end{bmatrix}
$$

### 2.3 测距系统原理
#### 2.3.1 伪距测量
伪距测量基于信号传播时间：
$$ P = c \cdot (t_r - t^s) $$
其中：
- $t_r$：接收机接收时刻（接收机本地时间）
- $t^s$：卫星发射时刻（卫星钟时间）
- $c$：光速（299,792,458 m/s）

真实伪距包含误差项：
$$ P_{meas} = \rho + c(\delta t_r - \delta t^s) + I + T + \epsilon $$
- $\rho$：卫星与接收机的几何距离
- $\delta t_r$：接收机钟差
- $\delta t^s$：卫星钟差
- $I$：电离层延迟
- $T$：对流层延迟
- $\epsilon$：测量噪声

#### 2.3.2 载波相位测量
载波相位观测值：
$$ \phi = \phi_r(t_r) - \phi^s(t^s) + N $$
- $\phi_r(t_r)$：接收机在$t_r$时刻的载波相位
- $\phi^s(t^s)$：卫星在$t^s$时刻的载波相位
- $N$：整周模糊度（整数常量）

载波相位距离表达：
$$ \lambda \phi = \rho + c(\delta t_r - \delta t^s) - I + T + \lambda N + \epsilon_\phi $$
其中$\lambda$为载波波长。

**主要GNSS系统载波信号**：
| 系统   | 频点   | 频率 (MHz) | 波长 (cm) |
|--------|--------|------------|-----------|
| GPS    | L1     | 1575.42    | 19.0      |
| GPS    | L2     | 1227.60    | 24.4      |
| 北斗   | B1     | 1561.098   | 19.2      |
| 北斗   | B3     | 1268.52    | 23.6      |
| Galileo| E1     | 1575.42    | 19.0      |

#### 2.3.3 多普勒测速
多普勒频移公式：
$$ f_d = \frac{v_{rel} \cdot \mathbf{e}}{\lambda} $$
- $v_{rel}$：卫星与接收机相对速度
- $\mathbf{e}$：卫星视线方向单位向量
- $\lambda$：载波波长

RTKlib实现：`ddres()`函数中多普勒残差计算

### 2.4 质量评估系统
#### 2.4.1 卫星端质量指标
1. **用户测距精度 (URA)**：
   - 导航电文参数：`eph.ura`
   - 换算公式：$\sigma_{URA} = 2^{(N/2 + 1)} \, \text{(m)}, \, N \in [0..15]$
   - RTKlib应用：加权矩阵对角元素赋值（`var_ura()`函数）

2. **卫星健康状态**：
   - 电文参数：`svh`（0表示健康）
   - 异常卫星剔除：`satexclude()`函数

#### 2.4.2 接收机端质量指标
1. **载噪比 (CN0)**：
   - 定义：$CN0 [dB-Hz] = 10 \log_{10}(P_s / P_n)$
   - RTKlib阈值处理：
     ```c
     /* 伪距加权系数与CN0关系 */
     var = var_meas * pow(10.0, 0.1*(cn0_min - cn0));
     ```

2. **锁定时间 (Lock Time)**：
   - 周跳检测依据：`locktime`连续计数
   - 周跳判定：`locktime`重置或小于阈值

3. **定位残差分析**：
   - 后验残差RMS：$\sigma_{res} = \sqrt{\frac{\mathbf{V}^T\mathbf{V}}{n-4}}$
   - 接收机自主完好性监测(RAIM)：`raim_fde()`函数

#### 2.4.3 数据可用性指标
| 指标          | 计算公式                     | 健康阈值 |
|---------------|------------------------------|----------|
| 卫星可见数    | $N_{sat}$                    | ≥6       |
| 几何精度因子  | $GDOP = \sqrt{\sigma_x^2 + \sigma_y^2 + \sigma_z^2 + \sigma_t^2}$ | <5       |
| 位置精度因子  | $PDOP = \sqrt{\sigma_x^2 + \sigma_y^2 + \sigma_z^2}$ | <4       |

RTKlib输出：`sol_t.dop`数组存储DOP值

### 2.5  导航电文结构，信号端的问题，暂不分析
若研究超紧组合，信号端的可能也要会，可能需要一些信号与系统的知识。


### 2.6  通过星历计算规划时间t_k系的位置与速度 以及加速度（插值使用）
接收机在t时刻下，计算得到规划时间t_k为，t_{oe}为星历的时间。
$$t_{k}=t-t_{oe}$$
计算卫星的平均角速度n
$$n=n_{0}+\Delta n$$
计算平近点角
$$M_{k}=M_{0}+nt_{k}$$
计算偏近点角，
$$E_{j}=M+e_{s}\sin(E_{j-1})$$
计算真近点角，
$$\nu=\arctan\left(\frac{\sqrt{1-e_s^2}\sin E}{\cos E-e_s}\right)$$
计算升交点角距
$$\Phi_{k}=\nu_{k}+\omega$$
计算摄动校正
$$ 
\begin{gathered}
\delta u_{k}=C_{us}\sin(2\Phi_{k})+C_{uc}\cos(2\Phi_{k})\\
\delta r_{k}=C_{rs}\sin(2\Phi_{k})+C_{rc}\cos(2\Phi_{k})\\
\delta i_{k}=C_{is}\sin(2\Phi_{k})+C_{ic}\cos(2\Phi_{k})
\end{gathered}
$$
计算升交点角距，矢径长度和轨道倾角
$$u_{k}=\Phi_{k}+\delta u_{k}$$
$$r_{k}=a_{s}\begin{pmatrix}{1-e_{s}}&{\cos E_{k}}\end{pmatrix}+\delta r_{k}$$
$$i_{k}=i_{0}+i\cdot t_{k}+\delta i_{k}$$
计算轨道平面的位置
$$
x_{k}^{\prime}=r_{k}\cos u_{k} \\
y_{k}^{\prime}=r_{k}\sin u_{k}
$$
计算升交点赤经
$$
\Omega_{k}=\Omega_{0}+(\dot{\Omega}-\dot{\Omega}_{e})t_{k}-\dot{\Omega}_{e}t_{oe}
$$
计算得到卫星在WGS84坐标系下的位置
$$
\begin{aligned}
&x_{k}=x_{k}^{\prime}\cos\Omega_{k}-y_{k}^{\prime}\cos i_{k}\sin\Omega_{k}\\
&y_{k}=x_{k}^{\prime}\sin\Omega_{k}+y_{k}^{\prime}\cos i_{k}\cos\Omega_{k}\\
&z_{k}=y_{k}^{\prime}\sin i_{k}
\end{aligned}
$$

也有取dt的方法确认速度。

计算信号发射时刻的 偏近点角速率
$$
\dot{E}_k=\frac{\dot{M}_k}{1-e_s\cos E_k} \\
\dot{E}_k=\frac{n}{1-e_s\cos E_k}
$$
计算信号发射时刻的 升交点赤经速率
$$
\dot{\Phi}_{k}=\dot{\nu}_{k} \\
=\frac{\left(1+e_{s}\cos\nu_{k}\right)\dot{E}_{k}\sin E_{k}}{\left(1-e_{s}\cos E_{k}\right)\sin\nu_{k}}\\ =\frac{\sqrt{1-e_{s}^{2}}\dot{E}_{k}}{1-e_{s}\cos E_{k}}
$$
计算摄动校正量
$$
\delta\dot{u}_{k}=2\dot{\Phi}_{k}\left(C_{us}\cos(2\Phi_{k})-C_{uc}\sin(2\Phi_{k})\right) \\
\delta\dot{r}_{k}=2\dot{\Phi}_{i:}\left(C_{rs}\cos(2\Phi_{k})-C_{rc}\sin(2\Phi_{k})\right) \\
\delta\dot{i}_{k}=2\dot{\Phi}_{k}\left(C_{is}\cos(2\Phi_{k})-C_{ic}\sin(2\Phi_{k})\right)
$$
计算其他的校正量
$$
\begin{gathered}
\dot{u}_{k}=\dot{\Phi}_{k}+\delta\dot{u}_{k}\\
\dot{r}_{k}=a_{s}e_{s}\dot{E}_{k}\sin E_{k}+\delta\dot{r}_{k}\\
i_{k}=i+\delta i_{k}\\
\dot{\Omega}_{k}=\dot{\Omega}-\dot{\Omega}_{e}
\end{gathered}
$$
得到轨道的速度
$$
\dot{x}_{k}^{\prime}=\dot{r}_{k}\cos u_{k}-r_{k}\dot{u}_{k}\sin u_{k} \\
\dot{y}_{k}^{\prime}=\dot{r}_{k}\sin u_{k}+r_{k}\dot{u}_{k}\cos u_{k}
$$
得到WGS84的速度
$$
\begin{aligned}
&\dot{x}_{k}=(\dot{x}_{k}^{\prime}-y_{k}^{\prime}\dot{\Omega}_{k}\cos i_{k})\cos\Omega_{k}-(x_{k}^{\prime}\dot{\Omega}_{k}+\dot{y}_{k}^{\prime}\cos i_{k}-y_{k}^{\prime}\dot{i}_{k}\sin i_{k})\sin\Omega_{k}\\
&=-y_{k}\dot{\Omega}_{k}-(\dot{y}_{k}^{\prime}\cos i_{k}-z_{k}\dot{i}_{k})\sin\Omega_{k}+\dot{x}_{k}^{\prime}\cos\Omega_{k}\\
&\dot{y}_{k}=(\dot{x}_{k}^{\prime}-y_{k}^{\prime}\dot{\Omega}_{k}\cos i_{k})\sin\Omega_{k}+(x_{k}^{\prime}\dot{\Omega}_{k}+\dot{y}_{k}^{\prime}\cos i_{k}-y_{k}^{\prime}i_{k}\sin i_{k})\cos\Omega_{k}\\
&=x_{k}\dot{\Omega}_{k}+(\dot{y}_{k}^{\prime}\cos i_{k}-z_{k}\dot{i}_{k})\cos\Omega_{k}+\dot{x}_{k}^{\prime}\sin\Omega_{k}\\
&z_k=\dot{y}_{k}^{\prime}\sin i_{k}+y_{k}^{\prime}i_{k}\cos i_{k}
\end{aligned}
$$

### 2.8 各个误差模型的建模
#### 2.8.1 卫星时钟校正
$$\delta t^{(s)}=\Delta t^{(s)}+\Delta t_{r}-T_{GD}$$
$$\Delta t_{r}=Fe_{s}\sqrt{a_{s}}\sin E_{k}$$
$$\Delta t^{(s)}=a_{f0}+a_{f1}(t-t_{oc})+a_{f2}(t-t_{oc})^{2}$$
$t_{oc}$ 为参考时间。
实际上还有一个信号传播延时需要扣除。
#### 2.8.1 电离层模型建模
$$
\rho_{1}=r+\delta t_{u}-\delta t^{(s)}+I_{1}+T+\varepsilon_{\rho_{1}} \\
\rho_{2}=r+\delta t_{u}-\delta t^{(s)}+I_{2}+T+\varepsilon_{\rho_{2}}
$$
$$
I_{1}=40.28\frac{N_{e}}{{f_{1}}^{2}} \\
I_{2}=40.28\frac{N_{e}}{{f_{2}}^{2}} \\
I_1=\frac{f_2^2}{f_1^2-f_2^2}(\rho_2-\rho_1)
$$
$$
\begin{aligned}
\rho_{1,2}&=\rho_{1}-I_{1}\approx r+\delta t_{u}-\delta t^{(s)}+T_{,}\\
&=\frac{f_{1}^{2}}{f_{1}^{2}-f_{2}^{2}}\rho_{1}-\frac{f_{2}^{2}}{f_{1}^{2}-f_{2}^{2}}\rho_{2} \\
&=2.546\rho_{1}-1.546\rho_{2}
\end{aligned}
$$
#### 2.8.2 对流层模型建模
$$
T=\frac{2.47}{\sin\theta+0.0121}
$$
#### 2.9 伪距定位原理
在观测到多个伪距方程
$$\rho^{(n)}=r^{(n)}+\delta t_{u}-\delta t^{(n)}+\hat{I}^{(n)}+T^{(n)}+\varepsilon_{\rho}^{(n)}$$
定义校正后的伪距观测为
$$\rho_c^{(n)}=\rho^{(n)}+\delta t^{(n)}-I^{(n)}-T^{(n)}$$
以及校正后的观测方程为
$$r^{(n)}+\delta t_{u}=\rho_{c}^{(n)}-\varepsilon_{\rho}^{(n)} $$
$$r^{(n)}=\left\|x^{(n)}-x\right\|=\sqrt{\left(x^{(n)}-x\right)^{2}+\left(y^{(n)}-y\right)^{2}+\left(z^{(n)}-z\right)^{2}}$$
本质是求解这样一个方程组
$$\sqrt{\left(x^{(1)}-x\right)^{2}+\left(y^{(1)}-y\right)^{2}+\left(z^{(1)}-z\right)^{2}}+\delta t_{u}=\rho_{c}^{(1)}\\
\sqrt{\left(x^{(2)}-x\right)^{2}+\left(y^{(2)}-y\right)^{2}+\left(z^{(2)}-z\right)^{2}}+\delta t_{u}=\rho_{c}^{(2)}\\
\cdots \\
\sqrt{\left(x^{(N)}-x\right)^{2}+\left(y^{(N)}-y\right)^{2}+\left(z^{(N)}-z\right)^{2}}+\delta t_{u}=\rho_{c}^{(N)}$$
线性化后，
$$G=\begin{bmatrix}-I_x^{(1)}(x_{k-1})&-I_y^{(1)}(x_{k-1})&-I_z^{(1)}(x_{k-1})&1\\-I_x^{(2)}(x_{k-1})&-I_y^{(2)}(x_{k-1})&-I_z^{(2)}(x_{k-1})&1\\\cdots&\cdots&\cdots&\cdots\\-I_x^{(N)}(x_{k-1})&-I_y^{(N)}(x_{k-1})&-I_z^{(N)}(x_{k-1})&1\end{bmatrix}=\begin{bmatrix}-[I^{(1)}(x_{k-1})]^\mathrm{T}&1\\-[I^{(2)}(x_{k-1})]^\mathrm{T}&1\\\cdots&\cdots\\-[I^{(N)}(x_{k-1})]^\mathrm{T}&1\end{bmatrix}$$
$$b=\begin{bmatrix}\rho_c^{(1)}-r^{(1)}(x_{k-1})-\delta t_{u,k-1}\\\rho_c^{(2)}-r^{(2)}(x_{k-1})-\delta t_{u,k-1}\\\cdots\\\rho_c^{(N)}-r^{(N)}(x_{k-1})-\delta t_{u,k-1}\end{bmatrix}$$
$$-I_{x}^{(n)}(x_{k-1})=\frac{-(x^{(n)}-x_{k-1})}{r^{(n)}(x_{k-1})}=\frac{-(x^{(n)}-x_{k-1})}{\left\|x^{(n)}-x_{k-1}\right\|}=\frac{\partial r^{(n)}}{\partial x}|_{x=x_{k-1}}$$
实际中，该方程并非难点，难点在于每颗星的定权。 \\
可知加权解为 
$$
\hat{x}=(H^TWH)^{-1}H^TWy\quad(J_{WLS}=v^TWv\to\min)
$$
#### 2.10 伪距定位误差分析
$$\sigma_{URE}^2=\sigma_{CS}^2+\sigma_{P}^2+\sigma_{RNM}^2$$
$$\begin{aligned}&=(G^{\mathrm{T}}G)^{-1}\sigma_{URE}^{2}\\&=H\sigma_{URE}^{2}\end{aligned}$$

#### 2.11 多普勒测速
观测方程为
$$\dot{\rho}^{(n)}=\dot{r}^{(n)}+\delta f_{u}-\delta f^{(n)}+\varepsilon_{\dot{\rho}}^{(n)}$$
在有多个观测时，
$$
-\nu\cdot I^{(n)}+\delta f_{u}=\left(\dot{\rho}^{(n)}-\nu^{(n)}\cdot I^{(n)}+\delta f^{(n)}\right)-\varepsilon_{\dot{\rho}}^{(n)}
$$
即有
$$
G\begin{bmatrix}\nu_x\\\nu_y\\\nu_z\\\delta f_u\end{bmatrix}=\dot{b}+\varepsilon_\rho
$$
同理加权解为
$$
\hat{x}=(H^TWH)^{-1}H^TWy\quad(J_{WLS}=v^TWv\to\min)
$$

### 2.12 RTK差分定位流程




## 3. 里程计与视觉定位部分 (视觉部分不熟悉)
**参考来源**：PINS
**核心观点**：在不引入回环检测、地标点及地图的情况下，视觉信息可简化为类似里程计的相对测量量。本节重点讨论使用速度或相对位移进行位置递推的方法。

### 3.1 里程计定位解算
#### 3.1.1 坐标系定义与基本假设
1. **坐标系定义**：
   - 里程仪测量坐标系（m系）：右手直角坐标系
     - Y轴：地平面内，指向车体正前方
     - Z轴：垂直于地平面向上
     - X轴：指向右方（"右-前-上"构型）
   - 导航坐标系（n系）：东北天（ENU）地理坐标系

2. **基本假设**：
   - 车轮紧贴路面，无打滑、滑行和弹跳
   - IMU坐标系（b系）与m系重合
   - 载体视为质点，里程计测量前向速度

#### 3.1.2 速度模型与坐标转换
里程仪速度输出模型：
$$
\boldsymbol{v}_{\mathrm{D}}^{m} = \begin{bmatrix} 0 \\ v_{\mathrm{D}} \\ 0 \end{bmatrix}
$$
其中$v_{\mathrm{D}}$为前向速度大小（前进正，倒车负）。

通过姿态矩阵$C_b^n$转换至导航系：
$$
\boldsymbol{v}_\mathrm{D}^n = C_b^n \boldsymbol{v}_\mathrm{D}^m
$$

#### 3.1.3 位置更新
位置微分方程：
$$
\dot{\boldsymbol{p}}_{\mathrm{D}} = \boldsymbol{M}_{p\nu\mathrm{D}} \boldsymbol{v}_{\mathrm{D}}^{n}
$$
其中：
- 位置向量：$\boldsymbol{p}_{\mathrm{D}} = \begin{bmatrix} L_{\mathrm{D}} \\ \lambda_{\mathrm{D}} \\ h_{\mathrm{D}} \end{bmatrix}$
- 转换矩阵：
  $$
  \boldsymbol{M}_{p\nu\mathbf{D}} = \begin{bmatrix} 
  0 & 1/R_{Mh\mathbf{D}} & 0 \\ 
  \sec L_{\mathbf{D}}/R_{Nh\mathbf{D}} & 0 & 0 \\ 
  0 & 0 & 1 
  \end{bmatrix}
  $$
- 曲率半径计算：
  $R_{MhD} = R_{MD} + h_{D}$（子午圈）
  $R_{NhD} = R_{ND} + h_{D}$（卯酉圈）

离散化位置更新：
$$
\begin{aligned}
L_{\mathrm{D}(j)} &= L_{\mathrm{D}(j-1)} + \frac{\Delta S_{\mathrm{N}(j)}}{R_{Mh\mathrm{D}(j-1)}} \\
\lambda_{\mathrm{D}(j)} &= \lambda_{\mathrm{D}(j-1)} + \frac{\Delta S_{\mathrm{E}(j)}\sec L_{\mathrm{D}(j-1)}}{R_{Nh\mathrm{D}(j-1)}} \\
h_{\mathrm{D}(j)} &= h_{\mathrm{D}(j-1)} + \Delta S_{\mathrm{U}(j)}
\end{aligned}
$$
其中$\Delta\boldsymbol{S}_{j}^{n} = \begin{bmatrix} \Delta S_{\mathrm{E}(j)} & \Delta S_{\mathrm{N}(j)} & \Delta S_{\mathrm{U}(j)} \end{bmatrix}^{T}$为位移增量。

#### 3.1.4 姿态更新
姿态矩阵微分方程：
$$
\dot{C}_{b}^{n} = C_{b}^{n}(\boldsymbol{\omega}_{ib}^{b}\times) - (\boldsymbol{\omega}_{in}^{n}\times)C_{b}^{n}
$$
其中：
$$
\boldsymbol{\omega}_{in}^n = \boldsymbol{\omega}_{ie}^n + \boldsymbol{\omega}_{en}^n
$$
$$
\boldsymbol{\omega}_{ie}^n = \begin{bmatrix} 0 \\ \omega_{ie}\cos L_{\mathrm{D}} \\ \omega_{ie}\sin L_{\mathrm{D}} \end{bmatrix}
$$
$$
\boldsymbol{\omega}_{en}^{n} = \begin{bmatrix} 
-\frac{\nu_{\mathrm{DN}}}{R_{Mh\mathrm{D}}} \\ 
\frac{\nu_{\mathrm{DE}}}{R_{Nh\mathrm{D}}} \\ 
\frac{\nu_{\mathrm{DE}}\tan L_{\mathrm{D}}}{R_{Nh\mathrm{D}}} 
\end{bmatrix}
$$

离散化姿态更新：
$$
C_{b(j)}^{n} = C_{n(j-1)}^{n(j)} C_{b(j-1)}^{n} C_{b(j)}^{b(j-1)}
$$
旋转矢量计算：
$$
\boldsymbol{\phi}_{in(j)}^n = T_j \begin{bmatrix} 0 \\ \omega_{ie}\cos L_{\mathrm{D}(j)} \\ \omega_{ie}\sin L_{\mathrm{D}(j)} \end{bmatrix} + \begin{bmatrix} 
-\Delta S_{\mathrm{DN}(j)}/R_{M\mathrm{D}(j)} \\ 
\Delta S_{\mathrm{DE}(j)}/R_{M\mathrm{D}(j)} \\ 
\Delta S_{\mathrm{DE}(j)}\tan L_{\mathrm{D}(j)}/R_{N\mathrm{D}(j)} 
\end{bmatrix}
$$

### 3.2 里程计误差分析
#### 3.2.1 安装偏差模型
m系到b系的变换矩阵：
$$
C_b^m = I + (\boldsymbol{\alpha}\times) = \begin{bmatrix}
1 & -\alpha_\psi & \alpha_\gamma \\ 
\alpha_\psi & 1 & -\alpha_\theta \\ 
-\alpha_\gamma & \alpha_\theta & 1 
\end{bmatrix}
$$
其中$\boldsymbol{\alpha} = \begin{bmatrix} \alpha_\theta & \alpha_\gamma & \alpha_\psi \end{bmatrix}^{\mathrm{T}}$为安装偏角。

#### 3.2.2 刻度系数误差
实际速度输出：
$$
\tilde{v}_{\mathrm{D}} = (1 + \delta K_{\mathrm{D}}) \nu_{\mathrm{D}}
$$

#### 3.2.3 导航系速度误差推导
实际速度在导航系投影：
$$
\begin{aligned}
\tilde{\boldsymbol{v}}_{\mathrm{D}}^{n} &= \tilde{\boldsymbol{C}}_{b}^{n}(\boldsymbol{C}_{b}^{m})^{\mathrm{T}}\tilde{\boldsymbol{v}}_{\mathrm{D}}^{m} \\
&= (\boldsymbol{I}-\boldsymbol{\phi}_{D}\times)\boldsymbol{C}_{b}^{n}(\boldsymbol{I}-\boldsymbol{\alpha}\times)(1+\delta K_{\mathrm{D}})\boldsymbol{v}_{\mathrm{D}}^{m} \\
&\approx \boldsymbol{v}_{\mathrm{D}}^{n} - (\boldsymbol{\phi}_{\mathrm{D}}\times)\boldsymbol{C}_{b}^{n}\boldsymbol{v}_{\mathrm{D}}^{m} - \boldsymbol{C}_{b}^{n}(\boldsymbol{\alpha}\times)\boldsymbol{v}_{\mathrm{D}}^{m} + \boldsymbol{C}_{b}^{n}\delta K_{\mathrm{D}}\boldsymbol{v}_{\mathrm{D}}^{m} \\
&= \boldsymbol{v}_{\mathrm{D}}^{n} + \boldsymbol{v}_{\mathrm{D}}^{n}\times\boldsymbol{\phi}_{\mathrm{D}} + \boldsymbol{C}_{b}^{n}(\boldsymbol{v}_{\mathrm{D}}^{m}\times)\boldsymbol{\alpha} + \boldsymbol{C}_{b}^{n}\boldsymbol{v}_{\mathrm{D}}^{m}\delta K_{\mathrm{D}}
\end{aligned}
$$

展开矩阵形式：
$$
\begin{aligned}
\tilde{\boldsymbol{v}}_{\mathrm{D}}^{n} &= \boldsymbol{v}_{\mathrm{D}}^{n} + \boldsymbol{v}_{\mathrm{D}}^{n}\times\boldsymbol{\phi}_{\mathrm{D}} + 
\begin{bmatrix}
C_{11} & C_{12} & C_{13} \\
C_{21} & C_{22} & C_{23} \\
C_{31} & C_{32} & C_{33}
\end{bmatrix}
\begin{bmatrix}
0 & 0 & v_{\mathrm{D}} \\
0 & 0 & 0 \\
-v_{\mathrm{D}} & 0 & 0
\end{bmatrix}
\boldsymbol{\alpha} \\
&+ \begin{bmatrix}
C_{11} & C_{12} & C_{13} \\
C_{21} & C_{22} & C_{23} \\
C_{31} & C_{32} & C_{33}
\end{bmatrix}
\begin{bmatrix}
0 \\ v_{\mathrm{D}} \\ 0
\end{bmatrix}
\delta K_{\mathrm{D}} \\
&= \boldsymbol{v}_{\mathrm{D}}^{n} + \boldsymbol{v}_{\mathrm{D}}^{n}\times\boldsymbol{\phi}_{\mathrm{D}} + v_{\mathrm{D}}
\begin{bmatrix}
-C_{13} & 0 & C_{11} \\
-C_{23} & 0 & C_{21} \\
-C_{33} & 0 & C_{31}
\end{bmatrix}
\boldsymbol{\alpha} + v_{\mathrm{D}}
\begin{bmatrix}
C_{12} \\ C_{22} \\ C_{32}
\end{bmatrix}
\delta K_{\mathrm{D}}
\end{aligned}
$$

简化形式（忽略滚动角）：
$$
\tilde{\boldsymbol{v}}_{\mathrm{D}}^{n} = \boldsymbol{v}_{\mathrm{D}}^{n} + \boldsymbol{v}_{\mathrm{D}}^{n}\times\boldsymbol{\phi}_{\mathrm{D}} + \boldsymbol{M}_{\nu\mathrm{iD}} \boldsymbol{\kappa}_{\mathrm{D}}
$$
其中：
$$
\boldsymbol{M}_{\nu\boldsymbol{i}\mathbf{D}} = \nu_{\mathbf{D}} \begin{bmatrix} 
-C_{13} & C_{12} & C_{11} \\ 
-C_{23} & C_{22} & C_{21} \\ 
-C_{33} & C_{32} & C_{31} 
\end{bmatrix}, \quad
\boldsymbol{\kappa}_\mathrm{D} = \begin{bmatrix} \alpha_\theta \\ \delta K_\mathrm{D} \\ \alpha_\psi \end{bmatrix}
$$

#### 3.2.4 位置误差方程
位置误差微分方程：
$$
\delta\dot{\boldsymbol{p}}_{\mathrm{D}} = \boldsymbol{M}_{p\nu\mathrm{D}} \delta\boldsymbol{v}_{\mathrm{D}}^{n} + \boldsymbol{M}_{pp\mathrm{D}} \delta\boldsymbol{p}_{\mathrm{D}}
$$
其中：
$$
\boldsymbol{M}_{p\nu\mathrm{D}}=\begin{bmatrix}0&1/R_{Mi\mathrm{D}}&0\\\sec L_\mathrm{D}/R_{Nh\mathrm{D}}&0&0\\0&0&1\end{bmatrix}  \\
\boldsymbol{M}_{pp\mathrm{D}} = \begin{bmatrix} 
0 & 0 & -\nu_{\mathrm{DN}}/R_{M\mathrm{D}}^2 \\ 
\nu_\mathrm{DE}\sec L_\mathrm{D}\tan L_\mathrm{D}/R_{Nh\mathrm{D}} & 0 & -\nu_\mathrm{DE}\sec L/R_{Nh\mathrm{D}}^2 \\ 
0 & 0 & 0 
\end{bmatrix}
$$

代入速度误差：
$$
\delta\dot{\boldsymbol{p}}_{\mathrm{D}} = \boldsymbol{M}_{p\nu\mathrm{D}} (\boldsymbol{v}_{\mathrm{D}}^{n}\times\boldsymbol{\phi}_{\mathrm{D}} + \boldsymbol{M}_{p\nu\mathrm{D}} \boldsymbol{M}_{\nu\mathrm{iD}} \boldsymbol{\kappa}_{\mathrm{D}} + \boldsymbol{M}_{pp\mathrm{D}} \delta\boldsymbol{p}_{\mathrm{D}}
$$

#### 3.2.5 姿态误差方程
姿态误差微分方程：
$$
\dot{\boldsymbol{\phi}}_{\mathrm{D}} = \boldsymbol{M}_{aa\mathrm{D}}^{\prime} \boldsymbol{\phi}_\mathrm{D} + \boldsymbol{M}_{a\nu\mathrm{D}} \delta\boldsymbol{v}_\mathrm{D}^{n} + \boldsymbol{M}_{ap\mathrm{D}} \delta\boldsymbol{p}_\mathrm{D} - \boldsymbol{C}_b^n \boldsymbol{\varepsilon}^b
$$
其中：
$$
\boldsymbol{M}_{aa\mathrm{D}}^{\prime} = -\left( \begin{bmatrix} 0 \\ \omega_{ie}\cos L_{\mathrm{D}} \\ \omega_{ie}\sin L_{\mathrm{D}} \end{bmatrix} + \begin{bmatrix} -\nu_{\mathrm{DN}}/R_{Mi\mathrm{D}} \\ \nu_{\mathrm{DE}}/R_{Ni\mathrm{D}} \\ \nu_{\mathrm{DE}}\tan L_{\mathrm{D}}/R_{Ni\mathrm{D}} \end{bmatrix} \right) \times
$$

代入速度、位置误差：
$$
\begin{aligned}
\dot{\boldsymbol{\phi}}_{\mathrm{D}} &= \boldsymbol{M}_{\mathrm{aaD}}^{\prime}\boldsymbol{\phi}_{\mathrm{D}} + \boldsymbol{M}_{\mathrm{anD}}(\boldsymbol{\nu}_{\mathrm{D}}^{n}\times\boldsymbol{\phi}_{\mathrm{D}} + \boldsymbol{M}_{\mathrm{anD}} \boldsymbol{M}_{\mathrm{\nu ibD}} \boldsymbol{\kappa}_{\mathrm{D}} + \boldsymbol{M}_{\mathrm{apD}} \delta\boldsymbol{p}_{\mathrm{D}} - \boldsymbol{C}_{b}^{n} \boldsymbol{\varepsilon}^{b} \\
&= \left[\boldsymbol{M}_{aa\mathrm{D}}^{\prime} + \boldsymbol{M}_{an\mathrm{D}} (\boldsymbol{v}_{\mathrm{D}}^{n} \times) \right] \boldsymbol{\phi}_{\mathrm{D}} + \boldsymbol{M}_{an\mathrm{D}} \boldsymbol{M}_{\nu i\mathrm{D}} \boldsymbol{\kappa}_{\mathrm{D}} + \boldsymbol{M}_{ap\mathrm{D}} \delta\boldsymbol{p}_{\mathrm{D}} - \boldsymbol{C}_{b}^{n} \boldsymbol{\varepsilon}^{b}
\end{aligned}
$$

最终形式：
$$
\dot{\boldsymbol{\phi}}_{\mathrm{D}} = \boldsymbol{M}_{aa\mathrm{D}} \boldsymbol{\phi}_\mathrm{D} + \boldsymbol{M}_{ak\mathrm{D}} \boldsymbol{\kappa}_\mathrm{D} + \boldsymbol{M}_{ap\mathrm{D}} \delta\boldsymbol{p}_\mathrm{D} - \boldsymbol{C}_b^n \boldsymbol{\varepsilon}^b
$$
其中：
$$
\boldsymbol{M}_{aa\mathrm{D}} = \boldsymbol{M}_{aa\mathrm{D}}^{\prime} + \boldsymbol{M}_{a\nu\mathrm{D}} (\boldsymbol{v}_{\mathrm{D}}^{n} \times) \\
M_{ak\mathrm{D}}=M_{av\mathrm{D}}M_{\nu k\mathrm{D}}
$$

### 3.3 EKF-SLAM 算法推导与误差分析
#### 3.3.1 状态空间定义
##### 3.3.1.1 状态向量构成
$$
\boldsymbol{x}_k = \begin{bmatrix} 
\boldsymbol{x}_{v,k} \\ 
\boldsymbol{m}_1 \\ 
\vdots \\ 
\boldsymbol{m}_n 
\end{bmatrix} \in \mathbb{R}^{3+2n}
$$
- **机器人位姿**：$\boldsymbol{x}_{v,k} = [x_k, y_k, \theta_k]^\top$ (2D 位置与航向)
- **路标位置**：$\boldsymbol{m}_i = [m_{i,x}, m_{i,y}]^\top$ (第 i 个路标的世界坐标)

##### 3.3.1.2 协方差矩阵结构
$$
\boldsymbol{P}_k = \begin{bmatrix}
\boldsymbol{P}_{vv,k} & \boldsymbol{P}_{vm,k} \\ 
\boldsymbol{P}_{vm,k}^\top & \boldsymbol{P}_{mm,k} 
\end{bmatrix}
$$
- $\boldsymbol{P}_{vv,k}$：位姿协方差 (3×3)
- $\boldsymbol{P}_{mm,k}$：路标位置协方差 (2n×2n)
- $\boldsymbol{P}_{vm,k}$：位姿-路标互协方差 (3×2n)

#### 3.3.2 运动模型（预测步骤）
##### 3.3.2.1 非线性运动模型
$$
\boldsymbol{x}_{v,k} = f(\boldsymbol{x}_{v,k-1}, \boldsymbol{u}_k, \boldsymbol{w}_k) = 
\begin{bmatrix}
x_{k-1} + \Delta T v_k \cos(\theta_{k-1} + \gamma_k) \\
y_{k-1} + \Delta T v_k \sin(\theta_{k-1} + \gamma_k) \\
\theta_{k-1} + \Delta T \omega_k
\end{bmatrix}
$$
- 控制输入：$\boldsymbol{u}_k = [v_k, \omega_k]^\top$ (线速度, 角速度)
- 过程噪声：$\boldsymbol{w}_k \sim \mathcal{N}(0, \boldsymbol{Q}_k)$

##### 3.3.2.2 雅可比矩阵计算
$$
\boldsymbol{F}_x = \frac{\partial f}{\partial \boldsymbol{x}_v} = 
\begin{bmatrix}
1 & 0 & -\Delta T v_k \sin(\theta_{k-1} + \gamma_k) \\
0 & 1 & \Delta T v_k \cos(\theta_{k-1} + \gamma_k) \\
0 & 0 & 1
\end{bmatrix}
$$
$$
\boldsymbol{F}_w = \frac{\partial f}{\partial \boldsymbol{w}_k} = 
\begin{bmatrix}
\Delta T \cos(\theta_{k-1} + \gamma_k) & 0 \\
\Delta T \sin(\theta_{k-1} + \gamma_k) & 0 \\
0 & \Delta T
\end{bmatrix}
$$

##### 3.3.2.3 预测方程
$$\begin{aligned}
\hat{\boldsymbol{x}}_{k|k-1} &= \begin{bmatrix} f(\boldsymbol{x}_{v,k-1}, \boldsymbol{u}_k, 0) \\ \boldsymbol{m}_{k-1} \end{bmatrix} \\
\boldsymbol{P}_{k|k-1} &= \boldsymbol{F} \boldsymbol{P}_{k-1} \boldsymbol{F}^\top + \boldsymbol{G} \boldsymbol{Q}_k \boldsymbol{G}^\top
\end{aligned}$$
其中：
$$
\boldsymbol{F} = \begin{bmatrix} \boldsymbol{F}_x & 0 \\ 0 & \boldsymbol{I}_{2n} \end{bmatrix}, \quad
\boldsymbol{G} = \begin{bmatrix} \boldsymbol{F}_w \\ 0 \end{bmatrix}
$$

#### 3.3.3 观测模型（更新步骤）
##### 3.3.3.1 非线性观测模型
$$
\boldsymbol{z}_k^i = h(\boldsymbol{x}_v, \boldsymbol{m}_i) + \boldsymbol{v}_k^i = 
\begin{bmatrix}
\sqrt{(m_{i,x} - x)^2 + (m_{i,y} - y)^2} \\
atan2(m_{i,y} - y, m_{i,x} - x) - \theta
\end{bmatrix}
$$
- 观测噪声：$\boldsymbol{v}_k^i \sim \mathcal{N}(0, \boldsymbol{R}_k)$

##### 3.3.3.2 观测雅可比矩阵
$$
\boldsymbol{H}^i = \frac{\partial h}{\partial \boldsymbol{x}} = 
\begin{bmatrix} 
\boldsymbol{H}_v^i & 0 \cdots 0 & \boldsymbol{H}_m^i & 0 \cdots 0 
\end{bmatrix}
$$
其中：
$$
\boldsymbol{H}_v^i = \begin{bmatrix}
-\frac{\Delta x}{d} & -\frac{\Delta y}{d} & 0 \\
\frac{\Delta y}{d^2} & -\frac{\Delta x}{d^2} & -1
\end{bmatrix}, \quad
\boldsymbol{H}_m^i = \begin{bmatrix}
\frac{\Delta x}{d} & \frac{\Delta y}{d} \\
-\frac{\Delta y}{d^2} & \frac{\Delta x}{d^2}
\end{bmatrix}
$$
- $\Delta x = m_{i,x} - x$, $\Delta y = m_{i,y} - y$, $d = \sqrt{(\Delta x)^2 + (\Delta y)^2}$

#### 3.3.4 数据关联与更新
##### 3.3.4.1 新路标初始化
若观测到未关联路标：
$$
\boldsymbol{m}_\text{new} = \begin{bmatrix}
x + d \cos(\theta + \phi) \\
y + d \sin(\theta + \phi)
\end{bmatrix}, \quad \boldsymbol{z} = [d, \phi]^\top
$$
协方差扩展：
$$
\boldsymbol{P}_\text{new} = \boldsymbol{J} \begin{bmatrix} \boldsymbol{P}_{vv} & 0 \\ 0 & \boldsymbol{R} \end{bmatrix} \boldsymbol{J}^\top, \quad
\boldsymbol{J} = \begin{bmatrix} \frac{\partial \boldsymbol{m}_\text{new}}{\partial \boldsymbol{x}_v} & \frac{\partial \boldsymbol{m}_\text{new}}{\partial \boldsymbol{z}} \end{bmatrix}
$$

##### 3.3.4.2 卡尔曼更新
$$\begin{aligned}
\text{新息：} \quad \boldsymbol{\nu}_k^i &= \boldsymbol{z}_k^i - h(\hat{\boldsymbol{x}}_{k|k-1}) \\
\text{协方差：} \quad \boldsymbol{S}_k^i &= \boldsymbol{H}^i \boldsymbol{P}_{k|k-1} (\boldsymbol{H}^i)^\top + \boldsymbol{R}_k \\
\text{增益：} \quad \boldsymbol{K}_k^i &= \boldsymbol{P}_{k|k-1} (\boldsymbol{H}^i)^\top (\boldsymbol{S}_k^i)^{-1} \\
\text{更新：} \quad \hat{\boldsymbol{x}}_{k|k} &= \hat{\boldsymbol{x}}_{k|k-1} + \boldsymbol{K}_k^i \boldsymbol{\nu}_k^i \\
\boldsymbol{P}_{k|k} &= (\boldsymbol{I} - \boldsymbol{K}_k^i \boldsymbol{H}^i) \boldsymbol{P}_{k|k-1}
\end{aligned}$$

#### 3.3.4.5 误差分析
##### 3.3.4.5.1 线性化误差
| **误差源**         | **数学描述**                          | **影响**               |
|--------------------|--------------------------------------|------------------------|
| **一阶近似残差**   | $\|f(\boldsymbol{x}) - f(\hat{\boldsymbol{x}}) - \boldsymbol{F}_x \delta\boldsymbol{x}\|$ | 模型失真导致发散       |
| **泰勒展开截断**   | $\mathcal{O}(\|\delta\boldsymbol{x}\|^2)$ | 大初始误差时不稳定    |

##### 3.3.4.5.2 数据关联误差
**误关联概率模型**：
$$ P(\text{错误关联}) = 1 - \int_{\mathcal{Z}} p(\boldsymbol{z} | \text{正确}) d\boldsymbol{z} $$
其中 $\mathcal{Z} = \{ \boldsymbol{z} : \|\boldsymbol{z} - \hat{\boldsymbol{z}}_i\|_{\boldsymbol{S}^{-1}} < \tau \}$

**误差传播**：
- 单次误关联 → 位姿误差增长 $\|\delta\boldsymbol{x}\| \propto \|\boldsymbol{K}\| \cdot \|\boldsymbol{\nu}_\text{err}\|$
- 连续误关联 → 协方差矩阵失去正定性

##### 3.3.4.5.3 计算复杂度
$$
\mathcal{O}(n^3) \quad \text{(矩阵求逆)} \quad \xrightarrow{\text{稀疏性利用}} \mathcal{O}(n^{1.5})
$$
- **存储需求**：$\frac{1}{2}(3+2n)(4+2n)$ → 路标数$n$较大时不可行

##### 3.3.4.5.4 一致性分析
**NEES检验**：
$$
\epsilon_k = (\boldsymbol{x}_k - \hat{\boldsymbol{x}}_k)^\top \boldsymbol{P}_k^{-1} (\boldsymbol{x}_k - \hat{\boldsymbol{x}}_k) \sim \chi^2_{\dim(\boldsymbol{x})}
$$
若 $\mathbb{E}[\epsilon_k] > \dim(\boldsymbol{x})$ 则滤波器乐观

#### 3.3.4.6 性能提升技术
##### 3.3.4.6.1 稀疏化处理
**协方差矩阵近似**：
$$
\boldsymbol{P} \approx \begin{bmatrix}
\boldsymbol{P}_{vv} & \boldsymbol{P}_{vm} \\
\boldsymbol{P}_{vm}^\top & \text{blkdiag}(\boldsymbol{P}_{m_im_i})
\end{bmatrix}
$$
- 忽略路标间相关性 → 计算降至$\mathcal{O}(n)$

##### 3.3.4.6.2 分治策略
**局部子图构建**：
1. 创建短期局部子图：$\boldsymbol{x}^\text{local} = [\boldsymbol{x}_v, \boldsymbol{m}_{\text{active}}]^\top$
2. 子图内EKF-SLAM
3. 子图合并：$\boldsymbol{x}_\text{global} = g(\boldsymbol{x}_\text{global}, \boldsymbol{x}_\text{local})$

## 4. 视觉基础部分
### 4.1 简介
**参考文献** 视觉SLAM14讲
视觉部分实际上我是比较抵触的，原因是其中提到的优化算法以及IMU的使用。 \
优化算法部分一直认为算是在作弊，实质上就是前后向滤波，后一帧使用前后向滤波融合的值，本质上就是算力换精度。 \
IMU使用与误差估计以及，一直认为其在使用IMU上有较大的问题，使用预积分怎么会用好IMU？  \
个人吐槽，以上不算正式的说明，不过相较于IMU与GNSS，OD严格的公式推导过程，即观测量分析，误差分析，随后带入方程即可；SLAM给人一种比较高层次的思维方式，即建立一个问题的数学模型，随后通过解数学模型的方法去求得解释。

### 4.2 定位建图问题的数学建模
定位问题建模，假设载体的前一时刻运动状态为$\boldsymbol{x}_{k-1}$，控制信息为$\boldsymbol{u}_k$，控制传感器造成的方差以及运动模型方差为$\boldsymbol{w}_k$,则定位问题模型为
$$
\boldsymbol{x}_k=f\left(\boldsymbol{x}_{k-1},\boldsymbol{u}_k,\boldsymbol{w}_k\right)
$$
观测问题建模，假设载体在这一时刻采集到了其他的信息$\boldsymbol y_{j} $，产生了一个观测数据$\boldsymbol z_{k,j}$，观测值噪声为$\boldsymbol v_{k,j}$，则
$$
\boldsymbol{z}_{k,j}=h\left(\boldsymbol{y}_j,\boldsymbol{x}_k,\boldsymbol{v}_{k,j}\right)
$$
当然上述建模过程中特征点可以被描述为地标点。这就是视觉耍赖的地方，算力大和存储空间大，通过记住之前的消息来优化它自己的结果。

### 4.3 预备知识
#### 4.3.1 视觉如何看旋转变化
旋转的数学法则基本与惯性导航一致，有几点不一致。
* 四元数的定义不一样，严老师的定义是实部在后，视觉这个是实部在前。 \
* 使用了数学定义的群去定义旋转与变换，惯性导航中只有旋转。 \
可能使用EIGEN库较为方便，其一是不需要单元测试，该库工程实践很多年，没有bug；其二是一些矩阵计算较为方便，特别是多维矩阵计算。 \
但还是想使用自己实现的矩阵运算库，其一是内存好控制，其二是方便单元测试，其三是熟悉旋转的数学运算。
看实际工程怎么实现吧。

#### 4.3.2 问题描述
问题描述，载体的姿态为$T$，观察到世界坐标为$p$的特征点，产生了观测数据为$z$，那么由
$$ z=T\boldsymbol{p}+\boldsymbol{w} $$
则观测方程可以被写为，
$$ e=\boldsymbol{z}-\boldsymbol{Tp} $$
找到最优的姿态$T$，使得整体的误差最小。
$$
\min_{\boldsymbol{T}}J(\boldsymbol{T})=\sum_{i=1}^{N}\|\boldsymbol{z}_{i}-\boldsymbol{T}\boldsymbol{p}_{i}\|_{2}^{2}
$$
对$R$进行一次扰动$\Delta R$，查看结果相对于扰动的变化率，假设扰动的李代数为$\boldsymbol\varphi$。则，
$$
\frac{\partial\left(\boldsymbol{R}\boldsymbol{p}\right)}{\partial\boldsymbol{\varphi}}=\lim_{\boldsymbol{\varphi}\to\boldsymbol{0}}\frac{\exp\left(\boldsymbol{\varphi}^{\wedge}\right)\exp\left(\boldsymbol{\phi}^{\wedge}\right)\boldsymbol{p}-\exp\left(\boldsymbol{\phi}^{\wedge}\right)\boldsymbol{p}}{\boldsymbol{\varphi}} \\
\begin{aligned}\frac{\partial\left(Rp\right)}{\partial\varphi}&=\lim_{\varphi\to0}\frac{\exp\left(\varphi^{\wedge}\right)\exp\left(\phi^{\wedge}\right)p-\exp\left(\phi^{\wedge}\right)p}{\varphi}\\&=\lim_{\varphi\to0}\frac{\left(\boldsymbol{I}+\boldsymbol{\varphi}^\wedge\right)\exp\left(\boldsymbol{\phi}^\wedge\right)\boldsymbol{p}-\exp\left(\boldsymbol{\phi}^\wedge\right)\boldsymbol{p}}{\varphi}\\&=\lim_{\varphi\to0}\frac{\varphi^{\wedge}Rp}{\varphi}=\lim_{\varphi\to0}\frac{-(Rp)^{\wedge}\varphi}{\varphi}=-(Rp)^{\wedge}\end{aligned}
$$
对于变换，则有
$$
\frac{\partial\left(\boldsymbol{T}\boldsymbol{p}\right)}{\partial\delta\boldsymbol{\xi}}=\lim_{\delta\boldsymbol{\xi}\to\boldsymbol{0}}\frac{\exp\left(\delta\boldsymbol{\xi}^\wedge\right)\exp\left(\boldsymbol{\xi}^\wedge\right)\boldsymbol{p}-\exp\left(\boldsymbol{\xi}^\wedge\right)\boldsymbol{p}}{\delta\boldsymbol{\xi}} \\
\begin{aligned}&=\lim_{\delta\boldsymbol{\xi}\to\mathbf{0}}\frac{\left(\boldsymbol{I}+\delta\boldsymbol{\xi}^{\wedge}\right)\exp\left(\boldsymbol{\xi}^{\wedge}\right)\boldsymbol{p}-\exp\left(\boldsymbol{\xi}^{\wedge}\right)\boldsymbol{p}}{\delta\boldsymbol{\xi}}\\&=\lim_{\delta\boldsymbol{\xi}\to\mathbf{0}}\frac{\delta\boldsymbol{\xi}^{\wedge}\exp\left(\boldsymbol{\xi}^{\wedge}\right)\boldsymbol{p}}{\delta\boldsymbol{\xi}}\\&=\lim_{\delta\boldsymbol{\xi}\to\mathbf{0}}\frac{\begin{bmatrix}\delta\boldsymbol{\phi}^\wedge&\delta\boldsymbol{\rho}\\\mathbf{0}^\mathrm{T}&0\end{bmatrix}\begin{bmatrix}\boldsymbol{R}\boldsymbol{p}+\boldsymbol{t}\\\\1\end{bmatrix}}{\delta\boldsymbol{\xi}}\\&=\lim_{\delta\boldsymbol{\xi}\to\mathbf{0}}\frac{\begin{bmatrix}\delta\phi^\wedge\left(\boldsymbol{Rp}+\boldsymbol{t}\right)+\delta\boldsymbol{\rho}\\\\\mathbf{0}^\mathrm{T}\end{bmatrix}}{[\delta\boldsymbol{\rho},\delta\boldsymbol{\phi}]^\mathrm{T}}=\begin{bmatrix}\boldsymbol{I}&-\left(\boldsymbol{Rp}+\boldsymbol{t}\right)^\wedge\\\\\mathbf{0}^\mathrm{T}&\mathbf{0}^\mathrm{T}\end{bmatrix}\overset{\mathrm{def}}{\operatorname*{=}}\boldsymbol{Tp}\end{aligned}
$$
#### 4.3.3 对旋转矩阵进行数学优化，使其满足某种数学运算，该数学运算有利于进行数值优化
通过对旋转矩阵进行微分，得到旋转矩阵可以由旋转角度的反对称阵计算得到。
$$
\boldsymbol{R}(t)=\exp{(\boldsymbol{\phi}_0^{\wedge}t)}
$$
注意此时 t=0，且假设旋转角度与时间无关，为一常数。 \
即，旋转角度形成的反对称阵，可以用于表达旋转矩阵的微分。 \
仔细想想，符合直觉，旋转的变化就是角度变化引起的，单位时间的旋转矩阵变化量就是旋转角度的方程，现在求出了这个方程的定义。\
对于变化阵，也有
$$
\xi^{\wedge}=\begin{bmatrix}\phi^{\wedge}&\rho\\0^{\mathrm{T}}&0\end{bmatrix}\in\mathbb{R}^{4\times4}
$$
将以上两式展开，
$$
\exp\left(\phi^{\wedge}\right)=\exp\left(\theta\boldsymbol{a}^{\wedge}\right)=\sum_{n=0}^{\infty}\frac{1}{n!}\left(\theta\boldsymbol{a}^{\wedge}\right)^n  \\
= \begin{aligned}&=\boldsymbol{I}+\theta\boldsymbol{a}^{\wedge}+\frac{1}{2!}\theta^{2}\boldsymbol{a}^{\wedge}\boldsymbol{a}^{\wedge}+\frac{1}{3!}\theta^{3}\boldsymbol{a}^{\wedge}\boldsymbol{a}^{\wedge}\boldsymbol{a}^{\wedge}+\frac{1}{4!}\theta^{4}(\boldsymbol{a}^{\wedge})^{4}+\cdots\\&=\boldsymbol{a}\boldsymbol{a}^\mathrm{T}-\boldsymbol{a}^\mathrm{\wedge}\boldsymbol{a}^\mathrm{\wedge}+\theta\boldsymbol{a}^\mathrm{\wedge}+\frac{1}{2!}\theta^2\boldsymbol{a}^\mathrm{\wedge}\boldsymbol{a}^\mathrm{\wedge}-\frac{1}{3!}\theta^3\boldsymbol{a}^\mathrm{\wedge}-\frac{1}{4!}\theta^4{(\boldsymbol{a}^\mathrm{\wedge})}^2+\cdots\\&=\boldsymbol{aa}^{\mathrm{T}}+\underbrace{\left(\theta-\frac{1}{3!}\theta^{3}+\frac{1}{5!}\theta^{5}-\cdots\right)}_{\sin\theta}\boldsymbol{a}^{\wedge}-\underbrace{\left(1-\frac{1}{2!}\theta^{2}+\frac{1}{4!}\theta^{4}-\cdots\right)}_{\cos\theta}\boldsymbol{a}^{\wedge}\boldsymbol{a}^{\wedge}\\&=a^\wedge a^\wedge+I+\sin\theta\boldsymbol{a}^\wedge-\cos\theta\boldsymbol{a}^\wedge\boldsymbol{a}^\wedge\\&=(1-\cos\theta)\boldsymbol{a}^\wedge\boldsymbol{a}^\wedge+\boldsymbol{I}+\sin\theta\boldsymbol{a}^\wedge\\&=\cos\theta\boldsymbol{I}+(1-\cos\theta)\boldsymbol{aa}^\mathrm{T}+\sin\theta\boldsymbol{a}^\mathrm{\wedge}.\end{aligned}
$$
即
$$
\phi=\ln\left(\boldsymbol{R}\right)^\vee=\left(\sum_{n=0}^\infty\frac{\left(-1\right)^n}{n+1}\left(\boldsymbol{R}-\boldsymbol{I}\right)^{n+1}\right)^\vee
$$
展开，
$$
\begin{aligned}\exp\left(\xi^{\wedge}\right)&=\begin{bmatrix}\sum_{n=0}^{\infty}\frac{1}{n!}(\phi^{\wedge})^{n}&\sum_{n=0}^{\infty}\frac{1}{(n+1)!}(\phi^{\wedge})^{n}\rho\\\\\mathbf{0}^{\mathrm{T}}&1\end{bmatrix}\\&\overset{\Delta}{\operatorname*{=}}\begin{bmatrix}R&J\rho\\\\0^\mathrm{T}&1\end{bmatrix}=T.\end{aligned}
$$
$$
\boldsymbol{J}=\frac{\sin\theta}{\theta}\boldsymbol{I}+\left(1-\frac{\sin\theta}{\theta}\right)\boldsymbol{a}\boldsymbol{a}^\mathrm{T}+\frac{1-\cos\theta}{\theta}\boldsymbol{a}^\mathrm{^{\prime}}
$$
考虑在一个姿态矩阵中，有一个微小的扰动，则此时对于李代数而言，
$$
\exp\left(\Delta\phi^{\wedge}\right)\exp\left(\phi^{\wedge}\right)=\exp\left(\left(\phi+J_{l}^{-1}\left(\phi\right)\Delta\phi\right)^{\wedge}\right) \\
\exp\left(\Delta\boldsymbol{\xi}^{\wedge}\right)\exp\left(\boldsymbol{\xi}^{\wedge}\right)\approx\exp\left(\left(\boldsymbol{J}_{l}^{-1}\Delta\boldsymbol{\xi}+\boldsymbol{\xi}\right)^{\wedge}\right),\exp\left(\boldsymbol{\xi}^{\wedge}\right)\exp\left(\Delta\boldsymbol{\xi}^{\wedge}\right)\approx\exp\left(\left(\boldsymbol{J}_{r}^{-1}\Delta\boldsymbol{\xi}+\boldsymbol{\xi}\right)^{\wedge}\right)
$$

#### 4.3.4 相机模型建立
##### 4.3.4.1 单目模型的成像过程以及数学模型的建立
假设世界坐标系有一个固定点P，世界坐标为$P_w$。 \
由于相机在运动，该运动可由旋转矩阵$R$和位移向量$t$描述。则世界坐标在相机为原点的坐标系下是$\hat P_c = RP_w + t$。 \
将其投影在归一化平面$Z = 1$，得到归一化坐标，$P_c = X/Z,Y/Z,1$\
有畸变的时候，根据畸变参数计算发生畸变后的坐标。 \
最后得到像素坐标，乘以内参，$P_{uv} = KP_c$

##### 4.3.4.2 双目模型的成像过程以及数学模型的建立
考虑一个空间点P，在左眼成像$P_L$，右眼成像$P_R$，由于基线存在，成像位置是不同的。 \
假设左右相机只在x轴上有位移，也就是u轴上有差异。左侧坐标为$u_L$，右侧坐标为$u_R$。 \
则有
$$
\frac{z-f}{z}=\frac{b-u_L+u_R}{b}
$$
进一步，有
$$
z=\frac{fb}{d},\quad d\triangleq u_{L}-u_{R}
$$

#### 4.3.5 非线性优化问题模型的建立
考虑到运动方程以及观测方程，
$$
\left.\left\{\begin{array}{l}\boldsymbol{x}_k=f\left(\boldsymbol{x}_{k-1},\boldsymbol{u}_k\right)+\boldsymbol{w}_k\\\boldsymbol{z}_{k,j}=h\left(\boldsymbol{y}_j,\boldsymbol{x}_k\right)+\boldsymbol{v}_{k,j}\end{array}\right.\right.
$$
假设在$\boldsymbol{x}_k$出在路标$\boldsymbol{y}_j$进行了一次观测，对应像素位置$boldsymbol{z}_{k,j}$，那么
$$
s\boldsymbol{z}_{k,j}=K(\boldsymbol{R}_k\boldsymbol{y}_j+t_k)
$$
并假设噪声为白噪声，则，
$$
w_k\sim\mathcal{N}\left(\mathbf{0},R_k\right),\boldsymbol{v}_k\sim\mathcal{N}\left(\mathbf{0},\boldsymbol{Q}_{k,j}\right)
$$
我们希望通过$z$和$u$推断位姿$x$和地图$y$，构成了状态估计问题。

#### 4.3.6 非线性优化引出
在1~N时刻，有M个路标点，定义位姿与路标点坐标为，
$$
x=\{x_1,\ldots,x_N\},\quad y=\{y_1,\ldots,y_M\}
$$
从概率学观点重新看，在已知输入数据$u$和观测数据$z$的条件下，求状态$x,y$的条件概率分布，
$$P((x,y)|(z,u))$$
当$u$未知时，即有$P((x,y)|(z))$ \
为估计状态量的条件分布，则有贝叶斯法则，有
$$
P\left(\boldsymbol{x},\boldsymbol{y}|\boldsymbol{z},\boldsymbol{u}\right)=\frac{P\left(\boldsymbol{z},\boldsymbol{u}|\boldsymbol{x},\boldsymbol{y}\right)P\left(\boldsymbol{x},\boldsymbol{y}\right)}{P\left(\boldsymbol{z},\boldsymbol{u}\right)}\propto\underbrace{P\left(\boldsymbol{z},\boldsymbol{u}|\boldsymbol{x},\boldsymbol{y}\right)}_{\text{似然}}\underbrace{P\left(\boldsymbol{x},\boldsymbol{y}\right)}_{\text{先验}}
$$
需要求得一个状态最优估计，使得该状态下后验概率最大化，
$$
(x,y)_{\mathrm{MAP}}^*=\arg\max P\left(x,y|z,u\right)=\arg\max P(z,u|x,y)P(x,y)
$$
在不知道机器人位姿和路标的位置时，可以求解最大似然估计。
$$
(\boldsymbol{x},\boldsymbol{y})_{\mathrm{MLE}}^*=\arg\max P(\boldsymbol{z},\boldsymbol{u}|\boldsymbol{x},\boldsymbol{y})
$$
现在的问题是求解上式的最大似然估计时，$x_k$的值， \
对于某次观测，有
$$
z_{k,j}=h\left(\boldsymbol{y}_j,\boldsymbol{x}_k\right)+\boldsymbol{v}_{k,j}
$$
则观测数据的条件概率为
$$
P(\boldsymbol{z}_{j,k}|\boldsymbol{x}_k,\boldsymbol{y}_j)=N\left(h(\boldsymbol{y}_j,\boldsymbol{x}_k),\boldsymbol{Q}_{k,j}\right)
$$
它依旧是一个高斯分布，使用最小化负对数的方法求解高斯分布的最大似然。
$$
P\left(\boldsymbol{x}\right)=\frac{1}{\sqrt{\left(2\pi\right)^{N}\det(\boldsymbol{\Sigma})}}\exp\left(-\frac{1}{2}(\boldsymbol{x}-\boldsymbol{\mu})^{\mathrm{T}}\boldsymbol{\Sigma}^{-1}\left(\boldsymbol{x}-\boldsymbol{\mu}\right)\right)
$$
$$
\left.-\ln\left(P\left(\boldsymbol{x}\right)\right)=\frac{1}{2}\ln\left(\left(2\pi\right)\right|^{N}\det\left(\boldsymbol{\Sigma}\right)\right)+\frac{1}{2}\left(\boldsymbol{x}-\boldsymbol{\mu}\right)^{\mathrm{T}}\boldsymbol{\Sigma}^{-1}\left(\boldsymbol{x}-\boldsymbol{\mu}\right)
$$
看上式，求上式的最小值，第一项与$x_k$无关，则求第二项的最小值。即
$$
\begin{aligned}(\boldsymbol{x}_k,\boldsymbol{y}_j)^*&=\arg\max\mathcal{N}(h(\boldsymbol{y}_j,\boldsymbol{x}_k),\boldsymbol{Q}_{k,j})\\&=\arg\min\left(\left(\boldsymbol{z}_{k,j}-h\left(\boldsymbol{x}_{k},\boldsymbol{y}_{j}\right)\right)^{\mathrm{T}}\boldsymbol{Q}_{k,j}^{-1}\left(\boldsymbol{z}_{k,j}-h\left(\boldsymbol{x}_{k},\boldsymbol{y}_{j}\right)\right)\right)\end{aligned}
$$
重新回到求解最大似然估计部分，有 
$$
P\left(\boldsymbol{z},\boldsymbol{u}|\boldsymbol{x},\boldsymbol{y}\right)=\prod_{k}P\left(\boldsymbol{u}_{k}|\boldsymbol{x}_{k-1},\boldsymbol{x}_{k}\right)\prod_{k,j}P\left(\boldsymbol{z}_{k,j}|\boldsymbol{x}_{k},\boldsymbol{y}_{j}\right)
$$
定义各次输入与观测数据与模型之间的误差为，
$$
e_{u,k}=x_{k}-f\left(\boldsymbol{x}_{k-1},\boldsymbol{u}_{k}\right)e_{z,j,k}=z_{k,j}-h\left(\boldsymbol{x}_{k},\boldsymbol{y}_{j}\right)
$$
则变为，
$$
\min J(x,y)=\sum_{k}e_{u,k}^{\mathrm{T}}R_{k}^{-1}e_{u,k}+\sum_{k}\sum_{j}e_{z,k,j}^{\mathrm{T}}Q_{k,j}^{-1}e_{z,k,j}
$$

### 4.4 视觉里程计--特征点法 前端实时
使用openCV获取特征点以及特征匹配。 \
特征点使用 FAST 角点，描述子为 BRIEF。 \
匹配算法是快速近似最近邻法。

### 4.5 2D-2D 对极几何
$$
x_2^\mathrm{T}t^\wedge Rx_1=0 \\
p_2^\mathrm{T}K^\mathrm{-T}t^\wedge RK^{-1}p_1=0
$$
有对极约束，
$$
E=t^\wedge R,\quad F=K^{-\mathrm{T}}EK^{-1},\quad x_2^\mathrm{T}Ex_1=p_2^\mathrm{T}Fp_1=0
$$
当有八个特征点时，有
$$
\begin{pmatrix}u_2,v_2,1\end{pmatrix}\begin{pmatrix}e_1&e_2&e_3\\\\e_4&e_5&e_6\\\\e_7&e_8&e_9\end{pmatrix}\begin{pmatrix}u_1\\\\v_1\\\\1\end{pmatrix}=0
$$
将矩阵展开，
$$
e=[e_1,e_2,e_3,e_4,e_5,e_6,e_7,e_8,e_9]^\mathrm{T}
$$
则有
$$
\begin{pmatrix}u_2^1u_1^1&u_2^1v_1^1&u_2^1&v_2^1u_1^1&v_2^1v_1^1&v_2^1&u_1^1&v_1^1&1\\u_2^2u_1^2&u_2^2v_1^2&u_2^2&v_2^2u_1^2&v_2^2v_1^2&v_2^2&u_1^2&v_1^2&1\\\vdots&\vdots&\vdots&\vdots&\vdots&\vdots&\vdots&\vdots\\u_2^8u_1^8&u_2^8v_1^8&u_2^8&v_2^8u_1^8&v_2^8v_1^8&v_2^8&u_1^8&v_1^8&1\end{pmatrix}\begin{pmatrix}e_1\\\\e_2\\\\e_3\\\\e_4\\\\e_5\\\\e_6\\\\e_7\\\\e_8\\\\e_9\end{pmatrix}
$$
设E的SVD分解为
$$
E=U\Sigma V^\mathrm{T}
$$
则t与R可以被计算为
$$
\begin{aligned}&t_{1}^{\wedge}=UR_{Z}(\frac{\pi}{2})\Sigma U^{\mathrm{T}},\quad R_{1}=UR_{Z}^{\mathrm{T}}(\frac{\pi}{2})V^{\mathrm{T}}\\&t_{2}^{\wedge}=UR_{Z}(-\frac{\pi}{2})\Sigma U^{\mathrm{T}},\quad R_{2}=UR_{Z}^{\mathrm{T}}(-\frac{\pi}{2})V^{\mathrm{T}}\end{aligned}
$$
将P点带入相机中，查看深度是否都为正，都为正的即为正确解。 \
为了防止E的病态，重新构造E矩阵
$$
E=U\mathrm{diag}(\frac{\sigma_1+\sigma_2}{2},\frac{\sigma_1+\sigma_2}{2},0)\boldsymbol{V}^\mathrm{T}
$$

在相机在统一平面旋转时，例如无人机俯视过程中，则有
$$
n^\mathrm{T}P+d=0
$$
$$
h_1u_1+h_2v_1+h_3-h_7u_1u_2-h_8v_1u_2=u_2 \\
h_4u_1+h_5v_1+h_6-h_7u_1v_2-h_8v_1v_2=v_2.
$$
在有八个特征点时，有
$$
\begin{pmatrix}u_1^1&v_1^1&1&0&0&0&-u_1^1u_2^1&-v_1^1u_2^1\\0&0&0&u_1^1&v_1^1&1&-u_1^1v_2^1&-v_1^1v_2^1\\u_1^2&v_1^2&1&0&0&0&-u_1^2u_2^2&-v_1^2u_2^2\\0&0&0&u_1^2&v_1^2&1&-u_1^2v_2^2&-v_1^2v_2^2\\u_1^3&v_1^3&1&0&0&0&-u_1^3u_2^3&-v_1^3u_2^3\\0&0&0&u_1^3&v_1^3&1&-u_1^3v_2^3&-v_1^3v_2^3\\u_1^4&v_1^4&1&0&0&0&-u_1^4u_2^4&-v_1^4u_2^4\\0&0&0&u_1^4&v_1^4&1&-u_1^4v_2^4&-v_1^4v_2^4\end{pmatrix}\begin{pmatrix}h_1\\h_2\\h_2\\h_3\\h_4\\h_5\\h_6\\h_7\\h_8\end{pmatrix}=\begin{pmatrix}u_2^1\\v_2^1\\u_2^2\\v_2^2\\u_2^3\\u_2^4\\v_2^4\\v_2^4\end{pmatrix}
$$

三角化法获得深度 \
$$
s_1x_1=s_2Rx_2+t
$$
$$
s_1x_1^\wedge x_1=0=s_2x_1^\wedge Rx_2+x_1^\wedge t
$$


### 4.5 3D-2D PnP
使用openCV 或 图优化
$$
s_i\boldsymbol{u}_i=K\exp\left(\boldsymbol{\xi}^\wedge\right)\boldsymbol{P}_i  \\
\xi^*=\arg\min_{\xi}\frac{1}{2}\sum_{i=1}^{n}\left\|u_{i}-\frac{1}{s_{i}}K\exp\left(\xi^{\wedge}\right)\boldsymbol{P}_{i}\right\|_{2}^{2}
$$
得到重投影关于李代数的一阶变化 \\
$$
\frac{\partial e}{\partial\delta\boldsymbol{\xi}}=-\begin{bmatrix}\frac{f_x}{Z^{\prime}}&0&-\frac{f_xX^{\prime}}{Z^{\prime2}}&-\frac{f_xX^{\prime}Y^{\prime}}{Z^{\prime2}}&f_x+\frac{f_xX^2}{Z^{\prime2}}&-\frac{f_xY^{\prime}}{Z^{\prime}}\\0&\frac{f_y}{Z^{\prime}}&-\frac{f_yY^{\prime}}{Z^{\prime2}}&-f_y-\frac{f_yY^{\prime2}}{Z^{\prime2}}&\frac{f_yX^{\prime}Y^{\prime}}{Z^{\prime2}}&\frac{f_yX^{\prime}}{Z^{\prime}}\end{bmatrix} \\
\left.\frac{\partial e}{\partial P}=-\left[\begin{array}{ccc}\frac{f_x}{Z^{\prime}}&0&-\frac{f_xX^{\prime}}{Z^{\prime2}}\\0&\frac{f_y}{Z^{\prime}}&-\frac{f_yY^{\prime}}{Z^{\prime2}}\end{array}\right.\right]R
$$


### 4.6 3D-3D ICP
$$
\min_{R,t}J=\frac{1}{2}\sum_{i=1}^{n}\left\|p_{i}-p-R\left(p_{i}^{\prime}-p^{\prime}\right)\right\|^{2}+\left\|p-Rp^{\prime}-t\right\|^{2} \\
\frac{1}{2}\sum_{i=1}^{n}\left\|q_{i}-Rq_{i}^{\prime}\right\|^{2}=\frac{1}{2}\sum_{i=1}^{n}q_{i}^{\mathrm{T}}q_{i}+q_{i}^{\prime\mathrm{T}}R^{\mathrm{T}}Rq_{i}^{\prime}-2q_{i}^{\mathrm{T}}Rq_{i}^{\prime}. \\
\sum_{i=1}^n-q_i^\mathrm{T}Rq_i^{\prime}=\sum_{i=1}^n-\mathrm{tr}\left(Rq_i^{\prime}q_i^\mathrm{T}\right)=-\mathrm{tr}\left(R\sum_{i=1}^nq_i^{\prime}q_i^\mathrm{T}\right) \\
W=\sum_{i=1}^nq_iq_i^{\prime\mathrm{T}} \\
W=U\Sigma V^\mathrm{T} \\
sort(\Sigma) max->min \\
R=UV^\mathrm{T} 
$$
使用SVD 或 图优化

## 4.7 视觉里程计 -- 稀疏光流法 前端实时
在一个t时刻，位于(x,y)处的像素，它的灰度为$I(x,y,t)$，假设同一个空间的像素灰度值在不同的图像中是固定不变的。那么就有，
$$
I(x+\mathrm{d}x,y+\mathrm{d}y,t+\mathrm{d}t)=I(x,y,t)
$$
展开，保留一阶项，
$$
I\left(x+\mathrm{d}x,y+\mathrm{d}y,t+\mathrm{d}t\right)\approx I\left(x,y,t\right)+\frac{\partial\boldsymbol{I}}{\partial x}\mathrm{d}x+\frac{\partial\boldsymbol{I}}{\partial u}\mathrm{d}y+\frac{\partial\boldsymbol{I}}{\partial t}\mathrm{d}t \\
\frac{\partial\boldsymbol{I}}{\partial x}\mathrm{d}x+\frac{\partial\boldsymbol{I}}{\partial y}\mathrm{d}y+\frac{\partial\boldsymbol{I}}{\partial t}\mathrm{d}t=0
$$
两边同时除以$dt$，
$$
\frac{\partial I}{\partial x}\frac{\mathrm{d}x}{\mathrm{d}t}+\frac{\partial I}{\partial y}\frac{\mathrm{d}y}{\mathrm{d}t}=-\frac{\partial I}{\partial t}
$$
于是有，
$$
\begin{bmatrix}I_x&I_y\end{bmatrix}\begin{bmatrix}u\\\\v\end{bmatrix}=-I_t
$$
假设，一个窗口$w$内的所有像素具有相同的变化。
$$
\begin{bmatrix}\boldsymbol{I}_x&\boldsymbol{I}_y\end{bmatrix}_k\begin{bmatrix}u\\\\v\end{bmatrix}=-\boldsymbol{I}_{tk},\quad k=1,\ldots,w^2 \\
\boldsymbol{A}=\begin{bmatrix}\left[\boldsymbol{I}_x,\boldsymbol{I}_y\right]_1\\\vdots\\\\\left[\boldsymbol{I}_x,\boldsymbol{I}_y\right]_k\end{bmatrix} \\
\boldsymbol{b}=\begin{bmatrix}\boldsymbol{I}_{t1}\\\vdots\\\\\boldsymbol{I}_{tk}\end{bmatrix}
$$
于是
$$
A\begin{bmatrix}u\\\\v\end{bmatrix}=-b \\
\begin{bmatrix}u\\\\v\end{bmatrix}^*=-\left(A^\mathrm{T}A\right)^{-1}A^\mathrm{T}b
$$
从优化的角度来讲，有
$$
\min_{\Delta x,\Delta y}\left\|\boldsymbol{I}_1\left(x,y\right)-\boldsymbol{I}_2\left(x+\Delta x,y+\Delta y\right)\right\|_2^2
$$

## 4.8 视觉里程计 -- 直接法 前端实时
有像素点与空间点的方程
$$
\boldsymbol{p}_1=\begin{bmatrix}u\\\\v\\\\1\end{bmatrix}_1=\frac{1}{Z_1}\boldsymbol{KP} \\
\boldsymbol{p}_2=\begin{bmatrix}u\\\\v\\\\1\end{bmatrix}_2=\frac{1}{Z_2}\boldsymbol{K}\left(\boldsymbol{R}\boldsymbol{P}+\boldsymbol{t}\right)=\frac{1}{Z_2}\boldsymbol{K}\left(\boldsymbol{T}\boldsymbol{P}\right)_{1:3}
$$
假设我们现在有一个位姿估计，找到了另一个位置下的像素点，我们要找到一个位姿计算两个像素点灰度值的差最小。
$$
e=\boldsymbol{I}_1\left(\boldsymbol{p}_1\right)-\boldsymbol{I}_2\left(\boldsymbol{p}_2\right) \\
\min_{\boldsymbol{T}}J\left(\boldsymbol{T}\right)=\sum_{i=1}^{N}e_{i}^{\mathrm{T}}e_{i},\quad e_{i}=\boldsymbol{I}_{1}\left(\boldsymbol{p}_{1,i}\right)-\boldsymbol{I}_{2}\left(\boldsymbol{p}_{2,i}\right)
$$
可以推导出优化的雅可比矩阵
$$
q=TP \\
u=\frac{1}{Z_2}Kq \\
e(T)=I_1(p_1)-I_2(u) \\
\frac{\partial e}{\partial\boldsymbol{T}}=\frac{\partial\boldsymbol{I}_2}{\partial\boldsymbol{u}}\frac{\partial\boldsymbol{u}}{\partial\boldsymbol{q}}\frac{\partial\boldsymbol{q}}{\partial\delta\boldsymbol{\xi}}\delta\boldsymbol{\xi} \\
\frac{\partial\boldsymbol{u}}{\partial\boldsymbol{q}}=\begin{bmatrix}\frac{\partial u}{\partial X}&\frac{\partial u}{\partial Y}&\frac{\partial u}{\partial Z}\\\frac{\partial v}{\partial X}&\frac{\partial v}{\partial Y}&\frac{\partial v}{\partial Z}\end{bmatrix}=\begin{bmatrix}\frac{f_x}{Z}&0&-\frac{f_xX}{Z^2}\\\\0&\frac{f_y}{Z}&-\frac{f_yY}{Z^2}\end{bmatrix} \\
\frac{\partial\boldsymbol{q}}{\partial\delta\boldsymbol{\xi}}=[I,-\boldsymbol{q}^{\wedge}] 
$$
将后两项组合，有
$$
\frac{\partial\boldsymbol{u}}{\partial\delta\boldsymbol{\xi}}=\begin{bmatrix}\frac{f_x}{Z}&0&-\frac{f_xX}{Z^2}&-\frac{f_xXY}{Z^2}&f_x+\frac{f_xX^2}{Z^2}&-\frac{f_xY}{Z}\\0&\frac{f_y}{Z}&-\frac{f_yY}{Z^2}&-f_y-\frac{f_yY^2}{Z^2}&\frac{f_yXY}{Z^2}&\frac{f_yX}{Z}\end{bmatrix} \\
J=-\frac{\partial\boldsymbol{I}_2}{\partial\boldsymbol{u}}\frac{\partial\boldsymbol{u}}{\partial\delta\boldsymbol{\xi}} 
$$


## 4.9 后端优化 建图
重述运动方程与观测方程，
$$
\begin{cases}\boldsymbol{x}_k=f\left(\boldsymbol{x}_{k-1},\boldsymbol{u}_k\right)+\boldsymbol{w}_k\\\boldsymbol{z}_{k,j}=h\left(\boldsymbol{y}_j,\boldsymbol{x}_k\right)+\boldsymbol{v}_{k,j}&\end{cases}\quad k=1,\ldots,N,j=1,\ldots,M
$$
假设，令$x_k$为k时刻的所有未知量，包含了当前相机的位姿与m个路标点，
$$
\boldsymbol{x}_k\overset{\mathrm{def}}{\operatorname*{\operatorname*{=}}}\{\boldsymbol{x}_k,\boldsymbol{y}_1,\ldots,\boldsymbol{y}_m\}
$$
则有
$$
\begin{cases}\boldsymbol{x}_k=f\left(\boldsymbol{x}_{k-1},\boldsymbol{u}_k\right)+\boldsymbol{w}_k\\\boldsymbol{z}_k=h\left(\boldsymbol{x}_k\right)+\boldsymbol{v}_k&\end{cases}
$$
我们希望用过去0到k时刻的数据估计现在的概率分布，
$$
P(x_k|x_0,u_{1:k},z_{1:k})
$$
将$z_k$与$x_k$交换位置，
$$
P\left(\boldsymbol{x}_k|\boldsymbol{x}_0,\boldsymbol{u}_{1:k},\boldsymbol{z}_{1:k}\right)\propto P\left(\boldsymbol{z}_k|\boldsymbol{x}_k\right)P\left(\boldsymbol{x}_k|\boldsymbol{x}_0,\boldsymbol{u}_{1:k},\boldsymbol{z}_{1:k-1}\right)
$$
在$x_{k-1}$时刻展开
$$
P\left(\boldsymbol{x}_k|\boldsymbol{x}_0,\boldsymbol{u}_{1:k},\boldsymbol{z}_{1:k-1}\right)=\int P\left(\boldsymbol{x}_k|\boldsymbol{x}_{k-1},\boldsymbol{x}_0,\boldsymbol{u}_{1:k},\boldsymbol{z}_{1:k-1}\right)P\left(\boldsymbol{x}_{k-1}|\boldsymbol{x}_0,\boldsymbol{u}_{1:k},\boldsymbol{z}_{1:k-1}\right)\mathrm{d}\boldsymbol{x}_{k-1}
$$
对于每次观测，有误差
$$
e=z-h(T,p)
$$
则多个观测量下，整体的代价函数为，
$$
\frac{1}{2}\sum_{i=1}^m\sum_{j=1}^n\|\boldsymbol{e}_{ij}\|^2=\frac{1}{2}\sum_{i=1}^m\sum_{j=1}^n\|\boldsymbol{z}_{ij}-h(\boldsymbol{T}_i,\boldsymbol{p}_j)\|^2
$$
针对整体的代价函数，定义自变量为
$$
\boldsymbol{x}=[T_1,\ldots,T_m,\boldsymbol{p}_1,\ldots,\boldsymbol{p}_n]^\mathrm{T}
$$
则目标函数为，
$$
\frac{1}{2}\left\|f(\boldsymbol{x}+\Delta\boldsymbol{x})\right\|^2\approx\frac{1}{2}\sum_{i=1}^m\sum_{j=1}^n\left\|\boldsymbol{e}_{ij}+\boldsymbol{F}_{ij}\Delta\boldsymbol{\xi}_i+\boldsymbol{E}_{ij}\Delta\boldsymbol{p}_j\right\|^2
$$
将位姿变量放在一起，空间点的变量也放在一起，
$$
\boldsymbol{x}_\mathrm{c}=[\boldsymbol{\xi}_1,\boldsymbol{\xi}_2,\ldots,\boldsymbol{\xi}_m]^\mathrm{T}\in\mathbb{R}^{6m} \\
\boldsymbol{x}_p=[\boldsymbol{p}_1,\boldsymbol{p}_2,\ldots,\boldsymbol{p}_n]^\mathrm{T}\in\mathbb{R}^{3n}
$$
则目标函数为
$$
\frac{1}{2}\left\|f(\boldsymbol{x}+\Delta\boldsymbol{x})\right\|^2=\frac{1}{2}\left\|\boldsymbol{e}+\boldsymbol{F}\Delta\boldsymbol{x}_c+\boldsymbol{E}\Delta\boldsymbol{x}_p\right\|^2
$$
此时J为
$$
J=[F \ E]
$$
因为雅可比矩阵非常大，所以针对其性质做数学上的变化。
$$
H=J^\mathrm{T}J=\begin{bmatrix}F^\mathrm{T}F&F^\mathrm{T}E\\\\E^\mathrm{T}F&E^\mathrm{T}E\end{bmatrix}
$$
在图论中，以点与与边形成的邻接矩阵与H阵表现一致。 \
于是得到线性方程组，
$$
\begin{bmatrix}B&E\\E^\mathrm{T}&C\end{bmatrix}\begin{bmatrix}\Delta x_\mathrm{c}\\\\\Delta x_p\end{bmatrix}=\begin{bmatrix}v\\\\w\end{bmatrix}
$$
将E消去，
$$
\begin{bmatrix}I&-EC^{-1}\\\\0&I\end{bmatrix}\begin{bmatrix}B&E\\\\E^{\mathrm{T}}&C\end{bmatrix}\begin{bmatrix}\Delta x_\mathrm{c}\\\\\Delta x_p\end{bmatrix}=\begin{bmatrix}I&-EC^{-1}\\\\0&I\end{bmatrix}\begin{bmatrix}v\\\\w\end{bmatrix}
$$
整理，得
$$
\begin{bmatrix}B-EC^{-1}E^\mathrm{T}&0\\\\E^\mathrm{T}&C\end{bmatrix}\begin{bmatrix}\Delta x_\mathrm{c}\\\\\Delta x_p\end{bmatrix}=\begin{bmatrix}v-EC^{-1}w\\\\w\end{bmatrix}
$$
则可以先固定位姿部分的增量方程：
$$
\begin{bmatrix}B-EC^{-1}E^\mathrm{T}\end{bmatrix}\Delta x_\mathrm{c}=v-EC^{-1}w
$$
随后，路标部分有
$$
\Delta\boldsymbol{x}_p=\boldsymbol{C}^{-1}(\boldsymbol{w}-\boldsymbol{E}^\mathrm{T}\Delta\boldsymbol{x}_\mathrm{c})
$$
实际上相当于
$$
P(\boldsymbol{x_\mathrm{c}},\boldsymbol{x_p})=P(\boldsymbol{x_\mathrm{c}}|\boldsymbol{x_p})P(\boldsymbol{x_p})
$$

## 4.9 滑动窗口优化 建图 (当前阶段认为与全局优化差不多，公式没复制全，先空着)
$$
p\left(\boldsymbol{x}_1,\ldots\boldsymbol{x}_4,\boldsymbol{y}_1,\ldots\boldsymbol{y}_6\right)=p\left(\boldsymbol{x}_2,\ldots,\boldsymbol{x}_4,\boldsymbol{y}_1,\ldots\boldsymbol{y}_6|\boldsymbol{x}_1\right)\underbrace{p\left(\boldsymbol{x}_1\right)}_{\text{舍去}}
$$
则可优化为
$$
\Delta\boldsymbol{\xi}_{ij}=\boldsymbol{\xi}_i^{-1}\circ\boldsymbol{\xi}_j=\ln\left(\boldsymbol{T}_i^{-1}\boldsymbol{T}_j\right)^\vee
$$
则误差方程为
$$
e_{ij}=\Delta\xi_{ij}\ln\left(T_{ij}^{-1}T_{i}^{-1}T_{j}\right)^{\vee}
$$
通过叽里咕噜的转换，得到
$$
\begin{aligned}\hat{e}_{ij}&=\ln\left(\boldsymbol{T}_{ij}^{-1}\boldsymbol{T}_i^{-1}\exp((-\boldsymbol{\delta}\boldsymbol{\xi}_i)^{\wedge})\exp(\delta\boldsymbol{\xi}_j^{\wedge})\boldsymbol{T}_j\right)^{\vee}\\&=\ln\left(T_{ij}^{-1}T_{i}^{-1}T_{j}\exp\left(\left(-\mathrm{Ad}(T_{j}^{-1})\delta\xi_{i}\right)^{\wedge}\right)\exp\left(\left(\mathrm{Ad}(T_{j}^{-1})\delta\xi_{j}\right)^{\wedge}\right)\right)^{\vee}\\&\approx\ln\left(\boldsymbol{T}_{ij}^{-1}\boldsymbol{T}_i^{-1}\boldsymbol{T}_j\left[\boldsymbol{I}-(\mathrm{Ad}(\boldsymbol{T}_j^{-1})\boldsymbol{\delta}\boldsymbol{\xi}_i)^{\wedge}+(\mathrm{Ad}(\boldsymbol{T}_j^{-1})\boldsymbol{\delta}\boldsymbol{\xi}_j)^{\wedge}\right]\right)^\vee\\&\approx\boldsymbol{e}_{ij}+\frac{\partial\boldsymbol{e}_{ij}}{\partial\boldsymbol{\delta}\boldsymbol{\xi}_{i}}\boldsymbol{\delta}\boldsymbol{\xi}_{i}+\frac{\partial\boldsymbol{e}_{ij}}{\partial\boldsymbol{\delta}\boldsymbol{\xi}_{i}}\boldsymbol{\delta}\boldsymbol{\xi}_{j}\end{aligned} \\
\frac{\partial\boldsymbol{e}_{ij}}{\partial\boldsymbol{\delta\xi}_i}=-\boldsymbol{J}_r^{-1}(\boldsymbol{e}_{ij})\mathrm{Ad}(\boldsymbol{T}_j^{-1}) \\
\frac{\partial\boldsymbol{e}_{ij}}{\partial\boldsymbol{\delta}\boldsymbol{\xi}_{i}}=\boldsymbol{J}_{r}^{-1}(\boldsymbol{e}_{ij})\mathrm{Ad}(\boldsymbol{T}_{j}^{-1}) \\
\mathcal{J}_r^{-1}(e_{ij})\approx I+\frac{1}{2}\begin{bmatrix}\phi_e^\wedge&\rho_e^\wedge\\0&\phi_e^\wedge\end{bmatrix}
$$
总体的目标函数为
$$
\min\frac{1}{2}\sum_{i,j\in\mathcal{E}}e_{ij}^\mathrm{T}\Sigma_{ij}^{-1}e_{ij}
$$

## 4.10 回环检测 建图
如何在仅有视觉信息的情况下，知道当初曾经来过这个地方？
我们使用单词组成的向量去描述一个图像，当判断两个图像的向量相似，则认为检测到了回环。
$$
s\left(\boldsymbol{a},\boldsymbol{b}\right)=1-\frac{1}{W}\left\|\boldsymbol{a}-\boldsymbol{b}\right\|_1
$$
### 4.10.1 生成词袋 机器学习方法 无监督学习 (无监督学习聚类的方法很多，为啥选择K-means)
使用K叉树生成字典。

### 4.10.2 检测回环


### 4.10.3 回环校正



## 5 基于kalman融合技术-算力受限的嵌入式上实现 穷哥们
### 5.1 MEMS 车载 GNSS/INS 松组合导航方案
MEMS 惯性传感器的误差特性复杂，在车载应用环境下，可利用的观测量（如 GNSS 位置/速度）相对有限，导致部分误差参数（如高阶随机过程参数）的可观测性较低，难以在线精确估计。因此，在状态量建模时，应优先关注对导航解算影响显著且相对可观测的误差项。

针对车载应用场景，采用以下核心状态向量进行估计：
$$X_k = [\delta \mathbf{\phi}^T, \delta \mathbf{v}^T, \delta \mathbf{p}^T, \mathbf{e}_b^T, \mathbf{d}_b^T]^T$$
**(状态量定义见下文)**。其中，部分难以在线稳定估计的参数（如杆臂，比例系数等）可考虑通过离线标定或后处理方式确定。

**状态量说明：**
*  $\delta \mathbf{\phi}$: 姿态误差角向量 (3x1)
*  $\delta \mathbf{v}$: 速度误差向量 (3x1)
*  $\delta \mathbf{p}$: 位置误差向量 (3x1)
*  $\mathbf{e}_b$: 陀螺仪零偏向量 (3x1)
*  $\mathbf{d}_b$: 加速度计零偏向量 (3x1)

**该模型的核心思想是：** 在保证主要误差项得到有效估计和补偿的前提下，通过精简状态维度和聚焦可观测参数，提升滤波器的稳定性和实时性，更适用于资源受限的MEMS车载平台。

## 6 优化技术-使用算力优势的平台 富少爷



## 5.  软件设计
**这部分文件内容为项目文档裁剪出来，部分字段以及说明未给出，并非完整的流程，仅作记录**

### 5.1 需求文档
**目的与范围**

定义GNSS/INS/OD/视觉多源融合组合导航系统的软件功能需求、性能指标及安全要求。文档覆盖传感器原始数据预处理（解码、时间同步、异常过滤）、多源数据时空对齐（坐标系转换、时间戳插值）、融合算法（松/紧耦合可选模式）、定位输出（位置/速度/姿态）、故障诊断及数据记录全流程。适用于车载嵌入式平台开发，满足ISO 26262功能安全要求（ASIL-B），作为设计、测试及ASPICE过程审核的基准依据。

**术语定义**



**参考文档**
| 文档名称 | 版本号 | 编写人 | 配置库地址 | 原因 |
| ------- | -------| -------| ------- | --- |
|系统需求规格说明书| v1.0 | 王麻子 | svn | 需求来源 |
|配置管理 | v1.0 |  | | 格式要求 |
|质量管理| v1.0 | | | 质量检查，升级，QA|
|需求追溯| v1.0 | | | 需求追溯 |
|计划书| | | | 日程以及人员安排|
|协议手册| | | | 硬件接口定义  |
|功能安全分析报告 | | | | 功能分解 |
|过程定义 | | QA | | 开发流程合规|


**约束**

硬件平台:   \
存储空间：  \
时间：   \
计算频率: \
RAM：   \
其他：  

**功能架构**


**功能需求**

数据采集 \
FR001：同步解析GNSS(NMEA-0183)、INS(SPI)、OD(CAN)、视觉(MIPI CSI-2)原始数据
FR002：支持PPS信号硬同步，时间戳对齐精度≤1ms

预处理 \
FR003：GNSS数据RAIM检测，自动剔除HDOP>2的无效卫星
FR004：视觉图像畸变校正（基于内参标定文件）

融合算法 \
FR005：支持松/紧耦合模式动态切换（根据GNSS信号质量）\
FR006：INS/OD组合计算航向角（消除轮速计滑移误差）\
FR007：视觉SLAM辅助闭环检测，修正长期漂移

输出 \
FR008：输出组合定位结果（WGS84坐标系，包含协方差）\
FR009：提供定位健康状态码（0-正常，1-警告，2-故障）

**非功能需求**

性能指标 \
NFR001：定位精度（GNSS可用时）：水平≤0.5m (2σ) \
NFR002：GNSS失效60s内：水平误差增长≤1%/s \
NFR003：启动冷初始化时间≤30s

安全与可靠性 \
NFR004：单点故障检测覆盖率≥90%（ASIL-B） \
NFR005：关键进程看狗监控超时≤50ms \
NFR006：数据完整性校验（CRC32）

其他 \
NFR007：在线标定（自动补偿OD轮径变化） \
NFR008：支持MDF4格式数据记录（触发式存储）

**软硬件接口**

输入 \
HSIIN001：GNSS - UART@115200bps (NMEA-0183) \
HSIIN002：INS - SPI@1MHz (16bit RAW数据) \
IN003：OD - CAN@500kbps (轮速脉冲计数) \
IN004：视觉 - MIPI CSI-2@720P 30fps

输出 \
HSIOUT001：定位结果 - Ethernet/UDP@100Hz
HSIOUT002：故障码 - CAN@10Hz (SAE J1939)

**测试案例推荐**

功能测试 \
TC001：模拟GNSS失效，验证INS/OD/视觉接力定位 \
TC002：注入视觉遮挡，测试SLAM重定位能力

性能测试 \
TC003：80km/h动态场景下验证定位精度（RTK基准对比）\
TC004：-40℃低温启动时间测试

安全测试 \
TC005：注入INS零偏突变，验证故障检测响应时间 \
TC006：RAM ECC错误注入测试

**沟通纪要**
需要会议纪要支持，编写完成后需要将文件分发至上下游。

| 日期       | 参与方         | 议题                | 决议                     |
|------------|----------------|---------------------|--------------------------|
| 2025-08-03 |     |      |   |
| 2025-08-10 |        | ASIL-B分解方案      | 采用双核锁步机制（见FSR）|
| 整改记录   |                |                     |                          |
| 问题ID     | 描述           | 整改措施            | 闭环日期                 |
| PQ001      |  |    |                |

### 5.2 软件架构设计




### 5.3 软件详细设计

#### 5.3.1 基础数据计算模块


#### 5.3.2 地球常量管理模块


#### 5.3.3 IMU模块


#### 5.3.4 GNSS模块


#### 5.3.5 OD模块


#### 5.3.6 融合模块


#### 5.3.7 数据解析模块


#### 5.3.8 日志模块


#### 5.3.9 数据统计模块



### 5.4 单元测试
#### 5.4.1 基础数据计算模块测试用例

#### 5.4.2 地球常量管理模块


### 5.5 软件模块测试



### 5.6 软件系统测试







