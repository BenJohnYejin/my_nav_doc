# 概述

**为应对频繁工作环境被损坏导致的数据、代码及工具丢失风险（即便采用移动硬盘备份仍存在意外删除的可能），特将部分技术资料、算法实现及学习心得集中归档于此文档。**  \

文档主体规划为六个章节：
1.  **第一章节** - 简单阐述惯性导航的软件实现。
2.  **第二章节** - 简单阐述卫星导航中SPP的软件实现。
3.  **第三章节** - 简单阐述里程计的递推软件实现。
4.  **第四章节** - 简单阐述视觉前端的软件实现。
5.  **第五章：V模型左侧 - 开发实现** - 聚焦工程化实现，包括实时导航软件的设计与开发。
3.  **第六章：V模型右侧 - 测试验证与分析** - 涵盖测试程序开发、数据处理脚本编写及事后数据分析方法。

**说明：** 除直接服务于客户端的实时导航软件外，一套完备的测试验证工具链（用于数据回放、结果分析、性能评估）及事后数据处理脚本对于保障系统质量和问题定位至关重要。此类工具链在逻辑上归属于第六章（V模型右侧）。

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

### 2.5 定位解算流程
#### 2.5.1 单点定位(SPP)流程
1. **数据准备**：
   - 读取观测值文件(`obsd_t`)
   - 读取广播星历(`eph_t`)

2. **误差修正**：
   - 卫星钟差：广播参数计算(`eph2clk()`)
   - 电离层延迟：Klobuchar模型(`ionmodel()`)
   - 对流层延迟：Saastamoinen模型(`tropmodel()`)

3. **最小二乘解算**：
   - 设计矩阵构建：`rescode()`
   - 加权最小二乘：`lsq()`
   - 结果输出：`sol_t`

#### 2.5.2 实时动态定位(RTK)流程


## 3. 里程计与视觉定位部分
**参考来源**：PINS、视觉SLAM十四讲、VINS  
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

#### 3.1.3 位置更新算法
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

#### 3.1.4 姿态更新算法
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

### 3.3 EKF_SLAM
## 4. EKF-SLAM 算法推导与误差分析
### 4.1 状态空间定义
#### 4.1.1 状态向量构成
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

#### 4.1.2 协方差矩阵结构
$$
\boldsymbol{P}_k = \begin{bmatrix}
\boldsymbol{P}_{vv,k} & \boldsymbol{P}_{vm,k} \\ 
\boldsymbol{P}_{vm,k}^\top & \boldsymbol{P}_{mm,k} 
\end{bmatrix}
$$
- $\boldsymbol{P}_{vv,k}$：位姿协方差 (3×3)
- $\boldsymbol{P}_{mm,k}$：路标位置协方差 (2n×2n)
- $\boldsymbol{P}_{vm,k}$：位姿-路标互协方差 (3×2n)

### 4.2 运动模型（预测步骤）
#### 4.2.1 非线性运动模型
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

#### 4.2.2 雅可比矩阵计算
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

#### 4.2.3 预测方程
$$\begin{aligned}
\hat{\boldsymbol{x}}_{k|k-1} &= \begin{bmatrix} f(\boldsymbol{x}_{v,k-1}, \boldsymbol{u}_k, 0) \\ \boldsymbol{m}_{k-1} \end{bmatrix} \\
\boldsymbol{P}_{k|k-1} &= \boldsymbol{F} \boldsymbol{P}_{k-1} \boldsymbol{F}^\top + \boldsymbol{G} \boldsymbol{Q}_k \boldsymbol{G}^\top
\end{aligned}$$
其中：
$$
\boldsymbol{F} = \begin{bmatrix} \boldsymbol{F}_x & 0 \\ 0 & \boldsymbol{I}_{2n} \end{bmatrix}, \quad
\boldsymbol{G} = \begin{bmatrix} \boldsymbol{F}_w \\ 0 \end{bmatrix}
$$

### 4.3 观测模型（更新步骤）
#### 4.3.1 非线性观测模型
$$
\boldsymbol{z}_k^i = h(\boldsymbol{x}_v, \boldsymbol{m}_i) + \boldsymbol{v}_k^i = 
\begin{bmatrix}
\sqrt{(m_{i,x} - x)^2 + (m_{i,y} - y)^2} \\
\atan2(m_{i,y} - y, m_{i,x} - x) - \theta
\end{bmatrix}
$$
- 观测噪声：$\boldsymbol{v}_k^i \sim \mathcal{N}(0, \boldsymbol{R}_k)$

#### 4.3.2 观测雅可比矩阵
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

### 4.4 数据关联与更新
#### 4.4.1 新路标初始化
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

#### 4.4.2 卡尔曼更新
$$\begin{aligned}
\text{新息：} \quad \boldsymbol{\nu}_k^i &= \boldsymbol{z}_k^i - h(\hat{\boldsymbol{x}}_{k|k-1}) \\
\text{协方差：} \quad \boldsymbol{S}_k^i &= \boldsymbol{H}^i \boldsymbol{P}_{k|k-1} (\boldsymbol{H}^i)^\top + \boldsymbol{R}_k \\
\text{增益：} \quad \boldsymbol{K}_k^i &= \boldsymbol{P}_{k|k-1} (\boldsymbol{H}^i)^\top (\boldsymbol{S}_k^i)^{-1} \\
\text{更新：} \quad \hat{\boldsymbol{x}}_{k|k} &= \hat{\boldsymbol{x}}_{k|k-1} + \boldsymbol{K}_k^i \boldsymbol{\nu}_k^i \\
\boldsymbol{P}_{k|k} &= (\boldsymbol{I} - \boldsymbol{K}_k^i \boldsymbol{H}^i) \boldsymbol{P}_{k|k-1}
\end{aligned}$$

### 4.5 误差分析
#### 4.5.1 线性化误差
| **误差源**         | **数学描述**                          | **影响**               |
|--------------------|--------------------------------------|------------------------|
| **一阶近似残差**   | $\|f(\boldsymbol{x}) - f(\hat{\boldsymbol{x}}) - \boldsymbol{F}_x \delta\boldsymbol{x}\|$ | 模型失真导致发散       |
| **泰勒展开截断**   | $\mathcal{O}(\|\delta\boldsymbol{x}\|^2)$ | 大初始误差时不稳定    |

#### 4.5.2 数据关联误差
**误关联概率模型**：
$$ P(\text{错误关联}) = 1 - \int_{\mathcal{Z}} p(\boldsymbol{z} | \text{正确}) d\boldsymbol{z} $$
其中 $\mathcal{Z} = \{ \boldsymbol{z} : \|\boldsymbol{z} - \hat{\boldsymbol{z}}_i\|_{\boldsymbol{S}^{-1}} < \tau \}$

**误差传播**：
- 单次误关联 → 位姿误差增长 $\|\delta\boldsymbol{x}\| \propto \|\boldsymbol{K}\| \cdot \|\boldsymbol{\nu}_\text{err}\|$
- 连续误关联 → 协方差矩阵失去正定性

#### 4.5.3 计算复杂度
$$
\mathcal{O}(n^3) \quad \text{(矩阵求逆)} \quad \xrightarrow{\text{稀疏性利用}} \mathcal{O}(n^{1.5})
$$
- **存储需求**：$\frac{1}{2}(3+2n)(4+2n)$ → 路标数$n$较大时不可行

#### 4.5.4 一致性分析
**NEES检验**：
$$
\epsilon_k = (\boldsymbol{x}_k - \hat{\boldsymbol{x}}_k)^\top \boldsymbol{P}_k^{-1} (\boldsymbol{x}_k - \hat{\boldsymbol{x}}_k) \sim \chi^2_{\dim(\boldsymbol{x})}
$$
若 $\mathbb{E}[\epsilon_k] > \dim(\boldsymbol{x})$ 则滤波器乐观

### 4.6 性能提升技术
#### 4.6.1 稀疏化处理
**协方差矩阵近似**：
$$
\boldsymbol{P} \approx \begin{bmatrix}
\boldsymbol{P}_{vv} & \boldsymbol{P}_{vm} \\
\boldsymbol{P}_{vm}^\top & \text{blkdiag}(\boldsymbol{P}_{m_im_i})
\end{bmatrix}
$$
- 忽略路标间相关性 → 计算降至$\mathcal{O}(n)$

#### 4.6.2 分治策略
**局部子图构建**：
1. 创建短期局部子图：$\boldsymbol{x}^\text{local} = [\boldsymbol{x}_v, \boldsymbol{m}_{\text{active}}]^\top$
2. 子图内EKF-SLAM
3. 子图合并：$\boldsymbol{x}_\text{global} = g(\boldsymbol{x}_\text{global}, \boldsymbol{x}_\text{local})$


## 5. msckf_vio 基于EKF的视觉融合
**参考文献** Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight


## 4.  融合技术
### 卡尔曼滤波技术




###  MEMS 车载 GNSS/INS 松组合导航方案

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







