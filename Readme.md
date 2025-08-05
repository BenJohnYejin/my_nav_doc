# 概述

**为应对频繁工作环境被损坏导致的数据、代码及工具丢失风险（即便采用移动硬盘备份仍存在意外删除的可能），特将部分技术资料、算法实现及学习心得集中归档于此文档。**  \
GITHUB的readme格式存在异常，使用VSCODE 打开后公式正常

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
$$\Delta\boldsymbol{\omega}_{arm} = \frac{1}{2} \left( 
\underbrace{\boldsymbol{\omega}_{ib}^b \times (\boldsymbol{r}_g \times \boldsymbol{\omega}_{ib}^b)}_{\text{离心项（向心加速度引起）}} 
+ \underbrace{\boldsymbol{r}_g \times \dot{\boldsymbol{\omega}}_{ib}^b}_{\text{欧拉项（切向加速度引起）}} 
\right)$$
其中$\boldsymbol{r}_g \in \mathbb{R}^3$为陀螺仪到IMU参考点的杆臂矢量（单位：m）。  
*工程补偿*：精密机械安装 + 标定后软件补偿（需已知$\boldsymbol{r}_g$和角加速度）

**b. 加速度计杆臂效应**  
加速度计空间偏移导致比力测量误差：
$$
\Delta\boldsymbol{f}_{arm} =  
\underbrace{\boldsymbol{r}_a \times \dot{\boldsymbol{\omega}}_{ib}^b}_{\text{切向加速度项}} 
+ \underbrace{\boldsymbol{\omega}_{ib}^b \times (\boldsymbol{r}_a \times \boldsymbol{\omega}_{ib}^b)}_{\text{向心加速度项}} 
+ \underbrace{2\boldsymbol{\omega}_{ib}^b \times \boldsymbol{v}_{rel}^b}_{\text{科氏力项}}
$$
其中$\boldsymbol{r}_a \in \mathbb{R}^3$为加速度计杆臂矢量，$\boldsymbol{v}_{rel}^b$为杆臂引起的相对速度。  
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
\boldsymbol{T} = 
\begin{bmatrix}
1 & -\beta_{yz} & \beta_{zy} \\
\beta_{xz} & 1 & -\beta_{zx} \\
-\beta_{xy} & \beta_{yx} & 1
\end{bmatrix}
+ \mathrm{diag}([\delta_{xx}, \delta_{yy}, \delta_{zz}])
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
\begin{aligned}\mathrm{\dot{\phi}}
&=\phi\times\omega_{in}^{n}+\delta\omega_{in}^{n}-\delta\omega_{ib}^{n}\\&=\phi\times\omega_{in}^{n}+(\delta\omega_{ie}^{n}+\delta\omega_{en}^{n})-C_{b}^{n}\delta\omega_{ib}^{b}\\
&=\boldsymbol{\phi}\times\boldsymbol{\omega}_{in}^{n}+(\boldsymbol{M}_{1}\delta\boldsymbol{p}+\boldsymbol{M}_{an}\delta\boldsymbol{v}^{n}+\boldsymbol{M}_{2}\delta\boldsymbol{p})-\boldsymbol{C}_{b}^{n}(\omega_{ibx}^{b}\delta\boldsymbol{K}_{\mathbf{G}x}+\omega_{iby}^{b}\delta\boldsymbol{K}_{\mathbf{G}y}+\omega_{ibz}^{b}\delta\boldsymbol{K}_{\mathbf{G}z}+\boldsymbol{\varepsilon}^{b})\\
&=-\omega_{in}^{n}\times\boldsymbol{\phi}+\boldsymbol{M}_{an}\delta\boldsymbol{\nu}^{n}+(\boldsymbol{M}_{1}+\boldsymbol{M}_{2})\delta\boldsymbol{p}-\omega_{ibx}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}x}-\omega_{iby}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}y}-\omega_{ibz}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}z}-\boldsymbol{C}_{iby}\\
&=\boldsymbol{M}_{aa}\boldsymbol{\phi}+\boldsymbol{M}_{a\nu}\delta\boldsymbol{\nu}^{n}+\boldsymbol{M}_{ap}\delta\boldsymbol{p}-\omega_{ibx}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}x}-\omega_{iby}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}y}-\omega_{ibz}^{b}\boldsymbol{C}_{b}^{n}\delta\boldsymbol{K}_{\mathbf{G}z}-\boldsymbol{C}_{b}^{n}\boldsymbol{\varepsilon}^{b}\end{aligned}
$$
速度误差方程为，
$$
\begin{gathered}\delta\dot{\nu}^{n}=\boldsymbol{f}_{\mathrm{sf}}^{n}\times\boldsymbol{\phi}+\left[(\boldsymbol{\nu}^{n}\times)\boldsymbol{M}_{cn}-(2\boldsymbol{\omega}_{i\boldsymbol{e}}^{n}+\boldsymbol{\omega}_{en}^{n})\times\right]\mathbf{\delta}\boldsymbol{\nu}^{n}+\left[(\boldsymbol{\nu}^{n}\times)(2\boldsymbol{M}_{1}+\boldsymbol{M}_{2})+\boldsymbol{M}_{3}\right]\mathbf{\delta}\boldsymbol{p}\\+C_{b}^{n}(f_{sfx}^{b}\delta K_{Ax}+f_{sfy}^{b}\delta K_{Ay}+f_{sfz}^{b}\delta K_{Az}+\nabla^{b})\end{gathered}
$$
其中
$$
\begin{aligned}&M_{_{va}}=(f_{\mathrm{sf}}^{n}\times)\\&\boldsymbol{M}_{_{w}}=(v^{n}\times)M_{av}-\left[(2\omega_{ie}^{n}+\omega_{en}^{n})\times\right]\\&\boldsymbol{M}_{\nu p}=(v^{n}\times)(2M_{1}+M_{2})+M_{3}\end{aligned}
$$

位置误差为
$$\delta\dot{p}=M_{pv}\delta v^{n}+M_{pp}\delta p$$
其中，
$$\boldsymbol{M}_{pv}=\begin{bmatrix}0&1/R_{Mh}&0\\\sec L/R_{Nh}&0&0\\0&0&1\end{bmatrix}$$
$$\left.\boldsymbol{M}_{pp}=\left\lfloor\begin{array}{ccc}{0}&{0}&{-\nu_{_\mathrm{N}}/R_{_\mathrm{Mh}}^{2}}\\{v_{_\mathrm{E}}\sec L\tan L/R_{_\mathrm{Nh}}}&{0}&{-\nu_{_\mathrm{E}}\sec L/R_{_\mathrm{Nh}}^{2}}\\{0}&{0}&{0}\end{array}\right.\right\rfloor$$


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







