# 单轴涡喷稳态示例（T-MATS GasTurbine_SS）

基于 NASA [T-MATS](https://github.com/nasa/T-MATS) v1.3.3 官方稳态示例 `GasTurbine_SS` 的学习与计算目录。模型是一台**单轴涡喷**（进气道—压气机—燃烧室—涡轮—喷管，一根轴），用牛顿—拉夫森求解器配平稳态共同工作点。

本仓库**不是**完整的 T-MATS 工具箱。要跑模型，本机仍需安装 T-MATS，并把库加到 MATLAB 路径。

官方模型说明见 NASA/TM-2014-218410。

## 设计点（海平面静止）

几何按下面这一点定尺寸（`iDesign = 0` 时算缩放和喷管面积）：

| 量 | 取值 |
|---|---|
| 高度 | 0 ft |
| 马赫数 | 0 |
| 温度偏差 | 0 |
| 燃油流量 *W*<sub>f</sub> | 3 pps（约 1.361 kg/s） |
| 设计转速 | 10000 rpm |

非设计点（节流、改高度/马赫数）时，三个旋转部件应保持 **`iDesign = 2`**，不要重新定尺寸。

稳态独立变量是 `[W; R-line; 涡轮 PR; N]`。燃油是常数输入，不是牛顿未知数。

## 环境

- MATLAB / Simulink **R2023b**（T-MATS v1.3.3 官方测试版本）
- 已安装并编译 T-MATS 库（MEX）

## 运行官方示例

在 MATLAB 中进入本目录后：

```matlab
GasTurbine_SS_setup_everything
```

会生成工作区变量 `MWS` 并打开 `GasTurbine_SS_Template.mdl`。然后按平时方式仿真即可。

结束后可调用 `GasTurbine_SS_Example_cleanup` 从路径里去掉本示例的 `SimSetup`。

**不要把官方 mdl 存盘。** 特性扫描脚本只会临时加记录模块，退出时会删掉。

三条教材特性（须先 `setup`，且 Compressor / Turbine / Nozzle 的 `iDesign = 2`）：

```matlab
results = run_throttle_char;    % 地面节流：H=0、Ma=0，扫燃油
results = run_altitude_char;    % 高度特性：默认 Ma=0.9、T4 不变，扫高度
results = run_speed_char;       % 速度特性：默认 H=0、T4 不变，扫马赫数
```

静止高度特性（Ma = 0）用 `run_altitude_char('MN', 0)`。已有 `.mat` 时，三个脚本都可用 `'PlotOnly', true` 只重画、不重算。

## 地面节流特性

`run_throttle_char.m` 在 **H = 0、Ma = 0、几何冻结** 下扫燃油。图上用**净推力** *F*<sub>n</sub> = *F*<sub>g</sub> − *F*<sub>ram</sub>；地面静止时冲压阻力为零，*F*<sub>n</sub> = *F*<sub>g</sub>。耗油率为 SFC = 3600 *W*<sub>f</sub> / *F*<sub>n</sub>。

```matlab
% 先 setup，确认 Compressor / Turbine / Nozzle 的 iDesign = 2
results = run_throttle_char;
```

默认从设计点 3.00 pps 往下收油，直到不收敛或碰到燃油下限。结果覆盖写入：

- `throttle_char_results.mat`
- `throttle_char.png` / `.fig`（净推力 *F*<sub>n</sub>、SFC 对压气机图换算转速 *N*<sub>c,map</sub>）
- `throttle_char_sfc_vs_fn.png` / `.fig`（SFC 对净推力 *F*<sub>n</sub>）
- `throttle_char_ops.png` / `.fig`（燃油、喘振裕度、*T*<sub>4</sub>、R-line）

在 MATLAB 里请打开 **`.fig`**（当前文件夹双击，或 `openfig('throttle_char.fig')`）。png 方便插入文档。

只改图、不重新仿真：

```matlab
results = run_throttle_char('PlotOnly', true);
```

横轴 *N*<sub>c,map</sub> 按压气机特性图范围画成 **0.50～1.05**，与本次实际算到的转速区间不是一回事。

## 飞行特性的控制规律

教材里三种稳态特性的分工：

| 特性 | 冻结 | 扫描 | 默认工况 |
|---|---|---|---|
| 节流 | *H* = 0、Ma = 0 | *W*<sub>f</sub> | 地面静止 |
| 高度 | 几何、Ma、*T*<sub>4</sub> | 高度 | Ma = 0.9 |
| 速度 | 几何、高度、*T*<sub>4</sub> | Ma | *H* = 0 |

三者都要求几何冻结、控制规律不变，只改表里那一项。推力一律用净推力 *F*<sub>n</sub> = *F*<sub>g</sub> − *F*<sub>ram</sub>。

本例默认控制规律是 **涡轮前总温 T<sub>4</sub> 不变**，钉住的是**海平面静止设计点**（*H* = 0、Ma = 0、*W*<sub>f</sub> = 3 pps）算出来的那个 *T*<sub>4</sub>（约 1710 K / 3078 R），不随高度或马赫改写目标。不要把日志里的 `T4=3078` 当成开尔文。

T-MATS 稳态模型里燃油是常数输入，牛顿未知数是 `[W; R-line; 涡轮 PR; N]`。所以每个飞行点有两层：

1. **内层**：给定 *W*<sub>f</sub>，牛顿法配平稳态。
2. **外层**：改 *W*<sub>f</sub>，直到 *T*<sub>4</sub> 落到容差内。

仍可改成钉住换算转速或物理转速（不是教材默认）：

```matlab
results = run_altitude_char('ThrottleMode', 'NcMap');  % NcMap = 1
results = run_speed_char('ThrottleMode', 'N');         % N = 10000 rpm
```

温度单位：气路总温、静温用**兰氏度 R**（英制绝对温标，R = °F + 459.67）。模型入口 `dTamb [degF]` 只是相对标准大气的**温差**，扫描脚本保持 `dTamb = 0`（标准日）。

## 高度特性

`run_altitude_char.m`：**几何冻结、马赫数固定、T<sub>4</sub> 不变，只扫高度。**

### 默认算什么

| 量 | 默认 |
|---|---|
| 飞行马赫数 | 0.9（飞行高度特性） |
| 控制规律 | *T*<sub>4</sub> = 设计点值，外层容差 20 R |
| 高度 | 0 km 起，名义步长 1 km，最小步长 0.25 km，目标上限 15 km |
| 燃油上限 | 4 pps |
| 大气 | 标准大气，`dTamb = 0` |
| 推力 | 净推力 *F*<sub>n</sub> = *F*<sub>g</sub> − *F*<sub>ram</sub> |

静止高度特性（Ma = 0）用：

```matlab
results = run_altitude_char('MN', 0);
```

### 计算步骤

1. 海平面静止、*W*<sub>f</sub> = 3 pps，记下设计点 *T*<sub>4</sub>。
2. 若目标 Ma > 0：在 **H = 0** 按 0.1 把马赫从 0 升到 0.9，每档配平 *T*<sub>4</sub>。这是牛顿延拓，**不记入高度曲线**。
3. 从 H = 0、Ma = 0.9 起按 1 km 往上扫。失败则对分；出压气机图（*N*<sub>c,map</sub> 超出 0.50～1.05）即停，**不按最高转速线外延**。

等 *T*<sub>4</sub> 爬高时进口变冷，换算转速升高。本机图只到 1.05，Ma = 0.9 时大约在对流层顶以下就会出图，扫不到 11 km 是图的范围，不是大气在 11 km 截断。

### 图与结果

四张子图（横轴高度 km）：单位推力 *F*<sub>s</sub> = *F*<sub>n</sub> / *W*、净推力 *F*<sub>n</sub>、空气流量 *W*、耗油率 SFC = 3600 *W*<sub>f</sub> / *F*<sub>n</sub>。另存 *N*<sub>c,map</sub> 随高度（纵轴为特性图 0.50～1.05）。

Ma = 0.9 时冲压阻力不可忽略：若误用毛推力 *F*<sub>g</sub> 算 SFC，对流层内会随高度上升，与教材相反。

```matlab
results = run_altitude_char;
results = run_altitude_char('PlotOnly', true);  % 只重画
```

覆盖写入 `altitude_char_results.mat`、`altitude_char.png` / `.fig`、`altitude_char_ncmap.png` / `.fig`。MATLAB 里请打开 **`.fig`**。

常用参数：`'MN'`、`'dMN'`（海平面加速步长，默认 0.1）、`'HMaxKm'`、`'dHKm'`、`'dHMinKm'`、`'WfMax'`、`'ThrottleMode'`。

## 速度特性

`run_speed_char.m`：**几何冻结、高度固定、T<sub>4</sub> 不变，只扫飞行马赫数。**

### 默认算什么

| 量 | 默认 |
|---|---|
| 高度 | 0 km（海平面速度特性） |
| 控制规律 | *T*<sub>4</sub> = 设计点值，外层容差 2 R |
| 马赫数 | 0 起，步长 0.05，最小步长 0.025，上限 2.5 |
| 燃油上限 | 8 pps |
| 大气 | 标准大气，`dTamb = 0` |
| 推力 | *F*<sub>n</sub> = *F*<sub>g</sub> − *W* *V*<sub>0</sub> / *g*<sub>c</sub> |

高空速度特性先在 Ma = 0 爬到指定高度（不记入曲线），再扫 Ma：

```matlab
results = run_speed_char('HKm', 11);
```

### 计算步骤

1. 海平面静止、*W*<sub>f</sub> = 3 pps，记下设计点 *T*<sub>4</sub>。
2. 若 `HKm > 0`：Ma = 0 爬高并配平 *T*<sub>4</sub>（不画进速度曲线）。
3. 在该高度上从 Ma = 0 按 0.05 往上扫。每档用冲压总压比放大 *W*、*W*<sub>f</sub> 初值，再内层牛顿、外层钉 *T*<sub>4</sub>。失败则对分；出图或燃油顶格则停止。

节流特性不需要这种流量放缩：进口条件不变，只改油门，上一档收敛解即可当牛顿初值。

### 曲线怎么读

*F*<sub>n</sub> = *F*<sub>s</sub> · *W*。等 *T*<sub>4</sub> 时：

- **F<sub>s</sub>** 随马赫大致单调下降，很高马赫时趋向 0。
- **W** 随冲压增大。
- **F<sub>n</sub>** 因此常为：先降（冲压阻力先露头）→ 再升（流量涨得比 F<sub>s</sub> 掉得快）→ 过峰值后再降。降到 0 要 F<sub>s</sub> → 0，往往高于本机图能算到的马赫。

本机压气机图 *N*<sub>c,map</sub> 为 0.50～1.05。马赫升高进口变热，换算转速下降，可能先碰到图的下沿或燃油上限（默认 `WfMax = 8` pps），不一定真能扫到 2.5。

纵轴按数据自动缩放。相邻点 *F*<sub>n</sub> 差千分之几，多半是 *T*<sub>4</sub> 配平残差或特性图插值，不是趋势反转。

### 图与结果

四张子图（横轴 Ma）：*F*<sub>s</sub>、*F*<sub>n</sub>、*W*、SFC。

```matlab
results = run_speed_char;
results = run_speed_char('PlotOnly', true);  % 只重画
```

覆盖写入 `speed_char_results.mat`、`speed_char.png` / `.fig`。MATLAB 里请打开 **`.fig`**。

常用参数：`'HKm'`、`'MNMax'`（默认 2.5）、`'dMN'`（默认 0.05）、`'dMNMin'`、`'WfMax'`、`'ThrottleMode'`。

## 目录

| 文件 / 目录 | 作用 |
|---|---|
| `GasTurbine_SS_Template.mdl` | 官方稳态模型 |
| `GasTurbine_SS_setup_everything.m` | 装载 `MWS`、打开模型 |
| `SimSetup/` | 部件特性图、求解器初值、喷管面积等 |
| `run_throttle_char.m` | 地面节流扫描与作图 |
| `run_altitude_char.m` | 高度特性扫描与作图（默认 *T*<sub>4</sub> 不变） |
| `run_speed_char.m` | 速度特性扫描与作图（默认 *T*<sub>4</sub> 不变） |
| `PlotSSData.m` | 官方站参数作图入口 |

## 来源

原始示例与模块来自 NASA Glenn Research Center 的 T-MATS（Apache 2.0）。节流、高度、速度扫描脚本是在该示例上为学习特性曲线增加的。
