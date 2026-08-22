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
| 燃油流量 \(W_f\) | 3 pps（约 1.361 kg/s） |
| 设计转速 | 10000 rpm |

非设计点（节流、改高度/马赫数）时，三个旋转部件应保持 **`iDesign = 2`**，不要重新定尺寸。

稳态独立变量是 \([W,\; R\text{-line},\; \text{涡轮}PR,\; N]\)。燃油是常数输入，不是牛顿未知数。

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

**不要把官方 mdl 存盘。** 节流脚本只会临时加记录模块，退出时会删掉。

## 地面节流特性

`run_throttle_char.m` 在 **H = 0、Ma = 0、几何冻结** 下扫燃油，求各平衡点的推力、耗油率、换算转速等。

```matlab
% 先 setup，确认 Compressor / Turbine / Nozzle 的 iDesign = 2
results = run_throttle_char;
```

默认从设计点 3.00 pps 往下收油，直到不收敛或碰到燃油下限。结果覆盖写入：

- `throttle_char_results.mat`
- `throttle_char.png` / `.fig`（推力、SFC 对压气机图换算转速 \(N_{c,\mathrm{map}}\)）
- `throttle_char_sfc_vs_fn.png` / `.fig`（SFC 对总推力）
- `throttle_char_ops.png` / `.fig`（燃油、喘振裕度、\(T_4\)、R-line）

在 MATLAB 里请打开 **`.fig`**（当前文件夹双击，或 `openfig('throttle_char.fig')`）。png 方便插入文档。

只改图、不重新仿真：

```matlab
results = run_throttle_char('PlotOnly', true);
```

横轴 \(N_{c,\mathrm{map}}\) 按压气机特性图范围画成 **0.50～1.05**，与本次实际算到的转速区间不是一回事。

## 目录

| 文件 / 目录 | 作用 |
|---|---|
| `GasTurbine_SS_Template.mdl` | 官方稳态模型 |
| `GasTurbine_SS_setup_everything.m` | 装载 `MWS`、打开模型 |
| `SimSetup/` | 部件特性图、求解器初值、喷管面积等 |
| `run_throttle_char.m` | 地面节流扫描与作图 |
| `PlotSSData.m` | 官方站参数作图入口 |

## 来源

原始示例与模块来自 NASA Glenn Research Center 的 T-MATS（Apache 2.0）。节流扫描脚本是在该示例上为学习地面节流特性增加的。
