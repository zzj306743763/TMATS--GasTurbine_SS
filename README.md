# T-MATS 单轴涡喷稳态示例

本仓库存放 NASA [T-MATS](https://github.com/nasa/T-MATS) 官方单轴涡喷**稳态**示例 `GasTurbine_SS`，并附带地面节流、高度特性和速度特性扫描脚本。

- **节流特性**：H = 0、Ma = 0，扫燃油。
- **高度特性**：默认 Ma = 0.9、*T*<sub>4</sub> 为设计点不变，扫高度；海平面先把马赫从 0 升到 0.9（不记入曲线）。压气机图出界即停，另存 *N*<sub>c,map</sub> 随高度。
- **速度特性**：默认 H = 0、*T*<sub>4</sub> 不变，Ma 从 0 按 0.05 扫到 2.5。图为 *F*<sub>s</sub>、*F*<sub>n</sub>、*W*、SFC；等 *T*<sub>4</sub> 时 *F*<sub>n</sub> 常先降再升，很高马赫才随 *F*<sub>s</sub> → 0 降下来。

三者推力均用教材净推力 *F*<sub>n</sub> = *F*<sub>g</sub> − *F*<sub>ram</sub>。完整设计点、步骤和参数见：

**[Example_GasTurbine_SS/README.md](Example_GasTurbine_SS/README.md)**

本机需要自行安装 T-MATS v1.3.3（建议 MATLAB R2023b），本仓库不包含工具箱本体。
