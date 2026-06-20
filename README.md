# MANUS 手套 → LZ-SG002 灵巧手 遥操系统

将 MANUS 数据手套捕捉的人手动作，通过 **dex-retargeting（DexPilot 优化式重映射）** 实时映射到
LZ-SG002 六自由度灵巧手（左右手），经 **RS485 / Modbus-RTU** 驱动，并提供 Web 调参面板与
力/角度/位置闭环反馈显示。

- 采用优化式重映射替代旧的「关节角度→线性归一化」方案，**免逐姿态标定**、拇指对掌正确。
- 混合架构：C++ 负责取手套数据（贴近 MANUS SDK），Python 负责重映射 + 驱动 + 面板。

---

## 1. 系统与硬件要求

| 项 | 要求 |
|---|---|
| 操作系统 | Windows 10/11 x64 |
| 手套 | MANUS 数据手套 + **MANUS Core**（已授权、可正常出数据） |
| 灵巧手 | LZ-SG002（右手从机地址 1 / 左手 2），48V 供电 |
| 总线 | USB→RS485 适配器，115200 8N1 |
| 编译 | Visual Studio 2022 或 VS BuildTools（v143, C++17） |
| Python | Anaconda/Miniconda（建表 conda 环境，Python 3.12） |

---

## 2. 完整数据流

```
┌─ MANUS Core ─────────────────────────────────────────────────────────────┐
│  数据手套 → 手套标定 → RawSkeleton（重映射前的真实手部 3D 骨架，单位 m）        │
└───────────────┬──────────────────────────────────────────────────────────┘
                │ ManusSDK (C++)
        ┌───────▼─────────────────────────────────────────┐
        │ src/keypoint_streamer  (ManusKeypointStreamer.exe)│
        │  · 订阅 RawSkeleton + NodeInfo                     │
        │  · 节点(ChainType/FingerJointType) → 21 个 MANO 点 │
        │  · 左右手各打一包                                   │
        └───────┬──────────────────────────────────────────┘
                │ UDP 127.0.0.1:9001  ('MNKP' v1, side, seq, 21×3 float32, ~90Hz)
        ┌───────▼───────────────────────────────────────────────────────────┐
        │ src/teleop  (python -m dexhand_teleop.main)                         │
        │  manus_receiver  → 每手最新关键点帧                                  │
        │  keypoints       → 平移到腕原点 + 估计腕坐标系(SVD) + OPERATOR2MANO   │
        │                    ⇒ MANO 规范系 21×3                                │
        │  retarget(DexPilot)→ 优化「腕→指尖 + 指尖间」向量 ⇒ qpos(11 关节, rad) │
        │                    (远端关节由 URDF <mimic> 耦合 + 低通 low_pass)     │
        │  取 6 个驱动关节   → reg0..5                                          │
        │  hand_driver     → qpos[qmin,qmax] 线性归一化 → 0..1000(+反向)        │
        │                    → Modbus-RTU 写寄存器 0..5（右 addr1 / 左 addr2）   │
        │                    ← Modbus-RTU 读 18..46（力/角度/电机位置）反馈      │
        │  3D 视图(meshcat)→ http://127.0.0.1:7000 真实 URDF：指令(实色)+实际(ghost) │
        │  panel(FastAPI)  → http://127.0.0.1:8090 精简调参 + 标定 + 反馈表       │
        └───────┬───────────────────────────────────────────────────────────┘
                │ RS485 115200
        ┌───────▼───────────┐
        │   LZ-SG002 灵巧手   │
        └────────────────────┘
```

各级单位/坐标：MANUS 位置=米；MANO 规范系（腕在原点，朝向由手几何估计，对尺寸/朝向不敏感）；
qpos=弧度；寄存器=最大行程千分之一（0–1000）。

**寄存器映射（详见 [_protocol_decoded.txt](_protocol_decoded.txt)）**

| reg | 含义 | 0 → 1000 | 驱动关节(右手) |
|---|---|---|---|
| 0 | 拇指翻转/对掌 | 不翻转 → 完全对掌 | `hand_r_thumb_joint1` |
| 1 | 拇指弯曲 | 伸直 → 完全弯曲 | `hand_r_thumb_joint2` |
| 2–5 | 食/中/无名/小指弯曲 | 伸直 → 完全弯曲 | `hand_r_{finger}_joint1` |
| 6–11 / 12–17 | 速度 / 力 | 0–100% 额定 | 启动一次性下发 |
| 18–30 / 31–40 / 41–46 | 指尖力(g) / 关节角(0.1°) / 电机位置(0–1000) | 只读反馈 | 面板显示 |

> URDF 每手 11 个关节，硬件仅 6 个执行器：每指远端关节用 `<mimic>` 耦合到近端（与参考 Inspire
> 手一致），故重映射输出可直接对应 6 个寄存器。

---

## 3. 目录结构

```
rs485_te_ws/
├─ src/                       ★ 第一方代码
│  ├─ teleop/                 Python 服务（重映射 + 驱动 + 面板 + 工具）
│  │  ├─ dexhand_teleop/      protocol / keypoints / manus_receiver / retarget / hand_driver / panel / main
│  │  ├─ configs/             *_dexpilot.yml（重映射）, drive.yml（COM/量程/速度力/UDP）
│  │  ├─ assets/              生成的 retarget URDF（+ meshes, gitignore）
│  │  ├─ tools/               make_retarget_urdf / fake_streamer / sim_view / test_*
│  │  └─ environment.yml · requirements.txt · README.md（面板/运维细节）
│  └─ keypoint_streamer/      C++ 关键点流送器（VS2022），引用 ../../ManusSDK_v3.1.1 的 SDK
├─ ManusSDK_v3.1.1/           第三方：MANUS 手套 SDK（示例 + include/lib）
├─ dex-retargeting/           第三方：重映射库（vendoring + editable，见 VENDORED.md）
├─ LZ-SG002-URDF.SLDASM/      输入：灵巧手 CAD/URDF（左右手，SolidWorks 导出）
├─ _protocol_decoded.txt      硬件寄存器协议（中文解码）
└─ 通信协议(1).docx           硬件协议原件
```

---

## 4. 依赖清单

| 依赖 | 来源 | 说明 |
|---|---|---|
| MANUS Core + ManusSDK | 第三方（仓库内 `ManusSDK_v3.1.1/`） | C++ 流送器编译/链接所需 include+lib+dll |
| pinocchio, nlopt | conda-forge | 机器人运动学 + 优化（pinocchio Windows 无 pip wheel） |
| torch (CPU) | pip | dex-retargeting 的损失/梯度计算 |
| dex-retargeting | 仓库内 `dex-retargeting/`（editable） | 重映射核心（依赖 `pin`，用 `--no-deps` 跳过，由 conda 的 pinocchio 满足） |
| numpy, pyyaml, lxml, pytransform3d, anytree, six | conda/pip | 数值、配置、URDF 解析 |
| pyserial | pip | RS485/Modbus |
| fastapi, uvicorn | pip | 调参面板 |
| matplotlib | pip | 仅 `sim_view` 三维可视化 |

---

## 5. 新机部署（按依赖顺序）

> 仓库已自带 `ManusSDK_v3.1.1/`、`dex-retargeting/`、`LZ-SG002-URDF.SLDASM/`，无需额外下载。

**① 安装 MANUS Core**：装好 MANUS 官方软件并完成授权，确认手套能在 Dashboard 出数据。

**② 创建 Python 环境**（含 pinocchio/torch 等所有运行依赖）：
```bat
cd /d <仓库>\src\teleop
conda env create -f environment.yml
conda activate teleop
```

**③ 安装重映射库（用仓库内 vendored 源码，editable）**：
```bat
:: 在仓库根目录
pip install -e .\dex-retargeting --no-deps
```
`--no-deps` 是必须的：它的依赖 `pin`(pinocchio) 在 Windows 上无 wheel、会编译失败；pinocchio 已由
conda-forge 提供。装好后 `import dex_retargeting` 解析到工作区内的 `dex-retargeting\src\`。

**④ 编译 C++ 关键点流送器**：
```bat
:: 用 VS2022 打开 src\keypoint_streamer\ManusKeypointStreamer.sln，选 Release/x64 生成；或：
MSBuild src\keypoint_streamer\ManusKeypointStreamer.vcxproj -p:Configuration=Release -p:Platform=x64
```
产物：`src\keypoint_streamer\Output\x64\Release\ManusKeypointStreamer_Windows.exe`（自动随附 ManusSDK.dll）。

**⑤ （可选）重生成 retarget URDF**（仓库已含生成结果，改了 CAD 才需要）：
```bat
cd /d <仓库>\src\teleop
python tools\make_retarget_urdf.py ..\..\LZ-SG002-URDF.SLDASM\LZ-SG002-URDF-R.SLDASM\urdf\LZ-SG002-URDF-R.SLDASM.urdf assets\right_hand.urdf
python tools\make_retarget_urdf.py ..\..\LZ-SG002-URDF.SLDASM\LG-SG002-URDF-L.SLDASM\urdf\LG-SG002-URDF-L.SLDASM.urdf assets\left_hand.urdf
```

**⑥ 离线自检（不接硬件，验证部署成功）**：
```bat
cd /d <仓库>\src\teleop
python tools\test_retarget.py --hand right   :: 期望 RESULT: PASS
python tools\test_modbus.py                  :: 期望 RESULT: PASS
python tools\sim_view.py snapshot --hand right --out sim.png   :: 出三维姿态图
```

---

## 6. 启动运行

**一键脚本（推荐，仓库根目录双击即可）**
| 脚本 | 用途 |
|---|---|
| `run_panel_sim.bat` | 空跑演示：合成数据 + 面板，不接硬件（自动开浏览器） |
| `run_teleop.bat` | 真机：启动流送器 + 重映射/驱动服务 + 面板（需先满足下方 6.2 的先决条件） |

下面 6.1 / 6.2 是这两个脚本所封装的等价手动命令。

### 6.1 空跑演示（无手套/无硬件）
```bat
:: 终端1：合成手套数据
conda activate teleop & cd /d <仓库>\src\teleop
python tools\fake_streamer.py --mode wave

:: 终端2：重映射服务 + 面板（drive.yml 中 port 留空 = 不写串口）
conda activate teleop & cd /d <仓库>\src\teleop
python -m dexhand_teleop.main
```
浏览器打开 **3D 视图 http://127.0.0.1:7000/static/**（看映射对不对）和 **调参面板 http://127.0.0.1:8090/**（脚本会自动开这两个）。

### 6.2 真机运行
1. 编辑 `src\teleop\configs\drive.yml`：填 `right.port`/`left.port`（如 `COM5`；共用一条 485 总线则填**相同** port，靠从机地址区分）；`common.speed/force` 起步调小。
2. 启动 MANUS Core、戴手套、确认识别到左右手。
3. 运行 `src\keypoint_streamer\Output\x64\Release\ManusKeypointStreamer_Windows.exe`。
4. `conda activate teleop & cd src\teleop & python -m dexhand_teleop.main`。
5. 打开 **3D 视图 http://127.0.0.1:7000/static/**（验证映射 + 看灵巧手实际状态）与 **调参面板 http://127.0.0.1:8090/**（第 7 节标定/微调）。

> 安全：首次先让 port 留空空跑确认方向/数值正确，再接真机；低速低力起步；面板「松开(归零)」随时可用，退出自动归零。

---

## 7. 校准流程

校准分三层，**只有手套层是必做前提，映射层免标定，驱动层是可选微调**：

| 层 | 是否必须 | 在哪做 | 做什么 |
|---|---|---|---|
| **① 手套** | **必须**（前提） | MANUS Core / Dashboard | 按 MANUS 流程贴合手型标定，保证 RawSkeleton 准确。没做好后续无法补救。 |
| **② 映射** | **免标定** | —— | DexPilot 用相对向量重映射，对人/机手尺寸不敏感，启动即用，无需采姿态。 |
| **③ 驱动行程/手感** | 可选微调 | Web 面板 | 让行程跑满、手感顺滑（见下）。 |

**驱动层微调步骤（面板，实时生效）**
1. 张开手 → 点「**捕获张开**」；用力握拳 → 点「**捕获握拳**」。系统把该范围标定为电机 0–1000 满行程。
2. 闭合幅度整体不够/过度 → 调 **scaling**（≈1.0 起）。
3. 抖动 → 调小 **low_pass** 或加 **寄存器平滑**；写入太频繁 → 加 **deadband**。
4. 个别手指方向反/量程不对 → 逐关节 **invert / qmin / qmax**。
5. 拇指对掌行程不足 → 调 reg0 的 `out_hi` 或 `qmax`。

---

## 8. 调参参考（drive.yml + 面板）

| 参数 | 位置 | 作用 |
|---|---|---|
| scaling_factor | *_dexpilot.yml / 面板 | 人手↔机器手尺寸比 |
| low_pass_alpha | *_dexpilot.yml / 面板 | 重映射低通（小=更平滑更滞后） |
| smoothing_alpha | drive.yml / 面板 | 寄存器空间 EMA（1=关闭） |
| deadband | drive.yml / 面板 | 最小寄存器变化才写入 |
| speed / force | drive.yml / 面板 | 寄存器 6–11 / 12–17 |
| qmin/qmax/out_lo/out_hi/invert | drive.yml / 面板 | 每关节 qpos→寄存器 标定 |
| rate_hz | drive.yml | 控制环+写串口频率 |
| udp.host/port, position_scale | drive.yml | 与流送器对齐；位置单位换算（MANUS 米=1.0） |

---

## 9. 故障排查

| 现象 | 排查 |
|---|---|
| 面板 connected=false | 流送器没跑 / MANUS 未识别手套 / UDP 端口与 `drive.yml` `udp.port` 不一致 |
| 写串口无反应 | COM 口、波特率 115200、从机地址（右1/左2）、RS485 接线与方向 |
| 手指方向/量程不对 | 面板「捕获张开/握拳」标定，或逐关节 invert/qmin/qmax |
| 抖动明显 | 调小 low_pass / 加 smoothing / 加 deadband |
| `OMP: Error #15` | 入口已设 `KMP_DUPLICATE_LIB_OK=TRUE`，正常不出现；手动跑脚本可 `set KMP_DUPLICATE_LIB_OK=TRUE` |
| `pip install pin` 失败 | 不要装 `pin`；pinocchio 用 conda-forge，dex-retargeting 用 `pip install -e .\dex-retargeting --no-deps` |

---

## 10. 关联文档
- 面板字段与运维细节：[src/teleop/README.md](src/teleop/README.md)
- 重映射库来源与更新：[dex-retargeting/VENDORED.md](dex-retargeting/VENDORED.md)
- 硬件寄存器协议：[_protocol_decoded.txt](_protocol_decoded.txt)
