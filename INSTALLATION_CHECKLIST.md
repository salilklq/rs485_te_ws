# 新电脑安装与真机运行清单

本文用于从 GitHub 克隆本项目后，在 Windows 新电脑上部署 MANUS 数据手套到 LZ-SG002 灵巧手的遥操作环境。

> 仓库地址：<https://github.com/salilklq/rs485_te_ws>
>
> 目标平台：Windows 10/11 x64

## 1. 硬件清单

- MANUS 数据手套、接收器/加密狗及有效授权
- LZ-SG002 灵巧手
- 48 V 电源及配套供电线
- USB 转 RS485 适配器
- RS485 通信线
- 急停或可快速切断的电源装置（建议）

通信参数：

| 项目 | 配置 |
|---|---|
| 协议 | Modbus-RTU |
| 波特率 | 115200 |
| 数据格式 | 8N1 |
| 右手从机地址 | 1 |
| 左手从机地址 | 2 |

## 2. 必须安装的软件和驱动

### 2.1 Git

用于克隆和更新代码：

<https://git-scm.com/download/win>

```powershell
git clone https://github.com/salilklq/rs485_te_ws.git
cd rs485_te_ws
```

该仓库是私有仓库，克隆账号必须已获得仓库访问权限。

### 2.2 Anaconda 或 Miniconda

推荐安装 Miniconda：

<https://docs.conda.io/projects/miniconda/en/latest/>

项目使用 Python 3.12，具体依赖由 `src/teleop/environment.yml` 安装。

### 2.3 Visual Studio 2022 或 Build Tools 2022

下载地址：

<https://visualstudio.microsoft.com/downloads/>

安装时必须勾选：

- Desktop development with C++（使用 C++ 的桌面开发）
- MSVC v143 C++ x64/x86 build tools
- Windows 10 SDK 或 Windows 11 SDK
- C++ CMake tools for Windows（建议）

### 2.4 MANUS Core

从 MANUS 官方渠道下载并安装与手套及授权匹配的 MANUS Core。安装后必须完成：

- 登录并激活授权
- 连接手套及接收器
- 更新固件（如 MANUS Core 要求）
- 在 Dashboard 中完成左右手识别
- 按 MANUS 流程完成手型标定
- 确认 Dashboard 能持续输出手部数据

MANUS 文档：

<https://docs.manus-meta.com/>

### 2.5 MANUS SDK v3.1.1 Windows 版

从 MANUS 官方开发者渠道下载 MANUS SDK v3.1.1 Windows 版，并将 SDK 文件放到以下固定目录：

```text
ManusSDK_v3.1.1\SDKMinimalClient_Windows\ManusSDK\
├── include\
│   ├── ManusSDK.h
│   ├── ManusSDKTypeInitializers.h
│   └── ManusSDKTypes.h
└── lib\
    ├── ManusSDK.lib
    └── ManusSDK.dll
```

这些文件是编译和运行 `ManusKeypointStreamer_Windows.exe` 的必要依赖。

> 注意：仓库的 `.gitignore` 排除了 `*.dll`、`*.lib` 和 `*.exe`，因此从 GitHub 克隆后不会包含 `ManusSDK.lib`、`ManusSDK.dll` 和已经编译好的流送器。必须从官方 SDK 补齐，或者从已经部署成功的电脑复制同版本文件。

### 2.6 USB-RS485 适配器驱动

根据适配器芯片安装对应 Windows 驱动，常见芯片包括：

- FTDI FT232
- WCH CH340/CH341
- Silicon Labs CP210x

安装后在“设备管理器 -> 端口 (COM 和 LPT)”中确认设备没有警告标志，并记下实际 COM 口，例如 `COM5`。

## 3. 创建 Python 环境

在仓库根目录执行：

```powershell
conda env create -f .\src\teleop\environment.yml
conda activate teleop
pip install -e .\dex-retargeting --no-deps
```

不要在 Windows 上直接执行 `pip install pin`。本项目通过 conda-forge 安装 Pinocchio，`dex-retargeting` 必须使用 `--no-deps` 安装。

验证主要依赖：

```powershell
python -c "import pinocchio, torch, nlopt, serial, fastapi, meshcat; print('Python dependencies: OK')"
```

## 4. 编译 MANUS 关键点流送器

打开“Developer PowerShell for VS 2022”，切换到仓库根目录后执行：

```powershell
MSBuild .\src\keypoint_streamer\ManusKeypointStreamer.vcxproj `
  -p:Configuration=Release `
  -p:Platform=x64
```

成功后应生成：

```text
src\keypoint_streamer\Output\x64\Release\
├── ManusKeypointStreamer_Windows.exe
└── ManusSDK.dll
```

如果提示找不到 `ManusSDK.h`、`ManusSDK.lib` 或 `ManusSDK.dll`，检查第 2.5 节所列目录是否完整。

## 5. 离线验证

先不要连接或驱动灵巧手。在仓库根目录运行：

```powershell
conda activate teleop
cd .\src\teleop
python tools\test_retarget.py --hand right
python tools\test_modbus.py
```

也可以双击仓库根目录的 `run_panel_sim.bat`。它使用合成手套数据，不向串口写入指令。

浏览器页面：

- 3D 视图：<http://127.0.0.1:7000/static/>
- 调参面板：<http://127.0.0.1:8090/>

## 6. 真机配置

编辑 `src/teleop/configs/drive.yml`：

1. 将 `right.port` 和/或 `left.port` 改为设备管理器显示的实际 COM 口。
2. 只使用一只手时，另一只手设置 `enabled: false`。
3. 右手从机地址使用 `slave: 1`，左手使用 `slave: 2`。
4. 两只手共用一条 RS485 总线时，两边填写相同 COM 口，通过从机地址区分。
5. 首次运行前降低 `common.speed` 和 `common.force`。

## 7. 首次运行安全检查

当前仓库配置中的 `speed: 1000`、`force: 1000` 是最大值，不应直接用于新设备首次运行。

推荐顺序：

1. 先把所有 `port` 留空，运行完整链路进行空跑。
2. 在 3D 视图确认左右手、手指方向、张开和闭合方向正确。
3. 断电状态下检查 48 V 电源和 RS485 的 A/B/GND 接线。
4. 填写真实 COM 口，并先使用较低速度和力度。
5. 保持手和其他物体远离夹持区域，确保可以立即断电。
6. 启动 MANUS Core，确认手套已经标定并稳定出数。
7. 运行 `run_teleop.bat`。
8. 如方向、量程或反馈异常，立即停止并检查 `invert`、`qmin/qmax`、`out_lo/out_hi` 和从机地址。

退出程序时会尝试将位置寄存器归零，但不能将软件退出替代硬件急停或断电措施。

## 8. 可选的 3D 网格文件

`src/teleop/assets/meshes/` 被 Git 忽略，因此新克隆不会包含其中的 STL 副本。这些文件只影响 MeshCat 中的实体外观显示，不影响关键点接收、重映射计算或 RS485 真机驱动。

需要完整 3D 外观时，可使用仓库已跟踪的 CAD/URDF 网格重新生成：

```powershell
conda activate teleop
cd .\src\teleop
python tools\make_retarget_urdf.py `
  ..\..\LZ-SG002-URDF.SLDASM\LZ-SG002-URDF-R.SLDASM\urdf\LZ-SG002-URDF-R.SLDASM.urdf `
  assets\right_hand.urdf
python tools\make_retarget_urdf.py `
  ..\..\LZ-SG002-URDF.SLDASM\LG-SG002-URDF-L.SLDASM\urdf\LG-SG002-URDF-L.SLDASM.urdf `
  assets\left_hand.urdf
```

## 9. 部署完成检查表

- [ ] Windows 10/11 x64
- [ ] Git 可用
- [ ] Conda 可用，`teleop` 环境已创建
- [ ] `dex-retargeting` 已以 editable 方式安装
- [ ] Visual Studio/Build Tools 的 v143 和 Windows SDK 已安装
- [ ] MANUS Core 已安装并授权
- [ ] MANUS 手套已识别和标定
- [ ] MANUS SDK v3.1.1 的 include/lib/dll 已放到固定目录
- [ ] `ManusKeypointStreamer_Windows.exe` 已成功编译
- [ ] USB-RS485 驱动正常，COM 口已确认
- [ ] 灵巧手 48 V 电源和 RS485 接线已检查
- [ ] 从机地址正确（右手 1、左手 2）
- [ ] 离线测试通过
- [ ] 已先完成空跑
- [ ] 首次真机运行已降低速度和力度
