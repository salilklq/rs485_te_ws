# 灵巧手遥操 Python 服务（dexhand_teleop）

> 📌 **项目总指南（系统要求 / 数据流 / 新机部署 / 启动 / 校准）见仓库根目录的
> [../../README.md](../../README.md)。** 本文档聚焦本 Python 服务的细节（面板、配置、工具、排障）。

把 MANUS 手套的真实手部骨架，用 **dex-retargeting（DexPilot 优化式重映射）** 映射到
LZ-SG002 六自由度灵巧手（左右手），通过 RS485/Modbus-RTU 驱动，并提供实时 Web 调参面板。

取代了旧的「关节角度 → 加权求和 → 线性归一化」C++ 映射（该方案的标定值 `openRaw=0/closedRaw=1`
与「角度(度)」单位不匹配，导致手指稍弯即满握、拇指对掌不准）。

## 架构

```
MANUS Core ─SDK─> [C++ ManusKeypointStreamer]  每手 21 个 MANO 关键点
       │  UDP 127.0.0.1:9001  (协议见 dexhand_teleop/protocol.py)
       ▼
[Python teleop 服务]  左右各一套
   manus_receiver → keypoints(MANO 规范化) → retarget(DexPilot) → qpos(11, 含 mimic)
   → 取 6 个驱动关节 → 0..1000 寄存器 → hand_driver(Modbus-RTU, pyserial)
   ← 读 18..46 反馈（指尖力 / 关节角 / 电机位置）
   + panel(FastAPI 实时调参, 左/右 Tab)
       ▼  RS485：右手从机=1，左手从机=2
```

每只手 URDF 有 11 个关节，硬件只有 6 个执行器。和参考的 Inspire 手一样：每指近端关节是
**优化目标**，远端关节用 URDF `<mimic>` 与近端耦合，于是优化器看到的就是 6 自由度手。
寄存器顺序：`reg0 拇指旋转/对掌, reg1 拇指弯曲, reg2..5 食/中/无名/小指`。

## 目录（仓库根）
```
src/                       ★ 全部自己写的代码
  teleop/                  本目录（Python 服务）
    assets/                retarget 用 URDF（含 mimic + 指尖 frame）+ meshes（自动生成）
    configs/
      right_hand_dexpilot.yml / left_hand_dexpilot.yml   dex-retargeting 配置
      drive.yml            COM/从机地址、关节→寄存器量程、速度/力、平滑、UDP
    dexhand_teleop/        Python 包（protocol/keypoints/manus_receiver/retarget/hand_driver/panel/main）
    tools/
      make_retarget_urdf.py 由 SolidWorks URDF 生成 retarget URDF（已运行过）
      fake_streamer.py     无手套时发合成关键点
      sim_view.py          matplotlib 三维可视化（snapshot/live）
      test_retarget.py / test_pinch.py / test_modbus.py   离线验证（无硬件）
    environment.yml / requirements.txt / README.md
  keypoint_streamer/       C++ 关键点流送器（VS2022），引用 ../../ManusSDK_v3.1.1 的 SDK
ManusSDK_v3.1.1/           第三方：手套 SDK（示例 + include/lib）
dex-retargeting/           第三方：重映射库（vendoring，editable 安装；见其 VENDORED.md）
LZ-SG002-URDF.SLDASM/      输入：灵巧手 CAD/URDF
```

## 1. 安装 Python 环境
```bash
# 在 src/teleop/ 下执行
conda env create -f environment.yml
conda activate teleop
# dex-retargeting 已 vendoring 到仓库根目录 ../../dex-retargeting（与手套 SDK 同层），
# 以 editable 方式装上，运行时直接用这份本地源码：
pip install -e ../../dex-retargeting --no-deps
```
> 为什么是本地 editable 而不是 `pip install dex-retargeting`：①它的依赖 `pin`(pinocchio) 在
> Windows 上没有 wheel、从源码编译会失败,所以 pinocchio 改用 conda-forge,装它时必须 `--no-deps`;
> ②把源码放进工作区(`dex-retargeting/`)使依赖可见、可离线复现、可随时改读。
> 若遇 `OMP: Error #15`,已在 `main.py` 顶部设 `KMP_DUPLICATE_LIB_OK=TRUE`(pinocchio 与 torch 的
> OpenMP 冲突的标准规避)。

## 2. 编译 C++ 关键点流送器（VS2022 / MSBuild）
```bash
# 打开 src/keypoint_streamer/ManusKeypointStreamer.sln，或命令行：
MSBuild src/keypoint_streamer/ManusKeypointStreamer.vcxproj \
        -p:Configuration=Release -p:Platform=x64
```
产物：`src/keypoint_streamer/Output/x64/Release/ManusKeypointStreamer_Windows.exe`（已随附 ManusSDK.dll）。

## 3. 离线自检（无手套、无硬件，强烈建议先跑）
```bash
conda run -n teleop python tools/test_retarget.py --hand right   # 重映射方向/量程合理 -> PASS
conda run -n teleop python tools/test_modbus.py                  # Modbus 帧/CRC -> PASS
# 合成关键点跑通整条链路 + 面板：
conda run -n teleop python tools/fake_streamer.py --mode grasp &
conda run -n teleop python -m dexhand_teleop.main                # 打开 http://127.0.0.1:8090/
```

## 4. 接真机运行
1. 编辑 `configs/drive.yml`：
   - `right.port` / `left.port` 填 COM 口（如 `COM5`）。**留空=空跑**（只算不写串口）。
   - 共用一条 485 总线时，两只手填**相同** port，按从机地址(1/2)区分。
   - `common.speed/force` 起步用较小值；`common.relax_on_exit: true`。
2. 启动 MANUS Core，戴好手套；运行 `ManusKeypointStreamer_Windows.exe`（默认发到 127.0.0.1:9001）。
3. `conda run -n teleop python -m dexhand_teleop.main`，浏览器开 `http://127.0.0.1:8090/`。
4. **安全**：首次低速低力，手离开危险区，随时可点面板「松开(归零)」。

## 5. 面板调参与标定
- **左/右 Tab**：分别调每只手。顶栏显示跟踪状态、速率、是否已连真机。
- **手指条**：蓝=下发寄存器，橙=真机电机位置反馈(+指尖力)。两者应同向跟随。
- **缩放 scaling**：人手/机器手尺寸比。手指闭合不够/过度时调它（≈1.0 起）。
- **低通 low_pass**：越小越平滑但越滞后（抖动大就调小）。
- **寄存器平滑 / 死区**：进一步抑制抖动 / 减少串口写入。
- **标定**：张开手→「捕获张开」，用力握拳→「捕获握拳」，自动把该范围标定为 0–1000 满行程。
- **逐关节高级**：手动改 `qmin/qmax`(弧度)、`out_lo/out_hi`、反向（个别手指方向反了用它）。

## 6. 拇指对掌（主诉问题）
DexPilot 直接优化「拇指指尖↔各指指尖」向量，对掌天然正确。自检里 `pinch/point` 姿态
`reg0`（拇指旋转）显著升高即为正常。若真机对掌行程不足，调 `reg0` 的 `out_hi` 或 `qmax`。

## 故障排查
- **`OMP: Error #15`**：确认用 `python -m dexhand_teleop.main`（已设 `KMP_DUPLICATE_LIB_OK`）；或手动 `set KMP_DUPLICATE_LIB_OK=TRUE`。
- **面板 connected=false**：检查 `ManusKeypointStreamer` 是否在跑、MANUS 是否识别到手套、UDP 端口是否一致（drive.yml `udp.port` 与 exe `--udp-port`）。
- **手指方向/量程不对**：用面板「捕获张开/握拳」标定，或逐关节 `invert`/`qmin/qmax`。
- **写串口无反应**：确认 COM 口、波特率 115200、从机地址（右1/左2）、RS485 接线与方向。
- **pinocchio 加载 URDF 报 mesh 路径**：retarget 不需要 mesh，可忽略；`assets/meshes/` 仅供可选可视化。
