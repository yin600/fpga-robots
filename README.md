# 异构协同具身智能移动机器人 — 完整源码

## 目录结构

```
机器人完整源码/
├── fpga/                    FPGA 端（任务 1~4，可直接烧录）
│   ├── rtl/                 16 个 Verilog 模块（顶层 robot_top.v）
│   ├── tb/                  3 个测试台（iverilog 仿真）
│   ├── constraints/         引脚约束 acg720_robot.cst + 时序约束 robot.sdc + 接线表
│   └── tools/regtool.py     PC 端串口调试工具
├── rk3588/                  RK3588 端（任务 5~6 + 声控）
│   ├── uart_link.py         FPGA 串口协议
│   ├── detector.py          YOLO 目标检测（RKNN）
│   ├── voice.py             语音识别（Vosk 离线中文）
│   ├── demo_voice.py        声控演示入口
│   └── demo_track.py        目标跟踪演示入口
└── README.md
```

## 一、FPGA 端：编译与烧录

1. 安装高云云源软件 Gowin EDA（本目录已含 `Gowin_V1.9.12_x64_win`）。
2. 新建工程，器件选择 `GW5AT-LV138PG484A`（即 ACG720 板载芯片）。
3. 将 `fpga/rtl/` 下全部 `.v` 文件加入工程，顶层模块设为 `robot_top`。
4. 加入约束文件 `fpga/constraints/acg720_robot.cst` 与 `fpga/constraints/robot.sdc`。
5. 综合 → 布局布线 → 生成比特流（`.fs`），用 Gowin Programmer 烧录。
6. 接线严格按 `fpga/constraints/接线表.md`（P7 扩展排针 40 针逐针对应）。

上板前必须现场确认（README 的"上板后必须现场确认"一节）：
- GPIO0 BANK 电压为 3.3V；
- 电机转向参数 `MOT_INV_L/R`、编码器方向 `ENC_INV_L/R`（前进时左右 `spd` 均为正）；
- IMU 轴向符号（左转时 yaw 应增大，否则改 `GZS`）；
- `KF_SPD` 按电机实际最大转速标定；
- 3S 电池满电 12.6V 超 ACG720 输入上限，必须经降压模块再接开发板。

## 二、串口协议（FPGA ↔ 上位机，115200 8N1）

命令帧（PC→FPGA，6 字节）：`0x55, cmd, addr, hi, lo, chk`，`chk=(cmd+addr+hi+lo)&0xFF`
- `cmd=0x01` 写寄存器；`cmd=0x02` 读寄存器（FPGA 回 `0x55,0x82,addr,hi,lo,chk`）

遥测帧（FPGA→PC，每 20ms，19 字节）：`0xA5,0x5A, 8×16bit 大端, chk`
- 字顺序：roll / pitch / yaw（Q7 度，即 度×128）、距离左/中/右（mm）、左速 / 右速（计数/10ms）

关键寄存器：`0x00` 模式、`0x01` 线速度、`0x02` 角速度、`0x03` 转角、`0x04` 控制
- 模式：`0` 停车、`1` 手动 v/w、`2` 直行+避障、`3` 原地旋转、`4` 航点任务
- 速度单位：编码器计数 / 10ms（四倍频后）；w>0 = 左转

## 三、RK3588 端：部署与运行

### 1. 安装依赖
```
pip install rknnlite numpy opencv-python pyserial sounddevice vosk
```
（`rknnlite` 需与板载系统版本匹配，按正点原子 RK3588 镜像说明安装）

### 2. 准备模型
- 目标检测：PyTorch 训练 YOLO → 导出 ONNX → RKNN-Toolkit2 转 INT8 `.rknn`
  （检测结果需解码为 `[cx,cy,w,h,obj,cls...]` 的 `[1,N,85]` 格式，person 类 id=0）
- 语音：下载 Vosk 中文小模型（如 `vosk-model-small-cn-0.22`），解压后传入 `--vosk-model`

### 3. 运行
```
python3 demo_voice.py --port /dev/ttyS1 --vosk-model ./vosk-model-small-cn-0.22
python3 demo_track.py --port /dev/ttyS1 --model ./best.rknn --cam 0
```
- `--port`：RK3588 连接 FPGA 的串口设备名（TTL 3.3V，共地）
- 目标跟踪采用视觉伺服：方位误差→角速度，框高误差→线速度；前向超声波 <250mm 时强制停车

## 四、演示对应关系

| 演示段 | 功能 | 代码入口 |
|---|---|---|
| 任务1 实时画面 | FPGA 采集+姿态叠加 | fpga 端自动运行 |
| 任务2 运动控制 | 前进/后退/左转/右转/原地旋转 | regtool.py 写 0x00/0x01/0x02 |
| 任务3 避障 | 直行+分级避障 | regtool.py 设 mode=2 |
| 任务4 航点导航 | 航点状态机 | regtool.py 写航点+mode=4 |
| 任务5/6 目标跟踪 | YOLO+跟踪 | demo_track.py |
| 声控 | 语音→运动指令 | demo_voice.py |

## 五、说明

- FPGA 端源码来自 `选题三_FPGA机器人_RTL起步包`（含可读注释，注释不影响综合）；RK3588 端为新写代码。
- 所有运动增益（`FORWARD_SPEED`、`KP_TURN`、`KP_DIST` 等）需按实际底盘标定后微调。
- 目标跟踪在 mode=1 下运行，超声波安全停车由 `demo_track.py` 主动读取遥测实现。
