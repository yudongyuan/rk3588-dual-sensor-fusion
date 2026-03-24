# rk3588-dual-sensor-fusion
A dual-light fusion thermal imaging system based on RK3588. Features V4L2 zero-copy capture , YOLOv5 object detection via RKNN , and real-time WebSocket streaming.

## 📖 项目简介 (Overview)
[cite_start]本项目是一个部署在 ARM 架构 RK3588 高性能平台上的端侧多模态视觉系统 [cite: 14, 16][cite_start]。系统通过驱动 HTPA 热成像模块与 SONY IMX415 高清摄像头 [cite: 16][cite_start]，在底层实现 V4L2 零拷贝图像采集 [cite: 20][cite_start]，并在 NPU 上利用 RKNN 异构部署 YOLOv5 进行目标检测 [cite: 22][cite_start]。最终通过 OpenCV 完成空间配准与热成像伪彩融合 [cite: 23][cite_start]，并基于 WebSocket 实现融合视频流的低延迟网络分发与控制 [cite: 24]。

🛠 技术栈 (Tech Stack)

C, C++ System/Architecture: Linux, ARM (RK3588) 

Multimedia/Vision: V4L2, OpenCV 

AI/Deep Learning: YOLOv5, RKNN Toolkit 

Network: WebSocket (libwebsockets) 

Protocols/Drivers: I2C, Device Tree 

---

## 🛠 硬件清单 (Hardware Bill of Materials)

| 硬件组件 (Component) | 规格说明 (Specification) | 接口类型 (Interface) | 作用 (Function) |
| :--- | :--- | :--- | :--- |
| **主控板 (Mainboard)** | 鲁班猫 RK3588 开发板 (如 Firefly / Radxa 等) | - | [cite_start]提供核心 CPU/GPU/NPU 算力及丰富外设接口 [cite: 14, 16] |
| **可见光相机** | SONY IMX415 图像传感器模块 | MIPI CSI-2 | [cite_start]采集高分辨率可见光图像，用于目标检测与场景识别 [cite: 16] |
| **热成像相机** | HTPA 热成像阵列传感器模块 | I2C | [cite_start]获取环境红外辐射数据，用于温度场分析与伪彩生成 [cite: 16] |
| **网络模块** | 千兆以太网口或 Wi-Fi 6 模块 | RJ45 / PCIe | [cite_start]负责与 Web 客户端的高速视频流传输及指令交互 [cite: 24] |
| **周边配件** | 12V电源适配器、杜邦线、HDMI显示器(可选调试用) | - | 系统供电与基础硬件调试 |

---

## ⚙️ 硬件组装与连接步骤 (Hardware Setup)

### 1. 摄像头物理连接
* **IMX415 连接：** 确保开发板断电。将 IMX415 摄像头的 FPC 排线插入 RK3588 对应的 `MIPI CSI` 接口。注意排线的金手指方向需与接口底座引脚对应，扣紧卡扣。
* **HTPA 热成像模块连接：** 使用杜邦线将 HTPA 模块接入 RK3588 的 GPIO 扩展排针组。官网https://www.heimannsensor.com/
* 具体接线如下：
  * `VCC` -> `3.3V`
  * `GND` -> `GND`
  * `SDA` -> `I2C_SDA` (特定 I2C 通道，例如 I2C2_SDA)
  * `SCL` -> `I2C_SCL` (特定 I2C 通道，例如 I2C2_SCL)
<img width="698" height="989" alt="image" src="https://github.com/user-attachments/assets/fa9b7fb1-5905-4225-8341-4ebb13de62e0" />

### 2. 底层驱动与设备树配置 (Device Tree Overlay)
[cite_start]由于热成像模块对 I2C 时序有严格要求，需通过设备树覆盖文件 (`.dtbo`) 修改内核 I2C 参数 ：
1. 编写包含 HTPA 节点和 I2C 时钟频率修改指令的 `htpa-i2c.dts` 文件。
2. 编译设备树：`dtc -I dts -O dtb -o htpa-i2c.dtbo htpa-i2c.dts`
3. 将生成的 `.dtbo` 文件导入 Linux 系统的 `/boot/dtb/overlays/` 目录，并修改 `/boot/uEnv.txt`（或对应引导配置文件）以加载该 Overlay。
4. 重启设备，使用 `i2cdetect -y [总线编号]` 验证热成像模块是否挂载成功。

### 3. V4L2 节点确认
[cite_start]重启后，在终端执行 `ls /dev/video*`，确认 IMX415 是否已成功注册为标准的 V4L2 视频设备节点（如 `/dev/video0`） [cite: 17]。

---

## 🚀 核心软件特性 (Key Software Features)
* [cite_start]**V4L2 零拷贝采集：** 抛弃低效的 read/write 模式，利用 mmap 共享内存机制，直接在用户空间获取 NV12 格式图像的物理内存映射，大幅降低 CPU 占用 。
* [cite_start]**定制化 I2C 交互：** 越过通用驱动限制，直接使用 `ioctl` 操作 I2C 字符设备，实现对 HTPA 寄存器的精细化读写与状态机控制 。
* [cite_start]**多线程并发控制：** 构建采图、推理、融合、推流四级流水线，引入 `pthread_mutex` 与 `pthread_cond` 保障数据帧在队列流转中的线程安全 [cite: 21]。
* [cite_start]**NPU 硬件加速：** 将 YOLOv5 转换为 RKNN 模型，充分压榨 RK3588 的 6TOPS NPU 算力，实现高帧率边缘检测 [cite: 22]。
* [cite_start]**轻量级网络推流：** 集成 `libwebsockets`，将融合后的图像帧高效编码并实时推送到前端页面，支持跨端跨平台查看 [cite: 24]。

---

## 📝 编译与运行 (Build & Run)

在终端命令行中输入对应指令即可
代码块
1 make    #编译出可执行文件app
2 makegdb #进入到gdb调试模式，在执行app时需要输入sudogdbserver:1234./app/dev/i2c-6/dev/i2c-5
3 makerun #自动编译并执行，会输入sudo./app/dev/i2c-5/dev/i2c-5/dev/video11
最终执行和退出
在执行app前，要给app授予下权限，然后再执行，否则无法执行！
使用ctrl+c退出，或者程序命令行中输入exit退出
代码块
sudo chmod 777 app  #授予权限
sudo./app /dev/i2c-5 /dev/i2c-5 /dev/video11#执行








