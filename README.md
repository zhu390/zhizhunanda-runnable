# 痣准安达-可运行程序

Jetson 负责相机采集、靶点识别和操作界面，RA6M5 负责机械臂运动和输出控制，两端通过串口通信。项目使用仿真人脸模型进行演示。

## 目录

- `jixiebi_project_20260818/`：Jetson 上位机、视觉算法、模型和相机标定文件。
- `Robot_FreeRTOS-Arm-main/`：RA6M5 / FreeRTOS 固件和 PC 调试工具。

## 界面演示

安装 Python 依赖后，可以在普通电脑上查看界面和模拟画面：

```bash
python -m pip install PyQt5 numpy opencv-python pyserial
cd jixiebi_project_20260818/arm_mole_qt
python -m app.main --demo --detector-mode spot --camera-layout mono
```

该命令不连接机械臂和相机。Linux 下的模拟串口联调入口是 `jixiebi_project_20260818/ra6m5_sim_check.py`：

```bash
cd jixiebi_project_20260818
/usr/bin/python3 ra6m5_sim_check.py --qt-demo
```

## Jetson 整机运行

将 `jixiebi_project_20260818` 部署到 `/home/jetson/jixiebi`，安装 Ultralytics 和与 JetPack 匹配的 PyTorch。包内 Orbbec 运行库对应 ARM64 / Python 3.10，安装方法见 `pyorbbecsdk/README.md`。

相机、串口和标定参数在 `arm_mole_qt/run_full_arm.sh` 中。运行前需要按实际设备修改，并确认 RGB-D 配对和瞄准标定；脚本里的参数来自现有样机。

```bash
cd /home/jetson/jixiebi/arm_mole_qt
bash run_full_arm.sh
```

操作顺序：开始 → 测距 → 采集三视图 → 选择视图 → 开始点痣 → 对准后按住 P000 → 完成。

## 控制板固件

使用 Keil 打开 `Robot_FreeRTOS-Arm-main/Robot_FreeRTOS.uvprojx`。工程使用 RA6M5、ARM Compiler 6.24 和 Renesas RA DFP 6.4.0。编译前将 `rasc_version.txt` 中的 RASC 路径改为本机安装位置。

PC 调试控制台位于 `Robot_FreeRTOS-Arm-main/UpperComputer/`，使用 PySide6 和 pyserial。
