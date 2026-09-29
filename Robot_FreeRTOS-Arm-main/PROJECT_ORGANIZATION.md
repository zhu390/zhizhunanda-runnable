# 项目整理说明

## 现在不要随便移动的目录

这些目录包含正在使用的代码入口，移动会影响 import 或运行命令：

```text
jetson_nano_vision/jetson_vision/
arm_mole_qt/app/
pyorbbecsdk/
```

其中 `pyorbbecsdk/` 是第三方 SDK 源码，体积较大，但 `shuangmu` 和 Qt 深度预览会依赖它。

## 当前建议保留的主文件

视觉主流程：

```text
jetson_nano_vision/jetson_vision/mole_system.py
jetson_nano_vision/jetson_vision/reference_capture.py
jetson_nano_vision/jetson_vision/vision/reference_mole_detect.py
jetson_nano_vision/jetson_vision/vision/mole_detect.py
jetson_nano_vision/jetson_vision/vision_protocol.py
jetson_nano_vision/jetson_vision/serial_sender.py
```

Qt 主流程：

```text
arm_mole_qt/app/main.py
arm_mole_qt/app/arm_controller.py
arm_mole_qt/run_qt_app.sh
arm_mole_qt/install_desktop_launcher.sh
```

新单目标定：

```text
shuangmu/mono_calib_images/
shuangmu/mono_calib_params.npz
```

Astra / RGB-D 实验：

```text
shuangmu/orbbec_rgb_depth_snapshot.py
shuangmu/orbbec_depth_click_3d.py
shuangmu/rgb_depth_*/
pyorbbecsdk/
```

## 明显属于缓存或环境的内容

这些不是源码，可以后续清理或加入忽略规则：

```text
arm_mole_qt/app/__pycache__/
jetson_nano_vision/jetson_vision/__pycache__/
jetson_nano_vision/jetson_vision/vision/__pycache__/
pyorbbecsdk/build/
pyorbbecsdk/Log/
shuangmu/Log/
```

这些生成物已经清理过；如果后续运行又生成，可以继续删除。

## 建议的下一阶段目录形态

后续如果要进一步整理，可以逐步收敛成：

```text
jixiebi/
  apps/
    qt_operator/          # 从 arm_mole_qt 迁入
  vision/
    jetson_vision/        # 从 jetson_nano_vision/jetson_vision 迁入
  calibration/
    mono_current/         # 当前单目标定图和 mono_calib_params.npz
    legacy/               # 后续需要长期保存的历史实验数据
  datasets/
    rgbd_snapshots/       # shuangmu/rgb_depth_* 数据
    test_results/         # 检测结果图
  third_party/
    pyorbbecsdk/          # Orbbec SDK
  docs/
```

不过这一步会改很多 import 和运行命令，建议等当前识别/Qt/串口流程稳定以后再做。

## 优先整理顺序

1. 先统一使用 `shuangmu/mono_calib_params.npz` 作为新单目相机参数。
2. 给 Qt 增加 `--calib-params` 和去畸变逻辑，让 Qt 与命令行检测保持一致。
3. 明确 `NO_TARGET`、`ALIGN_FAILED`、安全阻断在串口协议里的表达。
4. 把旧测试结果和 RGB-D 快照移动到 `datasets/legacy/` 或 `datasets/rgbd_snapshots/`。
5. 最后再考虑大规模重命名目录。

## 相关方案文档

- `docs/WORKFLOW_PLAN.md`：Qt 主控、RA6M5 预置动作、Astra 安全距离、医生选痣、单目精定位和点痣请求的完整流程方案。
