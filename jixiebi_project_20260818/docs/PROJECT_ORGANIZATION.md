# 项目整理说明

## 现在不要随便移动的目录

这些目录包含正在使用的代码入口，移动会影响 import 或运行命令：

```text
jetson_nano_vision/jetson_vision/
arm_mole_qt/app/
pyorbbecsdk/
```

其中 `pyorbbecsdk/` 是第三方 SDK 源码，体积较大，但 `shuangmu` 和 Qt 深度预览会依赖它。

## 当前文件和目录职责

根目录：

| 文件 | 作用 |
|---|---|
| `README.md` | 当前工程总入口、正式运行方式和安全边界 |
| `视觉协议(5).md` | Jetson 侧当前协议镜像；跨端联调须与 RA6M5 工程根目录 `视觉协议.md` 一致 |
| `项目演示(3).md` | 唯一正式比赛操作流程 |
| `视觉协议(3).md`、`视觉协议(4).md`、`项目演示(2).md` | 历史追溯资料，不参与正式实现 |
| `ra6m5_sim_check.py` | 协议 v5 状态机和伪串口端到端模拟检查 |
| `usb_camera_preview.py` | 简单 USB 摄像头预览辅助工具 |
| 两份 `.docx` | 早期项目方案材料，不是运行时配置或协议依据 |

`arm_mole_qt/` 正式应用：

| 文件 | 作用 |
|---|---|
| `app/main.py` | Qt 页面、顶部 RGB-D/末端单目线程、正式业务状态机和安全门 |
| `app/arm_controller.py` | Qt 对单一 `SerialSender` 的控制封装 |
| `run_qt_app.sh` | Jetson 上启动 Qt 的脚本入口 |
| `install_desktop_launcher.sh` | 安装桌面启动图标 |
| `requirements.txt` | Qt 应用 Python 依赖 |
| `README.md` | 按钮语义、正式时序和启动参数 |

`jetson_nano_vision/jetson_vision/` 公共视觉与协议包：

| 文件 | 作用 |
|---|---|
| `vision_protocol.py` | A5 5A 帧、CRC16、业务字段校验和状态流解析 |
| `serial_sender.py` | 后台心跳/RX、单串口发送、同字节重试、SEQ/ACK/会话门控 |
| `ra6m5_protocol_cli.py` | Qt 退出后的隔离协议台架工具 |
| `protocol_self_test.py` | 协议打包、解析、重同步和非法值自检 |
| `rgbd_distance.py` | RGB 检测框到 Depth 的对象归属和最小三维净距 |
| `scene_guard.py` | SAFE 样本开始后的患者位置、尺度、丢失和流中断守卫 |
| `target_lock.py` | 定靶会话内单目标身份锁和防自动换痣 |
| `mono_mole_target.py` | 末端单目独立预览及显式隔离串口联调 |
| `mole_system.py` | 基准采集、配准、检测的一体化实验工具；串口仅旧协议隔离调试 |
| `mole_preview.py` | 普通黑痣检测预览；串口仅旧协议隔离调试 |
| `mole_visual_follow.py` | 历史视觉跟随台架程序，只保留 legacy 调试 |
| `reference_capture.py`、`reference_mole_preview.py` | 无黑痣基准采集和基准差分预览 |
| `facial_candidate_preview.py` | 人脸区域候选点算法预览 |
| `camera_utils.py`、`camera_test.py`、`snapshot.py` | 相机打开、取流测试和截图 |
| `calib_capture.py`、`calib_solve.py`、`undistort.py` | 单目标定采集、求解和去畸变 |
| `detect_preview.py`、`mole_image_test.py`、`laser_offset_tuner.py` | 检测/偏置离线调参工具 |
| `_compat.py` | 不同 Jetson/Python 环境的兼容辅助 |

`jetson_nano_vision/jetson_vision/vision/` 算法实现：

| 文件 | 作用 |
|---|---|
| `facial_candidate.py` | 排除五官和背景后生成末端单目候选目标 |
| `mole_detect.py` | 肤色掩膜、black-hat 和形状过滤检测 |
| `reference_mole_detect.py` | 无黑痣基准配准和新增暗斑检测 |
| `target_stability.py` | 多帧候选稳定性确认 |
| `rect_detect.py` | 早期矩形靶检测辅助算法 |

`shuangmu/` 是 Astra/RGB-D 独立实验区。`yolo_rgb_depth_distance.py` 只显示 YOLO 与实验距离，正式串口输出已经禁用；`depth_roi_distance.py` 是手工 ROI 测距；`orbbec_*`、`rgb_*` 用于 Depth 点击、快照、预览和标定；`yolo_rgb_capture.py` 用于采集训练图；`openni_depth_test.cpp` 与其二进制用于底层 Depth 测试。

数据与第三方目录：

- `calibration/mono_end_camera/`：末端单目标定图和 `mono_calib_params.npz`。
- `calibration/astra_rgb/`：Astra RGB 标定图和 `rgb_calib_params.npz`；Depth 内参/外参运行时取自 SDK。
- `models/`：YOLO 等模型权重；`datasets/`：训练图片和标签。
- `jetson_nano_vision/reference/`：基准脸、安全距离参考图片和元数据。
- `jetson_nano_vision/test_results/`：算法输出图片，不是正式配置。
- `pyorbbecsdk/`：Orbbec 第三方 SDK 源码、绑定和示例，业务代码不要混入其中；当前 Python 3.10 运行模块及依赖库位于 `pyorbbecsdk/install/lib/`。
- `tests/`：前景归属/净距、场景守卫、串口门控、目标身份锁和人脸候选的单元回归测试。

`jetson_nano_vision/IMPLEMENTATION_PLAN.md`、`PROJECT_UPDATE.md`、`MOLE_DETECTION_TEST_RESULTS.md` 记录过去的实施和算法实验；与 v5/v3 冲突时只以根目录两份正式说明为准。

## 明显属于缓存或环境的内容

这些不是源码，可以后续清理或加入忽略规则：

```text
arm_mole_qt/app/__pycache__/
jetson_nano_vision/jetson_vision/__pycache__/
jetson_nano_vision/jetson_vision/vision/__pycache__/
pyorbbecsdk/build/
pyorbbecsdk/build_*/
pyorbbecsdk/.deps/
pyorbbecsdk/Log/
shuangmu/Log/
```

这些都属于可删除生成物；运行测试或 SDK 后可能再次出现，Git 已忽略其中的 Python/测试缓存。

## 建议的下一阶段目录形态

后续如果要进一步整理，可以逐步收敛成：

```text
jixiebi/
  apps/
    qt_operator/          # 从 arm_mole_qt 迁入
  vision/
    jetson_vision/        # 从 jetson_nano_vision/jetson_vision 迁入
  calibration/
    mono_end_camera/      # 当前末端单目标定图和 mono_calib_params.npz
    astra_rgb/            # Astra RGB 彩色镜头标定图和 rgb_calib_params.npz
    astra_depth/          # Astra Depth 参数记录，内参主要由 Orbbec SDK 读取
    legacy/               # 后续需要长期保存的历史实验数据
  datasets/
    rgbd_snapshots/       # 需要长期保存的 RGB-D 快照
    test_results/         # 检测结果图
  third_party/
    pyorbbecsdk/          # Orbbec SDK
  docs/
```

不过这一步会改很多 import 和运行命令，建议等当前识别/Qt/串口流程稳定以后再做。

## 优先整理顺序

1. 先统一使用 `calibration/mono_end_camera/mono_calib_params.npz` 作为新单目相机参数。
2. 给 Qt 增加 `--calib-params` 和去畸变逻辑，让 Qt 与命令行检测保持一致。
3. 正式串口保持 Qt 单进程所有权，视觉脚本只向 Qt 提供数据；协议行为以 `视觉协议(5).md` 与 RA6M5 工程根目录 `视觉协议.md` 共同确认的一致版本为准。
4. 双目安全距离主线使用 YOLO + Depth，并只在公共测距窗口消费；手动 ROI 只保留为调试工具。
5. 末端单目在 `SELECTED_VIEW_READY` 后用实时帧自动跟踪，禁止复用三视图静态坐标。
6. 最后再考虑大规模重命名目录。

## 相关方案文档

- `docs/CAMERA_CONFIG.md`：当前摄像头身份、固定路径、推荐分辨率、帧率和测试命令。
- `项目演示(3).md`：当前正式比赛操作流程。
- `视觉协议(5).md`：Jetson 侧当前通信协议镜像，须与 RA6M5 正式协议核对一致。
- `docs/WORKFLOW_PLAN.md`：Qt 单串口主控、RA6M5 固化点位、公共测距锁存、医生选择视图和末端单目自动定靶的完整流程方案。
