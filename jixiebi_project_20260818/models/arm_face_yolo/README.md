# Arm/Face YOLO Model

正式运行模型：

```text
models/arm_face_yolo/best.pt
```

类别：

```text
0: arm_tip
1: face
```

实时双目测距入口：

```bash
cd ~/jixiebi/shuangmu
DISPLAY=:0 PYTHONPATH=../pyorbbecsdk/install/lib python3 yolo_rgb_depth_distance.py
```

训练结果摘要：

```text
训练图片: 579
最佳轮次: epoch 52
Precision: 0.998
Recall: 1.000
mAP50: 0.995
mAP50-95: 0.811
```

当前正式模型已于 2026-07-20 更新，覆盖原始方向和新安装方向采集画面。

`training_report/` 中保留了训练曲线和混淆矩阵，便于后续复查。
