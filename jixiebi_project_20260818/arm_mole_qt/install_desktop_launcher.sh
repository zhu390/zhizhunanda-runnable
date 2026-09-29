#!/usr/bin/env bash
set -e

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
DESKTOP_DIR="$HOME/Desktop"
LAUNCHER="$DESKTOP_DIR/机械臂点痣装置.desktop"

mkdir -p "$DESKTOP_DIR"

cat > "$LAUNCHER" <<EOF
[Desktop Entry]
Type=Application
Name=机械臂点痣装置
Comment=Jetson Nano 离线机械臂点痣装置 Qt 界面
Exec=bash -lc 'cd "$APP_DIR" && python3 -m app.main'
Terminal=false
Categories=Utility;
EOF

chmod +x "$LAUNCHER"
chmod +x "$APP_DIR/run_qt_app.sh"

echo "已创建桌面图标: $LAUNCHER"
