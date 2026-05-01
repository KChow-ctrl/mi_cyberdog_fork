#!/bin/bash
# CyberDog Bridge V2 启动脚本
# 启动 bringup 后启动 bridge_node V2

set -e

HOST="${1:-199.166.55.21}"
PASS="${2:-123}"
DOMAIN="${3:-0}"

echo "=== CyberDog Bridge V2 启动 ==="
echo "目标: $HOST"
echo "ROS_DOMAIN_ID: $DOMAIN"

# 创建 CycloneDDS WiFi 配置
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no mi@$HOST '
  cat > /tmp/cyclonedds_wifi.xml << EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="0">
        <General>
            <NetworkInterfaceAddress>199.166.55.21</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
EOF
  echo "CycloneDDS 配置已创建"
'

# 杀掉旧 bridge_node
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no mi@$HOST \
  'pkill -f bridge_node 2>/dev/null || true; echo "旧进程已清理"'

# 确保 bringup 在跑
echo "检查 bringup 状态..."
BRINGUP_RUNNING=$(sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no mi@$HOST \
  'ps aux | grep "lc_bringup" | grep -v grep | wc -l' 2>/dev/null)
if [ "$BRINGUP_RUNNING" -eq "0" ]; then
  echo "⚠️ bringup 未运行，建议先 sudo systemctl restart cyberdog_ros2"
fi

# 启动 bridge_node V2
echo "启动 bridge_node V2..."
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no mi@$HOST \
  "export CYCLONEDDS_URI=file:///tmp/cyclonedds_wifi.xml && \
   export ROS_DOMAIN_ID=$DOMAIN && \
   export ROS_LOCALHOST_ONLY=0 && \
   cd /home/mi/cyberdog_ws && \
   source /opt/ros2/foxy/setup.bash && \
   source /opt/ros2/cyberdog/setup.bash && \
   source install/setup.bash && \
   exec ./install/lib/cyberdog_bridge/bridge_node &"

sleep 3

# 验证
echo ""
echo "=== 验证 ==="
curl -s http://$HOST:5555/health && echo ""
curl -s http://$HOST:5555/state && echo ""
echo ""
echo "启动完成"
