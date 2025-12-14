#!/bin/bash
set -e

DEVICE_IP="192.168.43.1:5555"

echo ""
echo "=== Robot Connect ==="
echo ""

export PATH="/Users/$(whoami)/Library/Android/sdk/platform-tools:$PATH"

# Ensure adb server is healthy
adb kill-server >/dev/null 2>&1
sleep 3
adb start-server >/dev/null

echo "Connecting to device: $DEVICE_IP"

# Disconnect first to avoid stale sessions
adb disconnect "$DEVICE_IP" >/dev/null 2>&1 || true

# Try connecting (retry up to 3 times)
for i in {1..3}; do
    if adb connect "$DEVICE_IP" 2>/dev/null | grep -q "connected"; then
        break
    fi
    sleep 1
done

# Verify connection
if adb devices | grep -q "$DEVICE_IP.*device"; then
    echo "✅ ADB connected to $DEVICE_IP"
else
    echo "❌ Failed to connect to $DEVICE_IP"
    adb devices
    exit 1
fi
