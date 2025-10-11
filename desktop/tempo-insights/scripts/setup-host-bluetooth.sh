#!/bin/bash
# ===================================
# Task 170: Setup Host Bluetooth
# ===================================
# Installs and configures BlueZ on the host system
# Required for bluetooth-scanner container to access hardware
#
# Usage:
#   sudo ./scripts/setup-host-bluetooth.sh

set -e

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ Error: This script must be run with sudo"
    echo "Usage: sudo ./scripts/setup-host-bluetooth.sh"
    exit 1
fi

echo "=========================================="
echo "Host Bluetooth Setup"
echo "=========================================="
echo ""
echo "This script configures BlueZ on the host system"
echo "for use with the Tempo Insights bluetooth-scanner."
echo ""

# ===================================
# Step 1: Install BlueZ
# ===================================
echo "Step 1: Installing BlueZ packages..."
echo "-------------------------------------------"

apt-get update

apt-get install -y \
    bluez \
    libbluetooth-dev \
    bluetooth \
    bluez-tools

echo "✓ BlueZ packages installed"
echo ""

# ===================================
# Step 2: Verify Installation
# ===================================
echo "Step 2: Verifying installation..."
echo "-------------------------------------------"

# Check bluetoothd version
if command -v bluetoothd &> /dev/null; then
    BLUEZ_VERSION=$(bluetoothd --version)
    echo "✓ bluetoothd version: $BLUEZ_VERSION"
else
    echo "❌ bluetoothd not found"
    exit 1
fi

# Check hciconfig
if command -v hciconfig &> /dev/null; then
    echo "✓ hciconfig available"
else
    echo "⚠️  hciconfig not available (deprecated in newer BlueZ)"
fi

echo ""

# ===================================
# Step 3: Enable Bluetooth Service
# ===================================
echo "Step 3: Enabling Bluetooth service..."
echo "-------------------------------------------"

systemctl enable bluetooth
systemctl start bluetooth

if systemctl is-active --quiet bluetooth; then
    echo "✓ Bluetooth service running"
else
    echo "❌ Bluetooth service failed to start"
    systemctl status bluetooth
    exit 1
fi

echo ""

# ===================================
# Step 4: Check Bluetooth Adapter
# ===================================
echo "Step 4: Checking Bluetooth adapter..."
echo "-------------------------------------------"

# List Bluetooth adapters
if command -v hciconfig &> /dev/null; then
    echo "Bluetooth adapters:"
    hciconfig -a || true
else
    # Use bluetoothctl for newer systems
    echo "Bluetooth controllers:"
    bluetoothctl list || true
fi

echo ""

# Check if any adapter is present
if ls /sys/class/bluetooth/hci* > /dev/null 2>&1; then
    echo "✓ Bluetooth adapter(s) detected"
    ls -la /sys/class/bluetooth/
else
    echo "⚠️  No Bluetooth adapters found"
    echo ""
    echo "If running on Raspberry Pi 5:"
    echo "  - Built-in Bluetooth should be available"
    echo "  - Check 'sudo dmesg | grep -i bluetooth'"
    echo "  - Ensure Bluetooth is enabled in config.txt"
fi

echo ""

# ===================================
# Step 5: Test Scanning
# ===================================
echo "Step 5: Testing Bluetooth scan..."
echo "-------------------------------------------"

echo "Attempting 5-second BLE scan..."
timeout 5s bluetoothctl scan on || true

echo ""
echo "If devices were found above, Bluetooth is working!"
echo ""

# ===================================
# Step 6: Configure D-Bus Permissions
# ===================================
echo "Step 6: Configuring D-Bus permissions..."
echo "-------------------------------------------"

# Create D-Bus policy for Docker containers
DBUS_POLICY="/etc/dbus-1/system.d/bluetooth-container.conf"

cat > "$DBUS_POLICY" <<'EOF'
<!DOCTYPE busconfig PUBLIC "-//freedesktop//DTD D-BUS Bus Configuration 1.0//EN"
 "http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <policy context="default">
    <allow send_destination="org.bluez"/>
    <allow send_interface="org.bluez.Adapter1"/>
    <allow send_interface="org.bluez.Device1"/>
    <allow send_interface="org.freedesktop.DBus.ObjectManager"/>
    <allow send_interface="org.freedesktop.DBus.Properties"/>
  </policy>
</busconfig>
EOF

echo "✓ D-Bus policy created: $DBUS_POLICY"

# Reload D-Bus
systemctl reload dbus

echo "✓ D-Bus reloaded"
echo ""

# ===================================
# Step 7: Verify D-Bus Access
# ===================================
echo "Step 7: Verifying D-Bus access..."
echo "-------------------------------------------"

if [ -S /run/dbus/system_bus_socket ]; then
    echo "✓ D-Bus system socket exists"
    ls -la /run/dbus/system_bus_socket
else
    echo "❌ D-Bus system socket not found"
fi

echo ""

# ===================================
# Summary
# ===================================
echo "=========================================="
echo "Host Bluetooth Setup Complete"
echo "=========================================="
echo ""
echo "✅ BlueZ installed and configured"
echo "✅ Bluetooth service enabled"
echo "✅ D-Bus permissions configured"
echo ""
echo "Next steps:"
echo "1. Verify adapter works:"
echo "   bluetoothctl"
echo "   > scan on"
echo ""
echo "2. Start Tempo bluetooth-scanner:"
echo "   docker compose up -d tempo-bt-scanner"
echo ""
echo "3. Check scanner logs:"
echo "   docker compose logs -f tempo-bt-scanner"
echo ""
echo "Troubleshooting:"
echo "- If scan fails: sudo hciconfig hci0 up"
echo "- Check adapter: ls /sys/class/bluetooth/"
echo "- View logs: journalctl -u bluetooth -f"
echo ""