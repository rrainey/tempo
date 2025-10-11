# Prerequisites Checklist

Complete checklist for deploying Tempo Insights on Raspberry Pi 5.

---

## Hardware Requirements

### ✅ Required Hardware

- [ ] **Raspberry Pi 5**
  - Minimum: 4GB RAM (development/QA)
  - Recommended: 8GB RAM (production)
  - CPU: ARM Cortex-A76 (quad-core, 2.4GHz)

- [ ] **Storage**
  - Minimum: 64GB microSD card (Class 10, UHS-I)
  - Recommended: 128GB+ NVMe SSD via PCIe adapter
  - Expected usage: ~10GB system + 20-800GB data

- [ ] **Power Supply**
  - Official Raspberry Pi 5 27W USB-C power adapter
  - Or: USB-C PD power supply (5V/5A minimum)

- [ ] **Network**
  - Ethernet connection (recommended)
  - Or: Built-in Wi-Fi (802.11ac)
  - Static IP recommended for production

### ✅ Optional Hardware

- [ ] **Tempo-BT Devices**
  - At least one device for testing
  - Multiple devices for production

- [ ] **UPS (Uninterruptible Power Supply)**
  - Recommended for production
  - Prevents data corruption on power loss

- [ ] **Cooling**
  - Active cooling fan (recommended for 24/7 operation)
  - Heatsink case
  - Ambient temperature <30°C (86°F)

- [ ] **Peripherals** (for initial setup)
  - USB keyboard
  - Monitor (micro-HDMI)
  - Or: SSH over network

---

## Software Requirements

### ✅ Operating System

- [ ] **Ubuntu 22.04 LTS (64-bit ARM)**
  - Download: https://ubuntu.com/download/raspberry-pi
  - Minimum: Ubuntu Server 22.04.3
  - Alternative: Raspberry Pi OS (64-bit) with modifications
  
- [ ] **Operating System Installed**
  - Flashed to microSD/SSD using Raspberry Pi Imager
  - Initial boot completed
  - User account created

### ✅ System Access

- [ ] **SSH Access Configured**
  - SSH enabled (default on Ubuntu Server)
  - SSH key authentication set up (recommended)
  - Firewall configured for SSH (port 22)

- [ ] **Sudo Privileges**
  - User has sudo access
  - Password configured
  - Can run: `sudo apt-get update`

### ✅ Network Configuration

- [ ] **Network Connected**
  - Internet access verified: `ping google.com`
  - DNS resolution working
  - NTP synchronized: `timedatectl status`

- [ ] **Hostname Set** (optional)
  ```bash
  sudo hostnamectl set-hostname tempo-insights
  ```

- [ ] **Static IP** (recommended for production)
  ```bash
  # Edit netplan configuration
  sudo nano /etc/netplan/50-cloud-init.yaml
  ```

---

## Pre-Installation Checks

### ✅ System Updates

```bash
# Update package lists
sudo apt-get update

# Upgrade installed packages
sudo apt-get upgrade -y

# Reboot if kernel updated
sudo reboot
```

### ✅ Disk Space

```bash
# Check available space (need 20GB+ free)
df -h /

# Output should show:
# Filesystem      Size  Used Avail Use% Mounted on
# /dev/mmcblk0p2   59G  3.5G   53G   7% /
```

- [ ] At least 20GB free space available
- [ ] No disk errors: `sudo dmesg | grep -i error`

### ✅ Memory

```bash
# Check RAM
free -h

# Output should show total RAM:
#               total        used        free
# Mem:          7.6Gi       500Mi       6.5Gi
```

- [ ] Minimum 4GB total RAM
- [ ] Swap configured (optional but recommended)

### ✅ Bluetooth Hardware

```bash
# Check for Bluetooth adapter
ls /sys/class/bluetooth/

# Should show: hci0 (or similar)
```

- [ ] Built-in Bluetooth detected
- [ ] Adapter shows up in system

---

## Software Dependencies (Auto-Installed)

The `configure-instance.sh` script installs these automatically, but verify prerequisites:

### ✅ Docker

Will be installed by script. Manual verification:
```bash
docker --version
# Should be: Docker version 24.0+ or later
```

### ✅ Node.js

Will be installed by script. Required version:
```bash
node --version
# Should be: v20.x.x or later
```

### ✅ BlueZ

Will be installed by script. Required for Bluetooth:
```bash
bluetoothd --version
# Should be: 5.6+ or later
```

---

## Network Ports

### ✅ Required Ports Available

Check these ports are not in use:

```bash
# Check port availability
sudo lsof -i :3000   # Tempo web app
sudo lsof -i :8000   # Supabase Kong gateway
sudo lsof -i :54322  # PostgreSQL (Supabase)
sudo lsof -i :6543   # Supavisor pooler
```

- [ ] Port 3000 available (web application)
- [ ] Port 8000 available (Supabase API)
- [ ] Port 54322 available (PostgreSQL direct)
- [ ] Port 6543 available (connection pooler)

### ✅ Firewall Configuration

```bash
# Check firewall status
sudo ufw status

# If active, ensure SSH is allowed
sudo ufw allow 22/tcp
```

- [ ] Firewall configured (if enabled)
- [ ] SSH access maintained
- [ ] Ports 3000, 8000 will be opened during install

---

## Security Prerequisites

### ✅ Access Control

- [ ] **Strong passwords set**
  - User account password (20+ characters)
  - Root disabled or strong password
  
- [ ] **SSH key authentication** (recommended)
  ```bash
  # On your local machine
  ssh-copy-id user@raspberry-pi-ip
  ```

- [ ] **No default passwords in use**
  - Changed default Ubuntu/Pi passwords
  - Will set strong passwords during deployment

### ✅ System Security

- [ ] **Automatic security updates enabled**
  ```bash
  sudo apt-get install -y unattended-upgrades
  sudo dpkg-reconfigure -plow unattended-upgrades
  ```

- [ ] **Fail2ban** (optional but recommended)
  ```bash
  sudo apt-get install -y fail2ban
  sudo systemctl enable fail2ban
  ```

---

## Environment-Specific Requirements

### Development Environment

- [ ] Can use weaker passwords (for convenience)
- [ ] Network access from development machine
- [ ] Git configured with credentials

### QA/Staging Environment

- [ ] Separate from development
- [ ] Real secrets (not development defaults)
- [ ] Network isolated from production

### Production Environment

- [ ] **UPS installed and configured**
- [ ] **Static IP address assigned**
- [ ] **Backup strategy planned**
- [ ] **Monitoring plan in place**
- [ ] **24/7 power and network**
- [ ] **Physical security for hardware**
- [ ] **Off-site backup storage**

---

## Pre-Deployment Testing

### ✅ System Stability Test

```bash
# Run stress test (optional)
sudo apt-get install -y stress
stress --cpu 4 --timeout 60s

# Monitor temperature
vcgencmd measure_temp
# Should stay below 80°C under load
```

- [ ] System stable under load
- [ ] Temperature acceptable (<80°C)
- [ ] No thermal throttling

### ✅ Bluetooth Test

```bash
# Scan for Bluetooth devices
sudo bluetoothctl
> scan on
# Should see nearby Bluetooth devices
> exit
```

- [ ] Bluetooth scanning works
- [ ] Can detect nearby devices

### ✅ Internet Connectivity

```bash
# Test DNS
nslookup github.com

# Test HTTPS
curl -I https://github.com

# Test Docker Hub
curl -I https://hub.docker.com
```

- [ ] DNS resolution working
- [ ] HTTPS connections successful
- [ ] Can reach Docker Hub

---

## Installation Readiness Checklist

Before running `configure-instance.sh`:

- [ ] All hardware connected and powered
- [ ] Ubuntu 22.04 installed and booted
- [ ] Network connection active
- [ ] SSH access working
- [ ] Sudo privileges confirmed
- [ ] At least 20GB free disk space
- [ ] All required ports available
- [ ] Bluetooth hardware detected
- [ ] System updated (`apt-get update && upgrade`)
- [ ] No critical system errors in `dmesg`

## Quick Verification Script

```bash
#!/bin/bash
# verify-prerequisites.sh

echo "Checking prerequisites..."

# OS
if grep -q "Ubuntu 22.04" /etc/os-release; then
    echo "✓ Ubuntu 22.04 detected"
else
    echo "❌ Not Ubuntu 22.04"
fi

# Disk space
FREE_GB=$(df -BG / | tail -1 | awk '{print $4}' | sed 's/G//')
if [ "$FREE_GB" -gt 20 ]; then
    echo "✓ Sufficient disk space (${FREE_GB}GB free)"
else
    echo "❌ Insufficient disk space (${FREE_GB}GB free, need 20GB+)"
fi

# RAM
TOTAL_GB=$(free -g | grep Mem | awk '{print $2}')
if [ "$TOTAL_GB" -ge 4 ]; then
    echo "✓ Sufficient RAM (${TOTAL_GB}GB)"
else
    echo "❌ Insufficient RAM (${TOTAL_GB}GB, need 4GB+)"
fi

# Bluetooth
if ls /sys/class/bluetooth/hci* > /dev/null 2>&1; then
    echo "✓ Bluetooth adapter detected"
else
    echo "❌ No Bluetooth adapter found"
fi

# Internet
if ping -c 1 github.com > /dev/null 2>&1; then
    echo "✓ Internet connectivity OK"
else
    echo "❌ No internet connection"
fi

# Ports
for port in 3000 8000 54322; do
    if ! sudo lsof -i :$port > /dev/null 2>&1; then
        echo "✓ Port $port available"
    else
        echo "❌ Port $port in use"
    fi
done

echo ""
echo "Verification complete!"
```

---

## Next Steps

Once all prerequisites are met:

1. **Download installation script:**
   ```bash
   wget https://raw.githubusercontent.com/YOUR-ORG/tempo-insights/main/scripts/configure-instance.sh
   chmod +x configure-instance.sh
   ```

2. **Run installation:**
   ```bash
   sudo ./configure-instance.sh
   ```

3. **Follow prompts** for environment selection and configuration

4. **Access application** at `http://raspberry-pi-ip:3000`

---

**Need Help?**

If any prerequisite checks fail, see:
- [Docker Deployment Guide](DOCKER_DEPLOYMENT.md)
- [Troubleshooting Guide](TROUBLESHOOTING.md)