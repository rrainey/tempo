# Docker Deployment Guide

Complete guide for deploying Tempo Insights using Docker on Raspberry Pi 5 with Ubuntu 22.04.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Quick Start](#quick-start)
3. [Manual Deployment](#manual-deployment)
4. [Configuration](#configuration)
5. [Operations](#operations)
6. [Troubleshooting](#troubleshooting)
7. [Upgrading](#upgrading)

---

## Prerequisites

### Hardware Requirements

**Minimum (Development/QA):**
- Raspberry Pi 5 (4GB RAM)
- 64GB microSD card
- Tempo-BT device for testing

**Recommended (Production):**
- Raspberry Pi 5 (8GB RAM)
- 128GB+ microSD card or SSD
- Multiple Tempo-BT devices
- Uninterruptible Power Supply (UPS)

### Software Requirements

- Ubuntu 22.04 LTS (64-bit ARM)
- Internet connection for initial setup
- SSH access (for remote deployment)

### Network Requirements

**Ports to expose:**
- `3000` - Tempo Insights web application
- `8000` - Supabase API gateway/Studio

**Optional ports (for debugging):**
- `54322` - Direct PostgreSQL access
- `6543` - Supavisor connection pooler

---

## Quick Start

### Automated Installation (Recommended)

For fresh Ubuntu 22.04 systems:

```bash
# 1. Download bootstrap script
wget https://raw.githubusercontent.com/YOUR-ORG/tempo-insights/main/scripts/configure-instance.sh

# 2. Make executable
chmod +x configure-instance.sh

# 3. Run as sudo
sudo ./configure-instance.sh
```

The script will:
- ✅ Install all system dependencies
- ✅ Set up Docker and BlueZ
- ✅ Clone repository
- ✅ Generate secure secrets
- ✅ Configure Supabase
- ✅ Build Docker images
- ✅ Initialize database
- ✅ Start all services

**Total time:** 15-30 minutes

---

## Manual Deployment

For developers or custom setups:

### Step 1: System Preparation

```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker

# Install Node.js 20
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo bash -
sudo apt-get install -y nodejs

# Install pnpm
sudo npm install -g pnpm

# Install BlueZ
sudo apt-get install -y bluez libbluetooth-dev bluetooth
sudo systemctl enable bluetooth
sudo systemctl start bluetooth
```

### Step 2: Clone Repository

```bash
git clone https://github.com/YOUR-ORG/tempo-insights.git
cd tempo-insights
```

### Step 3: Configure Secrets

```bash
# Generate secrets
./scripts/generate-secrets.sh

# Copy secrets to environment files
cp .env.example .env.docker
# Edit .env.docker with generated secrets
nano .env.docker
```

### Step 4: Setup Supabase

```bash
# Run Supabase setup
./scripts/setup-supabase.sh

# Or manually:
git clone --depth 1 https://github.com/supabase/supabase supabase-temp
mkdir -p supabase-stack
cp -r supabase-temp/docker/* supabase-stack/
rm -rf supabase-temp

# Configure Supabase
cp supabase-stack/.env.example supabase-stack/.env
# Edit with your secrets
nano supabase-stack/.env
```

### Step 5: Setup Host Bluetooth

```bash
sudo ./scripts/setup-host-bluetooth.sh
```

### Step 6: Build Images

```bash
./scripts/build-images.sh
```

### Step 7: Start Services

```bash
./scripts/start-tempo.sh
```

### Step 8: Initialize Database

```bash
# Run migrations
DATABASE_URL="postgresql://postgres:PASSWORD@localhost:54322/postgres" \
  pnpm prisma migrate deploy

# Seed initial data
DATABASE_URL="postgresql://postgres:PASSWORD@localhost:54322/postgres" \
  pnpm prisma db seed
```

---

## Configuration

### Environment Files

**`.env.docker`** - Application configuration
```bash
# Security
JWT_SECRET=<64-char-random-string>
WORKER_TOKEN=<64-char-random-string>

# Database (note: 'db' hostname for Docker)
DATABASE_URL=postgresql://postgres:PASSWORD@db:5432/postgres

# Supabase
NEXT_PUBLIC_SUPABASE_URL=http://localhost:8000
SUPABASE_SERVICE_KEY=<service-role-key>

# Bluetooth
DISCOVERY_WINDOW=300
SMPMGR_PLUGIN_PATH=/opt/smpmgr-extensions/plugins

# Storage
JUMP_LOG_STORAGE_BUCKET=jump-logs
JUMP_LOG_MAX_SIZE_MB=128
```

**`supabase-stack/.env`** - Database stack configuration
```bash
# PostgreSQL
POSTGRES_PASSWORD=<strong-random-password>

# JWT (must match .env.docker)
JWT_SECRET=<same-as-above>
ANON_KEY=<generated-from-jwt-secret>
SERVICE_ROLE_KEY=<generated-from-jwt-secret>

# Dashboard
DASHBOARD_USERNAME=admin
DASHBOARD_PASSWORD=<strong-password>
```

### Generating JWT Keys

Visit: https://supabase.com/docs/guides/self-hosting/docker#generate-api-keys

Use your `JWT_SECRET` to generate:
- `ANON_KEY`
- `SERVICE_ROLE_KEY`

---

## Operations

### Starting Services

```bash
# Start everything
./scripts/start-tempo.sh

# Or manually:
cd supabase-stack && docker compose up -d && cd ..
docker compose up -d
```

### Stopping Services

```bash
# Stop application only
./scripts/stop-tempo.sh

# Stop everything (app + database)
./scripts/stop-tempo.sh --all

# Stop and clean all data (DESTRUCTIVE!)
./scripts/stop-tempo.sh --clean
```

### Viewing Logs

```bash
# Follow all application logs
docker compose logs -f

# Specific service
docker compose logs -f tempo-web
docker compose logs -f tempo-bt-scanner
docker compose logs -f tempo-analysis

# Supabase logs
cd supabase-stack
docker compose logs -f postgres
docker compose logs -f kong
```

### Checking Status

```bash
# Application services
docker compose ps

# Supabase services
cd supabase-stack && docker compose ps

# Health checks
curl http://localhost:3000/api/health
curl http://localhost:8000/health
```

### Restarting Services

```bash
# Restart specific service
docker compose restart tempo-web

# Restart all
docker compose restart

# Rebuild and restart
docker compose up -d --build tempo-web
```

---

## Troubleshooting

### Common Issues

#### 1. Services Won't Start

**Symptom:** `docker compose up -d` fails or services show as unhealthy

**Checks:**
```bash
# Check environment
./scripts/validate-env.sh .env.docker

# Check Docker
docker info
docker compose ps

# Check logs
docker compose logs
```

**Solutions:**
- Ensure `.env.docker` exists with valid configuration
- Verify Supabase is running first
- Check for port conflicts: `sudo lsof -i :3000`

#### 2. Bluetooth Scanner Not Working

**Symptom:** Scanner container starts but doesn't detect devices

**Checks:**
```bash
# Check Bluetooth service
sudo systemctl status bluetooth

# Check adapter
bluetoothctl list

# Check scanner logs
docker compose logs tempo-bt-scanner
```

**Solutions:**
```bash
# Restart Bluetooth
sudo systemctl restart bluetooth

# Reset adapter
sudo hciconfig hci0 down
sudo hciconfig hci0 up

# Verify D-Bus permissions
ls -la /run/dbus/system_bus_socket

# Re-run host setup
sudo ./scripts/setup-host-bluetooth.sh
```

#### 3. Database Connection Errors

**Symptom:** Services log "database connection refused"

**Checks:**
```bash
# Check PostgreSQL
docker compose -f supabase-stack/docker-compose.yml exec postgres pg_isready

# Check connection string
grep DATABASE_URL .env.docker
```

**Solutions:**
- Verify `DATABASE_URL` uses `db` hostname (not `localhost`)
- Ensure Supabase stack is running
- Check credentials match between `.env.docker` and `supabase-stack/.env`

#### 4. Web Server 502/503 Errors

**Symptom:** Web application won't load

**Checks:**
```bash
# Check container
docker compose ps tempo-web

# Check health
curl http://localhost:3000/api/health

# Check logs
docker compose logs --tail=100 tempo-web
```

**Solutions:**
```bash
# Restart web service
docker compose restart tempo-web

# Check for Node.js errors
docker compose logs tempo-web | grep -i error

# Verify database migrations
pnpm prisma migrate deploy
```

#### 5. Out of Disk Space

**Symptom:** Services fail with "no space left on device"

**Checks:**
```bash
# Check disk usage
df -h
du -sh supabase-stack/volumes/*
docker system df
```

**Solutions:**
```bash
# Clean up Docker
docker system prune -a --volumes

# Clean old images
docker images | grep "<none>" | awk '{print $3}' | xargs docker rmi

# Rotate logs
find supabase-stack/volumes/logs -name "*.log" -mtime +7 -delete
```

---

## Upgrading

### Application Upgrade

```bash
# 1. Stop services
./scripts/stop-tempo.sh

# 2. Backup data
./scripts/backup-tempo.sh

# 3. Pull latest code
git pull origin main

# 4. Rebuild images
./scripts/build-images.sh

# 5. Run migrations
DATABASE_URL="postgresql://postgres:PASSWORD@localhost:54322/postgres" \
  pnpm prisma migrate deploy

# 6. Restart
./scripts/start-tempo.sh
```

### Supabase Upgrade

```bash
# 1. Stop application
./scripts/stop-tempo.sh

# 2. Backup database
cd supabase-stack
docker compose exec postgres pg_dump -U postgres postgres > backup.sql

# 3. Update Supabase
cd supabase-stack
docker compose pull
docker compose up -d

# 4. Verify health
docker compose ps

# 5. Restart application
cd ..
./scripts/start-tempo.sh
```

---

## Maintenance

### Backups

**Automated backup script:**
```bash
#!/bin/bash
# backup-tempo.sh

BACKUP_DIR="/backups/tempo-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP_DIR"

# Database
cd supabase-stack
docker compose exec postgres pg_dump -U postgres postgres > "$BACKUP_DIR/database.sql"

# Jump logs
tar -czf "$BACKUP_DIR/jump-logs.tar.gz" volumes/storage/

# Configuration
cp ../env.docker "$BACKUP_DIR/env-backup.txt"

echo "Backup complete: $BACKUP_DIR"
```

**Schedule with cron:**
```bash
# Daily at 2 AM
0 2 * * * /home/user/tempo-insights/scripts/backup-tempo.sh
```

### Log Rotation

Logs automatically rotate (configured in docker-compose.yml):
- Max size: 10MB per file
- Max files: 3
- Total per service: ~30MB

### Monitoring

**Key metrics to watch:**
```bash
# Disk usage
df -h

# Container health
docker compose ps

# Database size
docker compose -f supabase-stack/docker-compose.yml exec postgres \
  psql -U postgres -c "SELECT pg_size_pretty(pg_database_size('postgres'));"

# Jump log count
docker compose -f supabase-stack/docker-compose.yml exec postgres \
  psql -U postgres -c "SELECT COUNT(*) FROM \"JumpLog\";"
```

---

## Security Checklist

### Pre-Deployment

- [ ] Generate unique secrets (not defaults)
- [ ] Change Supabase dashboard password
- [ ] Configure firewall (UFW)
- [ ] Set up SSL/TLS (if internet-facing)
- [ ] Disable root SSH login
- [ ] Enable automatic security updates

### Post-Deployment

- [ ] Verify no default passwords remain
- [ ] Test backup/restore procedures
- [ ] Document secrets location
- [ ] Set up monitoring alerts
- [ ] Configure log retention

### Firewall Configuration

```bash
# Allow SSH
sudo ufw allow 22/tcp

# Allow web traffic
sudo ufw allow 3000/tcp
sudo ufw allow 8000/tcp

# Enable firewall
sudo ufw enable

# Check status
sudo ufw status
```

---

## Performance Tuning

### Raspberry Pi 5 Optimizations

**PostgreSQL tuning** (`supabase-stack/volumes/db/postgresql.conf`):
```conf
# Adjust for 8GB RAM
shared_buffers = 2GB
effective_cache_size = 6GB
maintenance_work_mem = 512MB
checkpoint_completion_target = 0.9
```

**Docker resource limits** (add to docker-compose.yml):
```yaml
services:
  tempo-web:
    deploy:
      resources:
        limits:
          memory: 1G
        reservations:
          memory: 512M
```

---

## Additional Resources

- [Environment Configuration Guide](ENVIRONMENT_CONFIGURATION.md)
- [Volumes and Persistence](VOLUMES_AND_PERSISTENCE.md)
- [Troubleshooting Guide](TROUBLESHOOTING.md)
- [API Documentation](API.md)
- [Supabase Self-Hosting Docs](https://supabase.com/docs/guides/self-hosting)

---

## Support

For issues and questions:
1. Check [Troubleshooting](#troubleshooting) section
2. Review logs: `docker compose logs`
3. Validate environment: `./scripts/validate-env.sh`
4. Open GitHub issue with logs and configuration details

---

**Last Updated:** 2025-01-10