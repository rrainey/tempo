# Tempo Insights - Docker Deployment Summary

Quick reference guide for deploying Tempo Insights using Docker on Raspberry Pi 5.

---

## Quick Links

- [Docker Deployment Guide](DOCKER_DEPLOYMENT.md) - Complete deployment instructions
- [Prerequisites Checklist](PREREQUISITES.md) - Hardware/software requirements
- [Security Checklist](SECURITY_CHECKLIST.md) - Production security hardening
- [Environment Configuration](ENVIRONMENT_CONFIGURATION.md) - .env file guide
- [Volumes & Persistence](VOLUMES_AND_PERSISTENCE.md) - Data storage guide

---

## One-Command Deployment

For **fresh Ubuntu 22.04 systems**:

```bash
wget https://raw.githubusercontent.com/YOUR-ORG/tempo-insights/main/scripts/configure-instance.sh
chmod +x configure-instance.sh
sudo ./configure-instance.sh
```

**Duration**: 15-30 minutes  
**Result**: Fully configured production-ready system

---

## What Gets Deployed

### Services

| Service | Container | Ports | Purpose |
|---------|-----------|-------|---------|
| **Next.js Web** | `tempo-web` | 3000 | User interface |
| **BT Scanner** | `tempo-bt-scanner` | host | Device discovery |
| **Analysis Worker** | `tempo-analysis` | none | Log processing |
| **PostgreSQL** | `postgres` | 54322 | Database |
| **Kong Gateway** | `kong` | 8000 | API gateway |
| **Supabase Studio** | `studio` | 8000 | Admin UI |
| + 8 more | Supabase stack | - | Backend services |

**Total**: 14 containers (3 app + 11 Supabase)

### Docker Images

- `tempo-web:latest` (~250MB)
- `tempo-bt-scanner:latest` (~400MB)
- `tempo-analysis:latest` (~250MB)
- Supabase images (~2GB total)

**Total Docker storage**: ~3GB images + data volumes

---

## Directory Structure

```
/home/user/tempo-insights/
├── .env.docker                    # App configuration
├── .secrets.prod                  # Generated secrets (BACKUP THIS!)
├── docker-compose.yml             # App services
├── docker/                        # Dockerfiles
│   ├── web/
│   ├── bluetooth-scanner/
│   └── analysis-worker/
├── supabase-stack/                # Database stack
│   ├── docker-compose.yml
│   ├── .env                       # Supabase config
│   └── volumes/                   # Persistent data
│       ├── db/data/              # PostgreSQL
│       └── storage/              # Jump logs
├── scripts/                       # Management scripts
│   ├── configure-instance.sh     # Bootstrap
│   ├── start-tempo.sh           # Start services
│   ├── stop-tempo.sh            # Stop services
│   ├── build-images.sh          # Build Docker images
│   └── validate-env.sh          # Verify config
└── docs/                          # Documentation
```

---

## Common Operations

### Start Everything
```bash
cd /home/user/tempo-insights
./scripts/start-tempo.sh
```

### Stop Everything
```bash
./scripts/stop-tempo.sh --all
```

### View Logs
```bash
# All services
docker compose logs -f

# Specific service
docker compose logs -f tempo-web
```

### Check Health
```bash
# Application
curl http://localhost:3000/api/health

# Database
docker compose -f supabase-stack/docker-compose.yml exec postgres pg_isready
```

### Restart Service
```bash
docker compose restart tempo-web
```

---

## Backup & Restore

### Backup

```bash
#!/bin/bash
# Quick backup
BACKUP_DIR="/backups/tempo-$(date +%Y%m%d)"
mkdir -p "$BACKUP_DIR"

# Database
cd supabase-stack
docker compose exec postgres pg_dump -U postgres postgres > "$BACKUP_DIR/db.sql"

# Jump logs
tar -czf "$BACKUP_DIR/logs.tar.gz" volumes/storage/

echo "Backup: $BACKUP_DIR"
```

### Restore

```bash
# Stop services
./scripts/stop-tempo.sh --all

# Restore database
cd supabase-stack
docker compose up -d postgres
docker compose exec -T postgres psql -U postgres postgres < /path/to/db.sql

# Restore files
tar -xzf /path/to/logs.tar.gz -C volumes/

# Restart
../scripts/start-tempo.sh
```

---

## Troubleshooting Quick Reference

### Services Won't Start

```bash
# Check environment
./scripts/validate-env.sh .env.docker

# Check logs
docker compose logs

# Check ports
sudo lsof -i :3000
sudo lsof -i :8000
```

### Bluetooth Not Working

```bash
# Check Bluetooth service
sudo systemctl status bluetooth

# Check adapter
bluetoothctl list

# Restart Bluetooth
sudo systemctl restart bluetooth
sudo hciconfig hci0 up

# Re-run setup
sudo ./scripts/setup-host-bluetooth.sh
```

### Database Connection Errors

```bash
# Verify Supabase running
cd supabase-stack && docker compose ps

# Check PostgreSQL
docker compose exec postgres pg_isready

# Verify DATABASE_URL
grep DATABASE_URL ../.env.docker
# Should use 'db' not 'localhost'
```

### Out of Disk Space

```bash
# Check usage
df -h
docker system df

# Clean up
docker system prune -a --volumes
```

---

## Environment Types

### Development
- **Secrets**: Weak OK (convenience)
- **Data**: Test/mock data
- **Backups**: Manual only
- **Monitoring**: Basic

### QA/Staging
- **Secrets**: Real but documented
- **Data**: Test scenarios
- **Backups**: Daily
- **Monitoring**: Standard

### Production
- **Secrets**: Maximum strength, unique
- **Data**: Real user data
- **Backups**: Hourly DB, daily full
- **Monitoring**: 24/7 with alerts

---

## Port Reference

| Port | Service | Internal/External | Purpose |
|------|---------|-------------------|---------|
| 3000 | Tempo Web | External | User access |
| 8000 | Kong/Studio | External | API & Admin |
| 5432 | PostgreSQL | Internal only | Database |
| 54322 | PostgreSQL | External (debug) | Direct DB |
| 6543 | Supavisor | Internal only | Connection pool |

**Firewall rules**:
```bash
sudo ufw allow 22/tcp    # SSH
sudo ufw allow 3000/tcp  # App
sudo ufw allow 8000/tcp  # Supabase
sudo ufw enable
```

---

## Resource Usage

### Typical (8GB Pi, 1000 jumps)

- **CPU**: 10-30% average (4 cores)
- **RAM**: 3-4GB used (OS + Docker + services)
- **Disk**: 30-50GB
  - System: ~10GB
  - Docker images: ~3GB
  - Database: ~5GB
  - Jump logs: ~10-30GB

### Growth Estimates

- **10,000 jumps**: ~150GB total
- **100,000 jumps**: ~500GB total

---

## Access Points

**After successful deployment:**

- **Web Application**: `http://<raspberry-pi-ip>:3000`
- **Supabase Studio**: `http://<raspberry-pi-ip>:8000`
- **API Gateway**: `http://<raspberry-pi-ip>:8000/rest/v1/`

**Default credentials** (if using configure-instance.sh):
- Supabase Studio: Set during installation
- First admin user: Created during database seed

---

## Maintenance Schedule

### Daily
- Check service health: `docker compose ps`
- Review logs: `docker compose logs --tail=100`

### Weekly
- Update system: `sudo apt-get update && upgrade`
- Check disk space: `df -h`
- Review error logs

### Monthly
- Test backup restore
- Update Docker images
- Security review

### Quarterly
- Rotate secrets
- Performance review
- Disaster recovery drill

---

## Getting Help

1. **Check logs**: `docker compose logs [service]`
2. **Validate environment**: `./scripts/validate-env.sh`
3. **Review documentation**: See links at top
4. **Test deployment**: `./scripts/test-deploy.sh`
5. **GitHub issues**: Include logs and config (redact secrets!)

---

## Key Files to Backup

**Critical** (backup immediately):
- `.secrets.prod` - Generated secrets
- `supabase-stack/.env` - Database configuration
- `.env.docker` - Application configuration

**Important** (backup regularly):
- `supabase-stack/volumes/db/data/` - Database
- `supabase-stack/volumes/storage/` - Jump logs

**Optional** (can regenerate):
- Docker images (can rebuild)
- System packages (can reinstall)

---

## Production Readiness Checklist

Before going live:

- [ ] All secrets changed from defaults
- [ ] Backups tested and automated
- [ ] Firewall configured
- [ ] SSL/TLS enabled (if internet-facing)
- [ ] Monitoring configured
- [ ] Team trained on procedures
- [ ] Disaster recovery plan documented
- [ ] `.secrets.prod` file backed up securely

---

## Version Information

**Tempo Insights**: v1.0.0  
**Docker**: 24.0+  
**Ubuntu**: 22.04 LTS  
**Node.js**: 20.x  
**PostgreSQL**: 15.x  
**Supabase**: Latest (self-hosted)

---

## Support Resources

- **Documentation**: `/docs` directory
- **Scripts**: `/scripts` directory
- **GitHub**: [Repository URL]
- **Issues**: [GitHub Issues URL]

---

**Last Updated**: 2025-01-10  
**Deployment Guide Version**: 1.0