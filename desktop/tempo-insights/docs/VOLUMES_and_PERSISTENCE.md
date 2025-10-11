# Tempo Insights - Volumes and Persistence

This document describes all persistent storage in the Tempo Insights system.

## Storage Architecture

All persistent data is managed by the Supabase stack. The application services (web, bluetooth-scanner, analysis-worker) are **stateless** and store no data locally.

```
┌─────────────────────────────────────────┐
│     Application Services (Stateless)    │
│  ┌─────────┐  ┌──────────┐  ┌─────────┐│
│  │   Web   │  │ BT Scan  │  │Analysis ││
│  └────┬────┘  └─────┬────┘  └────┬────┘│
│       │            │             │      │
│       └────────────┼─────────────┘      │
│                    │                     │
└────────────────────┼─────────────────────┘
                     │
┌────────────────────▼─────────────────────┐
│         Supabase Stack (Stateful)        │
│  ┌──────────────┐  ┌──────────────────┐ │
│  │  PostgreSQL  │  │  Storage (S3)    │ │
│  │   Database   │  │  Jump Log Files  │ │
│  └──────────────┘  └──────────────────┘ │
└──────────────────────────────────────────┘
```

---

## Task 158: Database Volume

### Location
```
supabase-stack/volumes/db/data/
```

### Purpose
PostgreSQL data directory containing all database tables, indexes, and WAL logs.

### Contents
- User accounts and authentication
- Device registrations
- Jump log metadata (hash, userId, timestamps, analysis results)
- Formation skydive records
- Group memberships and permissions

### Backup Strategy
```bash
# Manual backup
cd supabase-stack
docker compose exec postgres pg_dump -U postgres postgres > backup-$(date +%Y%m%d).sql

# Restore
docker compose exec -T postgres psql -U postgres postgres < backup-20250110.sql
```

### Size Estimates
- Fresh install: ~50 MB
- 100 users, 10,000 jumps: ~500 MB
- 1,000 users, 100,000 jumps: ~5 GB

### Configuration
Defined in `supabase-stack/docker-compose.yml`:
```yaml
volumes:
  - ./volumes/db/data:/var/lib/postgresql/data:Z
```

---

## Task 159: Storage Volume (Jump Logs)

### Location
```
supabase-stack/volumes/storage/
```

### Purpose
Supabase Storage bucket for jump log binary files.

### Contents
- Raw jump log files (flight.txt, etc.)
- Organized by path: `{userId}/{deviceId}/{hash}.log`
- File sizes: typically 500KB - 8MB per jump

### Storage Structure
```
volumes/storage/
└── jump-logs/               # Bucket name
    └── {userId}/
        └── {deviceId}/
            ├── abc123...def.log
            ├── 456789...xyz.log
            └── ...
```

### Access Methods
1. **Via Supabase Storage API** (application default):
   ```typescript
   const { data } = await supabase.storage
     .from('jump-logs')
     .download(storagePath);
   ```

2. **Direct file access** (backup/migration):
   ```bash
   ls supabase-stack/volumes/storage/jump-logs/
   ```

### Backup Strategy
```bash
# Backup all jump logs
tar -czf jump-logs-backup-$(date +%Y%m%d).tar.gz \
  supabase-stack/volumes/storage/

# Restore
tar -xzf jump-logs-backup-20250110.tar.gz -C supabase-stack/
```

### Size Estimates
- 1,000 jumps: ~2-8 GB
- 10,000 jumps: ~20-80 GB
- 100,000 jumps: ~200-800 GB

### Configuration
Defined in `supabase-stack/docker-compose.yml`:
```yaml
volumes:
  - ./volumes/storage:/var/lib/storage:z
```

---

## Task 160: Functions Volume

### Location
```
supabase-stack/volumes/functions/
```

### Purpose
Supabase Edge Functions (Deno runtime) for serverless functions.

### Contents
Currently contains sample functions:
- `hello/` - Example function
- `main/` - Main entry point

### Usage in Tempo Insights
**Not currently used** - all backend logic is in Next.js API routes.

Potential future use:
- Scheduled jobs (log cleanup)
- Webhooks (external integrations)
- Data transformations

### Configuration
```yaml
volumes:
  - ./volumes/functions:/home/deno/functions:Z
```

---

## Task 161: Logs Volume

### Location
```
supabase-stack/volumes/logs/
```

### Purpose
Vector log aggregation configuration and runtime logs.

### Contents
- `vector.yml` - Log aggregation config
- Runtime logs from all Supabase services

### Log Rotation
Configured in application `docker-compose.yml`:
```yaml
logging:
  driver: "json-file"
  options:
    max-size: "10m"
    max-file: "3"
```

Each service keeps max 3 files × 10MB = 30MB of logs.

### Viewing Logs
```bash
# Application services
docker compose logs -f tempo-web
docker compose logs -f tempo-bt-scanner
docker compose logs -f tempo-analysis

# Supabase services
cd supabase-stack
docker compose logs -f postgres
docker compose logs -f kong
```

### Configuration
```yaml
volumes:
  - ./volumes/logs:/var/log:z
```

---

## Task 162: Config Volume (pgsodium encryption)

### Location
```
supabase-stack/volumes/db-config/
```

### Purpose
PostgreSQL encryption key storage for pgsodium extension.

### Contents
- Encryption keys for sensitive database fields
- Auto-generated on first run
- **Critical for data security** - must be backed up

### Configuration
Named volume in `supabase-stack/docker-compose.yml`:
```yaml
volumes:
  db-config:
    driver: local
```

### Backup Strategy
```bash
# Backup encryption keys
docker volume inspect supabase_db-config
# Copy from the Mountpoint location

# For critical production systems, consider:
# - Encrypted backups stored off-site
# - Key management service (KMS)
```

---

## Complete Backup Procedure

### Full System Backup
```bash
#!/bin/bash
# backup-tempo-insights.sh

BACKUP_DIR="/backups/tempo-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP_DIR"

# 1. Database dump
cd supabase-stack
docker compose exec postgres pg_dump -U postgres postgres > "$BACKUP_DIR/database.sql"

# 2. Jump log files
tar -czf "$BACKUP_DIR/jump-logs.tar.gz" volumes/storage/

# 3. Configuration (excluding secrets)
cp .env.example "$BACKUP_DIR/env-template.txt"

# 4. Encryption keys
docker volume inspect supabase_db-config > "$BACKUP_DIR/db-config-info.json"

echo "Backup complete: $BACKUP_DIR"
echo "Size: $(du -sh $BACKUP_DIR | cut -f1)"
```

### Restore Procedure
```bash
#!/bin/bash
# restore-tempo-insights.sh BACKUP_DIR

BACKUP_DIR=$1

# 1. Stop services
cd supabase-stack
docker compose down

# 2. Restore database
docker compose up -d postgres
sleep 10
docker compose exec -T postgres psql -U postgres postgres < "$BACKUP_DIR/database.sql"

# 3. Restore jump logs
tar -xzf "$BACKUP_DIR/jump-logs.tar.gz" -C .

# 4. Restart services
docker compose up -d
cd ..
docker compose up -d
```

---

## Disaster Recovery

### Recovery Time Objective (RTO)
- Database restore: ~10 minutes
- Jump log restore: ~30 minutes (depends on size)
- Full system: ~1 hour

### Recovery Point Objective (RPO)
- Database: Daily backups (24-hour max data loss)
- Jump logs: Real-time (no loss if storage intact)

### Recommended Backup Schedule

**Development:**
- Manual backups before major changes
- No automated backups

**QA:**
- Daily database backups
- Weekly full backups
- 7-day retention

**Production:**
- Hourly database backups
- Daily full backups
- 30-day retention
- Off-site backup storage
- Encryption at rest

---

## Monitoring Disk Usage

```bash
# Check volume sizes
du -sh supabase-stack/volumes/*

# PostgreSQL database size
docker compose -f supabase-stack/docker-compose.yml exec postgres \
  psql -U postgres -c "SELECT pg_size_pretty(pg_database_size('postgres'));"

# Storage bucket size
du -sh supabase-stack/volumes/storage/jump-logs/

# Docker system usage
docker system df -v
```

---

## Volume Migration

### Moving to New Server
```bash
# On old server
./scripts/backup-tempo-insights.sh

# Copy backup to new server
scp -r /backups/tempo-20250110 newserver:/backups/

# On new server
sudo ./configure-instance.sh  # Set up fresh system
./scripts/restore-tempo-insights.sh /backups/tempo-20250110
```

### Expanding Storage
```bash
# If running out of space, options:

# 1. Add external volume
sudo mkdir /mnt/jump-logs
sudo mount /dev/sdb1 /mnt/jump-logs
sudo mv supabase-stack/volumes/storage/* /mnt/jump-logs/
sudo ln -s /mnt/jump-logs supabase-stack/volumes/storage

# 2. Clean old data (with user consent)
# Implement data retention policy in application
```

---

## Security Considerations

### File Permissions
```bash
# All volumes should be owned by appropriate user
sudo chown -R 999:999 supabase-stack/volumes/db/     # Postgres UID
sudo chown -R 1000:1000 supabase-stack/volumes/storage/
sudo chmod 700 supabase-stack/volumes/db/data/        # Database only
```

### Encryption
- **At rest**: Consider encrypting the entire volume mount point
- **In transit**: All API access over HTTPS
- **Database**: pgsodium extension for field-level encryption

### Access Control
- Volumes only accessible from Docker containers
- No direct user access in production
- All access via application APIs

---

## Summary

| Task | Volume | Purpose | Size (est) | Backup Frequency |
|------|--------|---------|------------|------------------|
| 158 | `db/data` | PostgreSQL | 5-50 GB | Hourly (prod) |
| 159 | `storage` | Jump logs | 20-800 GB | Daily (prod) |
| 160 | `functions` | Edge functions | <100 MB | Not needed |
| 161 | `logs` | Service logs | <500 MB | Not needed |
| 162 | `db-config` | Encryption keys | <1 MB | **Critical** |

**All persistence is in Supabase stack. Application containers are stateless.**