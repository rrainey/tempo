# Security & Production Readiness Checklist

Complete security hardening and production deployment checklist for Tempo Insights.

---

## Pre-Deployment Security

### ✅ Secrets Generation

- [ ] **All default secrets changed**
  - No "your-secret-here" or "change-this" values
  - No example/template secrets in use

- [ ] **Strong secret generation**
  ```bash
  ./scripts/generate-secrets.sh
  ```
  - [ ] JWT_SECRET: 64+ characters
  - [ ] WORKER_TOKEN: 64+ characters  
  - [ ] POSTGRES_PASSWORD: 32+ characters
  - [ ] All secrets cryptographically random

- [ ] **Unique secrets per environment**
  - [ ] Development secrets ≠ QA secrets
  - [ ] QA secrets ≠ Production secrets
  - [ ] Never reused across environments

- [ ] **Secrets stored securely**
  - [ ] `.secrets.prod` file backed up to secure location
  - [ ] Password manager entry created
  - [ ] Paper backup in safe (optional)
  - [ ] Secrets NOT in Slack/email/plaintext

### ✅ Environment Validation

```bash
# Run validation before deployment
./scripts/validate-env.sh .env.docker
```

- [ ] All required variables present
- [ ] No weak secrets detected
- [ ] No default passwords remaining
- [ ] DATABASE_URL uses correct hostname (`db` not `localhost`)

---

## System Hardening

### ✅ Operating System Security

- [ ] **System fully updated**
  ```bash
  sudo apt-get update && sudo apt-get upgrade -y
  sudo reboot
  ```

- [ ] **Automatic security updates enabled**
  ```bash
  sudo apt-get install -y unattended-upgrades
  sudo dpkg-reconfigure -plow unattended-upgrades
  ```

- [ ] **Unnecessary services disabled**
  ```bash
  sudo systemctl list-units --type=service --state=running
  # Disable any unused services
  ```

### ✅ User Account Security

- [ ] **Root login disabled** (SSH)
  ```bash
  sudo nano /etc/ssh/sshd_config
  # Set: PermitRootLogin no
  sudo systemctl restart sshd
  ```

- [ ] **Strong user password set**
  - Minimum 20 characters
  - Mix of upper/lower/numbers/symbols
  - Not based on dictionary words

- [ ] **SSH key authentication configured**
  ```bash
  # On your machine:
  ssh-copy-id user@tempo-server
  
  # Disable password auth (after testing):
  sudo nano /etc/ssh/sshd_config
  # Set: PasswordAuthentication no
  ```

- [ ] **Fail2ban installed** (brute-force protection)
  ```bash
  sudo apt-get install -y fail2ban
  sudo systemctl enable fail2ban
  sudo systemctl start fail2ban
  ```

### ✅ Firewall Configuration

- [ ] **UFW enabled with proper rules**
  ```bash
  # Allow SSH
  sudo ufw allow 22/tcp
  
  # Allow web traffic
  sudo ufw allow 3000/tcp  # Tempo app
  sudo ufw allow 8000/tcp  # Supabase
  
  # Enable firewall
  sudo ufw enable
  
  # Verify
  sudo ufw status verbose
  ```

- [ ] **Only necessary ports exposed**
  - Port 22: SSH (from specific IPs if possible)
  - Port 3000: Tempo web app
  - Port 8000: Supabase Studio/API
  - All other ports: BLOCKED

- [ ] **Rate limiting configured** (optional)
  ```bash
  sudo ufw limit 22/tcp
  sudo ufw limit 3000/tcp
  ```

---

## Application Security

### ✅ Database Security

- [ ] **PostgreSQL password changed**
  - Strong random password (32+ chars)
  - Different from all other passwords

- [ ] **Direct database access restricted**
  - Port 5432/54322 NOT exposed to internet
  - Only accessible via Docker network or localhost

- [ ] **Database backups encrypted**
  ```bash
  # Encrypt backup
  gpg --symmetric --cipher-algo AES256 backup.sql
  ```

- [ ] **Connection pooling enabled**
  - Using Supavisor (port 6543) for app connections
  - Not using direct PostgreSQL connections in production

### ✅ Supabase Security

- [ ] **Supabase Studio access secured**
  - Dashboard username changed (not "admin")
  - Strong dashboard password (20+ chars)
  - Dashboard only accessible on local network

- [ ] **JWT keys properly generated**
  - ANON_KEY generated from JWT_SECRET
  - SERVICE_ROLE_KEY generated from JWT_SECRET
  - Keys match between app and Supabase configs

- [ ] **Storage bucket secured**
  - `jump-logs` bucket has proper RLS policies
  - Only authenticated users can access their own files

### ✅ Docker Security

- [ ] **Containers run as non-root**
  - Verified in Dockerfiles (USER directive)
  - Web: runs as `nextjs` (UID 1001)
  - BT Scanner: runs as `btscanner` (UID 1001)
  - Analysis: runs as `analysis` (UID 1001)

- [ ] **Minimal capabilities granted**
  - Bluetooth scanner: Only NET_ADMIN (not privileged mode)
  - Other containers: No special capabilities

- [ ] **Secrets not in images**
  - All secrets passed via environment variables
  - `.env.docker` not copied into images
  - Image scan shows no embedded secrets

- [ ] **Images from trusted sources only**
  - Base images from official Docker Hub
  - Supabase images from official Supabase repo
  - No third-party images without verification

---

## Network Security

### ✅ Network Isolation

- [ ] **Containers on isolated network**
  - Application uses `tempo-network` bridge
  - Only necessary inter-container communication allowed

- [ ] **No unnecessary network exposure**
  - Analysis worker: No ports exposed
  - BT Scanner: Host network only for Bluetooth
  - Web: Only port 3000 exposed

### ✅ SSL/TLS (If Internet-Facing)

- [ ] **Reverse proxy configured** (if needed)
  ```bash
  # Using nginx with Let's Encrypt
  sudo apt-get install -y nginx certbot python3-certbot-nginx
  ```

- [ ] **HTTPS enabled**
  - Valid SSL certificate
  - HTTP redirects to HTTPS
  - HSTS header enabled

- [ ] **Strong TLS configuration**
  - TLS 1.2+ only
  - Strong cipher suites
  - Perfect forward secrecy

---

## Data Security

### ✅ Data at Rest

- [ ] **Disk encryption considered** (optional for Pi)
  ```bash
  # LUKS encryption for SSD
  # Note: Impacts performance on Pi
  ```

- [ ] **Sensitive fields encrypted**
  - Database uses pgsodium for encryption
  - Encryption keys backed up securely

- [ ] **File permissions correct**
  ```bash
  # Check volume permissions
  ls -la supabase-stack/volumes/db/
  # Should be: drwx------ (700) for database
  ```

### ✅ Data in Transit

- [ ] **Internal Docker network encrypted** (Docker default)
- [ ] **External API calls use HTTPS**
- [ ] **No sensitive data in logs**

### ✅ Data Retention

- [ ] **Backup retention policy defined**
  - Daily backups: 7 days
  - Weekly backups: 4 weeks
  - Monthly backups: 12 months

- [ ] **Data deletion procedures documented**
  - User data deletion process
  - GDPR compliance (if applicable)
  - Jump log retention limits

---

## Backup & Recovery

### ✅ Backup Configuration

- [ ] **Automated backups configured**
  ```bash
  # Add to crontab
  0 2 * * * /home/user/tempo-insights/scripts/backup-tempo.sh
  ```

- [ ] **Backup script tested**
  - Creates complete database dump
  - Archives jump log files
  - Saves configuration (without secrets)

- [ ] **Backup storage secured**
  - Stored on separate device/location
  - Encrypted backups
  - Off-site backup copy

- [ ] **Restore procedure tested**
  - Successfully restored from backup
  - Documented restore time (RTO)
  - Verified data integrity

### ✅ Disaster Recovery Plan

- [ ] **Recovery procedures documented**
  - Step-by-step restore process
  - Contact information for key personnel
  - Alternative hosting location identified

- [ ] **Recovery tested within last 30 days**
  - Test restore on separate system
  - Verify all services start
  - Validate data integrity

---

## Monitoring & Logging

### ✅ Log Management

- [ ] **Log rotation configured**
  - Docker logs: 10MB max, 3 files (already configured)
  - System logs: logrotate enabled

- [ ] **Logs reviewed regularly**
  - Daily: Check for errors/warnings
  - Weekly: Review security events
  - Monthly: Audit access logs

- [ ] **Sensitive data not logged**
  - No passwords in logs
  - No JWT tokens in logs
  - PII redacted or hashed

### ✅ Monitoring

- [ ] **Service health monitoring**
  ```bash
  # Check services daily
  docker compose ps
  curl http://localhost:3000/api/health
  ```

- [ ] **Disk space monitoring**
  ```bash
  # Alert when >80% full
  df -h /
  ```

- [ ] **Resource usage tracking**
  ```bash
  # Monitor CPU/RAM
  docker stats
  ```

- [ ] **Security monitoring** (optional)
  - fail2ban logs reviewed
  - Unusual login attempts tracked
  - Port scans detected

---

## Operational Security

### ✅ Access Control

- [ ] **Principle of least privilege**
  - Regular users: No sudo access
  - Service accounts: Minimal permissions
  - Database users: Scoped to needed tables

- [ ] **Multi-factor authentication** (if available)
  - SSH with hardware keys (YubiKey, etc.)
  - Or: 2FA for web dashboard access

- [ ] **Access audit trail**
  - SSH login logging enabled
  - Application access logs retained
  - Admin actions logged

### ✅ Change Management

- [ ] **Change control process documented**
  - Changes tested in QA first
  - Rollback plan for each change
  - Changes scheduled during maintenance windows

- [ ] **Version control used**
  - All configuration in git
  - Tagged releases for production
  - Change history maintained

### ✅ Incident Response

- [ ] **Incident response plan created**
  - Security incident procedures
  - Contact list (on-call)
  - Escalation path defined

- [ ] **Security contacts documented**
  - Who to notify for security issues
  - How to report vulnerabilities
  - External security contacts (if applicable)

---

## Compliance & Documentation

### ✅ Documentation

- [ ] **System documentation complete**
  - Architecture diagram
  - Network topology
  - Service dependencies

- [ ] **Runbook created**
  - Common operations (restart, backup, restore)
  - Troubleshooting procedures
  - Emergency contacts

- [ ] **Security policies documented**
  - Password policy
  - Access control policy
  - Data retention policy
  - Incident response policy

### ✅ Compliance (If Applicable)

- [ ] **GDPR compliance** (EU users)
  - Data processing agreements
  - Privacy policy published
  - User data export capability
  - Right to deletion implemented

- [ ] **HIPAA compliance** (if health data)
  - BAA signed
  - Audit logging enabled
  - Encryption at rest/in transit

---

## Production Launch Checklist

### Final Verification (Day Before Launch)

- [ ] All security items above completed
- [ ] Latest backups verified
- [ ] All services healthy
- [ ] Monitoring alerts configured
- [ ] Team trained on procedures
- [ ] Emergency rollback plan ready

### Launch Day

- [ ] Deploy during maintenance window
- [ ] Monitor logs for 1 hour post-launch
- [ ] Verify all health checks passing
- [ ] Test critical user flows
- [ ] Notify stakeholders of successful launch

### Post-Launch (First Week)

- [ ] Daily log review
- [ ] Monitor resource usage trends
- [ ] Verify backups running successfully
- [ ] Address any performance issues
- [ ] Collect user feedback

---

## Ongoing Security

### Daily

- [ ] Check service health
- [ ] Review error logs
- [ ] Verify backups completed

### Weekly

- [ ] Review security logs
- [ ] Check for failed login attempts
- [ ] Update system packages
- [ ] Verify disk space

### Monthly

- [ ] Test backup restore
- [ ] Review access logs
- [ ] Update Docker images
- [ ] Security audit

### Quarterly

- [ ] Rotate secrets (passwords, keys)
- [ ] Review and update documentation
- [ ] Security training for team
- [ ] Penetration testing (optional)

---

## Emergency Procedures

### Security Incident

1. **Isolate**: Disconnect from network if compromised
2. **Assess**: Determine scope of breach
3. **Contain**: Stop ongoing unauthorized access
4. **Eradicate**: Remove malicious code/accounts
5. **Recover**: Restore from clean backup
6. **Lessons Learned**: Document and improve

### Data Loss

1. **Stop writes**: Prevent further data loss
2. **Assess**: Determine what was lost
3. **Restore**: From most recent backup
4. **Verify**: Check data integrity
5. **Root cause**: Determine cause
6. **Prevent**: Implement safeguards

---

## Security Contacts

**Internal:**
- System Administrator: [contact]
- Security Lead: [contact]
- On-call rotation: [schedule]

**External:**
- Hosting provider support
- Security consultant (if any)
- Legal counsel (if applicable)

---

**Security is an ongoing process, not a one-time checklist!**

Regular reviews and updates to security posture are essential for maintaining a secure production environment.