# Phase 18 — Docker Packaging & Pi Deployment (Revised)

Each task is small, testable, and focused on one concern.

---

## A. Foundation & Environment Setup

### Task 113: Base environment template
**Do:** Create `.env.example` with all required keys (DATABASE_URL, JWT_SECRET, WORKER_TOKEN, DISCOVERY_WINDOW, Next.js vars)  
**Done when:** File exists with commented sections for each service; no secrets included.

### Task 114: Docker environment template
**Do:** Create `.env.docker` with Docker-specific settings (database URLs using container names, host network references)  
**Done when:** File exists with all vars pointing to Docker service names (e.g., `DATABASE_URL=postgresql://postgres:password@db:5432/tempo`).

### Task 115: .dockerignore file
**Do:** Create `.dockerignore` excluding node_modules, .next, .git, .env, volumes/, coverage/  
**Done when:** File exists with all development artifacts excluded.

---

## B. Next.js Web Server Container

### Task 116: Next.js Dockerfile - build stage
**Do:** Create `docker/web/Dockerfile` with build stage using node:20-bookworm base, copy package files, run `npm ci`  
**Done when:** Build stage completes; installs all dependencies including Prisma.

### Task 117: Next.js Dockerfile - standalone build
**Do:** Add build command `npm run build` with standalone output mode configured in next.config.js  
**Done when:** `.next/standalone` directory created in build stage.

### Task 118: Next.js Dockerfile - runtime stage
**Do:** Add runtime stage using node:20-bookworm-slim, copy standalone output and static assets  
**Done when:** Runtime stage < 250MB; includes only production files.

### Task 119: Next.js Dockerfile - Prisma client
**Do:** Copy Prisma client and schema, add postinstall to generate client in runtime stage  
**Done when:** Container can run Prisma queries without "Client not found" errors.

### Task 120: Next.js health check
**Do:** Add `/api/health` endpoint returning `{status: 'ok', timestamp: ...}` with DB ping  
**Done when:** Endpoint responds 200 with valid JSON.

### Task 121: Next.js Dockerfile - entrypoint
**Do:** Add CMD `["node", "server.js"]` and EXPOSE 3000; set ENV NODE_ENV=production  
**Done when:** Container starts and serves on port 3000.

### Task 122: Next.js container test build
**Do:** Build image locally with `docker build -t tempo-web:test -f docker/web/Dockerfile .`  
**Done when:** Build succeeds; no errors in output.

---

## C. Bluetooth Scanner Container

### Task 123: Bluetooth scanner Dockerfile - base stage
**Do:** Create `docker/bluetooth-scanner/Dockerfile` with node:20-bookworm base, install BlueZ runtime libs (libbluetooth3, libusb-1.0-0, udev)  
**Done when:** Base stage has all Bluetooth runtime dependencies.

### Task 124: Bluetooth scanner Dockerfile - Python stage
**Do:** Add Python 3.11 installation, pip, and build tools (python3-dev, libbluetooth-dev, build-essential)  
**Done when:** `python3 --version` shows 3.11.x; pip available.

### Task 125: Bluetooth scanner Dockerfile - smpmgr install
**Do:** Install smpmgr==0.12.0 via pip in virtual environment at /opt/venv  
**Done when:** `smpmgr --version` shows 0.12.0.

### Task 126: Bluetooth scanner Dockerfile - plugin copy
**Do:** Copy `smpmgr-extensions/plugins/` to `/opt/smpmgr-extensions/plugins/` in container  
**Done when:** Plugin directory exists with __init__.py and tempo_group.py.

### Task 127: Bluetooth scanner Dockerfile - Node.js deps
**Do:** Copy package.json, install dependencies, copy worker source (`src/workers/bluetooth-scanner.ts`, `src/lib/bluetooth/`, TempoBTClient)  
**Done when:** All TypeScript files and dependencies present.

### Task 128: Bluetooth scanner Dockerfile - compile TypeScript
**Do:** Add TypeScript compilation step (tsconfig for workers), output to `/app/dist`  
**Done when:** `bluetooth-scanner.js` exists in dist/ folder.

### Task 129: Bluetooth scanner Dockerfile - runtime stage
**Do:** Create slim runtime stage, copy only /opt/venv, /opt/smpmgr-extensions, compiled JS, and node_modules  
**Done when:** Runtime stage < 400MB.

### Task 130: Bluetooth scanner Dockerfile - entrypoint
**Do:** Set PATH to include /opt/venv/bin, CMD `["node", "dist/bluetooth-scanner.js"]`  
**Done when:** Container entry point set correctly.

### Task 131: Bluetooth scanner health logging
**Do:** Add heartbeat log every 60s in bluetooth-scanner.ts worker loop  
**Done when:** Log shows "Bluetooth scanner alive - cycle N" message.

### Task 132: Bluetooth scanner container test build
**Do:** Build with `docker build -t tempo-bt-scanner:test -f docker/bluetooth-scanner/Dockerfile .`  
**Done when:** Build succeeds without errors.

---

## D. Analysis Worker Container

### Task 133: Analysis worker Dockerfile - build stage
**Do:** Create `docker/analysis-worker/Dockerfile` using node:20-bookworm base, copy package files, install dependencies  
**Done when:** Dependencies installed including Prisma client.

### Task 134: Analysis worker Dockerfile - worker source
**Do:** Copy worker source files (`src/workers/log-processor.ts`, `src/lib/analysis/`), compile TypeScript  
**Done when:** Compiled `log-processor.js` in dist/ folder.

### Task 135: Analysis worker Dockerfile - runtime stage
**Do:** Create node:20-bookworm-slim runtime stage, copy compiled code and node_modules  
**Done when:** Runtime stage < 250MB.

### Task 136: Analysis worker Dockerfile - Prisma setup
**Do:** Copy prisma/ directory, set up Prisma client generation in entrypoint  
**Done when:** Worker can query database with Prisma.

### Task 137: Analysis worker health logging
**Do:** Add heartbeat log every 30s in log-processor.ts loop showing queue size  
**Done when:** Log shows "Analysis worker alive - pending: N" message.

### Task 138: Analysis worker Dockerfile - entrypoint
**Do:** Set CMD `["node", "dist/log-processor.js"]`, expose no ports  
**Done when:** Container starts and runs worker loop.

### Task 139: Analysis worker container test build
**Do:** Build with `docker build -t tempo-analysis:test -f docker/analysis-worker/Dockerfile .`  
**Done when:** Build succeeds without errors.

---

## E. Supabase Stack Setup

### Task 140: Clone Supabase Docker config
**Do:** Run `git clone --depth 1 https://github.com/supabase/supabase` in project root, copy docker/ to supabase/  
**Done when:** `supabase/docker/docker-compose.yml` exists.

### Task 141: Supabase environment configuration
**Do:** Copy `.env.example` to `supabase/.env`, generate secure secrets (POSTGRES_PASSWORD, JWT_SECRET)  
**Done when:** All placeholder passwords replaced with 24+ char secrets.

### Task 142: Generate Supabase JWT keys
**Do:** Use Supabase JWT generator to create ANON_KEY and SERVICE_ROLE_KEY from JWT_SECRET  
**Done when:** Both keys present in supabase/.env.

### Task 143: Configure Supabase dashboard credentials
**Do:** Set DASHBOARD_USERNAME and DASHBOARD_PASSWORD in supabase/.env  
**Done when:** Credentials set to non-default values.

### Task 144: Supabase volumes directory setup
**Do:** Create `supabase/volumes/{db,storage,functions,api,logs}` directories  
**Done when:** All volume mount points exist with correct permissions.

### Task 145: Test Supabase stack startup
**Do:** Run `cd supabase && docker compose up -d`, verify all 14 services healthy  
**Done when:** `docker compose ps` shows all services "running (healthy)".

### Task 146: Verify Supabase endpoints
**Do:** Test Kong gateway at http://localhost:8000, Studio at http://localhost:8000/  
**Done when:** Both endpoints respond correctly.

### Task 147: Create tempo database schema
**Do:** Run Prisma migrations against Supabase Postgres (localhost:5432)  
**Done when:** All tables exist; `prisma migrate deploy` succeeds.

### Task 148: Seed initial admin user
**Do:** Run `prisma db seed` against Supabase Postgres  
**Done when:** Admin user exists in database.

---

## F. Docker Compose Integration

### Task 149: Root docker-compose.yml - networks
**Do:** Create `docker-compose.yml` defining `tempo-network` bridge network  
**Done when:** Network configuration exists.

### Task 150: Root docker-compose.yml - database service
**Do:** Add external database reference pointing to Supabase Postgres  
**Done when:** Services can reference `db` hostname.

### Task 151: Root docker-compose.yml - web service
**Do:** Add tempo-web service using local Dockerfile, environment from .env.docker, port 3000:3000  
**Done when:** Service defined with all required env vars.

### Task 152: Root docker-compose.yml - bluetooth scanner service
**Do:** Add tempo-bt-scanner with network_mode: host, cap_add: NET_ADMIN, device mapping for /dev/bus/usb  
**Done when:** Service configured for Bluetooth access.

### Task 153: Root docker-compose.yml - bluetooth scanner volumes
**Do:** Mount /run/dbus:/run/dbus:ro and smpmgr-extensions plugins directory  
**Done when:** D-Bus socket and plugins accessible in container.

### Task 154: Root docker-compose.yml - analysis worker service
**Do:** Add tempo-analysis service on tempo-network, no ports exposed  
**Done when:** Worker service defined with database access.

### Task 155: Root docker-compose.yml - service dependencies
**Do:** Set depends_on for all services requiring database (web, workers)  
**Done when:** Startup order enforced via depends_on.

### Task 156: Root docker-compose.yml - health checks
**Do:** Add healthcheck configuration for web service (curl localhost:3000/api/health)  
**Done when:** Web service shows healthy status after startup.

### Task 157: Root docker-compose.yml - restart policies
**Do:** Set `restart: unless-stopped` for all services  
**Done when:** All services restart on failure.

---

## G. Volume & Persistence Configuration

### Task 158: Database volume configuration
**Do:** Define named volume `tempo-db-data` for Postgres persistence  
**Done when:** Volume defined in docker-compose.yml.

### Task 159: Storage volume configuration
**Do:** Define named volume `tempo-storage` for Supabase Storage files  
**Done when:** Volume mounted to storage service.

### Task 160: Functions volume configuration
**Do:** Define volume mount `./supabase/volumes/functions:/home/deno/functions`  
**Done when:** Edge functions directory persistent.

### Task 161: Logs volume configuration
**Do:** Define volume for Vector logs at `./supabase/volumes/logs`  
**Done when:** Log aggregation persists across restarts.

### Task 162: Config volume for pgsodium
**Do:** Add named volume `db-config` for pgsodium encryption key  
**Done when:** Encryption key persists.

---

## H. Environment & Secrets Management

### Task 163: Environment validation script
**Do:** Create `scripts/validate-env.sh` checking all required vars in .env.docker  
**Done when:** Script exits 1 if any required var missing; 0 if all present.

### Task 164: Secret generation script
**Do:** Create `scripts/generate-secrets.sh` producing random JWT_SECRET, POSTGRES_PASSWORD, etc.  
**Done when:** Running script outputs valid secrets for copy/paste.

### Task 165: Environment sync documentation
**Do:** Add README section explaining .env vs .env.docker usage  
**Done when:** Clear instructions on which file for development vs Docker.

---

## I. Build & Deployment Scripts

### Task 166: Build all images script
**Do:** Create `scripts/build-images.sh` building all three Docker images with version tags  
**Done when:** Script builds web, bluetooth-scanner, analysis-worker images.

### Task 167: Platform-specific build flag
**Do:** Add `--platform linux/arm64` flag to build script for Raspberry Pi  
**Done when:** Script supports PLATFORM env var (default arm64).

### Task 168: Docker Compose startup script
**Do:** Create `scripts/start-tempo.sh` running env validation, Supabase startup, then app services  
**Done when:** Single command starts entire stack.

### Task 169: Docker Compose shutdown script
**Do:** Create `scripts/stop-tempo.sh` gracefully stopping all services, optionally removing volumes  
**Done when:** Script stops services; `--clean` flag removes volumes.

### Task 170: Host BlueZ setup script
**Do:** Create `scripts/setup-host-bluetooth.sh` installing BlueZ, enabling service, testing hciconfig  
**Done when:** Script ensures BlueZ running on host Pi.

### Task 171: Quick test script
**Do:** Create `scripts/test-deploy.sh` building images, starting stack, running health checks  
**Done when:** Script verifies all services healthy.

---

## J. Documentation & Production Readiness

### Task 172: Docker deployment README
**Do:** Create `docs/DOCKER_DEPLOYMENT.md` with step-by-step Pi setup instructions  
**Done when:** Document covers prerequisites, build, deploy, troubleshooting.

### Task 173: Prerequisites checklist
**Do:** Document required Pi configuration (Docker installed, BlueZ, permissions, volumes)  
**Done when:** Checklist complete; testable on fresh Pi.

### Task 174: Port exposure documentation
**Do:** Document all exposed ports (3000, 8000, 8443, 6543) and their purposes  
**Done when:** Table in docs showing port mappings.

### Task 175: Security hardening checklist
**Do:** Create checklist: change Supabase defaults, firewall rules, non-root users, volume permissions  
**Done when:** Security tasks documented with verification steps.

### Task 176: Backup and restore procedures
**Do:** Document procedures for backing up volumes (db-data, storage) and restoring  
**Done when:** Step-by-step backup/restore instructions complete.

### Task 177: Troubleshooting guide
**Do:** Create troubleshooting section for common issues (Bluetooth permissions, network conflicts, health check failures)  
**Done when:** Guide includes symptoms, diagnosis, and solutions.

### Task 178: Production deployment checklist
**Do:** Create pre-flight checklist for production deployment  
**Done when:** All tasks (secrets, volumes, health checks, backups) verified.

---

## K. Testing & Validation

### Task 179: Isolated container tests
**Do:** Test each container individually with `docker run` and mock dependencies  
**Done when:** Each container starts and shows healthy logs in isolation.

### Task 180: Integration test - web only
**Do:** Start web container + Supabase, test /api/health and login  
**Done when:** Web app fully functional with database.

### Task 181: Integration test - bluetooth scanner
**Do:** Start scanner container with D-Bus access, verify it can run `smpmgr --version`  
**Done when:** Scanner logs show smpmgr available and scanning attempts.

### Task 182: Integration test - analysis worker
**Do:** Start analysis worker, seed test jump log, verify processing  
**Done when:** Worker processes log and updates initialAnalysisTimestamp.

### Task 183: Full stack integration test
**Do:** Start entire stack, verify all health checks pass, test end-to-end flow  
**Done when:** Can register user, assign device (mock), upload log, view analysis.

### Task 184: Raspberry Pi deployment test
**Do:** Deploy to actual Pi 5, verify Bluetooth hardware access, real device scan  
**Done when:** Scanner detects actual Tempo-BT device on Pi.

---
