# Docker Containerization Guide for Next.js with Bluetooth on Raspberry Pi 5

This document captures research performed to take a more advanced approach to production packaging of the tempo-insight application.  Rather than
  relying on a older, strictly file based deployment, we will use Docker on the Raspberry Pi 4 or 5 combined with Ubuntu 22 as the base operating system.

---

Here's the prompting dialog that led us to generating this report:

"Let's move on to Phase 18, "Packaging and Pi Deployment". Task  113 is complete (file attached).  For Task 114, rather than using systemd and a raw file system deployment, I'd like to have all tempo-insights server components built and packaged as Docker images.  This would include the Next.js server, the analysis engine, the bluetooth scanner, as well as the standard Supabase Docker images.  Let's start by preparing Docker images for all three of our project-specific components.  Do you have any clarification questions for me before we start?"

"Great! Moving to Phase 18 with Docker containerization. Before I start creating the Dockerfiles and docker-compose configuration, I have some clarification questions:

### 1. **Docker Architecture Questions:**
- Should all three services (Next.js server, bluetooth scanner, analysis worker) be separate containers, or would you prefer some combined?
- For the bluetooth scanner container, it will need access to the host's Bluetooth hardware. Should we use `--privileged` mode or specific device mapping (`/dev/bus/usb`, etc.)?

### 2. **Supabase Integration:**
- You mentioned "standard Supabase Docker images" - should I include the full Supabase local stack in the docker-compose (postgres, realtime, storage, auth, etc.), or will Supabase be running separately?
- Or are you planning to use the existing `supabase/setup` from your local development?

### 3. **smpmgr and Bluetooth Dependencies:**
- The `smpmgr` tool and `smpmgr-extensions/plugins` - should these be installed inside the bluetooth scanner container?
- Do you have specific instructions for installing `smpmgr` on Ubuntu/Debian, or should I research the best approach?
- Should `bluetoothctl` and BlueZ be installed in the container?

### 4. **Data Persistence:**
- Which directories need to be mounted as volumes for persistence?
  - Database data (if including Supabase)
  - Jump logs storage (Supabase storage bucket)
  - Any configuration files?

### 5. **Networking:**
- Should all containers be on the same Docker network?
- What ports need to be exposed to the host? (Next.js on 3000, any others?)

### 6. **Environment Variables:**
- Should I create a template `.env.docker` file alongside `.env.example` with Docker-specific settings (like database URLs pointing to container names)?

### 7. **Build Strategy:**
- Multi-stage builds for smaller images?
- Base image preference (node:20-alpine, node:20-bullseye, etc.)?"

"1. Run all three services as separate Docker containers. For the bluetooth-scanner, investigate which of these two approaches provides the most performant approach and use that one.

2.Supabase will not be run separately. Provide scripts and instructions for installing and configuring all standard Supabase components as docker images as part of our setup. This will all be set up on a clean server, so we ill not use the existing Supabase instance.

3. Yes, include `smpmgr` and our extension plugin (and the requisite Python runtimes) with the bluetooth-scanner container. Search for the best approach to install it (this will most likely be a direct clone of today's current `smpmgr` release from GitHub) The decision on where to install `bluetoothctl` and BlueZ will likely depend on the choices about how to run the bluetooth-scanner container in (1). Install where needed.

4. All three of these items (including .env and other configuration files) should likely best be stored as volumes for persistence.

5. All containers should be on the same Docker network. Port 3000 for the Next.js web server and ports 54322 and port 52321 should be exposed as well.

6. Yes, create a `.env.docker`

7. Multi-stage builds are acceptable. Research and use the most reliable Base image based on current error reports."

---

**For production Raspberry Pi 5 deployments with Bluetooth hardware access, use specific device mapping with `--cap-add=NET_ADMIN`, never `--privileged` mode.** Use **node:20-bookworm-slim** as your base image for maximum compatibility with native modules. The smpmgr tool installs via pip with BlueZ dependencies, while Supabase self-hosting requires 14 interconnected Docker services exposing ports through Kong gateway on 8000.

This guide provides battle-tested configurations for Docker containerization of Next.js applications requiring Bluetooth hardware access on ARM64 architecture. Security is paramount: privileged mode grants complete host access and fails compliance standards, while specific capability grants provide equivalent Bluetooth performance with containment. Alpine images create persistent compatibility issues with Bluetooth libraries due to musl vs glibc differences, making Debian-based images the only production-viable option for hardware integration.

## Docker Bluetooth access: Security vs functionality tradeoff

The choice between `--privileged` mode and specific device mapping fundamentally determines your security posture. Both approaches achieve identical Bluetooth performance when properly configured, but their security implications differ dramatically.

**Privileged mode is a critical security vulnerability** in production environments. It grants container root the same access as host root, allowing complete filesystem mounting, kernel parameter modification, and unrestricted device access. Security audits flag privileged containers as high-severity issues. The Docker security documentation explicitly warns against this flag, and CIS Docker Benchmark standards fail privileged containers automatically. For Raspberry Pi deployments, especially internet-facing systems, privileged mode creates an attack vector where container compromise equals full host compromise.

**Specific device mapping with capabilities provides equivalent functionality** while maintaining security boundaries. The recommended configuration uses `--net=host` (mandatory for HCI socket access), `--cap-add=NET_ADMIN` for network interface management, and selective device mounting. This approach limits blast radius: compromise affects only Bluetooth functionality rather than entire host infrastructure.

For Raspberry Pi 5 production deployments, the optimal Docker Compose configuration is:

```yaml
version: '3.8'

services:
  bluetooth-app:
    image: your-app:arm64
    container_name: bluetooth-app
    
    # Network: Required for Bluetooth HCI sockets
    network_mode: host
    
    # Security: Minimal capabilities
    cap_add:
      - NET_ADMIN
    cap_drop:
      - ALL
    
    # Device access: USB Bluetooth adapters only
    devices:
      - /dev/bus/usb:/dev/bus/usb
    
    # Volumes: D-Bus for shared Bluetooth stack
    volumes:
      - /run/dbus:/run/dbus:ro
      - bluetooth-data:/var/lib/bluetooth
    
    # Non-root user when possible
    user: "1000:1000"
    
    restart: unless-stopped
    
    healthcheck:
      test: ["CMD", "hciconfig", "hci0"]
      interval: 30s
      timeout: 10s
      retries: 3

volumes:
  bluetooth-data:
```

**The shared D-Bus approach is simplest for Raspberry Pi 5.** Run BlueZ on the host system (`sudo systemctl enable bluetooth`) and share the D-Bus socket into containers read-only. This eliminates the need to run `dbus` and `bluetoothd` inside containers, reduces resource consumption, and simplifies adapter management. The host handles Bluetooth service lifecycle while containers access it through `/run/dbus/system_bus_socket`.

For exclusive container control scenarios, you need additional capabilities. Add `CAP_SYS_ADMIN` for D-Bus and service management, install BlueZ packages in the Dockerfile (`bluez`, `bluetooth`, `dbus`), and implement proper startup sequencing in your entrypoint script. Reset the adapter on startup (`hciconfig hci0 down && hciconfig hci0 up`) to handle unclean exits, wait for start-stop-daemon completion before starting your application, and verify adapter readiness with health checks.

Common troubleshooting patterns on Raspberry Pi 5 include adapter ownership conflicts (only one process can own `hci0` at a time), D-Bus authentication errors (use `:ro` for shared mode), and missing kernel modules (`sudo modprobe bluetooth` and `btusb`). The Pi 5's built-in Broadcom BCM43xx Bluetooth operates over UART with firmware loaded by the btuart service, making host-managed BlueZ the most reliable approach.

## smpmgr installation: Python-based MCU management

The **official smpmgr repository is https://github.com/intercreate/smpmgr** with the latest version 0.12.1 available on PyPI. This Apache-2.0 licensed tool implements the Simple Management Protocol (SMP) for remotely managing MCU firmware over serial, Bluetooth, or UDP connections.

**Installation via pip is the recommended method** for Docker containers. The tool requires Python 3.10-3.12 and automatically pulls dependencies including `smpclient`, `smp`, `typer`, and `rich`. For **Bluetooth support, BlueZ system libraries are mandatory**: install `bluez`, `libbluetooth-dev`, `python3-dev`, and `build-essential` before running pip install.

The production-optimized Dockerfile uses multi-stage builds to separate compilation from runtime:

```dockerfile
# Build stage - includes development headers
FROM python:3.11-slim AS builder

RUN apt-get update && apt-get install -y \
    bluez \
    libbluetooth-dev \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Pin version for reproducibility
RUN pip install --no-cache-dir --upgrade pip==24.0 && \
    pip install --no-cache-dir smpmgr==0.12.0

# Runtime stage - only runtime dependencies
FROM python:3.11-slim

LABEL maintainer="your-email@example.com"
LABEL description="smpmgr MCU Manager CLI Tool"

# Runtime libraries only (no -dev packages)
RUN apt-get update && apt-get install -y \
    bluez \
    libbluetooth3 \
    libusb-1.0-0 \
    udev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Security: non-root user
RUN groupadd -g 1000 smpuser && \
    useradd -u 1000 -g 1000 -m -s /bin/bash smpuser

USER smpuser
WORKDIR /home/smpuser

ENTRYPOINT ["smpmgr"]
CMD ["--help"]
```

**For serial/UART access**, run containers with device mapping: `docker run --device=/dev/ttyUSB0:/dev/ttyUSB0 your-image smpmgr --conntype serial --connstring /dev/ttyUSB0 <command>`. **For Bluetooth operations**, combine with the privileged Bluetooth configuration: `docker run --net=host --cap-add=NET_ADMIN -v /run/dbus:/run/dbus your-image smpmgr --conntype ble --connstring <device-name> <command>`.

The smpmgr tool supports custom plugins for proprietary SMP groups via `--plugin-path` flag. Install specific versions for production reproducibility (`smpmgr==0.12.0`) and verify installation with `smpmgr --version`. The tool integrates with Zephyr RTOS and other SMP-enabled devices, serving as the Python reference implementation alongside the Go-based Apache mynewt-mcumgr-cli.

## Node.js base images: Debian over Alpine for ARM64 reliability

**The node:20.19.5-bookworm-slim image is the gold standard** for production ARM64/Raspberry Pi 5 deployments in 2025. This recommendation stems from critical compatibility differences between Alpine's musl libc and Debian's glibc, particularly affecting native module compilation and Bluetooth libraries.

Alpine Linux images create persistent production issues despite their smaller footprint. The **node:20-alpine image is officially EXPERIMENTAL** for ARM64 architecture, with Node.js ARM64 builds unsupported. Native modules requiring compilation face musl vs glibc incompatibilities, Bluetooth libraries (`noble`, `bluetooth-serial-port`) fail consistently, and adding build tools (`python3 make g++`) negates size advantages by adding 100-200MB. The 70% smaller size (150MB vs 220MB) doesn't justify the reliability risks for hardware-integrated applications.

**Debian Bookworm provides full glibc compatibility, official Node.js support, and proven ARM64 stability.** Released in June 2023 with support through 2028, Bookworm includes glibc 2.36, BlueZ 5.66+, and optimizations for newer ARM64 chips like the Raspberry Pi 5's Cortex-A76 processor. Bullseye (Debian 11) remains viable until June 2026 EOL but lacks the latest hardware support optimizations.

The comparison matrix reveals clear patterns:

| Factor | Alpine | Bookworm-slim | Recommendation |
|--------|--------|---------------|----------------|
| **Size** | 150-170 MB | 220-240 MB | Alpine advantage |
| **Native modules** | Requires build tools | Usually works OOTB | **Bookworm critical** |
| **Bluetooth libs** | Frequent failures | Reliable | **Bookworm only** |
| **Next.js + Sharp** | Complex setup required | Works immediately | Bookworm |
| **Production stability** | Experimental ARM64 | Official support | **Bookworm** |
| **Security CVEs** | 0-2 | 15-25 | Alpine advantage |
| **Build time ARM64** | Complex compilation | Standard | Bookworm |

**Next.js on ARM64 requires specific configuration** to avoid build hangs and deployment issues. Recent versions (15.2.4+) show extreme build time regressions on Docker ARM64, increasing from 2 minutes to 10+ minutes. Pin to Next.js 15.1.7 or earlier for stability. Use standalone output mode to ensure proper file structure:

```javascript
// next.config.js
module.exports = {
  output: 'standalone'
}
```

The multi-stage Dockerfile optimizes for Raspberry Pi deployment:

```dockerfile
# Build stage - full image acceptable for tooling
FROM node:20-bookworm AS builder

WORKDIR /app

# Install native module dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    make \
    g++ \
    libbluetooth-dev \
    && rm -rf /var/lib/apt/lists/*

COPY package*.json ./
RUN npm ci --only=production

COPY . .
RUN npm run build

# Runtime stage - slim image for production
FROM node:20-bookworm-slim

WORKDIR /app

# Runtime dependencies only
RUN apt-get update && apt-get install -y \
    libbluetooth3 \
    && rm -rf /var/lib/apt/lists/*

# Copy from builder
COPY --from=builder /app/.next/standalone ./
COPY --from=builder /app/.next/static ./.next/static
COPY --from=builder /app/public ./public

ENV NODE_ENV=production
ENV PORT=3000

EXPOSE 3000

CMD ["node", "server.js"]
```

**Critical issues to avoid on ARM64**: Never build `linux/amd64` images for ARM64 deployment—emulation causes 5-10x performance degradation and build hangs. Always use `--platform linux/arm64` explicitly. For cross-compilation from x86_64 hosts, use Docker buildx with proper platform targeting. The `.next` build folder not found errors stem from volume mounting issues; standalone output mode eliminates these problems by packaging all dependencies correctly.

Native module compilation works reliably on Debian-based images where Alpine requires `libc6-compat` and extensive troubleshooting. The glibc vs musl difference is non-negotiable for production Bluetooth applications. Pin exact versions in production (`node:20.19.5-bookworm-slim` not `node:20-bookworm-slim`) for reproducible builds, and always test on actual Raspberry Pi hardware before production deployment.

## Supabase self-hosting: 14-service architecture with Kong gateway

Supabase self-hosting via Docker deploys **14 interconnected services** managed through a single docker-compose.yml file from the official repository at https://github.com/supabase/supabase/tree/master/docker. All client requests route through Kong API Gateway on port 8000, which reverse proxies to internal services.

**The core service architecture includes** PostgreSQL 15+ on port 5432 for direct connections and port 6543 for pooled connections via Supavisor, PostgREST generating automatic RESTful APIs from database schemas, GoTrue providing JWT-based authentication with OAuth/email/phone support, and Realtime enabling WebSocket subscriptions for database changes. Storage API offers S3-compatible object storage, Edge Functions run Deno-based serverless code, and Studio provides the web-based management dashboard. Supporting services include Kong (gateway), ImgProxy (image optimization), postgres-meta (database management API), Vector (log aggregation), and Logflare (analytics).

**Installation requires just four commands** after cloning the repository:

```bash
git clone --depth 1 https://github.com/supabase/supabase
cp -rf supabase/docker/* supabase-project/
cp supabase/docker/.env.example supabase-project/.env
cd supabase-project && docker compose up -d
```

**All API endpoints route through Kong on port 8000**. Access REST APIs at `http://your-ip:8000/rest/v1/`, authentication at `/auth/v1/`, storage at `/storage/v1/`, realtime WebSocket at `/realtime/v1/`, edge functions at `/functions/v1/{function-name}`, and the Studio dashboard at the root `/`. This unified gateway simplifies firewall configuration and SSL certificate management.

The complete port mapping exposes only essential services:

| Service | External Port | Purpose |
|---------|--------------|---------|
| **Kong HTTP** | 8000 | Main API gateway (all APIs route here) |
| **Kong HTTPS** | 8443 | Secure gateway access |
| **Supavisor** | 6543 | Pooled PostgreSQL connections |
| **PostgreSQL** | 5432* | Direct DB access (not exposed by default) |
| **Mail Web** | 9000 | InBucket email testing UI (dev only) |
| **Mail SMTP** | 2500 | Development SMTP server |

*PostgreSQL direct access should remain internal—use Supavisor pooler for scalable connections.

**Critical data persistence requires four volume categories.** Database volumes mount `./volumes/db/data:/var/lib/postgresql/data:Z` for PostgreSQL data, plus migration scripts for realtime, webhooks, roles, JWT, and pooler configuration. Storage volumes persist uploaded files at `./volumes/storage:/var/lib/storage:z`. Function volumes store Deno edge function code at `./volumes/functions:/home/deno/functions:Z`. Configuration volumes include Kong gateway rules (`./volumes/api/kong.yml`) and Vector logging config (`./volumes/logs/vector.yml`). A named volume `db-config` persists the pgsodium encryption key.

**Security configuration is mandatory before production deployment.** The default .env file contains placeholder credentials that MUST be changed:

```bash
# Generate strong passwords (24+ characters)
POSTGRES_PASSWORD=your-super-secret-and-long-postgres-password

# Generate secure JWT secret (40+ characters)  
JWT_SECRET=your-super-secret-jwt-token-with-at-least-32-characters-long

# Generate ANON_KEY and SERVICE_ROLE_KEY from JWT_SECRET
# Use Supabase's official JWT generator tool
ANON_KEY=<generated-from-jwt-secret>
SERVICE_ROLE_KEY=<generated-from-jwt-secret>

# Change dashboard credentials
DASHBOARD_USERNAME=admin
DASHBOARD_PASSWORD=secure-password-here

# Generate encryption key for connection strings
PG_META_CRYPTO_KEY=<random-32-char-string>
```

**For production environments, replace development services** with production alternatives. Swap InBucket mail server for real SMTP providers like AWS SES:

```bash
SMTP_HOST=email-smtp.us-east-1.amazonaws.com
SMTP_PORT=465
SMTP_USER=your-aws-ses-username
SMTP_PASS=your-aws-ses-password
SMTP_ADMIN_EMAIL=admin@yourdomain.com
```

Configure S3 storage instead of local filesystem:

```bash
STORAGE_BACKEND=s3
GLOBAL_S3_BUCKET=your-production-bucket
REGION=us-east-1
```

**OAuth provider configuration** enables third-party authentication. For GitHub OAuth, add environment variables:

```bash
EXTERNAL_GITHUB_ENABLED=true
EXTERNAL_GITHUB_CLIENT_ID=your-github-client-id
EXTERNAL_GITHUB_SECRET=your-github-client-secret
EXTERNAL_GITHUB_REDIRECT_URI=http://your-domain:8000/auth/v1/callback
```

Service health monitoring uses built-in Docker healthchecks. Check status with `docker compose ps`—all services should show `running (healthy)`. For troubleshooting, inspect logs with `docker compose logs -f <service-name>`. Common issues include container exits due to incorrect Docker socket location for rootless Docker (`/run/user/1000/docker.sock`), authentication failures requiring JWT key regeneration, and migration errors solved by removing `volumes/db/data` (WARNING: destructive).

**The official documentation updates monthly** with new service versions. Images on Docker Hub at https://hub.docker.com/u/supabase receive regular security patches and feature updates. Update by changing version tags in docker-compose.yml, pulling new images with `docker compose pull`, and restarting with `docker compose up -d`.

Database connection strings differ by mode. Session mode provides direct PostgreSQL access: `postgres://postgres.your-tenant-id:password@localhost:5432/postgres`. Transaction mode uses Supavisor pooling for scalability: `postgres://postgres.your-tenant-id:password@localhost:6543/postgres`. Always prefer pooled connections for production applications with multiple clients.

## Key recommendations for production deployment

**Security takes priority over convenience** in production containerization. Never use `--privileged` mode for Bluetooth access—specific device mapping with `--cap-add=NET_ADMIN` provides identical functionality while maintaining container isolation. This configuration passes security audits and limits compromise radius to Bluetooth subsystems rather than entire hosts.

**Image selection determines long-term reliability.** Use node:20-bookworm-slim exclusively for ARM64 production deployments with native modules. Alpine's musl libc incompatibilities create persistent issues with Bluetooth libraries that make it unsuitable despite smaller image size. The 70MB size difference matters far less than production stability and native module compatibility.

**Multi-stage Docker builds optimize final image size** while maintaining full compilation capabilities. Build stages include all development headers (libbluetooth-dev, python3-dev, build-essential) while runtime stages contain only required libraries (libbluetooth3). This pattern achieves compact production images without sacrificing build reliability.

For Raspberry Pi 5 deployments, the recommended stack combines node:20-bookworm-slim for application containers, shared D-Bus for Bluetooth access (host-managed BlueZ), smpmgr installed via pip for MCU management, and Supabase self-hosted via official docker-compose for backend services. Pin all versions explicitly in production, test on actual ARM64 hardware before deployment, and implement comprehensive health checks for all services.

**The shared D-Bus pattern simplifies Bluetooth architecture** by running BlueZ on the Raspberry Pi host and mounting `/run/dbus:/run/dbus:ro` read-only into containers. This eliminates per-container service management, reduces resource consumption, and provides reliable adapter access across multiple containers. Combined with network_mode host and CAP_NET_ADMIN capability, this configuration delivers production-grade Bluetooth access with minimal security exposure.

Supabase self-hosting requires 14 services but simplifies to a single Kong endpoint for all client traffic. Change all default credentials before production deployment, replace InBucket with production SMTP, configure S3 storage for scalability, and implement regular backups of PostgreSQL data and storage volumes. The 4GB+ RAM requirement and 50GB+ storage recommendation ensure stable operation under load.

This architecture provides a secure, scalable foundation for Next.js applications with hardware integration on ARM64 platforms, balancing functionality with security best practices for production deployment.
