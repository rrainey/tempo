# ===================================
# STAGE 1: BASE WITH BLUETOOTH DEPS
# ===================================
FROM node:20-bookworm AS base

WORKDIR /app

# Install Bluetooth runtime libraries
RUN apt-get update && apt-get install -y \
    bluez \
    libbluetooth3 \
    libusb-1.0-0 \
    udev \
    && rm -rf /var/lib/apt/lists/*

# ===================================
# STAGE 2: PYTHON & BUILD TOOLS
# ===================================
FROM base AS python-builder

# Install Python 3.11 and build dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    libbluetooth-dev \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create Python virtual environment
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Upgrade pip
RUN pip install --no-cache-dir --upgrade pip==24.0

# Install smpmgr with pinned version
RUN pip install --no-cache-dir smpmgr==0.13.2

# ===================================
# STAGE 3: NODE.JS DEPENDENCIES
# ===================================
FROM base AS node-deps

WORKDIR /app

# Enable corepack for pnpm
RUN corepack enable && corepack prepare pnpm@latest --activate

# Copy package files
COPY package.json pnpm-lock.yaml ./

# Install dependencies
RUN pnpm install --frozen-lockfile

# ===================================
# STAGE 4: TYPESCRIPT COMPILATION
# ===================================
FROM node-deps AS builder

WORKDIR /app

# Copy source code
COPY . .

# Copy Prisma schema and generate client
COPY prisma ./prisma/
RUN npx prisma generate

# Compile TypeScript for workers
# Only compile bluetooth-scanner and its direct dependency (bluetooth service)
# Don't use wildcards to avoid pulling in unwanted files
RUN echo '{ \
  "compilerOptions": { \
    "target": "ES2020", \
    "module": "commonjs", \
    "lib": ["ES2020"], \
    "outDir": "./dist", \
    "strict": false, \
    "esModuleInterop": true, \
    "skipLibCheck": true, \
    "forceConsistentCasingInFileNames": true, \
    "resolveJsonModule": true, \
    "moduleResolution": "node" \
  }, \
  "include": [ \
    "src/workers/bluetooth-scanner.ts", \
    "src/lib/bluetooth/bluetooth.service.ts" \
  ] \
}' > tsconfig.workers.json

# Compile TypeScript
RUN npx tsc -p tsconfig.workers.json

# ===================================
# STAGE 5: RUNTIME
# ===================================
FROM node:20-bookworm-slim AS runner

WORKDIR /app

# Install runtime Bluetooth libraries only (no dev packages)
RUN apt-get update && apt-get install -y \
    bluez \
    libbluetooth3 \
    libusb-1.0-0 \
    udev \
    python3 \
    && rm -rf /var/lib/apt/lists/*

# Copy Python virtual environment with smpmgr
COPY --from=python-builder /opt/venv /opt/venv

# Copy smpmgr extensions/plugins
COPY --chown=node:node smpmgr-extensions/plugins /opt/smpmgr-extensions/plugins

# Copy compiled JavaScript and dependencies
COPY --from=builder /app/dist ./dist
COPY --from=builder /app/node_modules ./node_modules
COPY --from=builder /app/prisma ./prisma
COPY --from=builder /app/package.json ./

# Set PATH to include Python venv
ENV PATH="/opt/venv/bin:$PATH"

# Set environment for production
ENV NODE_ENV=production
ENV SMPMGR_PLUGIN_PATH=/opt/smpmgr-extensions/plugins

# Create non-root user for security
RUN useradd -u 1001 -g bluetooth -m -s /bin/bash btscanner

# Switch to non-root user
USER btscanner

# Verify smpmgr installation
RUN smpmgr --version

# Health logging - the worker itself logs heartbeats
# No port exposure needed (background service)

# Start the bluetooth scanner worker
CMD ["node", "dist/workers/bluetooth-scanner.js"]