# ===================================
# STAGE 1: DEPENDENCIES
# ===================================
FROM node:20-bookworm AS deps

WORKDIR /app

# Enable corepack for pnpm
RUN corepack enable && corepack prepare pnpm@latest --activate

# Copy package files
COPY package.json pnpm-lock.yaml ./

# Install ALL dependencies (including dev for build tools)
RUN pnpm install --frozen-lockfile

# ===================================
# STAGE 2: BUILD
# ===================================
FROM node:20-bookworm AS builder

WORKDIR /app

# Enable corepack for pnpm
RUN corepack enable && corepack prepare pnpm@latest --activate

# Copy dependencies from deps stage
COPY --from=deps /app/node_modules ./node_modules

# Copy application source
COPY . .

# Copy Prisma schema and generate client
COPY prisma ./prisma/
RUN npx prisma generate

# Compile TypeScript for analysis worker
# Only compile log-processor and analysis libraries it needs
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
    "src/workers/log-processor.ts", \
    "src/lib/analysis/**/*.ts" \
  ], \
  "exclude": [ \
    "**/*.test.ts", \
    "**/*.spec.ts", \
    "src/lib/analysis/event-detector.ts" \
  ] \
}' > tsconfig.analysis.json

# Compile TypeScript
RUN npx tsc -p tsconfig.analysis.json

# ===================================
# STAGE 3: RUNTIME
# ===================================
FROM node:20-bookworm-slim AS runner

WORKDIR /app

# Set production environment
ENV NODE_ENV=production

# Create non-root user for security
RUN groupadd -g 1001 nodejs && \
    useradd -u 1001 -g nodejs -m -s /bin/bash analysis

# Copy compiled JavaScript and dependencies
COPY --from=builder --chown=analysis:nodejs /app/dist ./dist
COPY --from=builder --chown=analysis:nodejs /app/node_modules ./node_modules
COPY --from=builder --chown=analysis:nodejs /app/prisma ./prisma
COPY --from=builder --chown=analysis:nodejs /app/package.json ./

# Switch to non-root user
USER analysis

# No port exposure needed (background service)

# Health logging - the worker itself logs heartbeats
# Worker logs: "Analysis worker alive - pending: N"

# Start the analysis worker
CMD ["node", "dist/workers/log-processor.js"]