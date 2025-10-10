# ===================================
# STAGE 1: DEPENDENCIES
# ===================================
# Install all dependencies including devDependencies
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
# Build the Next.js application with standalone output
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

# Build Next.js application with standalone output
# This creates a minimal production build
ENV NEXT_TELEMETRY_DISABLED 1
RUN pnpm run build

# ===================================
# STAGE 3: RUNNER (Production)
# ===================================
FROM node:20-bookworm-slim AS runner

WORKDIR /app

# Set production environment
ENV NODE_ENV=production
ENV NEXT_TELEMETRY_DISABLED=1

# Create non-root user for security
RUN groupadd -g 1001 nodejs && \
    useradd -u 1001 -g nodejs -m -s /bin/bash nextjs

# Copy standalone build from builder
# Next.js standalone output includes only necessary files
COPY --from=builder --chown=nextjs:nodejs /app/.next/standalone ./
COPY --from=builder --chown=nextjs:nodejs /app/.next/static ./.next/static
COPY --from=builder --chown=nextjs:nodejs /app/public ./public

# Copy entire node_modules for Prisma client (pnpm uses symlinks)
# The standalone output doesn't include native modules like Prisma
COPY --from=builder --chown=nextjs:nodejs /app/node_modules ./node_modules
COPY --from=builder --chown=nextjs:nodejs /app/prisma ./prisma

# Switch to non-root user
USER nextjs

# Expose port 3000
EXPOSE 3000

# Set port environment variable
ENV PORT=3000
ENV HOSTNAME="0.0.0.0"

# Health check for Docker/Kubernetes
# Uses Node.js built-in http module (no package manager needed)
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
  CMD node -e "require('http').get('http://localhost:3000/api/health', (r) => {process.exit(r.statusCode === 200 ? 0 : 1)})"

# Start the Next.js server
CMD ["node", "server.js"]