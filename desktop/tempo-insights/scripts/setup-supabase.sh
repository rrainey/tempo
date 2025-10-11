#!/bin/bash
# ===================================
# Tasks 140-148: Supabase Docker Setup
# ===================================
# This script sets up Supabase self-hosted stack
# Uses 'supabase-stack' directory to avoid conflict with existing 'supabase' directory

set -e

echo "================================================================="
echo "Supabase Docker Stack Setup"
echo "This builds the Docker configuration - not for development setup"
echo "================================================================="

# Task 140: Clone Supabase Docker config
echo ""
echo "Task 140: Cloning Supabase Docker configuration..."
echo "Note: Using 'supabase-stack' directory to preserve existing 'supabase' directory"

if [ -d "supabase-stack" ]; then
    echo "⚠️  supabase-stack directory already exists"
    read -p "Remove and re-clone? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf supabase-stack
    else
        echo "Skipping clone"
    fi
fi

if [ ! -d "supabase-stack" ]; then
    echo "Cloning Supabase repository..."
    git clone --depth 1 https://github.com/supabase/supabase supabase-temp
    
    echo "Copying Docker configuration..."
    mkdir -p supabase-stack
    cp -r supabase-temp/docker/* supabase-stack/
    
    echo "Cleaning up repository..."
    rm -rf supabase-temp
    
    echo "✓ Supabase Docker config ready in supabase-stack/"
else
    echo "✓ Using existing supabase-stack directory"
fi

# Task 144: Create volumes directory
echo ""
echo "Task 144: Creating volume directories..."
mkdir -p supabase-stack/volumes/{db,storage,functions,api,logs}
chmod -R 755 supabase-stack/volumes
echo "✓ Volume directories created"

# Task 141-143: Environment configuration
echo ""
echo "Task 141-143: Configuring environment..."

if [ ! -f "supabase-stack/.env" ]; then
    if [ -f "supabase-stack/.env.example" ]; then
        cp supabase-stack/.env.example supabase-stack/.env
        echo "✓ Created .env from .env.example"
    else
        echo "❌ Error: .env.example not found in supabase-stack/"
        exit 1
    fi
else
    echo "⚠️  .env already exists in supabase-stack/"
fi

echo ""
echo "=========================================="
echo "IMPORTANT: Security Configuration Required"
echo "=========================================="
echo ""
echo "You MUST change the following in supabase-stack/.env:"
echo ""
echo "1. POSTGRES_PASSWORD (currently: 'your-super-secret-and-long-postgres-password')"
echo "2. JWT_SECRET (currently: 'your-super-secret-jwt-token-with-at-least-32-characters-long')"
echo "3. ANON_KEY (generate from JWT_SECRET)"
echo "4. SERVICE_ROLE_KEY (generate from JWT_SECRET)"
echo "5. DASHBOARD_USERNAME and DASHBOARD_PASSWORD"
echo ""
echo "Generate secrets:"
echo "  - POSTGRES_PASSWORD: openssl rand -base64 32"
echo "  - JWT_SECRET: openssl rand -base64 32"
echo "  - JWT keys: https://supabase.com/docs/guides/self-hosting/docker#generate-api-keys"
echo ""
read -p "Press Enter to open .env file for editing..."
${EDITOR:-nano} supabase-stack/.env

# Verify critical values were changed
echo ""
echo "Verifying configuration..."

if grep -q "your-super-secret-and-long-postgres-password" supabase-stack/.env; then
    echo "⚠️  WARNING: POSTGRES_PASSWORD still set to default!"
fi

if grep -q "your-super-secret-jwt-token" supabase-stack/.env; then
    echo "⚠️  WARNING: JWT_SECRET still set to default!"
fi

echo ""
echo "Configuration summary:"
grep "^POSTGRES_PASSWORD=" supabase-stack/.env | sed 's/=.*/=***REDACTED***/'
grep "^JWT_SECRET=" supabase-stack/.env | sed 's/=.*/=***REDACTED***/'
grep "^DASHBOARD_USERNAME=" supabase-stack/.env
echo ""

# Task 145: Start Supabase stack
echo ""
echo "Task 145: Starting Supabase stack..."
read -p "Start Supabase services now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd supabase-stack
    docker compose up -d
    cd ..
    
    echo ""
    echo "Waiting for services to start (30 seconds)..."
    sleep 30
    
    # Task 146: Verify endpoints
    echo ""
    echo "Task 146: Verifying Supabase endpoints..."
    
    echo "Checking Kong API Gateway (port 8000)..."
    if curl -f -s http://localhost:8000/health > /dev/null 2>&1; then
        echo "✓ Kong API Gateway is responding"
    else
        echo "⚠️  Kong API Gateway not responding yet (may need more time)"
    fi
    
    echo "Checking Supabase Studio (port 8000)..."
    if curl -f -s http://localhost:8000 > /dev/null 2>&1; then
        echo "✓ Supabase Studio is accessible"
    else
        echo "⚠️  Supabase Studio not responding yet (may need more time)"
    fi
    
    # Task 147-148: Database setup
    echo ""
    echo "Task 147-148: Database setup..."
    echo ""
    echo "To set up the Tempo database schema:"
    echo "  1. Wait for all services to be healthy: cd supabase-stack && docker compose ps"
    echo "  2. Run Prisma migrations: DATABASE_URL='postgresql://postgres:PASSWORD@localhost:54322/postgres' pnpm prisma migrate deploy"
    echo "  3. Seed admin user: DATABASE_URL='postgresql://postgres:PASSWORD@localhost:54322/postgres' pnpm prisma db seed"
    echo ""
    
    echo "=========================================="
    echo "Supabase Stack Setup Complete!"
    echo "=========================================="
    echo ""
    echo "Services running:"
    cd supabase-stack
    docker compose ps
    cd ..
    echo ""
    echo "Access points:"
    echo "  - Studio Dashboard: http://localhost:8000"
    echo "  - API Gateway: http://localhost:8000/rest/v1/"
    echo "  - PostgreSQL: localhost:54322 (direct - note non-standard port!)"
    echo "  - Supavisor Pool: localhost:6543 (recommended)"
    echo ""
    echo "Directory structure:"
    echo "  - supabase/        - Your existing Supabase CLI config (PRESERVED)"
    echo "  - supabase-stack/  - Docker stack (NEW)"
    echo ""
    echo "Next steps:"
    echo "  1. Verify all services healthy: cd supabase-stack && docker compose ps"
    echo "  2. Update .env.docker with Supabase connection details"
    echo "  3. Run database migrations against port 54322"
    echo "  4. Seed initial data"
    echo ""
else
    echo "Supabase stack not started. To start manually:"
    echo "  cd supabase-stack && docker compose up -d"
fi