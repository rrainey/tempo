#!/bin/bash
# ===================================
# Task 168: Start Tempo Insights Stack
# ===================================
# Starts Supabase and application services in correct order
#
# Usage:
#   ./scripts/start-tempo.sh

set -e

echo "=========================================="
echo "Starting Tempo Insights"
echo "=========================================="
echo ""

# ===================================
# Step 1: Validate Environment
# ===================================
echo "Step 1: Validating environment configuration..."
echo "-------------------------------------------"

if [ ! -f ".env.docker" ]; then
    echo "❌ Error: .env.docker not found"
    echo ""
    echo "Please create .env.docker with required configuration."
    echo "See docs/ENVIRONMENT_CONFIGURATION.md for details."
    exit 1
fi

# Run validation script if it exists
if [ -f "scripts/validate-env.sh" ]; then
    if ! ./scripts/validate-env.sh .env.docker; then
        echo ""
        echo "❌ Environment validation failed"
        echo "Fix the issues above before starting services."
        exit 1
    fi
else
    echo "⚠️  Warning: validate-env.sh not found, skipping validation"
fi

echo ""

# ===================================
# Step 2: Check Docker
# ===================================
echo "Step 2: Checking Docker..."
echo "-------------------------------------------"

if ! command -v docker &> /dev/null; then
    echo "❌ Error: Docker not found"
    echo "Please install Docker first."
    exit 1
fi

if ! docker info &> /dev/null; then
    echo "❌ Error: Docker daemon not running"
    echo "Please start Docker first."
    exit 1
fi

echo "✓ Docker is running"
echo ""

# ===================================
# Step 3: Start Supabase Stack
# ===================================
echo "Step 3: Starting Supabase stack..."
echo "-------------------------------------------"

if [ ! -d "supabase-stack" ]; then
    echo "❌ Error: supabase-stack directory not found"
    echo ""
    echo "Run setup first:"
    echo "  ./scripts/setup-supabase.sh"
    echo "Or for production:"
    echo "  sudo ./scripts/configure-instance.sh"
    exit 1
fi

cd supabase-stack

# Check if already running
if docker compose ps | grep -q "Up"; then
    echo "⚠️  Supabase services already running"
    docker compose ps
else
    echo "Starting Supabase services..."
    docker compose up -d
    
    echo ""
    echo "Waiting for Supabase to be ready (30 seconds)..."
    sleep 30
fi

echo ""
echo "Supabase status:"
docker compose ps

cd ..
echo ""

# ===================================
# Step 4: Verify Supabase Health
# ===================================
echo "Step 4: Verifying Supabase health..."
echo "-------------------------------------------"

# Check Kong gateway
if curl -f -s http://localhost:8000/health > /dev/null 2>&1; then
    echo "✓ Kong API Gateway responding"
else
    echo "⚠️  Kong not responding (may need more time)"
fi

# Check PostgreSQL
if docker compose -f supabase-stack/docker-compose.yml exec -T postgres pg_isready -U postgres > /dev/null 2>&1; then
    echo "✓ PostgreSQL ready"
else
    echo "⚠️  PostgreSQL not ready (may need more time)"
fi

echo ""

# ===================================
# Step 5: Build Images (if needed)
# ===================================
echo "Step 5: Checking application images..."
echo "-------------------------------------------"

IMAGES_EXIST=true

for image in tempo-web tempo-bt-scanner tempo-analysis; do
    if ! docker images | grep -q "$image.*latest"; then
        echo "⚠️  Image $image:latest not found"
        IMAGES_EXIST=false
    else
        echo "✓ $image:latest found"
    fi
done

echo ""

if [ "$IMAGES_EXIST" = false ]; then
    echo "Building missing images..."
    if [ -f "scripts/build-images.sh" ]; then
        ./scripts/build-images.sh
    else
        echo "❌ Error: scripts/build-images.sh not found"
        echo "Please build images manually:"
        echo "  docker compose build"
        exit 1
    fi
    echo ""
fi

# ===================================
# Step 6: Start Application Services
# ===================================
echo "Step 6: Starting application services..."
echo "-------------------------------------------"

# Check if already running
if docker compose ps | grep -q "Up"; then
    echo "⚠️  Application services already running"
    docker compose ps
    echo ""
    read -p "Restart services? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Restarting services..."
        docker compose down
        docker compose up -d
    fi
else
    echo "Starting application services..."
    docker compose up -d
fi

echo ""
echo "Waiting for services to start (10 seconds)..."
sleep 10
echo ""

# ===================================
# Step 7: Verify Services
# ===================================
echo "Step 7: Verifying application services..."
echo "-------------------------------------------"

# Check web server health
if curl -f -s http://localhost:3000/api/health > /dev/null 2>&1; then
    echo "✓ Web server responding"
else
    echo "⚠️  Web server not responding yet"
fi

# Check all services status
echo ""
echo "Application services:"
docker compose ps

echo ""

# ===================================
# Summary
# ===================================
echo "=========================================="
echo "Tempo Insights Started"
echo "=========================================="
echo ""
echo "Services:"
echo "  - Supabase Studio: http://localhost:8000"
echo "  - Tempo Insights:   http://localhost:3000"
echo "  - API Gateway:      http://localhost:8000/rest/v1/"
echo ""
echo "View logs:"
echo "  docker compose logs -f tempo-web"
echo "  docker compose logs -f tempo-bt-scanner"
echo "  docker compose logs -f tempo-analysis"
echo ""
echo "Stop services:"
echo "  ./scripts/stop-tempo.sh"
echo ""
echo "✅ Startup complete!"