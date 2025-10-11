#!/bin/bash
# ===================================
# Task 171: Quick Test Deployment
# ===================================
# Builds images, starts stack, and runs health checks
# Useful for CI/CD or quick validation
#
# Usage:
#   ./scripts/test-deploy.sh

set -e

echo "=========================================="
echo "Tempo Insights - Test Deployment"
echo "=========================================="
echo ""

START_TIME=$(date +%s)
ERRORS=0

# ===================================
# Step 1: Environment Check
# ===================================
echo "Step 1: Checking environment..."
echo "-------------------------------------------"

if [ ! -f ".env.docker" ]; then
    echo "❌ .env.docker not found"
    ERRORS=$((ERRORS + 1))
else
    echo "✓ .env.docker exists"
    
    # Validate if script exists
    if [ -f "scripts/validate-env.sh" ]; then
        if ./scripts/validate-env.sh .env.docker > /dev/null 2>&1; then
            echo "✓ Environment validation passed"
        else
            echo "⚠️  Environment validation warnings"
        fi
    fi
fi

echo ""

# ===================================
# Step 2: Build Images
# ===================================
echo "Step 2: Building Docker images..."
echo "-------------------------------------------"

if [ -f "scripts/build-images.sh" ]; then
    if ./scripts/build-images.sh latest; then
        echo "✓ Images built successfully"
    else
        echo "❌ Image build failed"
        ERRORS=$((ERRORS + 1))
    fi
else
    echo "⚠️  build-images.sh not found, trying docker compose build..."
    if docker compose build; then
        echo "✓ Images built via docker compose"
    else
        echo "❌ Build failed"
        ERRORS=$((ERRORS + 1))
    fi
fi

echo ""

# ===================================
# Step 3: Start Services
# ===================================
echo "Step 3: Starting services..."
echo "-------------------------------------------"

if [ -f "scripts/start-tempo.sh" ]; then
    if ./scripts/start-tempo.sh; then
        echo "✓ Services started"
    else
        echo "❌ Service startup failed"
        ERRORS=$((ERRORS + 1))
    fi
else
    echo "⚠️  start-tempo.sh not found, trying manual start..."
    
    # Start Supabase
    if [ -d "supabase-stack" ]; then
        cd supabase-stack
        docker compose up -d
        cd ..
        echo "✓ Supabase started"
    fi
    
    # Start application
    docker compose up -d
    echo "✓ Application started"
    
    echo "Waiting for services (30 seconds)..."
    sleep 30
fi

echo ""

# ===================================
# Step 4: Health Checks
# ===================================
echo "Step 4: Running health checks..."
echo "-------------------------------------------"

# Check web server
echo -n "Web server (port 3000)... "
if curl -f -s http://localhost:3000/api/health > /dev/null 2>&1; then
    echo "✓ OK"
else
    echo "❌ FAIL"
    ERRORS=$((ERRORS + 1))
fi

# Check Supabase Kong
echo -n "Supabase API (port 8000)... "
if curl -f -s http://localhost:8000/health > /dev/null 2>&1; then
    echo "✓ OK"
else
    echo "⚠️  Not responding"
fi

# Check PostgreSQL
echo -n "PostgreSQL... "
if docker compose -f supabase-stack/docker-compose.yml exec -T postgres pg_isready -U postgres > /dev/null 2>&1; then
    echo "✓ OK"
else
    echo "❌ FAIL"
    ERRORS=$((ERRORS + 1))
fi

# Check Docker container status
echo ""
echo "Container status:"
docker compose ps

echo ""

# ===================================
# Step 5: Service Logs Check
# ===================================
echo "Step 5: Checking service logs for errors..."
echo "-------------------------------------------"

# Check for errors in logs (last 50 lines)
echo "Web server logs:"
if docker compose logs --tail=50 tempo-web 2>&1 | grep -i "error\|fatal\|exception" | head -5; then
    echo "⚠️  Errors found in web server logs (see above)"
else
    echo "✓ No critical errors in web server logs"
fi

echo ""
echo "Bluetooth scanner logs:"
if docker compose logs --tail=50 tempo-bt-scanner 2>&1 | grep -i "error\|fatal\|exception" | grep -v "No admin user found" | head -5; then
    echo "⚠️  Errors found in scanner logs (see above)"
else
    echo "✓ No critical errors in scanner logs"
fi

echo ""
echo "Analysis worker logs:"
if docker compose logs --tail=50 tempo-analysis 2>&1 | grep -i "error\|fatal\|exception" | head -5; then
    echo "⚠️  Errors found in analysis logs (see above)"
else
    echo "✓ No critical errors in analysis logs"
fi

echo ""

# ===================================
# Step 6: Database Check
# ===================================
echo "Step 6: Checking database schema..."
echo "-------------------------------------------"

# Check if key tables exist
echo -n "Checking for User table... "
if docker compose -f supabase-stack/docker-compose.yml exec -T postgres \
    psql -U postgres -d postgres -tAc "SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'User');" 2>/dev/null | grep -q "t"; then
    echo "✓ OK"
else
    echo "❌ FAIL - Run migrations: pnpm prisma migrate deploy"
    ERRORS=$((ERRORS + 1))
fi

echo -n "Checking for Device table... "
if docker compose -f supabase-stack/docker-compose.yml exec -T postgres \
    psql -U postgres -d postgres -tAc "SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'Device');" 2>/dev/null | grep -q "t"; then
    echo "✓ OK"
else
    echo "❌ FAIL"
    ERRORS=$((ERRORS + 1))
fi

echo -n "Checking for JumpLog table... "
if docker compose -f supabase-stack/docker-compose.yml exec -T postgres \
    psql -U postgres -d postgres -tAc "SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'JumpLog');" 2>/dev/null | grep -q "t"; then
    echo "✓ OK"
else
    echo "❌ FAIL"
    ERRORS=$((ERRORS + 1))
fi

echo ""

# ===================================
# Summary
# ===================================
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo "=========================================="
echo "Test Deployment Summary"
echo "=========================================="
echo ""
echo "Duration: ${DURATION}s"
echo ""

if [ $ERRORS -eq 0 ]; then
    echo "✅ ALL CHECKS PASSED"
    echo ""
    echo "Deployment is healthy!"
    echo ""
    echo "Access points:"
    echo "  - Web App: http://localhost:3000"
    echo "  - Supabase: http://localhost:8000"
    echo ""
    echo "View logs:"
    echo "  docker compose logs -f tempo-web"
    echo ""
    echo "Stop services:"
    echo "  ./scripts/stop-tempo.sh"
    echo ""
    exit 0
else
    echo "❌ DEPLOYMENT FAILED - $ERRORS error(s) found"
    echo ""
    echo "Common issues:"
    echo "1. Database not migrated:"
    echo "   pnpm prisma migrate deploy"
    echo ""
    echo "2. Services not healthy:"
    echo "   docker compose ps"
    echo "   docker compose logs [service-name]"
    echo ""
    echo "3. Ports already in use:"
    echo "   sudo lsof -i :3000"
    echo "   sudo lsof -i :8000"
    echo ""
    exit 1
fi