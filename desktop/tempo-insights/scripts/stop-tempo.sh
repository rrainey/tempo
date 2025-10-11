#!/bin/bash
# ===================================
# Task 169: Stop Tempo Insights Stack
# ===================================
# Gracefully stops application and optionally Supabase services
#
# Usage:
#   ./scripts/stop-tempo.sh          # Stop app only
#   ./scripts/stop-tempo.sh --all    # Stop app + Supabase
#   ./scripts/stop-tempo.sh --clean  # Stop + remove volumes

set -e

STOP_SUPABASE=false
CLEAN_VOLUMES=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --all)
            STOP_SUPABASE=true
            shift
            ;;
        --clean)
            CLEAN_VOLUMES=true
            STOP_SUPABASE=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--all] [--clean]"
            exit 1
            ;;
    esac
done

echo "=========================================="
echo "Stopping Tempo Insights"
echo "=========================================="
echo ""

# ===================================
# Step 1: Stop Application Services
# ===================================
echo "Step 1: Stopping application services..."
echo "-------------------------------------------"

if docker compose ps | grep -q "Up"; then
    docker compose down
    echo "✓ Application services stopped"
else
    echo "⚠️  Application services not running"
fi

echo ""

# ===================================
# Step 2: Stop Supabase (if requested)
# ===================================
if [ "$STOP_SUPABASE" = true ]; then
    echo "Step 2: Stopping Supabase stack..."
    echo "-------------------------------------------"
    
    if [ -d "supabase-stack" ]; then
        cd supabase-stack
        
        if docker compose ps | grep -q "Up"; then
            docker compose down
            echo "✓ Supabase stack stopped"
        else
            echo "⚠️  Supabase stack not running"
        fi
        
        cd ..
    else
        echo "⚠️  supabase-stack directory not found"
    fi
    
    echo ""
fi

# ===================================
# Step 3: Clean Volumes (if requested)
# ===================================
if [ "$CLEAN_VOLUMES" = true ]; then
    echo "Step 3: Cleaning volumes..."
    echo "-------------------------------------------"
    echo ""
    echo "⚠️  WARNING: This will DELETE all data!"
    echo "   - Database contents"
    echo "   - Jump log files"
    echo "   - All persistent storage"
    echo ""
    read -p "Are you sure? Type 'yes' to confirm: " -r
    echo
    
    if [[ $REPLY == "yes" ]]; then
        echo "Removing volumes..."
        
        # Remove application volumes
        docker compose down -v
        
        # Remove Supabase volumes
        if [ -d "supabase-stack" ]; then
            cd supabase-stack
            docker compose down -v
            
            # Remove volume directories
            if [ -d "volumes" ]; then
                echo "Removing volume directories..."
                rm -rf volumes/db/data/*
                rm -rf volumes/storage/*
                echo "✓ Volumes cleaned"
            fi
            
            cd ..
        fi
        
        echo ""
        echo "✅ All volumes removed"
    else
        echo "Volume cleanup cancelled"
    fi
    
    echo ""
fi

# ===================================
# Summary
# ===================================
echo "=========================================="
echo "Shutdown Summary"
echo "=========================================="
echo ""

# Check what's still running
RUNNING_CONTAINERS=$(docker ps --filter "name=tempo-" --format "{{.Names}}" | wc -l)

if [ "$RUNNING_CONTAINERS" -eq 0 ]; then
    echo "✅ All Tempo services stopped"
else
    echo "⚠️  Some containers still running:"
    docker ps --filter "name=tempo-" --format "table {{.Names}}\t{{.Status}}"
fi

echo ""

if [ "$CLEAN_VOLUMES" = true ]; then
    echo "💾 Volumes cleaned - data removed"
    echo ""
    echo "To start fresh:"
    echo "  ./scripts/start-tempo.sh"
    echo "  Then run database migrations:"
    echo "  pnpm prisma migrate deploy"
    echo "  pnpm prisma db seed"
elif [ "$STOP_SUPABASE" = true ]; then
    echo "To restart everything:"
    echo "  ./scripts/start-tempo.sh"
else
    echo "Supabase still running (database intact)"
    echo ""
    echo "To restart application:"
    echo "  ./scripts/start-tempo.sh"
    echo ""
    echo "To stop Supabase too:"
    echo "  ./scripts/stop-tempo.sh --all"
fi

echo ""