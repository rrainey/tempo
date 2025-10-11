#!/bin/bash
# ===================================
# Task 163: Environment Validation
# ===================================
# Validates that all required environment variables are set
# 
# Usage:
#   ./scripts/validate-env.sh [env-file]
#   ./scripts/validate-env.sh .env.docker

set -e

ENV_FILE="${1:-.env.docker}"

if [ ! -f "$ENV_FILE" ]; then
    echo "❌ Error: Environment file not found: $ENV_FILE"
    exit 1
fi

echo "=========================================="
echo "Environment Validation: $ENV_FILE"
echo "=========================================="
echo ""

# Required variables for all environments
REQUIRED_VARS=(
    "JWT_SECRET"
    "DATABASE_URL"
    "NEXT_PUBLIC_SUPABASE_URL"
    "SUPABASE_SERVICE_KEY"
    "WORKER_TOKEN"
    "DISCOVERY_WINDOW"
    "SMPMGR_PLUGIN_PATH"
    "JUMP_LOG_STORAGE_BUCKET"
    "JUMP_LOG_MAX_SIZE_MB"
)

# Optional but recommended
RECOMMENDED_VARS=(
    "ANON_KEY"
    "SERVICE_ROLE_KEY"
    "NEXTAUTH_URL"
    "NODE_ENV"
)

MISSING_REQUIRED=()
MISSING_RECOMMENDED=()
WEAK_SECRETS=()

# Function to check if variable is set and not empty
check_var() {
    local var_name=$1
    local var_value=$(grep "^${var_name}=" "$ENV_FILE" | cut -d'=' -f2- | tr -d '"' | tr -d "'")
    
    if [ -z "$var_value" ]; then
        return 1
    fi
    
    echo "$var_value"
    return 0
}

# Check required variables
echo "Checking required variables..."
for var in "${REQUIRED_VARS[@]}"; do
    if value=$(check_var "$var"); then
        echo "  ✓ $var is set"
        
        # Check for weak secrets
        case $var in
            JWT_SECRET|WORKER_TOKEN)
                if [ ${#value} -lt 32 ]; then
                    WEAK_SECRETS+=("$var (length: ${#value}, recommended: 32+)")
                fi
                
                # Check for default/example values
                if [[ "$value" == *"your-secret"* ]] || \
                   [[ "$value" == *"change-this"* ]] || \
                   [[ "$value" == *"example"* ]]; then
                    WEAK_SECRETS+=("$var (appears to be a default/example value)")
                fi
                ;;
        esac
    else
        MISSING_REQUIRED+=("$var")
        echo "  ❌ $var is NOT set"
    fi
done

echo ""

# Check recommended variables
echo "Checking recommended variables..."
for var in "${RECOMMENDED_VARS[@]}"; do
    if value=$(check_var "$var"); then
        echo "  ✓ $var is set"
    else
        MISSING_RECOMMENDED+=("$var")
        echo "  ⚠️  $var is NOT set (recommended)"
    fi
done

echo ""

# Validate DATABASE_URL format
if db_url=$(check_var "DATABASE_URL"); then
    if [[ ! "$db_url" =~ ^postgresql:// ]]; then
        echo "⚠️  DATABASE_URL doesn't start with 'postgresql://'"
    fi
    
    if [[ "$db_url" == *"localhost"* ]]; then
        echo "⚠️  DATABASE_URL uses 'localhost' - should use container name 'db' for Docker"
    fi
fi

# Validate NEXT_PUBLIC_SUPABASE_URL format
if supabase_url=$(check_var "NEXT_PUBLIC_SUPABASE_URL"); then
    if [[ ! "$supabase_url" =~ ^http ]]; then
        echo "⚠️  NEXT_PUBLIC_SUPABASE_URL should start with http:// or https://"
    fi
fi

# Validate DISCOVERY_WINDOW is numeric
if window=$(check_var "DISCOVERY_WINDOW"); then
    if ! [[ "$window" =~ ^[0-9]+$ ]]; then
        echo "⚠️  DISCOVERY_WINDOW should be a number (seconds)"
    fi
fi

echo ""
echo "=========================================="
echo "Validation Summary"
echo "=========================================="
echo ""

# Report results
if [ ${#MISSING_REQUIRED[@]} -eq 0 ]; then
    echo "✅ All required variables are set"
else
    echo "❌ Missing required variables:"
    for var in "${MISSING_REQUIRED[@]}"; do
        echo "   - $var"
    done
fi

echo ""

if [ ${#MISSING_RECOMMENDED[@]} -eq 0 ]; then
    echo "✅ All recommended variables are set"
else
    echo "⚠️  Missing recommended variables:"
    for var in "${MISSING_RECOMMENDED[@]}"; do
        echo "   - $var"
    done
fi

echo ""

if [ ${#WEAK_SECRETS[@]} -eq 0 ]; then
    echo "✅ No weak secrets detected"
else
    echo "⚠️  Potential security issues:"
    for issue in "${WEAK_SECRETS[@]}"; do
        echo "   - $issue"
    done
fi

echo ""

# Exit code based on results
if [ ${#MISSING_REQUIRED[@]} -gt 0 ]; then
    echo "❌ Validation FAILED - required variables missing"
    exit 1
elif [ ${#WEAK_SECRETS[@]} -gt 0 ]; then
    echo "⚠️  Validation PASSED with warnings - review security issues"
    exit 0
else
    echo "✅ Validation PASSED - environment is properly configured"
    exit 0
fi