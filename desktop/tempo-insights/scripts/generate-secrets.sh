#!/bin/bash
# ===================================
# Task 164: Secret Generation
# ===================================
# Generates cryptographically secure secrets for Tempo Insights
# 
# Usage:
#   ./scripts/generate-secrets.sh

echo "=========================================="
echo "Tempo Insights Secret Generator"
echo "=========================================="
echo ""
echo "This script generates secure random secrets."
echo "Copy and paste these into your .env files."
echo ""
echo "⚠️  IMPORTANT: Save these securely!"
echo "   Once you navigate away, these values are gone."
echo ""

# Function to generate secure random string
generate_secret() {
    local length=$1
    openssl rand -base64 $((length * 3 / 4)) | tr -d "=+/" | cut -c1-$length
}

# Generate secrets
JWT_SECRET=$(generate_secret 64)
WORKER_TOKEN=$(generate_secret 64)
POSTGRES_PASSWORD=$(generate_secret 32)
DASHBOARD_PASSWORD=$(generate_secret 24)

echo "=========================================="
echo "Generated Secrets"
echo "=========================================="
echo ""

echo "For .env.docker (Application):"
echo "-----------------------------------"
echo "JWT_SECRET=$JWT_SECRET"
echo "WORKER_TOKEN=$WORKER_TOKEN"
echo ""

echo "For supabase-stack/.env (Database):"
echo "-----------------------------------"
echo "POSTGRES_PASSWORD=$POSTGRES_PASSWORD"
echo "JWT_SECRET=$JWT_SECRET"
echo "DASHBOARD_PASSWORD=$DASHBOARD_PASSWORD"
echo ""

echo "=========================================="
echo "Next Steps"
echo "=========================================="
echo ""
echo "1. Generate Supabase JWT Keys:"
echo "   Visit: https://supabase.com/docs/guides/self-hosting/docker#generate-api-keys"
echo "   Use JWT_SECRET above to generate:"
echo "   - ANON_KEY"
echo "   - SERVICE_ROLE_KEY"
echo ""
echo "2. Update your environment files:"
echo "   - Copy secrets to .env.docker"
echo "   - Copy secrets to supabase-stack/.env"
echo "   - Add generated JWT keys to both files"
echo ""
echo "3. Validate configuration:"
echo "   ./scripts/validate-env.sh .env.docker"
echo ""
echo "=========================================="
echo ""
echo "⚠️  Save these secrets now!"
echo "   Consider storing in a password manager"
echo ""