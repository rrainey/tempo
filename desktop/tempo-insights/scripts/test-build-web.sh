#!/bin/bash
# ===================================
# Task 122: Next.js Container Test Build
# ===================================
# Test script for building the Next.js Docker image

set -e  # Exit on error

echo "=========================================="
echo "Task 122: Building Next.js Docker Image"
echo "=========================================="

# Check prerequisites
echo "✓ Checking prerequisites..."

if [ ! -f "docker/web/Dockerfile" ]; then
    echo "❌ Error: docker/web/Dockerfile not found"
    echo "   Please create the Dockerfile first"
    exit 1
fi

if [ ! -f "next.config.ts" ]; then
    echo "❌ Error: next.config.ts not found"
    exit 1
fi

if [ ! -f "package.json" ]; then
    echo "❌ Error: package.json not found"
    exit 1
fi

echo "✓ All prerequisite files exist"
echo ""

# Set platform for Raspberry Pi 5 (ARM64)
PLATFORM="linux/arm64"
IMAGE_NAME="tempo-web"
IMAGE_TAG="test"
FULL_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

echo "Building Docker image:"
echo "  Platform: ${PLATFORM}"
echo "  Image: ${FULL_IMAGE}"
echo ""

# Build the image
echo "Starting build..."
echo "----------------------------------------"

docker build \
    --platform "${PLATFORM}" \
    --file docker/web/Dockerfile \
    --tag "${FULL_IMAGE}" \
    --progress=plain \
    .

BUILD_EXIT_CODE=$?

echo "----------------------------------------"
echo ""

if [ $BUILD_EXIT_CODE -eq 0 ]; then
    echo "✅ BUILD SUCCESSFUL!"
    echo ""
    echo "Image details:"
    docker images "${IMAGE_NAME}" --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}"
    echo ""
    echo "Expected size: 220-300MB"
    echo ""
    echo "Next steps:"
    echo "  1. Test run: docker run --rm -p 3000:3000 ${FULL_IMAGE}"
    echo "  2. Check logs: docker logs <container-id>"
    echo "  3. Test health: curl http://localhost:3000/api/health"
else
    echo "❌ BUILD FAILED (exit code: ${BUILD_EXIT_CODE})"
    echo ""
    echo "Common issues:"
    echo "  - Missing dependencies in package.json"
    echo "  - Prisma schema errors"
    echo "  - TypeScript compilation errors"
    echo "  - Missing next.config.ts standalone output"
    exit 1
fi