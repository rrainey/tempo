#!/bin/bash
# ===================================
# Task 139: Analysis Worker Test Build
# ===================================

set -e

echo "=========================================="
echo "Task 139: Building Analysis Worker Image"
echo "=========================================="

# Check prerequisites
echo "✓ Checking prerequisites..."

if [ ! -f "docker/analysis-worker/Dockerfile" ]; then
    echo "❌ Error: docker/analysis-worker/Dockerfile not found"
    exit 1
fi

if [ ! -f "src/workers/log-processor.ts" ]; then
    echo "❌ Error: src/workers/log-processor.ts not found"
    exit 1
fi

if [ ! -d "src/lib/analysis" ]; then
    echo "❌ Error: src/lib/analysis directory not found"
    exit 1
fi

echo "✓ All prerequisite files exist"
echo ""

# Set platform for Raspberry Pi 5 (ARM64)
PLATFORM="linux/arm64"
IMAGE_NAME="tempo-analysis"
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
    --file docker/analysis-worker/Dockerfile \
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
    echo "Verification checks:"
    echo "  1. Check compiled files: docker run --rm ${FULL_IMAGE} ls -la dist/workers/"
    echo "  2. Verify Prisma: docker run --rm ${FULL_IMAGE} ls -la node_modules/.prisma/"
    echo ""
    echo "Note: Full testing requires database connection"
else
    echo "❌ BUILD FAILED (exit code: ${BUILD_EXIT_CODE})"
    echo ""
    echo "Common issues:"
    echo "  - TypeScript compilation errors"
    echo "  - Missing analysis library files"
    echo "  - Prisma schema errors"
    exit 1
fi