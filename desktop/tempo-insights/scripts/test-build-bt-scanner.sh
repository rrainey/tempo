#!/bin/bash
# ===================================
# Task 132: Bluetooth Scanner Test Build
# ===================================

set -e

echo "=========================================="
echo "Task 132: Building Bluetooth Scanner Image"
echo "=========================================="

# Check prerequisites
echo "✓ Checking prerequisites..."

if [ ! -f "docker/bluetooth-scanner/Dockerfile" ]; then
    echo "❌ Error: docker/bluetooth-scanner/Dockerfile not found"
    exit 1
fi

if [ ! -d "smpmgr-extensions/plugins" ]; then
    echo "❌ Error: smpmgr-extensions/plugins directory not found"
    exit 1
fi

if [ ! -f "smpmgr-extensions/plugins/tempo_group.py" ]; then
    echo "❌ Error: smpmgr-extensions/plugins/tempo_group.py not found"
    exit 1
fi

if [ ! -f "src/workers/bluetooth-scanner.ts" ]; then
    echo "❌ Error: src/workers/bluetooth-scanner.ts not found"
    exit 1
fi

echo "✓ All prerequisite files exist"
echo ""

# Set platform for Raspberry Pi 5 (ARM64)
PLATFORM="linux/arm64"
IMAGE_NAME="tempo-bt-scanner"
IMAGE_TAG="test"
FULL_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

echo "Building Docker image:"
echo "  Platform: ${PLATFORM}"
echo "  Image: ${FULL_IMAGE}"
echo ""

# Build the image
echo "Starting build (this may take 5-10 minutes)..."
echo "----------------------------------------"

docker build \
    --platform "${PLATFORM}" \
    --file docker/bluetooth-scanner/Dockerfile \
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
    echo "Expected size: 350-450MB"
    echo ""
    echo "Verification checks:"
    echo "  1. Verify smpmgr: docker run --rm ${FULL_IMAGE} smpmgr --version"
    echo "  2. Verify Python: docker run --rm ${FULL_IMAGE} python3 --version"
    echo "  3. Check plugins: docker run --rm ${FULL_IMAGE} ls -la /opt/smpmgr-extensions/plugins"
    echo ""
    echo "Note: Full testing requires Bluetooth hardware access with --net=host"
else
    echo "❌ BUILD FAILED (exit code: ${BUILD_EXIT_CODE})"
    echo ""
    echo "Common issues:"
    echo "  - Python/pip installation errors"
    echo "  - smpmgr installation timeout"
    echo "  - TypeScript compilation errors"
    echo "  - Missing BlueZ dependencies"
    exit 1
fi