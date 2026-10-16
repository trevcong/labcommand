#!/bin/bash
# Build LabCommand.app (Mac-native command center).
# Usage: ./build.sh [--no-install]
set -euo pipefail

NO_INSTALL=false
if [[ "${1:-}" == "--no-install" ]]; then
  NO_INSTALL=true
fi

ROOT="$(cd "$(dirname "$0")" && pwd)"
APP_NAME="LabCommand"
BUILD_DIR="${ROOT}/build"
APP="${BUILD_DIR}/${APP_NAME}.app"
INSTALL_DIR="${HOME}/Applications"
INSTALL_APP="${INSTALL_DIR}/${APP_NAME}.app"

mkdir -p "${BUILD_DIR}"
rm -rf "${APP}"

cd "${ROOT}"
echo "==> swift build (release) LabCommand"
swift build -c release --product LabCommand

BIN="$(swift build -c release --product LabCommand --show-bin-path)/${APP_NAME}"
if [[ ! -x "$BIN" ]]; then
  echo "Binary not found at $BIN" >&2
  exit 1
fi

mkdir -p "${APP}/Contents/MacOS" "${APP}/Contents/Resources"
cp -f "${BIN}" "${APP}/Contents/MacOS/${APP_NAME}"
cp -f "${ROOT}/Info.plist" "${APP}/Contents/Info.plist"
cp -f "${ROOT}/scripts/spark-capture.sh" "${APP}/Contents/Resources/spark-capture.sh"
chmod +x "${APP}/Contents/Resources/spark-capture.sh"

# Copy API fixtures for mock mode
if [[ -d "${ROOT}/fixtures/api" ]]; then
  echo "==> Copying API fixtures to app bundle"
  mkdir -p "${APP}/Contents/Resources/fixtures"
  cp -R "${ROOT}/fixtures/api" "${APP}/Contents/Resources/fixtures/api"
fi

# Hand goldens for the offline Room fixture. The gate stays --no-install.
HAND="${ROOT}/Tests/Fixtures/room/golden/hand"
if [[ -d "${HAND}" ]]; then
  echo "==> Copying room-memory hand goldens to app bundle"
  mkdir -p "${APP}/Contents/Resources/fixtures/room/golden/hand"
  cp -f "${HAND}/"*.hex "${APP}/Contents/Resources/fixtures/room/golden/hand/"
fi

# Copy sample PCD for 3D canvas (just the small one for testing)
if [[ -f "${ROOT}/fixtures/pointclouds/mid360-frame.pcd" ]]; then
  echo "==> Copying sample PCD to app bundle"
  mkdir -p "${APP}/Contents/Resources/fixtures/pointclouds"
  cp -f "${ROOT}/fixtures/pointclouds/mid360-frame.pcd" "${APP}/Contents/Resources/fixtures/pointclouds/"
fi

codesign -s - --force --deep "${APP}" 2>/dev/null || true

echo "Built  ${APP}"

if [[ "$NO_INSTALL" == "false" ]]; then
  mkdir -p "${INSTALL_DIR}"
  rm -rf "${INSTALL_APP}"
  cp -R "${APP}" "${INSTALL_APP}"
  touch "${INSTALL_APP}"
  /System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister -f "${INSTALL_APP}" 2>/dev/null || true
  echo "Installed ${INSTALL_APP}"
  echo "Open with: open \"${INSTALL_APP}\""
else
  echo "Skipped install (--no-install)"
  echo "Run with: open \"${APP}\""
fi
