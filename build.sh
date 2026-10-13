#!/bin/bash
# Build LabCommand.app (Mac-native command center).
# Usage: ./build.sh [--no-install]
#   --no-install  Build only; skip installing to ~/Applications
set -euo pipefail

NO_INSTALL=false
for arg in "$@"; do
  case "$arg" in
    --no-install) NO_INSTALL=true ;;
  esac
done

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

# Copy dual-sensor PCD fixtures for 3D canvas overlay
echo "==> Copying PCD fixtures to app bundle"
mkdir -p "${APP}/Contents/Resources/fixtures/pointclouds"
for pcd in mid360-frame.pcd jt128-frame.pcd; do
  if [[ -f "${ROOT}/fixtures/pointclouds/${pcd}" ]]; then
    cp -f "${ROOT}/fixtures/pointclouds/${pcd}" "${APP}/Contents/Resources/fixtures/pointclouds/"
  fi
done

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
