#!/usr/bin/env bash
set -e

# Resolve Windows user profile directory dynamically
WIN_HOME=$(wslpath "$(cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')")
EXTENSIONS_DIR="$WIN_HOME/.foxglove-studio/extensions"

if [ ! -d "$EXTENSIONS_DIR" ]; then
  echo "Foxglove extensions directory not found: $EXTENSIONS_DIR"
  exit 1
fi

# Build production bundle
npm run foxglove:prepublish

# Derive extension folder name from package.json
PKG_NAME=$(node -e "const p=require('./package.json'); console.log(p.name)")
PKG_VERSION=$(node -e "const p=require('./package.json'); console.log(p.version)")
PKG_PUBLISHER=$(node -e "const p=require('./package.json'); console.log(p.publisher.toLowerCase().replace(/[^a-z0-9]/g, ''))")
DEST="$EXTENSIONS_DIR/$PKG_PUBLISHER.$PKG_NAME-$PKG_VERSION"

echo "Installing to $DEST"
mkdir -p "$DEST"
cp -r dist package.json README.md CHANGELOG.md "$DEST/"
echo "Done. Restart Foxglove Studio on Windows to pick up the changes."
