#!/usr/bin/env bash
# Build the Polaris simulation image locally. Run from anywhere.
# First build: 30-60 min and several GB (dominated by ArduSub SITL and ardupilot_gazebo).
#
# The build context is a temporary directory holding only src/simulation/simulation.repos,
# so nothing from the workspace is sent to the daemon and workspace edits never bust the
# cache. Works with both the legacy builder and BuildKit/buildx.
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
TAG="${TAG:-polaris:sim}"
ctx="$(mktemp -d)"
trap 'rm -rf "$ctx"' EXIT
mkdir -p "$ctx/src/simulation"
cp "$repo/src/simulation/simulation.repos" "$ctx/src/simulation/"
docker build -f "$repo/docker/Dockerfile.sim" -t "$TAG" "$@" "$ctx"
