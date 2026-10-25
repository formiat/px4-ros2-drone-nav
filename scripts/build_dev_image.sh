#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build \
  --file "${repo_root}/docker/Dockerfile" \
  --target dev \
  --tag drone-gazebo-dev:latest \
  "${repo_root}"
