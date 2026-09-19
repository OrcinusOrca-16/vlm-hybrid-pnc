#!/usr/bin/env bash
set -euo pipefail

git config core.hooksPath .githooks
chmod +x .githooks/pre-commit

echo "Configured Git hooks from .githooks/"
python3 scripts/check_markdown_math.py
