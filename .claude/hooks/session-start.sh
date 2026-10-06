#!/bin/bash
# Dựng lại môi trường mô phỏng UAV cho phiên Claude Code trên cloud.
set -euo pipefail
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi
"$CLAUDE_PROJECT_DIR/scripts/uav/setup.sh"
