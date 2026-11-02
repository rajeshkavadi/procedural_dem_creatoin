#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$PROJ_DIR/.run"

[ -f "$RUN_DIR/tunnel.pid" ] && kill $(cat "$RUN_DIR/tunnel.pid") 2>/dev/null; rm -f "$RUN_DIR/tunnel.pid"
[ -f "$RUN_DIR/service.pid" ] && kill $(cat "$RUN_DIR/service.pid") 2>/dev/null; rm -f "$RUN_DIR/service.pid"

pkill -f "uvicorn pc_snapshot.api" 2>/dev/null || true
pkill -f "cloudflared tunnel" 2>/dev/null || true
echo "Stopped."
