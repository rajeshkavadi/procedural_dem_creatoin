#!/usr/bin/env bash
# Stop the snapshot service and the Cloudflare tunnel started by any of the
# start_* scripts.  Safe to run when nothing is running.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$PROJ_DIR/.run"

stop_pid_file() {
  local file="$1"
  if [ -f "$file" ]; then
    local pid
    pid="$(cat "$file" 2>/dev/null || true)"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      for _ in 1 2 3 4 5 6 7 8 9 10; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
      done
      kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$file"
  fi
}

stop_pid_file "$RUN_DIR/tunnel.pid"
stop_pid_file "$RUN_DIR/service.pid"

# Catch orphans whose pid file was lost (e.g. a previous shell was killed).
# Skip this script and its parent so a caller whose command line happens to
# contain the pattern (grep, editor, test harness) is never killed.
kill_matching() {
  local pattern="$1" pid
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill "$pid" 2>/dev/null || true
  done
}

kill_matching "python.*-m pc_snapshot"
kill_matching "uvicorn pc_snapshot.api"
kill_matching "cloudflared tunnel"
rm -f "$RUN_DIR/session.env"

echo "Stopped."
