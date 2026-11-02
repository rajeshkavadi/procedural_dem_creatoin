#!/usr/bin/env bash
# Backward-compatible entry point: bag + Cloudflare quick tunnel.
#
#   ./scripts/start_remote.sh /path/to/recording.bag [topic]
#
# Equivalent to ./scripts/start_cloudflare.sh --bag … --topic … (plus
# PCSNAP_GNSS when set).  See start_cloudflare.sh for every option.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BAG="${1:?usage: start_remote.sh /path/to/recording.bag [topic]}"
TOPIC="${2:-/livox/lidar}"

ARGS=(--bag "$BAG" --topic "$TOPIC")
[ -n "${PCSNAP_GNSS:-}" ] && ARGS+=(--gnss "$PCSNAP_GNSS")
[ -n "${PCSNAP_PORT:-}" ] && ARGS+=(--port "$PCSNAP_PORT")
[ -n "${PCSNAP_BAG_RATE:-}" ] && ARGS+=(--rate "$PCSNAP_BAG_RATE")
[ -n "${PCSNAP_API_TOKEN:-}" ] && ARGS+=(--token "$PCSNAP_API_TOKEN")

exec "$SCRIPT_DIR/start_cloudflare.sh" "${ARGS[@]}"
