/* LiDAR Live Viewer frontend.
 *
 * Two things only:
 *   Live  — one Start button opens the MJPEG point-cloud stream and the
 *           camera-angle selector switches between the five views.
 *   DEM   — the accumulated elevation model, re-fetched automatically every
 *           10 seconds while the tab is open.
 *
 * There is no API token field: the service runs without auth by default.
 */
"use strict";

const POLL_MS = 2500;        // health polling while the live view runs
const DEM_REFRESH_MS = 10000; // DEM auto-refresh interval

const $ = (id) => document.getElementById(id);

let liveOn = false;
let healthTimer = null;
let demTimer = null;
let demBusy = false;

function setConn(ok, text) {
  const dot = $("conn-dot");
  dot.className = "dot " + (ok === null ? "dot-unknown" : ok ? "dot-ok" : "dot-bad");
  $("conn-text").textContent = text;
}

async function getJSON(path, options) {
  const resp = await fetch(path, {
    ...(options || {}),
    signal: AbortSignal.timeout(15000),
  });
  let body = null;
  try { body = await resp.json(); } catch (_) { /* non-JSON error */ }
  if (!resp.ok) {
    throw new Error((body && body.detail ? body.detail : "") || ("HTTP " + resp.status));
  }
  return body;
}

// ---------------------------------------------------------------------------
// Live point cloud
// ---------------------------------------------------------------------------
function liveShowError(msg) {
  const el = $("live-error");
  el.hidden = !msg;
  el.textContent = msg || "";
}

function liveStatus(text) {
  $("live-status").textContent = text;
}

function startLive() {
  const view = $("view-select").value;
  const img = $("live-frame");
  liveShowError("");
  img.onerror = () => {
    if (!liveOn) return;
    liveShowError("Live stream failed to load — is the service still running?");
    liveStatus("stream error");
  };
  img.src = `/api/stream?view=${encodeURIComponent(view)}&t=${Date.now()}`;
  $("live-figure").hidden = false;
  $("live-idle").hidden = true;
  liveOn = true;
  $("btn-start").textContent = "Stop";
  $("btn-start").classList.remove("primary");
  liveStatus("connecting…");
  if (healthTimer) clearInterval(healthTimer);
  healthTimer = setInterval(updateHealth, POLL_MS);
  updateHealth();
}

function stopLive() {
  liveOn = false;
  const img = $("live-frame");
  img.onerror = null;
  img.src = ""; // closing the src aborts the multipart connection server-side
  $("live-figure").hidden = true;
  $("live-idle").hidden = false;
  $("btn-start").textContent = "Start";
  $("btn-start").classList.add("primary");
  liveStatus("stopped");
  if (healthTimer) {
    clearInterval(healthTimer);
    healthTimer = null;
  }
  liveShowError("");
}

// ---------------------------------------------------------------------------
// Health (connection dot + per-frame stats)
// ---------------------------------------------------------------------------
function fmtBagProgress(src) {
  return (src && src.kind === "bag" && src.progress != null)
    ? ` · bag ${Math.round(src.progress * 100)}%` : "";
}

async function updateHealth() {
  try {
    const h = await getJSON("/api/health");
    const pc = h.point_cloud || {};
    const src = h.source || {};
    $("topic").textContent = pc.topic || "—";

    if (src.kind && src.kind !== "none" && !src.alive) {
      const why = src.start_error || src.last_error;
      setConn(false, (src.kind === "bag" ? "Bag replay" : "Sensor") + " down" +
        (why ? " (" + why + ")" : ""));
      return;
    }
    if (!pc.message_received) {
      setConn(null, "waiting for point cloud");
      return;
    }
    setConn(true, `${pc.latest_point_count.toLocaleString()} pts/scan · frame ${pc.latest_frame_id || "?"}`);

    if (!liveOn) return;
    const s = h.stream || {};
    if (s.last_error) {
      liveShowError("Server: " + s.last_error);
      liveStatus("render error");
      return;
    }
    const age = s.last_frame_age_sec;
    if (s.clients === 0 || !s.render_loop_alive) {
      liveStatus("waiting for server…");
    } else if (age != null && age > 10) {
      liveStatus(`no new frames (${age.toFixed(0)}s old)`);
    } else {
      const fps = s.last_fps != null ? s.last_fps : 0;
      const pts = s.last_points != null ? s.last_points : 0;
      liveStatus(
        `${s.active_views ? Object.keys(s.active_views).join(", ") : "—"} · ` +
        `${pts.toLocaleString()} pts/frame · ${fps.toFixed(2)} fps` +
        fmtBagProgress(src));
    }
    liveShowError("");
  } catch (err) {
    setConn(false, "API unreachable: " + err.message);
  }
}

// ---------------------------------------------------------------------------
// DEM (auto-refreshed)
// ---------------------------------------------------------------------------
function demShowError(msg) {
  const el = $("dem-error");
  el.hidden = !msg;
  el.textContent = msg || "";
}

function fmtClock(unixSec) {
  return unixSec ? new Date(unixSec * 1000).toLocaleTimeString() : "—";
}

function fmtDuration(seconds) {
  const s = Math.max(0, Math.round(seconds));
  if (s < 60) return `${s}s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${String(s % 60).padStart(2, "0")}s`;
  return `${Math.floor(s / 3600)}h ${String(Math.floor((s % 3600) / 60)).padStart(2, "0")}m`;
}

function fmtWindow(win) {
  const a = win && win.start_walltime, b = win && win.end_walltime;
  if (!a || !b) return "—";
  return `${fmtClock(a)} → ${fmtClock(b)} (+${fmtDuration(b - a)})`;
}

async function fetchDem() {
  if (demBusy) return;
  demBusy = true;
  try {
    const data = await getJSON("/api/dem", { method: "POST" });
    displayDem(data);
    demShowError("");
  } catch (err) {
    const msg = err.message || "";
    // "nothing accumulated yet" is a normal state, not a failure to shout about.
    if (/no point cloud|no valid cells|has not|empty/i.test(msg)) {
      $("dem-status").textContent = "waiting for scans…";
      $("dem-idle").hidden = false;
      demShowError("");
    } else {
      $("dem-status").textContent = "error";
      demShowError("DEM update failed: " + msg);
    }
  } finally {
    demBusy = false;
  }
}

function displayDem(data) {
  const g = data.grid || {};
  $("img-dem").src = data.image + "?t=" + Date.now(); // cache-buster
  $("dem-figure").hidden = false;
  $("dem-idle").hidden = true;
  $("dem-meta").hidden = false;

  $("dm-scans").textContent = `${(data.scans_accumulated || 0).toLocaleString()} scans`;
  $("dm-points").textContent =
    `${(data.points_used || 0).toLocaleString()} / ${(data.points_in || 0).toLocaleString()}`;
  $("dm-coverage").textContent = g.valid_cells != null
    ? `${g.valid_cells.toLocaleString()} cells · ${g.coverage_m2} m²` : "—";
  $("dm-z").textContent = g.z_min != null && g.z_max != null
    ? `${g.z_min.toFixed(2)} … ${g.z_max.toFixed(2)} m` : "—";
  const shape = g.shape || [];
  $("dm-grid").textContent = shape.length === 2
    ? `${shape[1]} × ${shape[0]} @ ${g.resolution_m} m` : "—";
  $("dm-window").textContent = fmtWindow(data.window);
  $("dm-updated").textContent = new Date().toLocaleTimeString();

  $("dem-status").textContent = g.valid_cells != null && g.z_min != null
    ? `${g.valid_cells.toLocaleString()} cells · z ${g.z_min.toFixed(2)}…${g.z_max.toFixed(2)} m`
    : `${(data.scans_accumulated || 0).toLocaleString()} scans`;
}

function startDemPolling() {
  if (demTimer) return;
  fetchDem();
  demTimer = setInterval(fetchDem, DEM_REFRESH_MS);
}

function stopDemPolling() {
  if (demTimer) {
    clearInterval(demTimer);
    demTimer = null;
  }
}

// ---------------------------------------------------------------------------
// Tabs
// ---------------------------------------------------------------------------
function showTab(name) {
  for (const tab of document.querySelectorAll(".tab")) {
    tab.classList.toggle("active", tab.dataset.tab === name);
  }
  $("tab-live").hidden = name !== "live";
  $("tab-dem").hidden = name !== "dem";
  if (name === "dem") startDemPolling();
  else stopDemPolling();
}

function init() {
  $("btn-start").addEventListener("click", () => {
    if (liveOn) stopLive(); else startLive();
  });
  $("view-select").addEventListener("change", () => {
    if (liveOn) { stopLive(); startLive(); } // switch camera angle
  });
  for (const tab of document.querySelectorAll(".tab")) {
    tab.addEventListener("click", () => showTab(tab.dataset.tab));
  }
  updateHealth();
}

document.addEventListener("DOMContentLoaded", init);
