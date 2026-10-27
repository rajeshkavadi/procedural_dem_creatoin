/* Remote LiDAR Snapshot Viewer frontend.
 *
 * One-shot behavior by default: requests are only sent when the user clicks
 * "Fetch Point Cloud" (five rendered views) or "Get DEM" (the elevation
 * model accumulated since playback started). An opt-in Live View streams
 * rendered frames via multipart/x-mixed-replace (the <img> tag receives
 * frames natively).
 */
"use strict";

const VIEWS = ["top", "front", "back", "left", "right"];
const LS_BASE = "pcsnap_api_base";
const LS_TOKEN = "pcsnap_api_token";
const REQUEST_TIMEOUT_MS = 240000; // rendering can take a while on big clouds
const LIVE_STATUS_INTERVAL_MS = 2500;

const $ = (id) => document.getElementById(id);

let busy = false;
let demBusy = false;
let lastZipUrl = null;
let lastDemImageUrl = null;
let liveOn = false;
let liveStatusTimer = null;

function apiBase() {
  let base = ($("api-base").value || "").trim().replace(/\/+$/, "");
  if (!base) base = window.location.origin;
  return base;
}

function authHeaders() {
  const token = ($("api-token").value || "").trim();
  return token ? { "X-API-Token": token } : {};
}

function saveSettings() {
  localStorage.setItem(LS_BASE, $("api-base").value.trim());
  localStorage.setItem(LS_TOKEN, $("api-token").value.trim());
}

function loadSettings() {
  const base = localStorage.getItem(LS_BASE);
  const token = localStorage.getItem(LS_TOKEN);
  if (base) $("api-base").value = base;
  if (token) $("api-token").value = token;
}

function setBusy(state) {
  busy = state;
  $("btn-fetch").disabled = state;
  $("spinner").hidden = !state;
  $("btn-fetch").textContent = state ? "Capturing…" : "Fetch Point Cloud";
}

function showError(msg) {
  const el = $("error");
  el.hidden = !msg;
  el.textContent = msg || "";
}

function setConn(ok, text) {
  const dot = $("conn-dot");
  dot.className = "dot " + (ok === null ? "dot-unknown" : ok ? "dot-ok" : "dot-bad");
  $("conn-text").textContent = text;
}

function fmtTime(unixSec) {
  if (!unixSec && unixSec !== 0) return "—";
  const d = new Date(unixSec * 1000);
  return d.toLocaleString() + " (" + unixSec.toFixed(3) + ")";
}

async function checkHealth() {
  setConn(null, "Checking API…");
  try {
    const resp = await fetch(apiBase() + "/api/health", {
      headers: authHeaders(),
      signal: AbortSignal.timeout(10000),
    });
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    const h = await resp.json();
    const rosOk = h.ros && h.ros.connected;
    const received = h.point_cloud && h.point_cloud.message_received;
    let txt = "API online";
    if (h.ros && !h.ros.listener_alive) {
      txt += " — ROS listener DOWN";
      if (h.ros.listener_start_error) txt += " (" + h.ros.listener_start_error + ")";
      setConn(false, txt);
    } else if (!received) {
      txt += " — waiting for point cloud on " + (h.point_cloud ? h.point_cloud.topic : "?");
      setConn(null, txt);
    } else {
      const age = h.point_cloud.latest_message_age_sec;
      const stale = h.point_cloud.stale ? " (stale)" : "";
      txt += ` — cloud OK, ${h.point_cloud.latest_point_count} pts, age ${age}s${stale}`;
      setConn(true, txt);
    }
    $("status").textContent =
      `Topic: ${(h.point_cloud || {}).topic || "?"} | renderer: ${h.render ? h.render.renderer : "?"}` +
      demStatusText(h.dem);
    return true;
  } catch (err) {
    setConn(false, "API unreachable: " + err.message);
    return false;
  }
}

/** Compact one-line DEM summary for the status bar (from /api/health). */
function demStatusText(d) {
  if (!d) return "";
  if (!d.enabled) return " | DEM: off" + (d.reason ? ` (${d.reason})` : "");
  if (!d.scans_accumulated) return " | DEM: accumulating…";
  const cells = (d.valid_cells || 0).toLocaleString();
  const z = (d.z_min != null && d.z_max != null)
    ? `, z ${d.z_min.toFixed(2)}…${d.z_max.toFixed(2)} m` : "";
  return ` | DEM: ${d.scans_accumulated} scans · ${cells} cells${z}`;
}

async function fetchSnapshot() {
  if (busy) return; // prevent accidental repeated requests
  setBusy(true);
  showError("");
  saveSettings();
  try {
    const resp = await fetch(apiBase() + "/api/snapshot", {
      method: "POST",
      headers: authHeaders(),
      signal: AbortSignal.timeout(REQUEST_TIMEOUT_MS),
    });
    let body = null;
    try { body = await resp.json(); } catch (_) { /* non-JSON error */ }
    if (!resp.ok) {
      const detail = body && body.detail ? body.detail : "HTTP " + resp.status;
      throw new Error(detail);
    }
    displaySnapshot(body);
    checkHealth();
  } catch (err) {
    showError("Snapshot failed: " + err.message);
  } finally {
    setBusy(false);
  }
}

function displaySnapshot(data) {
  for (const view of VIEWS) {
    const url = data.images[view] + "?t=" + Date.now(); // cache-buster
    $("img-" + view).src = url;
    $("dl-" + view).href = data.images[view];
  }
  lastZipUrl = data.download_zip;
  $("btn-zip").disabled = false;

  $("m-id").textContent = (data.snapshot_id || "").slice(0, 12) + "…";
  $("m-capture").textContent = fmtTime(data.capture_timestamp);
  $("m-ros").textContent = data.ros_message_stamp
    ? fmtTime(data.ros_message_stamp) : "not available";
  $("m-frame").textContent = data.frame_id || "not available";
  $("m-topic").textContent = data.topic || "—";
  $("m-points").textContent =
    `${(data.valid_point_count || 0).toLocaleString()} / ${(data.rendered_point_count || 0).toLocaleString()}` +
    (data.original_point_count && data.original_point_count !== data.valid_point_count
      ? ` (msg had ${data.original_point_count.toLocaleString()})` : "");
  $("m-rendertime").textContent = (data.render_seconds || 0).toFixed(2) + " s";

  const warnRow = $("m-warnings-row");
  if (data.warnings && data.warnings.length) {
    $("m-warnings").textContent = data.warnings.join("; ");
    warnRow.hidden = false;
  } else {
    warnRow.hidden = true;
  }
  $("meta").hidden = false;
}

// ---------------------------------------------------------------------------
// Get DEM (incremental elevation model accumulated since playback started)
// ---------------------------------------------------------------------------
function demShowError(msg) {
  const el = $("dem-error");
  el.hidden = !msg;
  el.textContent = msg || "";
}

function demSetStatus(text) {
  $("dem-status").textContent = text;
}

function setDemBusy(state) {
  demBusy = state;
  $("btn-dem").disabled = state;
  $("dem-spinner").hidden = !state;
  $("btn-dem").textContent = state ? "Rendering DEM…" : "Get DEM";
  if (state) demSetStatus("merging scans…");
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
  if (demBusy) return; // prevent accidental repeated requests
  setDemBusy(true);
  demShowError("");
  saveSettings();
  try {
    const resp = await fetch(apiBase() + "/api/dem", {
      method: "POST",
      headers: authHeaders(),
      signal: AbortSignal.timeout(REQUEST_TIMEOUT_MS),
    });
    let body = null;
    try { body = await resp.json(); } catch (_) { /* non-JSON error */ }
    if (!resp.ok) {
      const detail = body && body.detail ? body.detail : "HTTP " + resp.status;
      throw new Error(detail);
    }
    displayDem(body);
    checkHealth();
  } catch (err) {
    demShowError("Get DEM failed: " + err.message);
    demSetStatus("error");
  } finally {
    setDemBusy(false);
  }
}

function displayDem(data) {
  $("img-dem").src = data.image + "?t=" + Date.now(); // cache-buster
  $("dl-dem").href = data.image;
  lastDemImageUrl = data.image;
  $("btn-dem-dl").disabled = false;

  const g = data.grid || {};
  const f = data.filters || {};
  const shape = g.shape || [];
  $("dm-scans").textContent =
    `${(data.scans_accumulated || 0).toLocaleString()} scans`;
  $("dm-points").textContent =
    `${(data.points_used || 0).toLocaleString()} merged / ` +
    `${(data.points_in || 0).toLocaleString()} seen`;
  $("dm-window").textContent = fmtWindow(data.window);
  $("dm-grid").textContent = shape.length === 2
    ? `${shape[1]} × ${shape[0]} cells @ ${g.resolution_m} m · ` +
      `${(g.extent_m || []).join(" × ")} m · agg=${g.aggregation}`
    : "—";
  $("dm-coverage").textContent = g.valid_cells != null
    ? `${g.valid_cells.toLocaleString()} cells · ${g.coverage_m2} m²`
    : "—";
  $("dm-z").textContent = g.z_min != null && g.z_max != null
    ? `${g.z_min.toFixed(2)} … ${g.z_max.toFixed(2)} m`
    : "—";
  $("dm-filters").textContent =
    `clip ${f.clip_range_m || "off"} m · min ${f.min_range_m} m · ${f.ground_filter}`;
  $("dm-rendertime").textContent = (data.render_seconds || 0).toFixed(2) + " s";

  const warnRow = $("dm-warnings-row");
  if (data.warnings && data.warnings.length) {
    $("dm-warnings").textContent = data.warnings.join("; ");
    warnRow.hidden = false;
  } else {
    warnRow.hidden = true;
  }
  $("dem-meta").hidden = false;
  $("dem-figure").hidden = false;
  demSetStatus(g.valid_cells != null && g.z_min != null
    ? `${g.valid_cells.toLocaleString()} cells · ` +
      `z ${g.z_min.toFixed(2)}…${g.z_max.toFixed(2)} m`
    : "rendered");
}

async function downloadViaFetch(url, filename) {
  // fetch -> blob -> object URL so cross-origin downloads work reliably
  const resp = await fetch(url, { headers: authHeaders() });
  if (!resp.ok) throw new Error("download failed: HTTP " + resp.status);
  const blob = await resp.blob();
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}

// ---------------------------------------------------------------------------
// Live view (multipart/x-mixed-replace)
// ---------------------------------------------------------------------------
function liveShowError(msg) {
  const el = $("live-error");
  el.hidden = !msg;
  el.textContent = msg || "";
}

function liveSetStatus(text) {
  $("live-status").textContent = text;
}

function startLive() {
  const view = $("live-view").value;
  const token = ($("api-token").value || "").trim();
  let url = `${apiBase()}/api/stream?view=${encodeURIComponent(view)}`;
  if (token) url += `&token=${encodeURIComponent(token)}`;

  liveShowError("");
  const img = $("live-frame");
  // Clear any previous stream before assigning a new one.
  img.onload = null;
  img.src = url;
  $("live-figure").hidden = false;
  liveOn = true;
  $("btn-live").textContent = "Stop Live View";
  $("btn-live").classList.remove("primary");
  liveSetStatus("connecting…");

  // The browser loads frames silently; poll /api/health for real stats.
  if (liveStatusTimer) clearInterval(liveStatusTimer);
  liveStatusTimer = setInterval(updateLiveStatus, LIVE_STATUS_INTERVAL_MS);
  updateLiveStatus();

  img.onerror = () => {
    // Only meaningful while we expect the stream to be up.
    if (!liveOn) return;
    liveShowError("Live stream failed to load (401? wrong token? service down?).");
    liveSetStatus("error");
  };
}

function stopLive() {
  liveOn = false;
  const img = $("live-frame");
  img.onerror = null;
  img.src = ""; // closing the src aborts the multipart connection server-side
  $("live-figure").hidden = true;
  $("btn-live").textContent = "Start Live View";
  $("btn-live").classList.add("primary");
  liveSetStatus("idle");
  if (liveStatusTimer) {
    clearInterval(liveStatusTimer);
    liveStatusTimer = null;
  }
  liveShowError("");
}

async function updateLiveStatus() {
  if (!liveOn) return;
  try {
    const resp = await fetch(apiBase() + "/api/health", {
      headers: authHeaders(),
      signal: AbortSignal.timeout(8000),
    });
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    const h = await resp.json();
    const s = h.stream || {};
    if (s.last_error) {
      liveShowError("Server: " + s.last_error);
      liveSetStatus("render error");
      return;
    }
    const fps = s.last_fps != null ? s.last_fps : 0;
    const pts = s.last_points != null ? s.last_points : 0;
    const age = s.last_frame_age_sec;
    if (s.clients === 0 || !s.render_loop_alive) {
      liveSetStatus("waiting for server…");
    } else if (age != null && age > 10) {
      liveSetStatus(`no new frames (${age}s old) — is the bag playing?`);
    } else {
      liveSetStatus(`${fps} fps · ${pts.toLocaleString()} pts`);
    }
    liveShowError("");
  } catch (err) {
    liveSetStatus("health check failed: " + err.message);
  }
}

function init() {
  loadSettings();
  $("api-base").value = $("api-base").value || window.location.origin;
  $("btn-fetch").addEventListener("click", fetchSnapshot);
  $("btn-dem").addEventListener("click", fetchDem);
  $("btn-check").addEventListener("click", checkHealth);
  $("api-base").addEventListener("change", () => {
    saveSettings(); checkHealth();
    if (liveOn) { stopLive(); startLive(); } // re-point the stream
  });
  $("api-token").addEventListener("change", () => {
    saveSettings(); checkHealth();
    if (liveOn) { stopLive(); startLive(); }
  });
  $("btn-live").addEventListener("click", () => {
    if (liveOn) stopLive(); else startLive();
  });
  $("live-view").addEventListener("change", () => {
    if (liveOn) { stopLive(); startLive(); } // switch camera angle
  });
  $("btn-zip").addEventListener("click", async () => {
    if (!lastZipUrl) return;
    $("btn-zip").disabled = true;
    try {
      await downloadViaFetch(lastZipUrl, "point_cloud_snapshot.zip");
    } catch (err) {
      showError("ZIP download failed: " + err.message);
    } finally {
      $("btn-zip").disabled = false;
    }
  });
  $("btn-dem-dl").addEventListener("click", async () => {
    if (!lastDemImageUrl) return;
    $("btn-dem-dl").disabled = true;
    try {
      await downloadViaFetch(lastDemImageUrl, "dem.png");
    } catch (err) {
      demShowError("PNG download failed: " + err.message);
    } finally {
      $("btn-dem-dl").disabled = false;
    }
  });
  // Individual downloads go through fetch for cross-origin reliability.
  for (const view of VIEWS) {
    $("dl-" + view).addEventListener("click", async (ev) => {
      ev.preventDefault();
      try {
        await downloadViaFetch(ev.target.href, view + ".png");
      } catch (err) {
        showError("PNG download failed: " + err.message);
      }
    });
  }
  checkHealth();
}

document.addEventListener("DOMContentLoaded", init);
