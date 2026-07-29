#include "HttpsGpsServer.h"
#include <ESP8266WebServerSecure.h>
#include "GpsHttpsCert.h"

namespace {
BearSSL::ESP8266WebServerSecure* server = nullptr;
BearSSL::ServerSessions* sessionCache = nullptr;

void handleGpsRoot();

const char GPS_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>GPS - Kubotio</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; min-width: 0; }
    html, body { max-width: 100%; overflow-x: hidden; }
    body { font-family: -apple-system, Arial, sans-serif; margin: 0; padding: 16px; background: #f8f9fa; color: #2c3e50; }
    h1 { font-size: 1.3em; }
    a { color: #007bff; text-decoration: none; }
    .card { background: #fff; border-radius: 10px; padding: 16px; box-shadow: 0 2px 4px rgba(0,0,0,0.08); margin-bottom: 12px; }
    .row { display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; padding: 4px 0; gap: 4px; }
    .label { color: #6c757d; font-size: 0.9em; }
    .value { font-weight: 600; }
    button { background: #007bff; color: #fff; border: none; border-radius: 6px; padding: 10px 16px; font-size: 0.95em; cursor: pointer; margin: 4px 4px 4px 0; }
    button.secondary { background: #6c757d; }
    button.danger { background: #dc3545; }
    button:disabled { background: #adb5bd; }
    .big { font-size: 2em; font-weight: 700; text-align: center; margin: 8px 0; }
    .deviation-bar { position: relative; height: 24px; background: #e9ecef; border-radius: 12px; margin: 12px 0; overflow: hidden; }
    .deviation-marker { position: absolute; top: 0; bottom: 0; width: 4px; background: #007bff; left: 50%; transform: translateX(-50%); }
    .deviation-center { position: absolute; top: 0; bottom: 0; left: 50%; width: 2px; background: #6c757d; }
    table { width: 100%; border-collapse: collapse; }
    th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid #e9ecef; font-size: 0.85em; }
    .note { font-size: 0.8em; color: #6c757d; }
  </style>
</head>
<body>
  <a id="backLink" href="#">&larr; Back to dashboard (HTTP)</a>
  <h1>GPS Tracking</h1>

  <div class="card">
    <div class="row"><span class="label">Permission</span><span class="value" id="permStatus">not requested</span></div>
    <div class="row"><span class="label">Accuracy</span><span class="value" id="accuracy">--</span></div>
    <div class="big" id="speed">-- km/h</div>
    <button id="startBtn" onclick="requestGps()">Enable GPS</button>
  </div>

  <div class="card">
    <h2 style="font-size:1em;margin-top:0;">Track recording</h2>
    <button id="recBtn" onclick="toggleRecording()" disabled>Start recording</button>
    <div id="sessionList"></div>
  </div>

  <div class="card">
    <h2 style="font-size:1em;margin-top:0;">Straight-line guidance (AB line)</h2>
    <button onclick="setPointA()" disabled id="setABtn">Set A (here)</button>
    <button onclick="setPointB()" disabled id="setBBtn">Set B (here)</button>
    <button class="secondary" onclick="clearAB()">Clear</button>
    <div class="deviation-bar" id="deviationBar" style="display:none;">
      <div class="deviation-center"></div>
      <div class="deviation-marker" id="deviationMarker"></div>
    </div>
    <div class="row" id="deviationRow" style="display:none;">
      <span class="label">Off line</span><span class="value" id="deviationValue">--</span>
    </div>
    <div class="note">Drive to A, tap "Set A", drive toward where you want the line to end, tap "Set B". The bar then shows how far off that line you are as you drive.</div>
  </div>

  <div class="card">
    <h2 style="font-size:1em;margin-top:0;">Parcel &amp; coverage</h2>
    <div class="row">
      <input type="text" id="refcatInput" placeholder="Referencia catastral (20 chars)" style="flex:1;padding:8px;border:1px solid #e9ecef;border-radius:6px;">
      <button onclick="fetchParcel()">Fetch</button>
    </div>
    <div class="note" id="parcelInfo">No parcel loaded yet.</div>
    <canvas id="parcelCanvas" style="width:100%;height:300px;display:block;"></canvas>
    <div class="row">
      <span class="label">Working width</span>
      <span><input type="number" id="swathInput" value="3" min="0.5" step="0.5" style="width:70px;" onchange="loadParcel()"> m</span>
    </div>
    <div class="row"><span class="label">Coverage</span><span class="value" id="coveragePct">--</span></div>
    <button class="secondary" onclick="clearParcel()">Clear stored parcel</button>
    <div class="note">Fetched once from Sede del Catastro (needs internet - do this while on
      home WiFi) and IGN's aerial photo service, then stored in this browser for reuse
      offline. Coverage is tracked in cells the size of your working width, filled in as
      you drive with GPS enabled above (recording on/off doesn't matter for this).</div>
  </div>

  <script>
    document.getElementById('backLink').href = 'http://' + location.hostname + '/';

    // --- IndexedDB: one object store, points tagged with a sessionId (session start ms) ---
    const DB_NAME = 'kubotio-gps', STORE = 'points';
    function openDb() {
      return new Promise((resolve, reject) => {
        const req = indexedDB.open(DB_NAME, 1);
        req.onupgradeneeded = () => {
          const db = req.result;
          const store = db.createObjectStore(STORE, { keyPath: 'id', autoIncrement: true });
          store.createIndex('sessionId', 'sessionId');
        };
        req.onsuccess = () => resolve(req.result);
        req.onerror = () => reject(req.error);
      });
    }
    async function savePoint(sessionId, point) {
      const db = await openDb();
      return new Promise((resolve, reject) => {
        const tx = db.transaction(STORE, 'readwrite');
        tx.objectStore(STORE).add({ sessionId, ...point });
        tx.oncomplete = () => resolve();
        tx.onerror = () => reject(tx.error);
      });
    }
    async function getAllPoints() {
      const db = await openDb();
      return new Promise((resolve, reject) => {
        const tx = db.transaction(STORE, 'readonly');
        const req = tx.objectStore(STORE).getAll();
        req.onsuccess = () => resolve(req.result);
        req.onerror = () => reject(req.error);
      });
    }
    async function deleteSession(sessionId) {
      const db = await openDb();
      return new Promise((resolve, reject) => {
        const tx = db.transaction(STORE, 'readwrite');
        const idx = tx.objectStore(STORE).index('sessionId');
        const req = idx.openCursor(IDBKeyRange.only(sessionId));
        req.onsuccess = () => {
          const cursor = req.result;
          if (cursor) { cursor.delete(); cursor.continue(); }
        };
        tx.oncomplete = () => resolve();
        tx.onerror = () => reject(tx.error);
      });
    }
    function groupBySession(points) {
      const bySession = {};
      points.forEach(p => { (bySession[p.sessionId] = bySession[p.sessionId] || []).push(p); });
      return bySession;
    }

    // --- Small "components": pure functions, data in -> display strings ---
    const Fmt = {
      kmh: mps => (mps == null ? '--' : (mps * 3.6).toFixed(1)) + ' km/h',
      meters: m => m.toFixed(1) + ' m',
      duration: ms => Math.round(ms / 1000) + ' s',
      distanceKm: points => (trackDistanceMeters(points) / 1000).toFixed(2) + ' km',
    };

    function haversine(a, b) {
      const R = 6371000, toRad = d => d * Math.PI / 180;
      const dLat = toRad(b.lat - a.lat), dLon = toRad(b.lon - a.lon);
      const s = Math.sin(dLat / 2) ** 2 + Math.cos(toRad(a.lat)) * Math.cos(toRad(b.lat)) * Math.sin(dLon / 2) ** 2;
      return 2 * R * Math.asin(Math.sqrt(s));
    }
    function trackDistanceMeters(points) {
      let total = 0;
      for (let i = 1; i < points.length; i++) total += haversine(points[i - 1], points[i]);
      return total;
    }

    function gpxForPoints(points) {
      const trkpts = points.map(p =>
        `<trkpt lat="${p.lat}" lon="${p.lon}"><time>${new Date(p.t).toISOString()}</time></trkpt>`).join('\n      ');
      return `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="kubotio">
  <trk><name>Kubotio track</name><trkseg>
      ${trkpts}
  </trkseg></trk>
</gpx>`;
    }
    function downloadText(filename, text) {
      const blob = new Blob([text], { type: 'application/gpx+xml' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = filename;
      a.click();
      URL.revokeObjectURL(a.href);
    }

    function renderSessionRow(sessionId, points) {
      const tr = document.createElement('tr');
      const start = new Date(Number(sessionId));
      tr.innerHTML = `<td>${start.toLocaleString()}</td><td>${points.length} pts</td><td>${Fmt.distanceKm(points)}</td>`;
      const actionsCell = document.createElement('td');
      const dlBtn = document.createElement('button');
      dlBtn.textContent = 'GPX';
      dlBtn.onclick = () => downloadText(`track-${sessionId}.gpx`, gpxForPoints(points));
      const delBtn = document.createElement('button');
      delBtn.textContent = 'Delete';
      delBtn.className = 'danger';
      delBtn.onclick = async () => { await deleteSession(sessionId); refreshSessionList(); };
      actionsCell.append(dlBtn, delBtn);
      tr.appendChild(actionsCell);
      return tr;
    }

    async function refreshSessionList() {
      const points = await getAllPoints();
      const bySession = groupBySession(points);
      const container = document.getElementById('sessionList');
      const ids = Object.keys(bySession).sort().reverse();
      if (!ids.length) { container.innerHTML = '<p class="note">No tracks recorded yet.</p>'; return; }
      const table = document.createElement('table');
      table.innerHTML = '<thead><tr><th>Started</th><th>Points</th><th>Distance</th><th></th></tr></thead>';
      const tbody = document.createElement('tbody');
      ids.forEach(id => tbody.appendChild(renderSessionRow(id, bySession[id])));
      table.appendChild(tbody);
      container.innerHTML = '';
      container.appendChild(table);
    }

    // --- Geolocation ---
    let lastPosition = null, recordingSessionId = null, pointA = null, pointB = null;

    function onPosition(pos) {
      const c = pos.coords;
      lastPosition = { lat: c.latitude, lon: c.longitude, t: pos.timestamp };
      document.getElementById('accuracy').textContent = Fmt.meters(c.accuracy);
      document.getElementById('speed').textContent = Fmt.kmh(c.speed);

      if (recordingSessionId) savePoint(recordingSessionId, lastPosition);
      if (pointA && pointB) updateDeviation(lastPosition);
      if (parcel) markCoverage(lastPosition);
    }
    function onError(err) {
      document.getElementById('permStatus').textContent = 'error: ' + err.message;
    }

    function requestGps() {
      if (!navigator.geolocation) { document.getElementById('permStatus').textContent = 'not supported'; return; }
      navigator.geolocation.watchPosition(onPosition, onError, { enableHighAccuracy: true, maximumAge: 1000, timeout: 10000 });
      document.getElementById('permStatus').textContent = 'requested...';
      document.getElementById('startBtn').disabled = true;
      document.getElementById('recBtn').disabled = false;
      document.getElementById('setABtn').disabled = false;
      document.getElementById('setBBtn').disabled = false;
      // First successful fix confirms the permission was granted.
      const check = setInterval(() => {
        if (lastPosition) { document.getElementById('permStatus').textContent = 'granted'; clearInterval(check); }
      }, 500);
    }

    function toggleRecording() {
      const btn = document.getElementById('recBtn');
      if (recordingSessionId) {
        recordingSessionId = null;
        btn.textContent = 'Start recording';
        btn.classList.remove('danger');
        refreshSessionList();
      } else {
        recordingSessionId = String(Date.now());
        btn.textContent = 'Stop recording';
        btn.classList.add('danger');
      }
    }

    // Flat-earth local projection (fine for AB lines a few hundred meters long).
    function toLocalXY(p, refLat) {
      const R = 6371000, toRad = d => d * Math.PI / 180;
      return { x: toRad(p.lon) * R * Math.cos(toRad(refLat)), y: toRad(p.lat) * R };
    }
    function crossTrackMeters(pos, a, b) {
      const A = toLocalXY(a, a.lat), B = toLocalXY(b, a.lat), P = toLocalXY(pos, a.lat);
      const abx = B.x - A.x, aby = B.y - A.y;
      const len = Math.hypot(abx, aby) || 1;
      // signed cross product distance: + = right of A->B, - = left
      return ((P.x - A.x) * aby - (P.y - A.y) * abx) / len;
    }

    function setPointA() { if (lastPosition) pointA = { ...lastPosition }; }
    function setPointB() { if (lastPosition) pointB = { ...lastPosition }; }
    function clearAB() {
      pointA = null; pointB = null;
      document.getElementById('deviationBar').style.display = 'none';
      document.getElementById('deviationRow').style.display = 'none';
    }
    function updateDeviation(pos) {
      const meters = crossTrackMeters(pos, pointA, pointB);
      document.getElementById('deviationBar').style.display = 'block';
      document.getElementById('deviationRow').style.display = 'flex';
      const clamped = Math.max(-20, Math.min(20, meters)); // +-20m maps to the full bar width
      document.getElementById('deviationMarker').style.left = (50 + clamped / 20 * 50) + '%';
      document.getElementById('deviationValue').textContent =
        Fmt.meters(Math.abs(meters)) + (meters > 0.2 ? ' right' : meters < -0.2 ? ' left' : ' on line');
    }

    // --- Parcel boundary + coverage tracking ---
    // Fetched once from public, key-free, CORS-open government services (needs
    // internet - do this on home WiFi), then cached in localStorage for offline reuse:
    //   - Sede del Catastro INSPIRE WFS: boundary polygon by cadastral reference
    //   - IGN PNOA WMS: aerial photo background
    const PARCEL_KEY = 'kubotio_parcel';
    const COVERAGE_KEY_PREFIX = 'kubotio_coverage_';
    let parcel = null;      // in-memory, rebuilt from localStorage on load
    let parcelPhotoImg = null;
    let coverageSaveTimer = null;

    function toLocalMeters(lat, lon, originLat, originLon) {
      const R = 6371000, toRad = d => d * Math.PI / 180;
      return {
        x: toRad(lon - originLon) * R * Math.cos(toRad(originLat)),
        y: toRad(lat - originLat) * R,
      };
    }
    function toLatLon(x, y, originLat, originLon) {
      const R = 6371000, toDeg = r => r * 180 / Math.PI;
      return {
        lat: originLat + toDeg(y / R),
        lon: originLon + toDeg(x / (R * Math.cos(originLat * Math.PI / 180))),
      };
    }
    function pointInPolygon(x, y, poly) {
      let inside = false;
      for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const xi = poly[i].x, yi = poly[i].y, xj = poly[j].x, yj = poly[j].y;
        if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) inside = !inside;
      }
      return inside;
    }

    async function fetchParcel() {
      const ref = document.getElementById('refcatInput').value.trim();
      if (!ref) return;
      document.getElementById('parcelInfo').textContent = 'Fetching...';
      try {
        const url = 'https://ovc.catastro.meh.es/INSPIRE/wfsCP.aspx?service=wfs&request=GetFeature' +
          '&STOREDQUERIE_ID=GetParcel&REFCAT=' + encodeURIComponent(ref) + '&SRSNAME=EPSG::4326';
        const xmlText = await (await fetch(url)).text();
        const xml = new DOMParser().parseFromString(xmlText, 'text/xml');
        const posLists = xml.getElementsByTagNameNS('http://www.opengis.net/gml/3.2', 'posList');
        if (!posLists.length) { document.getElementById('parcelInfo').textContent = 'Parcel not found.'; return; }

        // Multi-part parcels can have several rings; use the largest as the main boundary.
        let best = posLists[0];
        for (const el of posLists) if (el.textContent.trim().split(/\s+/).length > best.textContent.trim().split(/\s+/).length) best = el;
        const nums = best.textContent.trim().split(/\s+/).map(Number);
        const points = [];
        for (let i = 0; i < nums.length; i += 2) points.push([nums[i], nums[i + 1]]); // [lat, lon]

        const areaEl = xml.getElementsByTagNameNS('http://inspire.ec.europa.eu/schemas/cp/4.0', 'areaValue')[0];
        const areaM2 = areaEl ? Number(areaEl.textContent) : null;

        localStorage.setItem(PARCEL_KEY, JSON.stringify({ ref, points, areaM2 }));
        localStorage.removeItem(COVERAGE_KEY_PREFIX + ref);
        parcelPhotoImg = null;
        await loadParcel();
      } catch (e) {
        document.getElementById('parcelInfo').textContent = 'Fetch failed: ' + e.message;
      }
    }

    async function loadParcel() {
      const raw = localStorage.getItem(PARCEL_KEY);
      if (!raw) return;
      const data = JSON.parse(raw);

      const lats = data.points.map(p => p[0]), lons = data.points.map(p => p[1]);
      const originLat = Math.min(...lats), originLon = Math.min(...lons);
      const localPts = data.points.map(([lat, lon]) => toLocalMeters(lat, lon, originLat, originLon));
      const maxX = Math.max(...localPts.map(p => p.x)), maxY = Math.max(...localPts.map(p => p.y));

      const cellSize = Number(document.getElementById('swathInput').value) || 3;
      const gridW = Math.max(1, Math.ceil(maxX / cellSize));
      const gridH = Math.max(1, Math.ceil(maxY / cellSize));
      const eligible = new Uint8Array(gridW * gridH);
      for (let gy = 0; gy < gridH; gy++) {
        for (let gx = 0; gx < gridW; gx++) {
          if (pointInPolygon((gx + 0.5) * cellSize, (gy + 0.5) * cellSize, localPts)) eligible[gy * gridW + gx] = 1;
        }
      }

      const savedCoverage = localStorage.getItem(COVERAGE_KEY_PREFIX + data.ref);
      const covered = new Set(savedCoverage ? JSON.parse(savedCoverage) : []);

      parcel = { ref: data.ref, areaM2: data.areaM2, points: data.points, originLat, originLon, localPts, maxX, maxY, cellSize, gridW, gridH, eligible, covered };
      document.getElementById('parcelInfo').textContent =
        parcel.ref + (parcel.areaM2 ? ' - ' + (parcel.areaM2 / 10000).toFixed(2) + ' ha' : '');
      await drawParcelCanvas();
      updateCoveragePct();
    }

    async function drawParcelCanvas() {
      const canvas = document.getElementById('parcelCanvas');
      const w = canvas.clientWidth || 600, h = 300;
      canvas.width = w; canvas.height = h;
      const ctx = canvas.getContext('2d');

      const padX = parcel.maxX * 0.1 || 10, padY = parcel.maxY * 0.1 || 10;
      const viewMinX = -padX, viewMaxX = parcel.maxX + padX, viewMinY = -padY, viewMaxY = parcel.maxY + padY;
      const xForM = mx => (mx - viewMinX) / (viewMaxX - viewMinX) * w;
      const yForM = my => h - (my - viewMinY) / (viewMaxY - viewMinY) * h;

      if (!parcelPhotoImg || parcelPhotoImg.dataset.ref !== parcel.ref) {
        const minCorner = toLatLon(viewMinX, viewMinY, parcel.originLat, parcel.originLon);
        const maxCorner = toLatLon(viewMaxX, viewMaxY, parcel.originLat, parcel.originLon);
        const bbox = [minCorner.lon, minCorner.lat, maxCorner.lon, maxCorner.lat].join(',');
        const wmsUrl = 'https://www.ign.es/wms-inspire/pnoa-ma?service=WMS&version=1.1.1&request=GetMap' +
          '&layers=OI.OrthoimageCoverage&styles=&srs=EPSG:4326&bbox=' + bbox +
          '&width=' + w + '&height=' + h + '&format=image/jpeg';
        const img = new Image();
        img.crossOrigin = 'anonymous';
        img.dataset.ref = parcel.ref;
        await new Promise(resolve => { img.onload = resolve; img.onerror = resolve; img.src = wmsUrl; });
        parcelPhotoImg = img;
      }

      ctx.clearRect(0, 0, w, h);
      if (parcelPhotoImg.complete && parcelPhotoImg.naturalWidth) ctx.drawImage(parcelPhotoImg, 0, 0, w, h);

      ctx.fillStyle = 'rgba(40,167,69,0.55)';
      const cellPxW = cellSizePx(parcel.cellSize, viewMinX, viewMaxX, w);
      const cellPxH = cellSizePx(parcel.cellSize, viewMinY, viewMaxY, h);
      for (let gy = 0; gy < parcel.gridH; gy++) {
        for (let gx = 0; gx < parcel.gridW; gx++) {
          const idx = gy * parcel.gridW + gx;
          if (parcel.eligible[idx] && parcel.covered.has(idx)) {
            const x = xForM(gx * parcel.cellSize), y = yForM((gy + 1) * parcel.cellSize);
            ctx.fillRect(x, y, cellPxW, cellPxH);
          }
        }
      }

      ctx.strokeStyle = '#ffeb3b';
      ctx.lineWidth = 3;
      ctx.beginPath();
      parcel.localPts.forEach((p, i) => {
        const x = xForM(p.x), y = yForM(p.y);
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.stroke();

      if (lastPosition) {
        const p = toLocalMeters(lastPosition.lat, lastPosition.lon, parcel.originLat, parcel.originLon);
        ctx.fillStyle = '#007bff';
        ctx.beginPath();
        ctx.arc(xForM(p.x), yForM(p.y), 6, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
    function cellSizePx(cellMeters, viewMin, viewMax, pxSize) {
      return cellMeters / (viewMax - viewMin) * pxSize;
    }

    function markCoverage(pos) {
      const p = toLocalMeters(pos.lat, pos.lon, parcel.originLat, parcel.originLon);
      const gx = Math.floor(p.x / parcel.cellSize), gy = Math.floor(p.y / parcel.cellSize);
      if (gx < 0 || gy < 0 || gx >= parcel.gridW || gy >= parcel.gridH) return;
      const idx = gy * parcel.gridW + gx;
      const isNew = parcel.eligible[idx] && !parcel.covered.has(idx);
      parcel.covered.add(idx);
      if (isNew) {
        updateCoveragePct();
        drawParcelCanvas();
        clearTimeout(coverageSaveTimer);
        coverageSaveTimer = setTimeout(saveCoverage, 5000); // throttle localStorage writes
      }
    }
    function saveCoverage() {
      if (parcel) localStorage.setItem(COVERAGE_KEY_PREFIX + parcel.ref, JSON.stringify(Array.from(parcel.covered)));
    }
    function updateCoveragePct() {
      let eligibleCount = 0, coveredCount = 0;
      for (let i = 0; i < parcel.eligible.length; i++) {
        if (parcel.eligible[i]) { eligibleCount++; if (parcel.covered.has(i)) coveredCount++; }
      }
      document.getElementById('coveragePct').textContent =
        eligibleCount ? (coveredCount / eligibleCount * 100).toFixed(1) + '%' : '--';
    }
    function clearParcel() {
      if (parcel) localStorage.removeItem(COVERAGE_KEY_PREFIX + parcel.ref);
      localStorage.removeItem(PARCEL_KEY);
      parcel = null;
      parcelPhotoImg = null;
      document.getElementById('parcelInfo').textContent = 'No parcel loaded yet.';
      document.getElementById('coveragePct').textContent = '--';
      const canvas = document.getElementById('parcelCanvas');
      canvas.getContext('2d').clearRect(0, 0, canvas.width, canvas.height);
    }

    loadParcel();
    refreshSessionList();
  </script>
</body>
</html>
)=====";

void handleGpsRoot() {
  server->send_P(200, "text/html", GPS_PAGE);
}
}  // namespace

bool gps_https_start() {
  if (server != nullptr) return true;  // already running

  server = new BearSSL::ESP8266WebServerSecure(443);
  sessionCache = new BearSSL::ServerSessions(5);
  server->getServer().setRSACert(new BearSSL::X509List(GPS_HTTPS_CERT), new BearSSL::PrivateKey(GPS_HTTPS_KEY));
  server->getServer().setCache(sessionCache);
  server->on("/", handleGpsRoot);
  server->begin();
  return true;
}

void gps_https_stop() {
  if (server == nullptr) return;
  server->close();
  delete server;
  delete sessionCache;
  server = nullptr;
  sessionCache = nullptr;
}

bool gps_https_active() {
  return server != nullptr;
}

void gps_https_handle_client() {
  if (server != nullptr) server->handleClient();
}
