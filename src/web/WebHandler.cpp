#include "WebHandler.h"
#include <ArduinoJson.h>
#include "../charging/output_component.h"
#include "../connectivity/MqttPublisher.h"
#include "HttpsGpsServer.h"
#include <time.h>

namespace {
// Before NTP sync completes, time(nullptr) returns a small/near-zero value.
// Anything before ~2023 is treated as "not synced yet".
constexpr time_t kMinPlausibleEpoch = 1700000000;

uint32_t current_epoch_or_zero() {
  time_t now = time(nullptr);
  return now >= kMinPlausibleEpoch ? static_cast<uint32_t>(now) : 0;
}
}

ESP8266WebServer server(80);
extern OutputComponent alternator;
extern DataPoint history[];
extern byte historyIndex;
extern bool engine_running;
extern bool engine_probing;
extern bool overvoltage_alert;
extern bool wifiConnectedSta;
extern unsigned long maxLoopDurationUs;
extern MqttPublisher mqttPublisher;
extern const char* mqtt_host;
bool web_initialized = false;

void setupWebServer() {
  if (!web_initialized) {
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/history", handleHistory);
    server.on("/autotune/start", handleAutotuneStart);
    server.on("/maintenance/reset", handleMaintenanceReset);
    server.on("/maintenance/interval", handleMaintenanceInterval);
    server.on("/maintenance", handleMaintenancePage);
    server.on("/maintenance/log", handleMaintenanceLogList);
    server.on("/maintenance/log/add", handleMaintenanceLogAdd);
    server.on("/gps/enable", handleGpsEnable);
    server.on("/gps/disable", handleGpsDisable);
    server.on("/gps/status", handleGpsStatus);
    server.on("/restart", handleRestart); // remote recovery when there's no physical/USB access
    server.on("/mqtt/test", handleMqttTest);
    server.onNotFound(handleRoot); // captive portal: any unknown host/path -> dashboard
    server.begin();
    Serial.println("Web server started");
    web_initialized = true;
  }
}

void handleRoot() {
  String html = R"=====(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Tractor Monitor</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      :root {
        --bg: #f8f9fa; --card: #ffffff; --text: #2c3e50; --label: #6c757d;
        --border: #e9ecef; --accent: #007bff; --good: #28a745; --warn: #ffc107; --bad: #dc3545;
      }
      * { box-sizing: border-box; min-width: 0; }
      html, body { max-width: 100%; overflow-x: hidden; }
      body { font-family: -apple-system, Arial, sans-serif; margin: 0; padding: 16px; background: var(--bg); color: var(--text); }
      h1 { font-size: 1.4em; margin: 0 0 16px; }
      .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(min(260px, 100%), 1fr)); gap: 12px; }
      .card { background: var(--card); border-radius: 10px; padding: 16px; box-shadow: 0 2px 4px rgba(0,0,0,0.08); }
      .card h2 { font-size: 1em; margin: 0 0 12px; color: #495057; }
      .row { display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; padding: 4px 0; gap: 4px; }
      .label { color: var(--label); font-size: 0.9em; }
      .value { font-weight: 600; }
      .pill { padding: 3px 10px; border-radius: 12px; font-size: 0.85em; font-weight: 600; }
      .pill-good { background: #d4edda; color: #155724; }
      .pill-bad { background: #f8d7da; color: #721c24; }
      .pill-warn { background: #fff3cd; color: #856404; }
      .bar { width: 100%; height: 16px; background: var(--border); border-radius: 8px; overflow: hidden; margin: 6px 0; }
      .bar-fill { height: 100%; background: var(--accent); transition: width 0.4s ease; }
      button { background: var(--accent); color: #fff; border: none; border-radius: 6px; padding: 8px 14px; font-size: 0.9em; cursor: pointer; }
      button:disabled { background: #adb5bd; cursor: default; }
      button.secondary { background: #6c757d; }
      input[type=number] { width: 70px; padding: 4px 6px; border: 1px solid var(--border); border-radius: 6px; }
      #chartCard canvas { width: 100%; height: 260px; display: block; }
      .note { font-size: 0.8em; color: var(--label); margin-top: 8px; }
      select, input, button, img, canvas { max-width: 100%; }
      .row .label { flex: 1 1 auto; }
      .row .value, .row .pill { flex: 0 0 auto; text-align: right; }
      @media (max-width: 400px) {
        body { padding: 10px; }
        .card { padding: 12px; }
      }
    </style>
  </head>
  <body>
    <h1>Tractor Battery Monitor</h1>
    <div class="grid">

      <div class="card">
        <h2>Charging Status</h2>
        <div class="row"><span class="label">Voltage</span><span class="value" id="voltage">--</span></div>
        <div class="row"><span class="label">Mode</span><span class="value" id="outputMode">--</span></div>
        <div class="bar" id="pwmBar" style="display:none;"><div class="bar-fill" id="pwmFill" style="width:0%;"></div></div>
        <div class="row" id="pwmRow" style="display:none;"><span class="label">PWM</span><span class="value" id="pwmValue">0%</span></div>
        <div class="row"><span class="label">Engine</span><span class="pill" id="engineStatus">--</span></div>
        <div class="row" id="overvoltageRow" style="display:none;"><span class="label">⚠️ Overvoltage cutoff</span><span class="pill pill-bad">ACTIVE</span></div>
      </div>

      <div class="card">
        <h2>System</h2>
        <div class="bar"><div class="bar-fill" id="heapFill" style="width:0%;"></div></div>
        <div class="row"><span class="label">RAM used</span><span class="value" id="heapValue">--</span></div>
        <div class="row"><span class="label">Loop time (peak)</span><span class="value" id="loopTime">--</span></div>
        <div class="row"><span class="label">Refresh rate</span>
          <span><select id="pollInterval" onchange="setPollInterval()">
            <option value="1000">1s</option>
            <option value="2000">2s</option>
            <option value="5000">5s</option>
            <option value="10000">10s</option>
            <option value="30000">30s</option>
          </select></span>
        </div>
        <button class="secondary" onclick="restartDevice()">Restart device</button>
      </div>

      <div class="card">
        <h2>Connectivity</h2>
        <div class="row"><span class="label">WiFi</span><span class="pill" id="wifiMode">--</span></div>
        <div class="note">Home Assistant/MQTT only reachable while on home WiFi (STA); AP fallback is offline-only.</div>
      </div>

      <div class="card">
        <h2>MQTT / Home Assistant</h2>
        <div class="row"><span class="label">Status</span><span class="pill" id="mqttStatus">--</span></div>
        <div class="row"><span class="label">Last sent</span><span class="value" id="mqttLastSent">--</span></div>
        <button onclick="testMqtt()">Test connection</button>
      </div>

      <div class="card">
        <h2>GPS Mode</h2>
        <div class="row"><span class="label">Status</span><span class="pill" id="gpsStatus">off</span></div>
        <button id="gpsToggleBtn" onclick="toggleGps()">Enable</button>
        <div class="note">
          Opens a separate HTTPS page (self-signed cert - your browser will warn
          "not secure", that's expected for a LAN device with no internet CA; accept
          it once per device) needed for GPS to work at all in the browser.
          <span id="gpsLink"></span>
        </div>
      </div>

      <div class="card">
        <h2>PID Autotune</h2>
        <div class="row"><span class="label">Status</span><span class="pill" id="autotuneStatus">idle</span></div>
        <button id="autotuneBtn" onclick="startAutotune()">Start autotune</button>
        <div class="note">Briefly swings charge output to measure the alternator's response and tune the PID automatically.</div>
      </div>

      <div class="card">
        <h2>Maintenance</h2>
        <div class="row"><span class="label">Power-ons</span><span class="value" id="bootCount">--</span></div>
        <div class="row"><span class="label">Charging hours (total)</span><span class="value" id="runHours">--</span></div>
        <div class="row"><span class="label">Since last service</span><span class="value" id="serviceHours">--</span></div>
        <div class="row"><span class="label">Service interval</span>
          <span><input type="number" id="intervalInput" min="1"> h <button onclick="setInterval_()">Set</button></span>
        </div>
        <div class="row"><span class="label">Due for maintenance</span><span class="pill" id="maintenanceDue">--</span></div>
        <button class="secondary" onclick="resetMaintenance()">Mark serviced (reset)</button>
        <div class="note"><a href="/maintenance">View maintenance logbook &rarr;</a></div>
      </div>

      <div class="card" id="chartCard" style="grid-column: 1 / -1;">
        <h2>Voltage History</h2>
        <canvas id="voltageChart"></canvas>
      </div>

    </div>

    <script>
      // --- Small render "components": each takes raw data and returns {text, cls} or
      // a plain string. No framework - just named functions so call sites read like a
      // description of the UI instead of a wall of DOM calls. ---
      const Fmt = {
        volts: v => v.toFixed(2) + ' V',
        hours: h => h.toFixed(1) + ' h',
        percent: p => p + '%',
      };

      const WIFI_LABELS = { sta: 'Home WiFi', connecting: 'Connecting...', ap_fallback: 'AP fallback (offline)' };
      const WIFI_CLASSES = { sta: 'pill-good', connecting: 'pill-warn', ap_fallback: 'pill-warn' };

      const Components = {
        engineStatus: (running, probing) => probing
          ? { text: 'PROBING', cls: 'pill-warn' }
          : { text: running ? 'RUNNING' : 'STOPPED', cls: running ? 'pill-good' : 'pill-bad' },
        wifiMode: mode => ({ text: WIFI_LABELS[mode] || mode, cls: WIFI_CLASSES[mode] || 'pill-warn' }),
        autotuneStatus: active => ({ text: active ? 'running' : 'idle', cls: active ? 'pill-warn' : 'pill-good' }),
        maintenanceDue: due => ({ text: due ? 'DUE' : 'OK', cls: due ? 'pill-bad' : 'pill-good' }),
      };

      // --- DOM helpers: the only place that touches document.getElementById ---
      function setText(id, text) { document.getElementById(id).textContent = text; }
      function setPill(id, { text, cls }) {
        const el = document.getElementById(id);
        el.textContent = text;
        el.className = 'pill ' + cls;
      }

      function updateChargingStatus(data) {
        setText('voltage', Fmt.volts(data.voltage));

        const isPWM = data.outputMode === 'pwm';
        setText('outputMode', isPWM ? 'PWM' : 'RELAY');
        document.getElementById('pwmBar').style.display = isPWM ? 'block' : 'none';
        document.getElementById('pwmRow').style.display = isPWM ? 'flex' : 'none';
        if (isPWM) {
          setText('pwmValue', Fmt.percent(data.pwmPercentage));
          document.getElementById('pwmFill').style.width = Fmt.percent(data.pwmPercentage);
        }

        setPill('engineStatus', Components.engineStatus(data.engineRunning, data.engineProbing));
        document.getElementById('overvoltageRow').style.display = data.overvoltageAlert ? 'flex' : 'none';
      }

      function updateConnectivity(data) {
        setPill('wifiMode', Components.wifiMode(data.wifiMode));
      }

      function updateMqtt(data) {
        setPill('mqttStatus', data.mqttConnected
          ? { text: 'connected', cls: 'pill-good' } : { text: 'disconnected', cls: 'pill-bad' });
        setText('mqttLastSent', data.mqttHasLastPublish
          ? Fmt.volts(data.mqttLastVoltage) + ', ' + data.mqttLastPwmPercent + '% PWM (' + data.mqttLastSentAgoS + 's ago)'
          : 'never');
      }

      async function testMqtt() {
        await fetch('/mqtt/test');
        setTimeout(fetchData, 500);
      }

      async function restartDevice() {
        if (!confirm('Restart the device now?')) return;
        await fetch('/restart');
      }

      function updateAutotune(data) {
        setPill('autotuneStatus', Components.autotuneStatus(data.autotuneActive));
        document.getElementById('autotuneBtn').disabled = data.autotuneActive;
      }

      function updateMaintenance(data) {
        setText('bootCount', data.bootCount);
        setText('runHours', Fmt.hours(data.runHours));
        setText('serviceHours', Fmt.hours(data.secondsSinceService_h));
        if (document.activeElement.id !== 'intervalInput') {
          document.getElementById('intervalInput').value = data.serviceIntervalHours;
        }
        setPill('maintenanceDue', Components.maintenanceDue(data.maintenanceDue));
      }

      const TOTAL_HEAP_BYTES = 81920; // ESP8266 total RAM
      function updateSystem(data) {
        const usedPct = (TOTAL_HEAP_BYTES - data.freeHeap) / TOTAL_HEAP_BYTES * 100;
        document.getElementById('heapFill').style.width = usedPct.toFixed(0) + '%';
        setText('heapValue', usedPct.toFixed(0) + '% (' + (data.freeHeap / 1024).toFixed(1) + ' KB free)');
        setText('loopTime', (data.loopTimeUs / 1000).toFixed(1) + ' ms');
      }

      function updateStatus(data) {
        updateChargingStatus(data);
        updateConnectivity(data);
        updateAutotune(data);
        updateMaintenance(data);
        updateSystem(data);
        updateMqtt(data);
      }

      // Self-contained line chart, no external libraries (must work fully offline in AP mode).
      function drawChart(points) {
        const canvas = document.getElementById('voltageChart');
        const dpr = window.devicePixelRatio || 1;
        const cssWidth = canvas.clientWidth || 600;
        const cssHeight = 260;
        canvas.width = cssWidth * dpr;
        canvas.height = cssHeight * dpr;
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, cssWidth, cssHeight);
        if (!points.length) return;

        const padL = 34, padR = 8, padT = 8, padB = 18;
        const w = cssWidth - padL - padR, h = cssHeight - padT - padB;
        const minV = 12, maxV = 16;
        const t0 = points[0].timestamp, t1 = points[points.length - 1].timestamp;
        const xFor = t => padL + (t1 === t0 ? 0 : (t - t0) / (t1 - t0) * w);
        const yFor = v => padT + h - ((Math.min(Math.max(v, minV), maxV) - minV) / (maxV - minV)) * h;

        ctx.strokeStyle = '#e9ecef';
        ctx.fillStyle = '#6c757d';
        ctx.font = '11px sans-serif';
        ctx.lineWidth = 1;
        for (let v = minV; v <= maxV; v++) {
          const y = yFor(v);
          ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + w, y); ctx.stroke();
          ctx.fillText(v + 'V', 2, y + 4);
        }

        ctx.beginPath();
        points.forEach((p, i) => {
          const x = xFor(p.timestamp), y = yFor(p.voltage);
          if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = '#007bff';
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.lineTo(xFor(points[points.length - 1].timestamp), padT + h);
        ctx.lineTo(xFor(points[0].timestamp), padT + h);
        ctx.closePath();
        ctx.fillStyle = 'rgba(0,123,255,0.1)';
        ctx.fill();
      }

      function updateGps(data) {
        setPill('gpsStatus', data.active
          ? { text: 'on', cls: 'pill-good' } : { text: 'off', cls: 'pill-bad' });
        document.getElementById('gpsToggleBtn').textContent = data.active ? 'Disable' : 'Enable';
        document.getElementById('gpsLink').innerHTML = data.active
          ? ` <a href="https://${location.hostname}:443/gps">Open GPS page &rarr;</a>` : '';
      }

      async function fetchData() {
        try {
          const [statusRes, historyRes, gpsRes] = await Promise.all([fetch('/data'), fetch('/history'), fetch('/gps/status')]);
          updateStatus(await statusRes.json());
          drawChart(await historyRes.json());
          updateGps(await gpsRes.json());
        } catch (error) {
          console.error('Update failed:', error);
        }
      }

      async function toggleGps() {
        const enabling = document.getElementById('gpsToggleBtn').textContent === 'Enable';
        await fetch(enabling ? '/gps/enable' : '/gps/disable');
        fetchData();
      }

      async function startAutotune() {
        await fetch('/autotune/start');
        fetchData();
      }
      async function resetMaintenance() {
        await fetch('/maintenance/reset');
        fetchData();
      }
      async function setInterval_() {
        const hours = document.getElementById('intervalInput').value;
        await fetch('/maintenance/interval?hours=' + encodeURIComponent(hours));
        fetchData();
      }

      let pollTimer = null;
      function setPollInterval() {
        const ms = Number(document.getElementById('pollInterval').value);
        localStorage.setItem('kubotio_poll_ms', ms);
        if (pollTimer) clearInterval(pollTimer);
        pollTimer = setInterval(fetchData, ms);
      }
      document.getElementById('pollInterval').value = localStorage.getItem('kubotio_poll_ms') || '5000';
      setPollInterval();
      window.addEventListener('resize', fetchData);
      fetchData();
    </script>
  </body>
  </html>
  )=====";

  server.send(200, "text/html", html);
}

void handleData() {
  DynamicJsonDocument doc(512);
  doc["voltage"] = history[(historyIndex + HISTORY_SIZE - 1) % HISTORY_SIZE].voltage;
  doc["outputMode"] = alternator.isPWMEnabled() ? "pwm" : "relay";
  doc["pwmPercentage"] = alternator.getPWMPercent();
  doc["engineRunning"] = engine_running;
  doc["engineProbing"] = engine_probing;
  doc["overvoltageAlert"] = overvoltage_alert;

  doc["wifiMode"] = wifiConnectedSta ? "sta" : "ap_fallback";

  doc["autotuneActive"] = autotuneActive;
  doc["runHours"] = usageCounters.totalRunSeconds() / 3600.0;
  doc["secondsSinceService_h"] = usageCounters.secondsSinceService() / 3600.0;
  doc["serviceIntervalHours"] = usageCounters.serviceIntervalHours();
  doc["maintenanceDue"] = usageCounters.isMaintenanceDue();
  doc["bootCount"] = usageCounters.bootCount();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["resetReason"] = ESP.getResetReason();
  doc["resetInfo"] = ESP.getResetInfo();
  doc["wifiModeRaw"] = (int)WiFi.getMode();
  doc["softApIp"] = WiFi.softAPIP().toString();
  doc["softApStations"] = WiFi.softAPgetStationNum();
  doc["maxFreeBlock"] = ESP.getMaxFreeBlockSize();
  doc["heapFrag"] = ESP.getHeapFragmentation();
  doc["loopTimeUs"] = maxLoopDurationUs;

  doc["mqttConnected"] = mqttPublisher.isConnected();
  doc["mqttHasLastPublish"] = mqttPublisher.hasLastPublished();
  if (mqttPublisher.hasLastPublished()) {
    const MqttReading& last = mqttPublisher.lastPublished();
    doc["mqttLastVoltage"] = last.voltage;
    doc["mqttLastPwmPercent"] = alternator.getMaxPWM() ? (last.pwmValue * 100 / alternator.getMaxPWM()) : 0;
    doc["mqttLastSentAgoS"] = (millis() - mqttPublisher.lastPublishMillis()) / 1000;
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleHistory() {
  DynamicJsonDocument doc(4096);
  JsonArray array = doc.to<JsonArray>();

  for(int i = 0; i < HISTORY_SIZE; i++) {
    int idx = (historyIndex + i) % HISTORY_SIZE;
    if(history[idx].timestamp == 0) continue;

    JsonObject point = array.createNestedObject();
    point["timestamp"] = history[idx].timestamp;
    point["voltage"] = history[idx].voltage;
    point["mode"] = history[idx].outputMode ? "pwm" : "relay";
    point["pwm"] = history[idx].pwmValue;
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleAutotuneStart() {
  start_pid_autotune();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleMaintenanceReset() {
  usageCounters.resetMaintenanceCounter();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleMaintenanceInterval() {
  if (server.hasArg("hours")) {
    long hours = server.arg("hours").toInt();
    if (hours > 0) usageCounters.setServiceIntervalHours(static_cast<uint32_t>(hours));
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleMaintenancePage() {
  String html = R"=====(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Maintenance Logbook</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      * { box-sizing: border-box; min-width: 0; }
      html, body { max-width: 100%; overflow-x: hidden; }
      body { font-family: -apple-system, Arial, sans-serif; margin: 0; padding: 16px; background: #f8f9fa; color: #2c3e50; }
      h1 { font-size: 1.4em; }
      a { color: #007bff; text-decoration: none; }
      .card { background: #fff; border-radius: 10px; padding: 16px; box-shadow: 0 2px 4px rgba(0,0,0,0.08); margin-bottom: 12px; }
      table { width: 100%; border-collapse: collapse; }
      th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid #e9ecef; font-size: 0.9em; }
      th { color: #6c757d; font-weight: 600; }
      input[type=text] { width: 100%; padding: 8px; border: 1px solid #e9ecef; border-radius: 6px; margin-bottom: 8px; }
      button { background: #007bff; color: #fff; border: none; border-radius: 6px; padding: 8px 14px; cursor: pointer; }
      .empty { color: #6c757d; font-size: 0.9em; }
    </style>
  </head>
  <body>
    <a href="/">&larr; Back to dashboard</a>
    <h1>Maintenance Logbook</h1>

    <div class="card">
      <h2 style="font-size:1em;margin-top:0;">Log a maintenance event</h2>
      <input type="text" id="note" placeholder="e.g. Oil change, filter replaced" maxlength="39">
      <button onclick="addEntry()">Add entry</button>
    </div>

    <div class="card">
      <table>
        <thead><tr><th>Date</th><th>Hours</th><th>Note</th></tr></thead>
        <tbody id="rows"></tbody>
      </table>
      <div class="empty" id="empty" style="display:none;">No maintenance logged yet.</div>
    </div>

    <script>
      const Fmt = {
        date: epoch => epoch ? new Date(epoch * 1000).toLocaleDateString() : 'unknown date',
        hours: h => h + 'h',
      };

      // One log entry -> one <tr>. textContent for the note: safe against injected markup.
      function renderLogRow(entry) {
        const tr = document.createElement('tr');
        const dateCell = document.createElement('td');
        const hoursCell = document.createElement('td');
        const noteCell = document.createElement('td');
        dateCell.textContent = Fmt.date(entry.epoch);
        hoursCell.textContent = Fmt.hours(entry.hours);
        noteCell.textContent = entry.note;
        tr.append(dateCell, hoursCell, noteCell);
        return tr;
      }

      async function loadEntries() {
        const res = await fetch('/maintenance/log');
        const entries = await res.json();
        const rows = document.getElementById('rows');
        rows.innerHTML = '';
        document.getElementById('empty').style.display = entries.length ? 'none' : 'block';
        entries.slice().reverse().forEach(e => rows.appendChild(renderLogRow(e)));
      }

      async function addEntry() {
        const note = document.getElementById('note').value.trim();
        if (!note) return;
        await fetch('/maintenance/log/add?note=' + encodeURIComponent(note));
        document.getElementById('note').value = '';
        loadEntries();
      }

      loadEntries();
    </script>
  </body>
  </html>
  )=====";

  server.send(200, "text/html", html);
}

void handleMaintenanceLogList() {
  MaintenanceLogEntry entries[64];
  size_t count = maintenanceLog.getEntries(entries, 64);

  DynamicJsonDocument doc(4096);
  JsonArray array = doc.to<JsonArray>();
  for (size_t i = 0; i < count; i++) {
    JsonObject e = array.createNestedObject();
    e["epoch"] = entries[i].epochSeconds;
    e["hours"] = entries[i].runHours;
    e["note"] = entries[i].note;
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleMaintenanceLogAdd() {
  String note = server.hasArg("note") ? server.arg("note") : "";
  maintenanceLog.addEntry(current_epoch_or_zero(), usageCounters.totalRunSeconds() / 3600, note.c_str());
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleMqttTest() {
  mqttPublisher.forceReconnectNow();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleGpsEnable() {
  gps_https_start();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleGpsDisable() {
  gps_https_stop();
  if (server.hasArg("redirect")) {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleGpsStatus() {
  String json = String("{\"active\":") + (gps_https_active() ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleRestart() {
  server.send(200, "application/json", "{\"ok\":true}");
  server.client().flush();
  delay(100); // let the response actually go out before rebooting
  ESP.restart();
}
