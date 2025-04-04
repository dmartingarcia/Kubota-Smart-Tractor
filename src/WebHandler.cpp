#include "WebHandler.h"
#include <ArduinoJson.h>
#include "output_component.h"

ESP8266WebServer server(80);
extern OutputComponent alternator;
extern DataPoint history[];
extern byte historyIndex;
extern bool engine_running;

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/history", handleHistory);
  server.begin();
}

void handleRoot() {
  String html = R"=====(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Tractor Monitor</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
      body { font-family: Arial; padding: 20px; background: #f8f9fa; }
      .status-box {
        padding: 20px;
        margin: 10px;
        background: white;
        border-radius: 10px;
        box-shadow: 0 2px 4px rgba(0,0,0,0.1);
      }
      #chartContainer { max-width: 800px; margin: 20px 0; }
      .status-label { font-weight: bold; color: #6c757d; }
      .status-value { color: #2c3e50; }
      .mode-pwm { color: #28a745; }
      .mode-relay { color: #ffc107; }
      .pwm-bar {
        width: 100%;
        height: 20px;
        background: #e9ecef;
        border-radius: 10px;
        overflow: hidden;
        margin: 10px 0;
      }
      .pwm-fill {
        height: 100%;
        background: #007bff;
        transition: width 0.5s ease;
      }
      .pwm-labels {
        display: flex;
        justify-content: space-between;
        color: #6c757d;
        font-size: 0.9em;
      }
      .engine-status {
        padding: 8px 12px;
        border-radius: 20px;
        font-weight: bold;
      }
      .engine-running { background: #d4edda; color: #155724; }
      .engine-stopped { background: #f8d7da; color: #721c24; }
    </style>
  </head>
  <body>
    <h1 style="color: #2c3e50;">Tractor Battery Monitor</h1>

    <div class="status-box">
      <h2 style="color: #495057; margin-top: 0;">Current Status</h2>

      <div class="status-item">
        <span class="status-label">Voltage:</span>
        <span class="status-value" id="voltage">--</span> V
      </div>

      <div class="status-item">
        <span class="status-label">Output Mode:</span>
        <span id="modeIndicator">
          <span class="mode-pwm" style="display:none;">PWM (<span id="pwmValue">0</span>%)</span>
          <span class="mode-relay" style="display:none;">RELAY</span>
        </span>
      </div>

      <div id="pwmContainer" style="display: none;">
        <div class="pwm-bar">
          <div class="pwm-fill" style="width: 0%;"></div>
        </div>
        <div class="pwm-labels">
          <span>0%</span>
          <span>100%</span>
        </div>
      </div>

      <div class="status-item">
        <span class="status-label">Engine Status:</span>
        <span class="engine-status" id="engineStatus">--</span>
      </div>
    </div>

    <div class="status-box">
      <h2 style="color: #495057;">Voltage History</h2>
      <canvas id="voltageChart"></canvas>
    </div>

    <script>
      const ctx = document.getElementById('voltageChart').getContext('2d');
      const chart = new Chart(ctx, {
        type: 'line',
        data: {
          labels: [],
          datasets: [{
            label: 'Battery Voltage',
            data: [],
            borderColor: '#007bff',
            backgroundColor: 'rgba(0,123,255,0.1)',
            tension: 0.3,
            fill: true
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            y: {
              min: 12,
              max: 16,
              grid: { color: '#e9ecef' },
              ticks: { color: '#6c757d' }
            },
            x: {
              grid: { display: false },
              ticks: { color: '#6c757d' }
            }
          },
          plugins: {
            tooltip: {
              callbacks: {
                title: (items) => new Date(items[0].raw.x).toLocaleTimeString(),
                label: (item) => `${item.raw.y}V (${item.raw.mode})`
              }
            }
          }
        }
      });

      function updateStatus(data) {
        // Update voltage
        document.getElementById('voltage').textContent = data.voltage.toFixed(2);

        // Update output mode
        const isPWM = data.outputMode === 'pwm';
        document.querySelector('.mode-pwm').style.display = isPWM ? 'inline' : 'none';
        document.querySelector('.mode-relay').style.display = isPWM ? 'none' : 'inline';

        // Update PWM elements
        document.getElementById('pwmContainer').style.display = isPWM ? 'block' : 'none';
        if(isPWM) {
          document.getElementById('pwmValue').textContent = data.pwmPercentage;
          document.querySelector('.pwm-fill').style.width = `${data.pwmPercentage}%`;
        }

        // Update engine status
        const engineElem = document.getElementById('engineStatus');
        engineElem.textContent = data.engineRunning ? 'RUNNING' : 'STOPPED';
        engineElem.className = `engine-status ${data.engineRunning ? 'engine-running' : 'engine-stopped'}`;
      }

      function updateChart(history) {
        chart.data.labels = history.map(p => new Date(p.timestamp));
        chart.data.datasets[0].data = history.map(p => ({
          x: p.timestamp,
          y: p.voltage,
          mode: p.mode
        }));
        chart.update();
      }

      async function fetchData() {
        try {
          const [statusRes, historyRes] = await Promise.all([
            fetch('/data'),
            fetch('/history')
          ]);

          const statusData = await statusRes.json();
          const historyData = await historyRes.json();

          updateStatus(statusData);
          updateChart(historyData);
        } catch(error) {
          console.error('Update failed:', error);
        }
      }

      // Update every 2 seconds
      setInterval(fetchData, 2000);
      fetchData();
    </script>
  </body>
  </html>
  )=====";

  server.send(200, "text/html", html);
}

void handleData() {
  DynamicJsonDocument doc(256);
  doc["voltage"] = history[(historyIndex + 119) % 120].voltage;
  doc["outputMode"] = alternator.isPWMEnabled() ? "pwm" : "relay";
  doc["pwmPercentage"] = alternator.getPWMPercent();
  doc["engineRunning"] = engine_running;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleHistory() {
  DynamicJsonDocument doc(4096);
  JsonArray array = doc.to<JsonArray>();

  for(int i = 0; i < 120; i++) {
    int idx = (historyIndex + i) % 120;
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