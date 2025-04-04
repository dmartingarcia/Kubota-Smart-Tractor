#include "WebHandler.h"

ESP8266WebServer server(80);
extern DataPoint history[];
extern byte historyIndex;
extern bool relay_state;
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
      body { font-family: Arial; padding: 20px; }
      .status-box { padding: 20px; margin: 10px; background: #f0f0f0; border-radius: 5px; }
      #chartContainer { max-width: 800px; margin: 20px 0; }
      .status-label { font-weight: bold; color: #444; }
      .status-value { color: #2c3e50; }
      .engine-running { color: #27ae60; }
      .engine-stopped { color: #c0392b; }
    </style>
  </head>
  <body>
    <h1>Tractor Battery Monitor</h1>

    <div class="status-box">
      <h2>Current Status</h2>
      <p class="status-label">Voltage: <span class="status-value" id="voltage">--</span> V</p>
      <p class="status-label">Alternator: <span class="status-value" id="relayState">--</span></p>
      <p class="status-label">Engine: <span class="status-value" id="engineStatus">--</span></p>
    </div>

    <div id="chartContainer">
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
            borderColor: 'rgb(75, 192, 192)',
            tension: 0.1
          }]
        },
        options: {
          responsive: true,
          scales: {
            y: {
              min: 12,
              max: 16
            }
          }
        }
      });

      function updateStatusColors() {
        const engineElem = document.getElementById('engineStatus');
        engineElem.classList.remove('engine-running', 'engine-stopped');
        engineElem.classList.add(
          engineElem.textContent === 'RUNNING' ? 'engine-running' : 'engine-stopped'
        );
      }

      function updateData() {
        fetch('/data')
          .then(r => r.json())
          .then(data => {
            document.getElementById('voltage').textContent = data.voltage.toFixed(2);
            document.getElementById('relayState').textContent =
              data.relayState ? "ACTIVE" : "INACTIVE";
            document.getElementById('engineStatus').textContent =
              data.engineRunning ? "RUNNING" : "STOPPED";
            updateStatusColors();
          });

        fetch('/history')
          .then(r => r.json())
          .then(history => {
            chart.data.labels = history.map(p => new Date(p.timestamp).toLocaleTimeString());
            chart.data.datasets[0].data = history.map(p => p.voltage);
            chart.update();
          });
      }

      // Update every 2 seconds
      setInterval(updateData, 2000);
      updateData();
    </script>
  </body>
  </html>
  )=====";

  server.send(200, "text/html", html);
}

void handleData() {
  DynamicJsonDocument doc(128);
  doc["voltage"] = round(history[(historyIndex + 119) % 120].voltage * 100) / 100.0;
  doc["relayState"] = relay_state;
  doc["engineRunning"] = engine_running;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleHistory() {
  DynamicJsonDocument doc(2500);
  JsonArray array = doc.to<JsonArray>();

  for(int i = 0; i < 120; i++) {
    int idx = (historyIndex + i) % 120;
    if(history[idx].timestamp == 0) continue;

    JsonObject obj = array.createNestedObject();
    obj["timestamp"] = history[idx].timestamp;
    obj["voltage"] = round(history[idx].voltage * 100) / 100.0;
    obj["relayState"] = history[idx].relayState;
    obj["engineRunning"] = history[idx].engineRunning;
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}