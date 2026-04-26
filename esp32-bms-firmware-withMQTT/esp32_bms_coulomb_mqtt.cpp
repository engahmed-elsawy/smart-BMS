#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <math.h>

/*
  ESP32 BMS Firmware — Coulomb Counting + Live Web Dashboard + MQTT
  ==================================================================
  Features:
    - CD4051 8-channel MUX reading (6 cells + current + pack voltage)
    - Coulomb Counting SOC with voltage-based re-sync at rest
    - Automatic cell balancing
    - Live web dashboard with Chart.js graphs (SOC, V, I, cell voltages)
    - JSON API endpoints: /api/status  /api/history
    - MQTT publish every 5 s (status + per-cell topics)
    - MQTT subscribe: remote commands (balance on/off, auto balance)

  Required library: PubSubClient by Nick O'Leary
    Arduino IDE  → Sketch → Include Library → Manage Libraries → search "PubSubClient"
    PlatformIO   → lib_deps = knolleary/PubSubClient @ ^2.8

  MUX channel mapping (CD4051):
    CH0 -> ADC_C1   (cell 1 voltage)
    CH1 -> ADC_C2   (cell 2 voltage)
    CH2 -> ADC_C3   (cell 3 voltage)
    CH3 -> ADC_C4   (cell 4 voltage)
    CH4 -> ADC_C5   (cell 5 voltage)
    CH5 -> ADC_C6   (cell 6 voltage)
    CH6 -> ADC_ip   (pack current via shunt/sensor)
    CH7 -> ADC_vb   (pack voltage)

  MQTT topics published (prefix = MQTT_TOPIC_PREFIX):
    bms/status          → full JSON payload (same as /api/status)
    bms/soc             → SOC % as plain number
    bms/pack_voltage    → pack voltage V
    bms/pack_current    → pack current A
    bms/cell/1 … /6    → individual cell voltages
    bms/charged_ah      → cumulative Ah charged
    bms/discharged_ah   → cumulative Ah discharged

  MQTT topics subscribed (commands):
    bms/cmd/balance_auto   → payload "ON" / "OFF"
    bms/cmd/balance_all    → payload "OFF"
    bms/cmd/balance_set    → payload "cell:state"  e.g. "3:1" or "3:0"

  BEFORE FIRST USE — tune the values in USER CONFIG and CALIBRATION below.
*/

// =====================================================================
// USER CONFIG
// =====================================================================
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ── MQTT ──────────────────────────────────────────────────────────────
// Broker address: IP or hostname of your Mosquitto / HiveMQ / etc.
// Free public test broker (no account needed, not for production):
//   test.mosquitto.org  or  broker.hivemq.com
const char* MQTT_BROKER   = "192.168.1.100";   // ← change to your broker IP
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = "";                // leave empty if no auth
const char* MQTT_PASS_STR = "";
const char* MQTT_CLIENT_ID     = "esp32_bms_01";
const char* MQTT_TOPIC_PREFIX  = "bms";        // all topics start with this

// How often to publish (ms). Keep ≥ 1000 to avoid flooding the broker.
const uint32_t MQTT_PUBLISH_INTERVAL_MS = 5000;

// Set false to disable MQTT entirely (HTTP dashboard still works)
bool mqttEnabled = true;
// ─────────────────────────────────────────────────────────────────────

// GPIO pin assignments — change to match your actual wiring
static const int PIN_MUX_S0  = 25;
static const int PIN_MUX_S1  = 26;
static const int PIN_MUX_S2  = 27;
static const int PIN_MUX_OUT = 34;   // ADC1 channel only (GPIO 32-39)

static const int PIN_BAL_1 = 16;
static const int PIN_BAL_2 = 17;
static const int PIN_BAL_3 = 18;
static const int PIN_BAL_4 = 19;
static const int PIN_BAL_5 = 21;
static const int PIN_BAL_6 = 22;

static const int PIN_STATUS_LED = 2;   // set to -1 if unused

// ADC settings
static const uint8_t  ADC_SAMPLES_PER_READ = 16;
static const uint16_t MUX_SETTLE_MS        = 5;
static const float    ADC_VREF             = 3.30f;
static const int      ADC_MAX              = 4095;

// =====================================================================
// CALIBRATION  — measure on your real board and update these values
// =====================================================================

// Per-cell voltage divider scale: actual_cell_V = adc_V * cellScale[i]
float cellScale[6] = { 1.400f, 1.400f, 1.400f, 1.400f, 1.400f, 1.400f };

// Pack voltage divider scale: pack_V = adc_V * packScale
float packScale = 9.125f;

// Current sensor: I(A) = (adc_V - offset) * gain
// Example: ACS712-20A → offset=1.65V, gain=10.0 A/V
float currentOffsetV   = 1.650f;
float currentGainAperV = 10.000f;

// =====================================================================
// COULOMB COUNTING CONFIG
// =====================================================================

// Nominal pack capacity in Ah (e.g. 6S 18650 pack with 3× 3.5Ah cells = 10.5Ah)
float packCapacityAh = 10.0f;

// Coulomb efficiency factors (typical Li-ion values)
float chargeEfficiency    = 0.98f;
float dischargeEfficiency = 1.00f;

// Re-sync parameters: when |I| < threshold for restDuration ms,
// trust voltage-SOC to gently correct CC drift.
float    ccResyncCurrentThreshA  = 0.20f;
uint32_t ccResyncRestDurationMs  = 60000UL;   // 60 s

// =====================================================================
// SOC VOLTAGE LOOKUP TABLE  (per-cell voltage → SOC %)
// Calibrate for your specific cell chemistry
// =====================================================================
static const uint8_t SOC_TABLE_SIZE = 11;

float socVoltageTable[SOC_TABLE_SIZE] = {
  3.00f, 3.20f, 3.30f, 3.40f, 3.50f,
  3.60f, 3.70f, 3.80f, 3.95f, 4.10f, 4.20f
};
float socPercentTable[SOC_TABLE_SIZE] = {
   0.0f, 10.0f, 20.0f, 30.0f, 40.0f,
  50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f
};

bool  useMinCellForSocSafety = true;
float socLowClampCellV       = 3.00f;
float socHighClampCellV      = 4.20f;

// =====================================================================
// BALANCING CONFIG
// =====================================================================
bool  autoBalanceEnabled         = false;
float balanceStartVoltage        = 4.10f;   // start bleeding a cell above this V
float balanceStopDelta           = 0.03f;   // stop when max-min spread < this V
float minPackVoltageForBalancing = 20.0f;   // don't balance if pack too low

// =====================================================================
// HISTORY RING BUFFER  (for web graphs)
// =====================================================================
static const uint16_t HIST_SIZE = 200;

struct HistPoint {
  uint32_t t;
  float    packV;
  float    packI;
  float    soc;
  float    cell[6];
};

HistPoint histBuf[HIST_SIZE];
uint16_t  histHead  = 0;
uint16_t  histCount = 0;

// =====================================================================
// INTERNAL STATE
// =====================================================================
WebServer server(80);

// MQTT client
WiFiClient   wifiClientMqtt;
PubSubClient mqttClient(wifiClientMqtt);

struct BmsData {
  uint16_t raw[8];
  float    adcVoltage[8];
  float    cellVoltage[6];
  float    packVoltage;
  float    packCurrent;
  float    avgCellVoltage;
  float    minCellVoltage;
  float    maxCellVoltage;
  float    socPercent;       // fused SOC (Coulomb Counting + re-sync)
  float    socVoltage;       // pure voltage-SOC (reference / debug)
  float    chargedAh;        // total Ah pushed in since boot
  float    dischargedAh;     // total Ah taken out since boot
  bool     balanceState[6];
  unsigned long sampleCounter;
};

BmsData bms;

// Coulomb Counting internal state
bool     ccInitialized  = false;
float    ccSocPercent   = 50.0f;
uint32_t ccLastSampleMs = 0;
uint32_t ccRestStartMs  = 0;
bool     ccInRest       = false;

const int balancePins[6] = {
  PIN_BAL_1, PIN_BAL_2, PIN_BAL_3, PIN_BAL_4, PIN_BAL_5, PIN_BAL_6
};

const char* muxChannelNames[8] = {
  "ADC_C1", "ADC_C2", "ADC_C3", "ADC_C4",
  "ADC_C5", "ADC_C6", "ADC_ip", "ADC_vb"
};

unsigned long printIntervalMs = 2000;

// =====================================================================
// UTILITY HELPERS
// =====================================================================

void setMuxChannel(uint8_t ch) {
  digitalWrite(PIN_MUX_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S2, (ch & 0x04) ? HIGH : LOW);
}

uint16_t readMuxRaw(uint8_t ch) {
  setMuxChannel(ch);
  delay(MUX_SETTLE_MS);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES_PER_READ; i++) {
    sum += analogRead(PIN_MUX_OUT);
    delayMicroseconds(200);
  }
  return (uint16_t)(sum / ADC_SAMPLES_PER_READ);
}

float rawToVoltage(uint16_t raw) {
  return ((float)raw * ADC_VREF) / (float)ADC_MAX;
}

void setBalance(uint8_t idx, bool on) {
  if (idx >= 6) return;
  bms.balanceState[idx] = on;
  digitalWrite(balancePins[idx], on ? HIGH : LOW);
}

void allBalanceOff() {
  for (uint8_t i = 0; i < 6; i++) setBalance(i, false);
}

float getMaxCellVoltage() {
  float v = bms.cellVoltage[0];
  for (uint8_t i = 1; i < 6; i++)
    if (bms.cellVoltage[i] > v) v = bms.cellVoltage[i];
  return v;
}

float getMinCellVoltage() {
  float v = bms.cellVoltage[0];
  for (uint8_t i = 1; i < 6; i++)
    if (bms.cellVoltage[i] < v) v = bms.cellVoltage[i];
  return v;
}

float getAverageCellVoltage() {
  float s = 0.0f;
  for (uint8_t i = 0; i < 6; i++) s += bms.cellVoltage[i];
  return s / 6.0f;
}

float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// Linear interpolation through the SOC lookup table
float interpolateSocFromCellVoltage(float cellV) {
  cellV = clampf(cellV, socLowClampCellV, socHighClampCellV);
  if (cellV <= socVoltageTable[0]) return socPercentTable[0];
  for (uint8_t i = 1; i < SOC_TABLE_SIZE; i++) {
    if (cellV <= socVoltageTable[i]) {
      float x0 = socVoltageTable[i - 1];
      float x1 = socVoltageTable[i];
      float y0 = socPercentTable[i - 1];
      float y1 = socPercentTable[i];
      return y0 + (cellV - x0) / (x1 - x0) * (y1 - y0);
    }
  }
  return socPercentTable[SOC_TABLE_SIZE - 1];
}

// =====================================================================
// COULOMB COUNTING
// =====================================================================
/*
  Algorithm summary:
    1. First call: seed CC SOC from voltage table (no current history yet).
    2. Each subsequent call: integrate  dSOC = I * dt / capacity * 100
       - Positive I  → charging   → apply charge efficiency
       - Negative I  → discharging → divide by discharge efficiency
    3. After the pack rests (|I| < threshold) for ccResyncRestDurationMs,
       gently blend CC toward voltage-SOC (10% pull per cycle) to correct
       any accumulated integration error.
*/
void updateCoulombCounting() {
  uint32_t now = millis();

  // --- seed on first call ---
  if (!ccInitialized) {
    float vSocIn = bms.avgCellVoltage;
    if (useMinCellForSocSafety)
      vSocIn = min(bms.avgCellVoltage, bms.minCellVoltage + 0.03f);
    ccSocPercent    = interpolateSocFromCellVoltage(vSocIn);
    bms.socPercent  = ccSocPercent;
    ccLastSampleMs  = now;
    ccRestStartMs   = now;
    ccInitialized   = true;
    return;
  }

  // dt in hours
  float dtH = (float)(now - ccLastSampleMs) / 3600000.0f;
  ccLastSampleMs = now;

  float I = bms.packCurrent;

  if (I > 0.0f) {
    // Charging — apply charge efficiency (some energy lost as heat)
    float dAh = I * dtH * chargeEfficiency;
    bms.chargedAh += dAh;
    ccSocPercent  += (dAh / packCapacityAh) * 100.0f;
  } else if (I < 0.0f) {
    // Discharging — apply discharge efficiency factor
    float dAh = fabsf(I) * dtH / dischargeEfficiency;
    bms.dischargedAh += dAh;
    ccSocPercent     -= (dAh / packCapacityAh) * 100.0f;
  }

  ccSocPercent = clampf(ccSocPercent, 0.0f, 100.0f);

  // --- voltage re-sync at rest ---
  bool isResting = (fabsf(I) < ccResyncCurrentThreshA);
  if (isResting) {
    if (!ccInRest) {
      ccInRest      = true;
      ccRestStartMs = now;
    }
    if ((now - ccRestStartMs) >= ccResyncRestDurationMs) {
      float vSocIn = bms.avgCellVoltage;
      if (useMinCellForSocSafety)
        vSocIn = min(bms.avgCellVoltage, bms.minCellVoltage + 0.03f);
      float voltSoc = interpolateSocFromCellVoltage(vSocIn);
      // Soft blend: 90% CC + 10% voltage — corrects drift without jumping
      ccSocPercent = ccSocPercent * 0.90f + voltSoc * 0.10f;
    }
  } else {
    ccInRest = false;
  }

  bms.socPercent = ccSocPercent;

  // Update voltage-SOC reference (for display)
  float vSocIn = bms.avgCellVoltage;
  if (useMinCellForSocSafety)
    vSocIn = min(bms.avgCellVoltage, bms.minCellVoltage + 0.03f);
  bms.socVoltage = interpolateSocFromCellVoltage(vSocIn);
}

// =====================================================================
// AUTO BALANCING
// =====================================================================
void updateAutoBalancing() {
  if (!autoBalanceEnabled) { allBalanceOff(); return; }
  if (bms.packVoltage < minPackVoltageForBalancing) { allBalanceOff(); return; }

  float maxCell = getMaxCellVoltage();
  float minCell = getMinCellVoltage();

  if ((maxCell - minCell) < balanceStopDelta) { allBalanceOff(); return; }

  for (uint8_t i = 0; i < 6; i++) {
    bool en = (bms.cellVoltage[i] >= balanceStartVoltage) &&
              ((bms.cellVoltage[i] - minCell) >= balanceStopDelta);
    setBalance(i, en);
  }
}

// =====================================================================
// SAMPLING — reads all 8 MUX channels and computes all values
// =====================================================================
void sampleAllChannels() {
  for (uint8_t ch = 0; ch < 8; ch++) {
    bms.raw[ch]        = readMuxRaw(ch);
    bms.adcVoltage[ch] = rawToVoltage(bms.raw[ch]);
  }

  for (uint8_t i = 0; i < 6; i++)
    bms.cellVoltage[i] = bms.adcVoltage[i] * cellScale[i];

  bms.packCurrent    = (bms.adcVoltage[6] - currentOffsetV) * currentGainAperV;
  bms.packVoltage    = bms.adcVoltage[7] * packScale;
  bms.minCellVoltage = getMinCellVoltage();
  bms.maxCellVoltage = getMaxCellVoltage();
  bms.avgCellVoltage = getAverageCellVoltage();

  updateCoulombCounting();
  updateAutoBalancing();
  bms.sampleCounter++;

  // Push to history ring buffer
  histBuf[histHead] = {
    millis(),
    bms.packVoltage,
    bms.packCurrent,
    bms.socPercent,
    { bms.cellVoltage[0], bms.cellVoltage[1], bms.cellVoltage[2],
      bms.cellVoltage[3], bms.cellVoltage[4], bms.cellVoltage[5] }
  };
  histHead  = (histHead + 1) % HIST_SIZE;
  if (histCount < HIST_SIZE) histCount++;
}

// =====================================================================
// JSON BUILDERS
// =====================================================================
String jsonStatus() {
  String s = "{";
  s += "\"samples\":"       + String(bms.sampleCounter)      + ",";
  s += "\"pack_voltage\":"  + String(bms.packVoltage, 3)     + ",";
  s += "\"pack_current\":"  + String(bms.packCurrent, 3)     + ",";
  s += "\"avg_cell_v\":"    + String(bms.avgCellVoltage, 3)  + ",";
  s += "\"min_cell_v\":"    + String(bms.minCellVoltage, 3)  + ",";
  s += "\"max_cell_v\":"    + String(bms.maxCellVoltage, 3)  + ",";
  s += "\"soc_cc\":"        + String(bms.socPercent, 2)      + ",";
  s += "\"soc_v\":"         + String(bms.socVoltage, 2)      + ",";
  s += "\"charged_ah\":"    + String(bms.chargedAh, 4)       + ",";
  s += "\"discharged_ah\":" + String(bms.dischargedAh, 4)    + ",";

  s += "\"cell_voltages\":[";
  for (uint8_t i = 0; i < 6; i++) {
    s += String(bms.cellVoltage[i], 3);
    if (i < 5) s += ",";
  }
  s += "],\"balance\":[";
  for (uint8_t i = 0; i < 6; i++) {
    s += bms.balanceState[i] ? "true" : "false";
    if (i < 5) s += ",";
  }
  s += "],\"adc_raw\":[";
  for (uint8_t i = 0; i < 8; i++) {
    s += String(bms.raw[i]);
    if (i < 7) s += ",";
  }
  s += "]}";
  return s;
}

// Returns last HIST_SIZE samples as parallel arrays suitable for Chart.js
String jsonHistory() {
  uint16_t start = (histCount < HIST_SIZE) ? 0 : histHead;

  String s = "{\"t\":[";
  for (uint16_t k = 0; k < histCount; k++) {
    uint16_t idx = (start + k) % HIST_SIZE;
    s += String(histBuf[idx].t);
    if (k < histCount - 1) s += ",";
  }
  s += "],\"packV\":[";
  for (uint16_t k = 0; k < histCount; k++) {
    uint16_t idx = (start + k) % HIST_SIZE;
    s += String(histBuf[idx].packV, 2);
    if (k < histCount - 1) s += ",";
  }
  s += "],\"packI\":[";
  for (uint16_t k = 0; k < histCount; k++) {
    uint16_t idx = (start + k) % HIST_SIZE;
    s += String(histBuf[idx].packI, 2);
    if (k < histCount - 1) s += ",";
  }
  s += "],\"soc\":[";
  for (uint16_t k = 0; k < histCount; k++) {
    uint16_t idx = (start + k) % HIST_SIZE;
    s += String(histBuf[idx].soc, 1);
    if (k < histCount - 1) s += ",";
  }
  s += "],\"cells\":[";
  for (uint8_t c = 0; c < 6; c++) {
    s += "[";
    for (uint16_t k = 0; k < histCount; k++) {
      uint16_t idx = (start + k) % HIST_SIZE;
      s += String(histBuf[idx].cell[c], 3);
      if (k < histCount - 1) s += ",";
    }
    s += "]";
    if (c < 5) s += ",";
  }
  s += "]}";
  return s;
}

// =====================================================================
// WEB DASHBOARD  (served at /)
// Live-updating page with Chart.js — no page reload needed.
// Polls /api/status every second.
// =====================================================================
void handleRoot() {
  String html = R"rawhtml(
<!DOCTYPE html><html><head>
<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>ESP32 BMS Dashboard</title>
<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js'></script>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:sans-serif;background:#0f172a;color:#e2e8f0;padding:16px}
  h1{font-size:18px;font-weight:500;margin-bottom:16px;color:#94a3b8}
  .cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin-bottom:20px}
  .card{background:#1e293b;border-radius:10px;padding:12px 14px}
  .card .lbl{font-size:11px;color:#64748b;margin-bottom:4px}
  .card .val{font-size:22px;font-weight:500;color:#f1f5f9}
  .card .unit{font-size:12px;color:#94a3b8;margin-left:2px}
  .charts{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:20px}
  @media(max-width:600px){.charts{grid-template-columns:1fr}}
  .chart-box{background:#1e293b;border-radius:10px;padding:14px}
  .chart-box h3{font-size:12px;color:#64748b;margin-bottom:10px;font-weight:400}
  .cells{display:grid;grid-template-columns:repeat(6,1fr);gap:8px;margin-bottom:16px}
  .cell{background:#1e293b;border-radius:8px;padding:10px 6px;text-align:center}
  .cell .cv{font-size:15px;font-weight:500}
  .cell .cl{font-size:10px;color:#64748b;margin-top:2px}
  .cell.bal{border:1px solid #f59e0b}
  .soc-bar{height:14px;background:#0f172a;border-radius:7px;overflow:hidden;margin-top:6px}
  .soc-fill{height:100%;border-radius:7px;transition:width .4s}
  .footer{font-size:11px;color:#475569;text-align:center;margin-top:8px}
  .ctrl{display:flex;gap:8px;margin-bottom:16px;flex-wrap:wrap}
  .btn{background:#1e293b;border:1px solid #334155;color:#94a3b8;padding:6px 14px;border-radius:6px;cursor:pointer;font-size:12px}
  .btn:hover{background:#334155}
  .btn.on{border-color:#22c55e;color:#22c55e}
</style>
</head><body>
<h1>ESP32 BMS — Live Dashboard</h1>

<div class='cards'>
  <div class='card'><div class='lbl'>Pack voltage</div><div class='val' id='pv'>—<span class='unit'>V</span></div></div>
  <div class='card'><div class='lbl'>Current</div><div class='val' id='pi'>—<span class='unit'>A</span></div></div>
  <div class='card'>
    <div class='lbl'>SOC — Coulomb</div>
    <div class='val' id='soc'>—<span class='unit'>%</span></div>
    <div class='soc-bar'><div class='soc-fill' id='socbar' style='width:0%;background:#22c55e'></div></div>
  </div>
  <div class='card'><div class='lbl'>SOC — voltage</div><div class='val' id='socv'>—<span class='unit'>%</span></div></div>
  <div class='card'><div class='lbl'>Charged</div><div class='val' id='chAh'>—<span class='unit'>Ah</span></div></div>
  <div class='card'><div class='lbl'>Discharged</div><div class='val' id='dcAh'>—<span class='unit'>Ah</span></div></div>
</div>

<div class='ctrl'>
  <button class='btn' id='btnBalOff'  onclick="fetch('/balance/off')">All balance OFF</button>
  <button class='btn' id='btnBalAuto' onclick="toggleAutoBalance()">Auto balance</button>
  <span style='font-size:11px;color:#475569;align-self:center' id='balStatus'></span>
</div>

<div class='cells' id='cellGrid'></div>

<div class='charts'>
  <div class='chart-box'><h3>SOC % (Coulomb counting)</h3><div style='position:relative;height:160px'><canvas id='cSOC' role='img' aria-label='SOC over time'>SOC chart</canvas></div></div>
  <div class='chart-box'><h3>Pack voltage (V)</h3><div style='position:relative;height:160px'><canvas id='cV' role='img' aria-label='Pack voltage over time'>Voltage chart</canvas></div></div>
  <div class='chart-box'><h3>Pack current (A)</h3><div style='position:relative;height:160px'><canvas id='cI' role='img' aria-label='Pack current over time'>Current chart</canvas></div></div>
  <div class='chart-box'><h3>Cell voltages (V)</h3><div style='position:relative;height:160px'><canvas id='cCells' role='img' aria-label='Individual cell voltages'>Cell voltage chart</canvas></div></div>
</div>

<div class='footer' id='ft'>Connecting…</div>

<script>
const COLORS=['#38bdf8','#4ade80','#fb923c','#f472b6','#a78bfa','#facc15'];
const MAX_PTS=120;
let autoBalOn=false;

function mkChart(id,label,color,yMin,yMax){
  return new Chart(document.getElementById(id),{
    type:'line',
    data:{labels:[],datasets:[{label,data:[],borderColor:color,borderWidth:1.5,pointRadius:0,fill:false,tension:0.3}]},
    options:{responsive:true,maintainAspectRatio:false,animation:false,
      scales:{x:{display:false},y:{min:yMin??undefined,max:yMax??undefined,
        ticks:{color:'#94a3b8',font:{size:10}},grid:{color:'rgba(255,255,255,.05)'}}},
      plugins:{legend:{display:false}}}
  });
}

function mkCellChart(id){
  return new Chart(document.getElementById(id),{
    type:'line',
    data:{labels:[],datasets:COLORS.map((c,i)=>({label:'C'+(i+1),data:[],borderColor:c,borderWidth:1.2,pointRadius:0,fill:false,tension:0.3}))},
    options:{responsive:true,maintainAspectRatio:false,animation:false,
      scales:{x:{display:false},y:{ticks:{color:'#94a3b8',font:{size:10}},grid:{color:'rgba(255,255,255,.05)'}}},
      plugins:{legend:{labels:{color:'#94a3b8',font:{size:10},boxWidth:8}}}}
  });
}

const socChart=mkChart('cSOC','SOC','#22c55e',0,100);
const vChart  =mkChart('cV','V','#38bdf8',null,null);
const iChart  =mkChart('cI','A','#fb923c',null,null);
const cellChart=mkCellChart('cCells');

function pushLine(chart,label,...vals){
  chart.data.labels.push(label);
  if(chart.data.labels.length>MAX_PTS) chart.data.labels.shift();
  vals.forEach((v,i)=>{
    chart.data.datasets[i].data.push(v);
    if(chart.data.datasets[i].data.length>MAX_PTS) chart.data.datasets[i].data.shift();
  });
  chart.update('none');
}

function socColor(s){ return s>50?'#22c55e':s>20?'#f59e0b':'#ef4444'; }

function toggleAutoBalance(){
  autoBalOn=!autoBalOn;
  fetch('/balance/auto/'+(autoBalOn?'on':'off'));
  document.getElementById('btnBalAuto').className='btn'+(autoBalOn?' on':'');
}

let lastSamples=0;
async function fetchStatus(){
  try{
    const r=await fetch('/api/status');
    const d=await r.json();
    const t=new Date().toLocaleTimeString();

    document.getElementById('pv').childNodes[0].textContent=d.pack_voltage.toFixed(2);
    document.getElementById('pi').childNodes[0].textContent=d.pack_current.toFixed(2);
    const s=parseFloat(d.soc_cc);
    document.getElementById('soc').childNodes[0].textContent=s.toFixed(1);
    document.getElementById('socv').childNodes[0].textContent=parseFloat(d.soc_v).toFixed(1);
    document.getElementById('chAh').childNodes[0].textContent=parseFloat(d.charged_ah).toFixed(3);
    document.getElementById('dcAh').childNodes[0].textContent=parseFloat(d.discharged_ah).toFixed(3);

    const bar=document.getElementById('socbar');
    bar.style.width=s.toFixed(1)+'%';
    bar.style.background=socColor(s);

    const grid=document.getElementById('cellGrid');
    if(grid.children.length===0){
      for(let i=0;i<6;i++){
        const c=document.createElement('div');
        c.id='cell'+i;c.className='cell';
        c.innerHTML=`<div class='cv' id='cv${i}'>—</div><div class='cl'>Cell ${i+1}</div>`;
        grid.appendChild(c);
      }
    }
    d.cell_voltages.forEach((v,i)=>{
      document.getElementById('cv'+i).textContent=v.toFixed(3);
      document.getElementById('cell'+i).className='cell'+(d.balance[i]?' bal':'');
    });

    if(d.samples!==lastSamples){
      pushLine(socChart,t,s);
      pushLine(vChart,t,d.pack_voltage);
      pushLine(iChart,t,d.pack_current);
      cellChart.data.labels.push(t);
      if(cellChart.data.labels.length>MAX_PTS) cellChart.data.labels.shift();
      d.cell_voltages.forEach((v,i)=>{
        cellChart.data.datasets[i].data.push(v);
        if(cellChart.data.datasets[i].data.length>MAX_PTS) cellChart.data.datasets[i].data.shift();
      });
      cellChart.update('none');
      lastSamples=d.samples;
    }

    document.getElementById('ft').textContent='Updated '+t+' | Samples: '+d.samples;
  }catch(e){
    document.getElementById('ft').textContent='Connection error — retrying…';
  }
}

fetchStatus();
setInterval(fetchStatus,1000);
</script>
</body></html>
)rawhtml";
  server.send(200, "text/html", html);
}

// =====================================================================
// HTTP ROUTE HANDLERS
// =====================================================================
void handleApiStatus() {
  server.send(200, "application/json", jsonStatus());
}

void handleApiHistory() {
  server.send(200, "application/json", jsonHistory());
}

void handleAllBalanceOff() {
  autoBalanceEnabled = false;
  allBalanceOff();
  server.send(200, "text/plain", "All balancing OFF");
}

void handleAutoOn() {
  autoBalanceEnabled = true;
  server.send(200, "text/plain", "Auto balancing ON");
}

void handleAutoOff() {
  autoBalanceEnabled = false;
  allBalanceOff();
  server.send(200, "text/plain", "Auto balancing OFF");
}

void handleBalanceManual() {
  if (!server.hasArg("cell") || !server.hasArg("state")) {
    server.send(400, "text/plain", "Usage: /balance/set?cell=1..6&state=0|1");
    return;
  }
  int cell  = server.arg("cell").toInt();
  int state = server.arg("state").toInt();
  if (cell < 1 || cell > 6) { server.send(400, "text/plain", "cell must be 1..6"); return; }
  autoBalanceEnabled = false;
  setBalance(cell - 1, state != 0);
  server.send(200, "text/plain", "OK");
}

// =====================================================================
// SERIAL REPORT
// =====================================================================
void printSerialReport() {
  Serial.println("====================================================");
  Serial.printf("Samples : %lu\n", bms.sampleCounter);
  Serial.printf("Pack    : %.3f V | %.3f A\n", bms.packVoltage, bms.packCurrent);
  Serial.printf("SOC (CC): %.2f %%  |  SOC (V): %.2f %%\n", bms.socPercent, bms.socVoltage);
  Serial.printf("Charged : %.4f Ah  |  Discharged: %.4f Ah\n", bms.chargedAh, bms.dischargedAh);
  for (uint8_t i = 0; i < 6; i++) {
    Serial.printf("Cell %u  : %.3f V  BAL=%s\n",
                  i + 1, bms.cellVoltage[i],
                  bms.balanceState[i] ? "ON" : "OFF");
  }
  Serial.println("Raw ADC:");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.printf("  CH%u (%s): raw=%u  adc=%.4f V\n",
                  i, muxChannelNames[i], bms.raw[i], bms.adcVoltage[i]);
  }
}

// =====================================================================
// MQTT
// =====================================================================

// Build a full topic string:  "bms/subtopic"
String mqttTopic(const char* sub) {
  return String(MQTT_TOPIC_PREFIX) + "/" + sub;
}

// Publish a single float value as a plain string
void mqttPublishFloat(const char* sub, float val, int decimals = 3) {
  if (!mqttClient.connected()) return;
  mqttClient.publish(mqttTopic(sub).c_str(), String(val, decimals).c_str(), true);
}

// Publish the full JSON status payload
void mqttPublishStatus() {
  if (!mqttClient.connected()) return;
  String payload = jsonStatus();
  mqttClient.publish(mqttTopic("status").c_str(), payload.c_str(), false);
}

// Publish individual topics (useful for Home Assistant auto-discovery)
void mqttPublishAll() {
  mqttPublishStatus();
  mqttPublishFloat("soc",           bms.socPercent,    1);
  mqttPublishFloat("pack_voltage",  bms.packVoltage,   3);
  mqttPublishFloat("pack_current",  bms.packCurrent,   3);
  mqttPublishFloat("charged_ah",    bms.chargedAh,     4);
  mqttPublishFloat("discharged_ah", bms.dischargedAh,  4);

  for (uint8_t i = 0; i < 6; i++) {
    String sub = "cell/" + String(i + 1);
    mqttClient.publish(mqttTopic(sub.c_str()).c_str(),
                       String(bms.cellVoltage[i], 3).c_str(), true);
  }
}

/*
  MQTT command callback — handles incoming messages on subscribed topics.

  bms/cmd/balance_auto  → "ON"  enable auto balance
                        → "OFF" disable
  bms/cmd/balance_all   → "OFF" force all channels off
  bms/cmd/balance_set   → "N:S" e.g. "3:1" to turn cell 3 on, "3:0" off
*/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  String topicStr(topic);

  if (topicStr == mqttTopic("cmd/balance_auto")) {
    if (msg == "ON") {
      autoBalanceEnabled = true;
      Serial.println("[MQTT] Auto balance ON");
    } else {
      autoBalanceEnabled = false;
      allBalanceOff();
      Serial.println("[MQTT] Auto balance OFF");
    }

  } else if (topicStr == mqttTopic("cmd/balance_all")) {
    if (msg == "OFF") {
      autoBalanceEnabled = false;
      allBalanceOff();
      Serial.println("[MQTT] All balance OFF");
    }

  } else if (topicStr == mqttTopic("cmd/balance_set")) {
    // Expect format "cell:state"  e.g. "3:1"
    int sep = msg.indexOf(':');
    if (sep > 0) {
      int cell  = msg.substring(0, sep).toInt();
      int state = msg.substring(sep + 1).toInt();
      if (cell >= 1 && cell <= 6) {
        autoBalanceEnabled = false;
        setBalance(cell - 1, state != 0);
        Serial.printf("[MQTT] Balance cell %d → %s\n", cell, state ? "ON" : "OFF");
      }
    }
  }
}

// Connect / reconnect to broker (non-blocking attempt)
bool mqttReconnect() {
  if (!mqttEnabled) return false;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (mqttClient.connected()) return true;

  Serial.print("[MQTT] Connecting to ");
  Serial.print(MQTT_BROKER);
  Serial.print("…");

  bool ok;
  if (strlen(MQTT_USER) > 0) {
    ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS_STR);
  } else {
    ok = mqttClient.connect(MQTT_CLIENT_ID);
  }

  if (ok) {
    Serial.println(" connected.");
    // Subscribe to command topics
    mqttClient.subscribe(mqttTopic("cmd/balance_auto").c_str());
    mqttClient.subscribe(mqttTopic("cmd/balance_all").c_str());
    mqttClient.subscribe(mqttTopic("cmd/balance_set").c_str());
    Serial.println("[MQTT] Subscribed to cmd topics.");
  } else {
    Serial.printf(" failed (rc=%d) — will retry.\n", mqttClient.state());
  }
  return ok;
}

void setupMqtt() {
  if (!mqttEnabled) return;
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);   // larger buffer for JSON status payload
  mqttReconnect();
}

// =====================================================================
// WIFI SETUP
  if (String(WIFI_SSID) == "YOUR_WIFI_SSID") {
    Serial.println("Wi-Fi not configured — running offline.");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected — IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi timeout — continuing offline.");
  }
}

void setupServer() {
  server.on("/",                  handleRoot);
  server.on("/api/status",        handleApiStatus);
  server.on("/api/history",       handleApiHistory);
  server.on("/balance/off",       handleAllBalanceOff);
  server.on("/balance/auto/on",   handleAutoOn);
  server.on("/balance/auto/off",  handleAutoOff);
  server.on("/balance/set",       handleBalanceManual);
  server.begin();
  Serial.println("HTTP server started.");
}

void setupPins() {
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(balancePins[i], OUTPUT);
    digitalWrite(balancePins[i], LOW);
    bms.balanceState[i] = false;
  }
  if (PIN_STATUS_LED >= 0) {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
  }
}

void setupAdc() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

// =====================================================================
// MAIN
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ESP32 BMS — Coulomb Counting + Live Dashboard + MQTT — starting…");

  setupPins();
  setupAdc();
  allBalanceOff();
  setupWiFi();
  setupServer();
  setupMqtt();

  // First sample seeds Coulomb Counting from voltage
  sampleAllChannels();
  Serial.printf("CC SOC seeded at %.1f %% from voltage.\n", bms.socPercent);
  Serial.println("Ready.");
}

void loop() {
  static unsigned long lastSample = 0;
  static unsigned long lastPrint  = 0;
  static unsigned long lastMqtt   = 0;
  static unsigned long lastMqttReconnect = 0;
  static bool ledState = false;

  unsigned long now = millis();

  // ── Sample ADC every 300 ms ────────────────────────────────────────
  if (now - lastSample >= 300) {
    lastSample = now;
    sampleAllChannels();
    if (PIN_STATUS_LED >= 0) {
      ledState = !ledState;
      digitalWrite(PIN_STATUS_LED, ledState ? HIGH : LOW);
    }
  }

  // ── Serial report ──────────────────────────────────────────────────
  if (now - lastPrint >= printIntervalMs) {
    lastPrint = now;
    printSerialReport();
  }

  // ── MQTT reconnect (every 10 s if disconnected) ────────────────────
  if (mqttEnabled && !mqttClient.connected()) {
    if (now - lastMqttReconnect >= 10000) {
      lastMqttReconnect = now;
      mqttReconnect();
    }
  }

  // ── MQTT publish ───────────────────────────────────────────────────
  if (mqttEnabled && mqttClient.connected()) {
    if (now - lastMqtt >= MQTT_PUBLISH_INTERVAL_MS) {
      lastMqtt = now;
      mqttPublishAll();
    }
    mqttClient.loop();   // process incoming commands
  }

  // ── HTTP server ────────────────────────────────────────────────────
  server.handleClient();
}
