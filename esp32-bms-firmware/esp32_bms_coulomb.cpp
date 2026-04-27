#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

/*
  ESP32 BMS Firmware — v1 — Coulomb Counting + Live Web Dashboard
  ================================================================
  Hardware: as per schematic (Ahmed Hatem Elsawy — 27/4/2026)
    - CD4051BM  8-ch MUX  (S0=IO25, S1=IO26, S2=IO27, OUT→GPIO34)
    - LM358P    differential cell voltage buffers (gain=0.5 per cell)
    - LM358P    pack voltage buffer  (R49=13K / R51=1.6K divider)
    - ACS712-30A current sensor, VCC=5V, through LM358 gain-2 amp
      + external 18K/33K voltage divider to protect ESP32 ADC
    - IRLZ44N   balancing MOSFETs via optocouplers

  Current sensor signal chain:
    ACS712-30A (VCC=5V) → zero=2.5V, sensitivity=66mV/A
    → LM358 gain-2  → zero=5.0V, gain=132mV/A
    → 18K/33K divider (×0.647) → zero=3.235V, gain=85.4mV/A
    → ESP32 ADC (safe ≤3.3V)
    currentGainAperV = 1 / 0.0854 = 11.7 A/V

  Cell voltage signal chain:
    Cell differential voltage → LM358 unity-gain buffer
    → 1M/1M output divider → ADC_Cx
    cellScale = 2.0  (ADC reads half the real cell voltage)

  MUX channel mapping:
    CH0=ADC_C1, CH1=ADC_C2, CH2=ADC_C3, CH3=ADC_C4,
    CH4=ADC_C5, CH5=ADC_C6, CH6=ADC_ip, CH7=ADC_vb
*/

// =====================================================================
// USER CONFIG
// =====================================================================
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// GPIO pins — matched to schematic
static const int PIN_MUX_S0  = 25;
static const int PIN_MUX_S1  = 26;
static const int PIN_MUX_S2  = 27;
static const int PIN_MUX_OUT = 34;   // ADC1 only (GPIO 32–39)

static const int PIN_BAL_1 = 16;
static const int PIN_BAL_2 = 17;
static const int PIN_BAL_3 = 18;
static const int PIN_BAL_4 = 19;
static const int PIN_BAL_5 = 21;
static const int PIN_BAL_6 = 22;

static const int PIN_STATUS_LED = 2;  // set -1 if unused

// ADC
static const uint8_t  ADC_SAMPLES_PER_READ = 16;
static const uint16_t MUX_SETTLE_MS        = 5;
static const float    ADC_VREF             = 3.30f;
static const int      ADC_MAX              = 4095;

// =====================================================================
// CALIBRATION
// =====================================================================

// Cell: LM358 differential buffer → 1M/1M divider → ADC
// actual_cell_V = adc_V * 2.0
// Fine-tune each channel after measuring with a multimeter.
float cellScale[6] = { 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f };

// Pack: R49=13K / R51=1.6K divider → LM358 unity buffer
// packScale = (13K + 1.6K) / 1.6K = 9.125
float packScale = 9.125f;

// Current: ACS712-30A (5V) → LM358 gain-2 → 18K/33K divider → ADC
// Zero-current ADC voltage = 2.5V * 2 * 0.647 = 3.235V
// Gain = (1/0.066) / 2 / (1/0.647) = 11.70 A/V
// Sign convention: positive = charging, negative = discharging
// If current sign is inverted, swap sensor wiring or negate gain below.
float currentOffsetV   = 3.235f;
float currentGainAperV = 11.70f;

// =====================================================================
// COULOMB COUNTING CONFIG
// =====================================================================
float packCapacityAh = 10.0f;   // ← set to your real pack capacity in Ah

float chargeEfficiency    = 0.98f;
float dischargeEfficiency = 1.00f;

float    ccResyncCurrentThreshA = 0.20f;
uint32_t ccResyncRestDurationMs = 60000UL;  // 60 s rest before voltage re-sync

// =====================================================================
// SOC VOLTAGE TABLE  (per-cell voltage → SOC %)
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
float balanceStartVoltage        = 4.10f;
float balanceStopDelta           = 0.03f;
float minPackVoltageForBalancing = 20.0f;

// =====================================================================
// HISTORY RING BUFFER
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

struct BmsData {
  uint16_t raw[8];
  float    adcVoltage[8];
  float    cellVoltage[6];
  float    packVoltage;
  float    packCurrent;
  float    avgCellVoltage;
  float    minCellVoltage;
  float    maxCellVoltage;
  float    socPercent;
  float    socVoltage;
  float    chargedAh;
  float    dischargedAh;
  bool     balanceState[6];
  unsigned long sampleCounter;
};

BmsData bms;

bool     ccInitialized  = false;
float    ccSocPercent   = 50.0f;
uint32_t ccLastSampleMs = 0;
uint32_t ccRestStartMs  = 0;
bool     ccInRest       = false;

const int balancePins[6] = {
  PIN_BAL_1, PIN_BAL_2, PIN_BAL_3, PIN_BAL_4, PIN_BAL_5, PIN_BAL_6
};

const char* muxChannelNames[8] = {
  "ADC_C1","ADC_C2","ADC_C3","ADC_C4",
  "ADC_C5","ADC_C6","ADC_ip","ADC_vb"
};

unsigned long printIntervalMs = 2000;

// =====================================================================
// HELPERS
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

float interpolateSocFromCellVoltage(float cellV) {
  cellV = clampf(cellV, socLowClampCellV, socHighClampCellV);
  if (cellV <= socVoltageTable[0]) return socPercentTable[0];
  for (uint8_t i = 1; i < SOC_TABLE_SIZE; i++) {
    if (cellV <= socVoltageTable[i]) {
      float x0 = socVoltageTable[i - 1], x1 = socVoltageTable[i];
      float y0 = socPercentTable[i - 1], y1 = socPercentTable[i];
      return y0 + (cellV - x0) / (x1 - x0) * (y1 - y0);
    }
  }
  return socPercentTable[SOC_TABLE_SIZE - 1];
}

// =====================================================================
// COULOMB COUNTING
// =====================================================================
void updateCoulombCounting() {
  uint32_t now = millis();

  if (!ccInitialized) {
    float vIn = bms.avgCellVoltage;
    if (useMinCellForSocSafety)
      vIn = min(bms.avgCellVoltage, bms.minCellVoltage + 0.03f);
    ccSocPercent   = interpolateSocFromCellVoltage(vIn);
    bms.socPercent = ccSocPercent;
    ccLastSampleMs = now;
    ccRestStartMs  = now;
    ccInitialized  = true;
    return;
  }

  float dtH = (float)(now - ccLastSampleMs) / 3600000.0f;
  ccLastSampleMs = now;

  float I = bms.packCurrent;

  if (I > 0.0f) {
    float dAh = I * dtH * chargeEfficiency;
    bms.chargedAh += dAh;
    ccSocPercent  += (dAh / packCapacityAh) * 100.0f;
  } else if (I < 0.0f) {
    float dAh = fabsf(I) * dtH / dischargeEfficiency;
    bms.dischargedAh += dAh;
    ccSocPercent     -= (dAh / packCapacityAh) * 100.0f;
  }

  ccSocPercent = clampf(ccSocPercent, 0.0f, 100.0f);

  bool isResting = (fabsf(I) < ccResyncCurrentThreshA);
  if (isResting) {
    if (!ccInRest) { ccInRest = true; ccRestStartMs = now; }
    if ((now - ccRestStartMs) >= ccResyncRestDurationMs) {
      float vIn = bms.avgCellVoltage;
      if (useMinCellForSocSafety)
        vIn = min(bms.avgCellVoltage, bms.minCellVoltage + 0.03f);
      float voltSoc = interpolateSocFromCellVoltage(vIn);
      ccSocPercent = ccSocPercent * 0.90f + voltSoc * 0.10f;
    }
  } else {
    ccInRest = false;
  }

  bms.socPercent = ccSocPercent;

  float vIn = bms.avgCellVoltage;
  if (useMinCellForSocSafety)
    vIn = min(bms.avgCellVoltage, bms.minCellVoltage + 0.03f);
  bms.socVoltage = interpolateSocFromCellVoltage(vIn);
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
// SAMPLING
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

  histBuf[histHead] = {
    millis(), bms.packVoltage, bms.packCurrent, bms.socPercent,
    { bms.cellVoltage[0], bms.cellVoltage[1], bms.cellVoltage[2],
      bms.cellVoltage[3], bms.cellVoltage[4], bms.cellVoltage[5] }
  };
  histHead  = (histHead + 1) % HIST_SIZE;
  if (histCount < HIST_SIZE) histCount++;
}

// =====================================================================
// JSON
// =====================================================================
String jsonStatus() {
  String s = "{";
  s += "\"samples\":"       + String(bms.sampleCounter)     + ",";
  s += "\"pack_voltage\":"  + String(bms.packVoltage, 3)    + ",";
  s += "\"pack_current\":"  + String(bms.packCurrent, 3)    + ",";
  s += "\"avg_cell_v\":"    + String(bms.avgCellVoltage, 3) + ",";
  s += "\"min_cell_v\":"    + String(bms.minCellVoltage, 3) + ",";
  s += "\"max_cell_v\":"    + String(bms.maxCellVoltage, 3) + ",";
  s += "\"soc_cc\":"        + String(bms.socPercent, 2)     + ",";
  s += "\"soc_v\":"         + String(bms.socVoltage, 2)     + ",";
  s += "\"charged_ah\":"    + String(bms.chargedAh, 4)      + ",";
  s += "\"discharged_ah\":" + String(bms.dischargedAh, 4)   + ",";
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
// WEB DASHBOARD
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
  <button class='btn' onclick="fetch('/balance/off')">All balance OFF</button>
  <button class='btn' id='btnBal' onclick="toggleBal()">Auto balance</button>
</div>
<div class='cells' id='cellGrid'></div>
<div class='charts'>
  <div class='chart-box'><h3>SOC % (Coulomb counting)</h3><div style='position:relative;height:160px'><canvas id='cSOC'></canvas></div></div>
  <div class='chart-box'><h3>Pack voltage (V)</h3><div style='position:relative;height:160px'><canvas id='cV'></canvas></div></div>
  <div class='chart-box'><h3>Pack current (A)</h3><div style='position:relative;height:160px'><canvas id='cI'></canvas></div></div>
  <div class='chart-box'><h3>Cell voltages (V)</h3><div style='position:relative;height:160px'><canvas id='cCells'></canvas></div></div>
</div>
<div class='footer' id='ft'>Connecting…</div>
<script>
const COLORS=['#38bdf8','#4ade80','#fb923c','#f472b6','#a78bfa','#facc15'];
const MAX_PTS=120;
let autoBalOn=false;
function mkChart(id,label,color,yMin,yMax){
  return new Chart(document.getElementById(id),{type:'line',
    data:{labels:[],datasets:[{label,data:[],borderColor:color,borderWidth:1.5,pointRadius:0,fill:false,tension:0.3}]},
    options:{responsive:true,maintainAspectRatio:false,animation:false,
      scales:{x:{display:false},y:{min:yMin??undefined,max:yMax??undefined,ticks:{color:'#94a3b8',font:{size:10}},grid:{color:'rgba(255,255,255,.05)'}}},
      plugins:{legend:{display:false}}}});
}
function mkCellChart(id){
  return new Chart(document.getElementById(id),{type:'line',
    data:{labels:[],datasets:COLORS.map((c,i)=>({label:'C'+(i+1),data:[],borderColor:c,borderWidth:1.2,pointRadius:0,fill:false,tension:0.3}))},
    options:{responsive:true,maintainAspectRatio:false,animation:false,
      scales:{x:{display:false},y:{ticks:{color:'#94a3b8',font:{size:10}},grid:{color:'rgba(255,255,255,.05)'}}},
      plugins:{legend:{labels:{color:'#94a3b8',font:{size:10},boxWidth:8}}}}});
}
const socChart=mkChart('cSOC','SOC','#22c55e',0,100);
const vChart=mkChart('cV','V','#38bdf8',null,null);
const iChart=mkChart('cI','A','#fb923c',null,null);
const cellChart=mkCellChart('cCells');
function push(chart,label,...vals){
  chart.data.labels.push(label);
  if(chart.data.labels.length>MAX_PTS)chart.data.labels.shift();
  vals.forEach((v,i)=>{chart.data.datasets[i].data.push(v);if(chart.data.datasets[i].data.length>MAX_PTS)chart.data.datasets[i].data.shift();});
  chart.update('none');
}
function socColor(s){return s>50?'#22c55e':s>20?'#f59e0b':'#ef4444';}
function toggleBal(){autoBalOn=!autoBalOn;fetch('/balance/auto/'+(autoBalOn?'on':'off'));document.getElementById('btnBal').className='btn'+(autoBalOn?' on':'');}
let lastSamples=0;
async function tick(){
  try{
    const d=await(await fetch('/api/status')).json();
    const t=new Date().toLocaleTimeString();
    document.getElementById('pv').childNodes[0].textContent=d.pack_voltage.toFixed(2);
    document.getElementById('pi').childNodes[0].textContent=d.pack_current.toFixed(2);
    const s=parseFloat(d.soc_cc);
    document.getElementById('soc').childNodes[0].textContent=s.toFixed(1);
    document.getElementById('socv').childNodes[0].textContent=parseFloat(d.soc_v).toFixed(1);
    document.getElementById('chAh').childNodes[0].textContent=parseFloat(d.charged_ah).toFixed(3);
    document.getElementById('dcAh').childNodes[0].textContent=parseFloat(d.discharged_ah).toFixed(3);
    const bar=document.getElementById('socbar');
    bar.style.width=s.toFixed(1)+'%';bar.style.background=socColor(s);
    const grid=document.getElementById('cellGrid');
    if(grid.children.length===0)for(let i=0;i<6;i++){const c=document.createElement('div');c.id='cell'+i;c.className='cell';c.innerHTML=`<div class='cv' id='cv${i}'>—</div><div class='cl'>Cell ${i+1}</div>`;grid.appendChild(c);}
    d.cell_voltages.forEach((v,i)=>{document.getElementById('cv'+i).textContent=v.toFixed(3);document.getElementById('cell'+i).className='cell'+(d.balance[i]?' bal':'');});
    if(d.samples!==lastSamples){
      push(socChart,t,s);push(vChart,t,d.pack_voltage);push(iChart,t,d.pack_current);
      cellChart.data.labels.push(t);if(cellChart.data.labels.length>MAX_PTS)cellChart.data.labels.shift();
      d.cell_voltages.forEach((v,i)=>{cellChart.data.datasets[i].data.push(v);if(cellChart.data.datasets[i].data.length>MAX_PTS)cellChart.data.datasets[i].data.shift();});
      cellChart.update('none');lastSamples=d.samples;
    }
    document.getElementById('ft').textContent='Updated '+t+' | Samples: '+d.samples;
  }catch(e){document.getElementById('ft').textContent='Connection error — retrying…';}
}
tick();setInterval(tick,1000);
</script>
</body></html>
)rawhtml";
  server.send(200, "text/html", html);
}

// =====================================================================
// HTTP HANDLERS
// =====================================================================
void handleApiStatus()   { server.send(200, "application/json", jsonStatus()); }
void handleApiHistory()  { server.send(200, "application/json", jsonHistory()); }
void handleAllBalanceOff() { autoBalanceEnabled=false; allBalanceOff(); server.send(200,"text/plain","OK"); }
void handleAutoOn()  { autoBalanceEnabled=true;  server.send(200,"text/plain","Auto ON"); }
void handleAutoOff() { autoBalanceEnabled=false; allBalanceOff(); server.send(200,"text/plain","Auto OFF"); }

void handleBalanceManual() {
  if (!server.hasArg("cell")||!server.hasArg("state")){server.send(400,"text/plain","Use ?cell=1..6&state=0|1");return;}
  int cell=server.arg("cell").toInt(), state=server.arg("state").toInt();
  if(cell<1||cell>6){server.send(400,"text/plain","cell 1..6");return;}
  autoBalanceEnabled=false;
  setBalance(cell-1,state!=0);
  server.send(200,"text/plain","OK");
}

// =====================================================================
// SERIAL REPORT
// =====================================================================
void printSerialReport() {
  Serial.println("====================================================");
  Serial.printf("Samples : %lu\n", bms.sampleCounter);
  Serial.printf("Pack    : %.3f V | %.3f A\n", bms.packVoltage, bms.packCurrent);
  Serial.printf("SOC(CC) : %.2f %%  |  SOC(V): %.2f %%\n", bms.socPercent, bms.socVoltage);
  Serial.printf("Charged : %.4f Ah  |  Discharged: %.4f Ah\n", bms.chargedAh, bms.dischargedAh);
  for (uint8_t i=0;i<6;i++)
    Serial.printf("Cell %u  : %.3f V  BAL=%s\n", i+1, bms.cellVoltage[i], bms.balanceState[i]?"ON":"OFF");
  Serial.println("Raw ADC:");
  for (uint8_t i=0;i<8;i++)
    Serial.printf("  CH%u (%s): raw=%u  adc=%.4fV\n", i, muxChannelNames[i], bms.raw[i], bms.adcVoltage[i]);
}

// =====================================================================
// SETUP
// =====================================================================
void setupWiFi() {
  if (String(WIFI_SSID)=="YOUR_WIFI_SSID"){Serial.println("Wi-Fi not configured — offline.");return;}
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID,WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  unsigned long t=millis();
  while(WiFi.status()!=WL_CONNECTED&&millis()-t<15000){delay(500);Serial.print(".");}
  Serial.println();
  if(WiFi.status()==WL_CONNECTED){Serial.print("Connected — IP: ");Serial.println(WiFi.localIP());}
  else Serial.println("Wi-Fi timeout — offline.");
}

void setupServer() {
  server.on("/",                 handleRoot);
  server.on("/api/status",       handleApiStatus);
  server.on("/api/history",      handleApiHistory);
  server.on("/balance/off",      handleAllBalanceOff);
  server.on("/balance/auto/on",  handleAutoOn);
  server.on("/balance/auto/off", handleAutoOff);
  server.on("/balance/set",      handleBalanceManual);
  server.begin();
  Serial.println("HTTP server started.");
}

void setupPins() {
  pinMode(PIN_MUX_S0,OUTPUT); pinMode(PIN_MUX_S1,OUTPUT); pinMode(PIN_MUX_S2,OUTPUT);
  for(uint8_t i=0;i<6;i++){pinMode(balancePins[i],OUTPUT);digitalWrite(balancePins[i],LOW);bms.balanceState[i]=false;}
  if(PIN_STATUS_LED>=0){pinMode(PIN_STATUS_LED,OUTPUT);digitalWrite(PIN_STATUS_LED,LOW);}
}

void setupAdc() { analogReadResolution(12); analogSetAttenuation(ADC_11db); }

// =====================================================================
// MAIN
// =====================================================================
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("\nESP32 BMS v1 — Web Dashboard — starting…");
  setupPins(); setupAdc(); allBalanceOff();
  setupWiFi(); setupServer();
  sampleAllChannels();
  Serial.printf("CC SOC seeded at %.1f%% from voltage.\n", bms.socPercent);
  Serial.println("Ready.");
}

void loop() {
  static unsigned long lastSample=0, lastPrint=0;
  static bool ledState=false;
  unsigned long now=millis();

  if(now-lastSample>=300){
    lastSample=now; sampleAllChannels();
    if(PIN_STATUS_LED>=0){ledState=!ledState;digitalWrite(PIN_STATUS_LED,ledState);}
  }
  if(now-lastPrint>=printIntervalMs){lastPrint=now;printSerialReport();}
  server.handleClient();
}
