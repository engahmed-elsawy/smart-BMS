# ESP32 BMS — Coulomb Counting + Live Web Dashboard

ESP32 firmware for monitoring a 6S Li-ion battery pack.
Reads 6 cells + pack current + pack voltage through a CD4051 analog MUX,
estimates SOC using Coulomb Counting, and serves a live dashboard in the browser.

---

## Repository Structure

```
esp32_bms_coulomb/
└── esp32_bms_coulomb.cpp
```

---

## Requirements

### Hardware

| Component | Details |
|---|---|
| ESP32 DevKit | Any standard ESP32 board |
| CD4051 | 8-channel analog multiplexer |
| Current sensor | ACS712 or similar (output 0–3.3 V) |
| Voltage dividers | One per cell + one for pack — output must be ≤ 3.3 V |
| Balancing circuit | MOSFET or transistor per cell |

### Software

- [Arduino IDE](https://www.arduino.cc/en/software) version 2.x
- **ESP32 board package** — in Arduino IDE:  
  `File → Preferences → Additional boards manager URLs`, add:
  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```
  Then `Tools → Board → Boards Manager`, search for **esp32** and install it.

No additional libraries are needed — the code uses `WiFi.h` and `WebServer.h`
which ship with the ESP32 package.

---

## Configuration

Open `esp32_bms_coulomb.cpp` and edit the **USER CONFIG** section at the top.

### 1. Wi-Fi credentials

```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";    // your Wi-Fi network name
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD"; // your Wi-Fi password
```

### 2. GPIO pin mapping

```cpp
static const int PIN_MUX_S0  = 25;   // MUX select bit 0
static const int PIN_MUX_S1  = 26;   // MUX select bit 1
static const int PIN_MUX_S2  = 27;   // MUX select bit 2
static const int PIN_MUX_OUT = 34;   // MUX output → ADC (use GPIO 32–39 only)

static const int PIN_BAL_1 = 16;     // balance output — cell 1
static const int PIN_BAL_2 = 17;
static const int PIN_BAL_3 = 18;
static const int PIN_BAL_4 = 19;
static const int PIN_BAL_5 = 21;
static const int PIN_BAL_6 = 22;
```

Change these numbers to match your actual wiring.

### 3. Voltage calibration

```cpp
// cell_V = adc_V × cellScale  — one scale factor per cell
float cellScale[6] = { 1.400f, 1.400f, 1.400f, 1.400f, 1.400f, 1.400f };

// pack_V = adc_V × packScale
float packScale = 9.125f;

// current sensor:  I(A) = (adc_V - offset) × gain
// ACS712-20A example: offset = 1.65 V, gain = 10.0 A/V
float currentOffsetV   = 1.650f;
float currentGainAperV = 10.000f;
```

**How to calibrate `cellScale`:**  
Measure the real cell voltage with a multimeter, then read the ADC voltage
for the same channel from Serial Monitor.
```
cellScale = real_voltage / adc_voltage
```
Example: multimeter reads 3.80 V, ADC reads 0.271 V → `cellScale = 14.02`

### 4. Pack capacity (Coulomb Counting)

```cpp
// Set this to the real capacity of your pack in Ah
float packCapacityAh = 10.0f;
```

---

## MUX Channel Mapping (CD4051)

| Channel | Signal | Description |
|---|---|---|
| CH0 | ADC_C1 | Cell 1 voltage |
| CH1 | ADC_C2 | Cell 2 voltage |
| CH2 | ADC_C3 | Cell 3 voltage |
| CH3 | ADC_C4 | Cell 4 voltage |
| CH4 | ADC_C5 | Cell 5 voltage |
| CH5 | ADC_C6 | Cell 6 voltage |
| CH6 | ADC_ip | Pack current |
| CH7 | ADC_vb | Pack voltage |

---

## Web Dashboard

After flashing, open Serial Monitor at **115200 baud**. You will see:

```
Connected — IP: 192.168.1.45
HTTP server started.
```

Open a browser on any device on the same network and go to:

```
http://192.168.1.45/
```

The page updates every second without reloading.

### What the dashboard shows

- **Pack Voltage** and **Pack Current** — live readings
- **SOC (Coulomb Counting)** with a color bar (green → yellow → red as it drops)
- **SOC (Voltage)** — reference value for comparison
- **Charged Ah / Discharged Ah** — cumulative totals since boot
- **6 cell cards** — individual voltages, highlighted when balancing is active
- **4 live charts** — SOC %, Pack Voltage, Pack Current, and all Cell Voltages

### HTTP Endpoints

| URL | Description |
|---|---|
| `http://[IP]/` | Main live dashboard |
| `http://[IP]/api/status` | Full JSON snapshot of all values |
| `http://[IP]/api/history` | Last 200 data points as arrays (used by charts) |
| `http://[IP]/balance/off` | Turn off all balance outputs |
| `http://[IP]/balance/auto/on` | Enable automatic balancing |
| `http://[IP]/balance/auto/off` | Disable automatic balancing |
| `http://[IP]/balance/set?cell=3&state=1` | Manually switch one cell on (state=1) or off (state=0) |

---

## Coulomb Counting — How It Works

1. **First boot:** SOC is seeded from the voltage lookup table since no current history exists yet.
2. **During operation:** integrates `dSOC = I × dt / capacity × 100` every 300 ms.
   - Positive current (charging) → multiplied by `chargeEfficiency`.
   - Negative current (discharging) → divided by `dischargeEfficiency`.
3. **Rest re-sync:** when `|I| < 0.2 A` for 60 consecutive seconds the pack is
   considered at rest. The voltage-based SOC then gently corrects any accumulated
   integration drift using a 90 % CC + 10 % voltage blend per cycle.

---

## Serial Monitor Output

A report is printed every 2 seconds:

```
====================================================
Samples : 1547
Pack    : 24.350 V | -2.100 A
SOC (CC): 67.45 %  |  SOC (V): 65.20 %
Charged : 0.0000 Ah  |  Discharged: 0.1234 Ah
Cell 1  : 4.058 V  BAL=OFF
Cell 2  : 4.061 V  BAL=OFF
Cell 3  : 4.055 V  BAL=OFF
Cell 4  : 4.059 V  BAL=OFF
Cell 5  : 4.060 V  BAL=OFF
Cell 6  : 4.057 V  BAL=OFF
Raw ADC:
  CH0 (ADC_C1): raw=336  adc=0.2710 V
  ...
```

---

## Next Step

If you need to push data to Home Assistant, Grafana, or control the BMS
remotely over the network — see the MQTT version in the `esp32_bms_mqtt` folder.
