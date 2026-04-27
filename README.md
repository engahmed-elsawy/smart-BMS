# ESP32 Smart BMS

An open-source Battery Management System for 6S Li-ion packs built around an ESP32.
The project covers hardware design, embedded firmware, and a planned machine learning
pipeline that will train on the data the system collects.

---

## Project Status

| Phase | Status |
|---|---|
| Hardware design (schematic + PCB) | ✅ Done |
| Firmware v1 — Coulomb Counting + Web Dashboard | ✅ Done |
| Firmware v2 — MQTT data pipeline | ✅ Done |
| ML model — SOC / SOH estimation | 🔜 Planned |

---

## Repository Structure

```
esp32-smart-bms/
│
├── hardware/
│   └── schematic/               BMS schematic (KiCad / PDF)
│
├── esp32_bms_coulomb/           Firmware v1 — Web dashboard only
│   ├── esp32_bms_coulomb.cpp
│   └── README.md
│
├── esp32_bms_mqtt/              Firmware v2 — Web dashboard + MQTT
│   ├── esp32_bms_coulomb_mqtt.cpp
│   └── README.md
│
└── README.md                    ← this file
```

---

## What the System Does

```
┌─────────────────────────────────────────────────────────┐
│                     6S Li-ion Pack                      │
│  Cell 1 │ Cell 2 │ Cell 3 │ Cell 4 │ Cell 5 │ Cell 6   │
└────┬─────────┬────────┬────────┬────────┬────────┬──────┘
     │         │        │        │        │        │
     └─────────┴────────┴───── CD4051 ────┴────────┘
                                  │
                              ESP32 ADC
                                  │
              ┌───────────────────┼───────────────────┐
              │                   │                   │
        Web Dashboard          MQTT Broker        Serial Monitor
        (browser, live)     (Home Assistant /    (115200 baud)
                              Grafana / ML)
                                  │
                          ┌───────┴───────┐
                          │  (planned)    │
                       Dataset CSV    ML Model
                      (SOC / SOH /   (TensorFlow
                       temperature)    Lite / sklearn)
```

---

## Hardware Overview

### Core Components

| Component | Role |
|---|---|
| ESP32 DevKit | Main MCU — ADC, Wi-Fi, control logic |
| CD4051 | 8-channel analog MUX — reads all cells through one ADC pin |
| ACS712 (or similar) | Current sensor — measures pack charge / discharge current |
| Voltage dividers | Scale each cell voltage and pack voltage down to ≤ 3.3 V |
| MOSFETs / transistors | Balance outputs — one per cell |

### Key Design Decisions

**Single ADC pin via MUX** — The CD4051 lets the ESP32 read 8 analog channels
through a single ADC input. This saves GPIOs and avoids the inter-channel
crosstalk that comes from using multiple ADC pins simultaneously on the ESP32.

**All analog signals ≤ 3.3 V** — The ESP32 ADC is not 5 V tolerant.
Every voltage divider and sensor output is designed to stay within 0–3.3 V.

**ADC1 only** — ADC2 on the ESP32 is shared with the Wi-Fi radio and gives
unreliable readings when Wi-Fi is active. `PIN_MUX_OUT` is wired to a GPIO
in the ADC1 group (GPIO 32–39).

**Hardware schematic** is in the `hardware/schematic/` folder.

---

## Firmware

### v1 — Web Dashboard (`esp32_bms_coulomb/`)

Standalone firmware. No external services needed.

- Reads all 8 MUX channels every 300 ms
- Coulomb Counting SOC with voltage re-sync at rest
- Auto cell balancing (configurable thresholds)
- Live browser dashboard with 4 Chart.js graphs — updates every second
- JSON API at `/api/status` and `/api/history`

### v2 — MQTT (`esp32_bms_mqtt/`)

Everything in v1, plus:

- Publishes to MQTT broker every 5 seconds
- Individual topics per value (`bms/soc`, `bms/cell/1`, etc.)
- Subscribes to command topics for remote balance control
- Compatible with Home Assistant, Node-RED, Grafana, and any MQTT client
- **All published data is logged by the broker** — this is the dataset that
  will feed the ML pipeline

See each folder's `README.md` for full setup instructions.

---

## MQTT Data Format

Every publish cycle sends these topics:

```
bms/status         →  full JSON snapshot
bms/soc            →  67.4
bms/pack_voltage   →  24.350
bms/pack_current   →  -2.100
bms/cell/1  …  /6  →  4.058  (one topic per cell)
bms/charged_ah     →  1.2340
bms/discharged_ah  →  3.5670
```

To log everything to a CSV file using Node-RED or a simple subscriber:

```bash
# log all bms topics to a file
mosquitto_sub -h 192.168.1.100 -t "bms/#" -v >> bms_log.txt
```

---

## ML Pipeline (Planned)

The MQTT broker accumulates time-series data across real charge / discharge cycles.
That dataset will be used to train a model that estimates SOC and SOH more
accurately than the current voltage-table approach.

### Planned Features

- **Better SOC estimation** — replace the voltage lookup table with a learned
  model that accounts for temperature, aging, and load history
- **SOH (State of Health)** — track capacity fade over hundreds of cycles
- **Anomaly detection** — flag unusual cell behavior (high self-discharge,
  internal short, accelerated aging)
- **Remaining useful life (RUL)** — predict how many cycles the pack has left

### Planned Stack

```
Data collection  →  MQTT broker  →  CSV / InfluxDB
Feature engineering  →  Python (pandas, numpy)
Model training   →  scikit-learn / TensorFlow / PyTorch
Deployment       →  TensorFlow Lite on ESP32  or  inference on a Pi / server
```

### Features the Model Will Use

| Feature | Source |
|---|---|
| Cell voltages (V1–V6) | `bms/cell/1` … `/6` |
| Pack current (A) | `bms/pack_current` |
| Pack voltage (V) | `bms/pack_voltage` |
| SOC from Coulomb Counting | `bms/soc` |
| Charged Ah | `bms/charged_ah` |
| Discharged Ah | `bms/discharged_ah` |
| Timestamp | logged by subscriber |
| Temperature (future) | planned sensor addition |

### Data Collection Advice

The more cycles logged, the better the model. To build a useful dataset:

1. Flash the v2 MQTT firmware.
2. Set up a broker and a subscriber that appends every message to a CSV with a timestamp.
3. Run the pack through full charge / discharge cycles under realistic loads.
4. Aim for at least 20–30 full cycles before starting model training.

---

## Quick Start

### 1. Flash the firmware

1. Install [Arduino IDE 2.x](https://www.arduino.cc/en/software).
2. Add the ESP32 board package URL in `File → Preferences`:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Open `esp32_bms_coulomb/esp32_bms_coulomb.cpp`.
4. Set your Wi-Fi credentials and GPIO pins in the **USER CONFIG** section.
5. Set `packCapacityAh` to your pack's real capacity.
6. Flash to your ESP32.

### 2. Open the dashboard

Find the IP in Serial Monitor (115200 baud) and open it in a browser.

### 3. Add MQTT (optional)

Switch to the `esp32_bms_mqtt/` version, install **PubSubClient**, set
`MQTT_BROKER` to your broker's IP, and reflash.

---

## Calibration Checklist

Before trusting the readings, go through this checklist:

- [ ] Measure each cell voltage with a multimeter and calculate the correct `cellScale[i]`
- [ ] Measure pack voltage and calculate the correct `packScale`
- [ ] Verify current sensor zero-current offset matches `currentOffsetV`
- [ ] Set `packCapacityAh` to the actual rated capacity of your cells
- [ ] Update the SOC voltage table if your cells have a different chemistry

---

## Contributing

Pull requests are welcome. If you are working on the ML pipeline, data collection
scripts, or a new sensor integration, open an issue first to discuss the approach.

---

## License

MIT
