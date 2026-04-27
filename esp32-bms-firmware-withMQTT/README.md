# ESP32 BMS — Coulomb Counting + Live Dashboard + MQTT

Extended version of the BMS firmware that adds MQTT on top of the web dashboard.
The ESP32 publishes live battery data to a broker every 5 seconds and accepts
remote commands to control cell balancing from any device on the network.

> The web dashboard from `esp32_bms_coulomb` is still fully functional in this
> version — MQTT is added on top without removing anything.

---

## Repository Structure

```
esp32_bms_mqtt/
└── esp32_bms_coulomb.cpp
```

---

## Additional Requirements

### Extra Library

**PubSubClient** by Nick O'Leary

**Arduino IDE:**  
`Sketch → Include Library → Manage Libraries` → search for `PubSubClient` → Install

**PlatformIO** — add to `platformio.ini`:
```ini
lib_deps = knolleary/PubSubClient @ ^2.8
```

### MQTT Broker

You need a running MQTT broker accessible from the ESP32.

| Option | Best for | Setup |
|---|---|---|
| **Mosquitto** on Raspberry Pi or PC | Permanent local use | `sudo apt install mosquitto mosquitto-clients` |
| **Home Assistant** built-in broker | If you already run HA | Enable the Mosquitto add-on inside HA |
| **HiveMQ Cloud** (free tier) | Cloud access, no local server | [hivemq.com/mqtt-cloud-broker](https://www.hivemq.com/mqtt-cloud-broker/) |
| **test.mosquitto.org** | Quick testing only | Public broker — do not use in production |

---

## Configuration

Open `esp32_bms_coulomb.cpp` and edit the **USER CONFIG** section at the top.

### 1. Wi-Fi credentials

```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";    // your Wi-Fi network name
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD"; // your Wi-Fi password
```

### 2. MQTT broker address ← most important step

```cpp
const char* MQTT_BROKER   = "192.168.1.100"; // ← IP or hostname of your broker
const uint16_t MQTT_PORT  = 1883;            // default MQTT port (use 8883 for TLS)
const char* MQTT_USER     = "";              // leave empty if no authentication
const char* MQTT_PASS_STR = "";              // leave empty if no authentication
const char* MQTT_CLIENT_ID    = "esp32_bms_01"; // unique name for this device
const char* MQTT_TOPIC_PREFIX = "bms";          // all topics will start with this
```

**How to find your broker IP:**

- **Mosquitto on Raspberry Pi:** the broker IP is the Pi's IP address.  
  Run `hostname -I` in the Pi terminal to find it.
- **Mosquitto on Windows/Linux PC:** use the machine's local IP.  
  On Windows: `ipconfig` in Command Prompt → look for IPv4 Address.  
  On Linux: `ip a` → look for your network interface IP.
- **Home Assistant:** go to `Settings → System → Network` to find the HA IP.
- **HiveMQ Cloud:** your dashboard provides a hostname — paste it in place of the IP  
  and change `MQTT_PORT` to `8883`. TLS support requires additional setup.
- **Quick test (no broker):** set `MQTT_BROKER = "test.mosquitto.org"` temporarily.

**If your broker requires a username and password:**

```cpp
const char* MQTT_USER     = "myusername";
const char* MQTT_PASS_STR = "mypassword";
```

### 3. Publish interval

```cpp
// How often to publish data in milliseconds — keep at 1000 or above
const uint32_t MQTT_PUBLISH_INTERVAL_MS = 5000;  // every 5 seconds
```

### 4. Disable MQTT without removing the code

```cpp
bool mqttEnabled = false;   // web dashboard still works normally
```

---

## Topics Published by the ESP32

All topics start with the prefix defined in `MQTT_TOPIC_PREFIX` (default: `bms`).

| Topic | Type | Example value |
|---|---|---|
| `bms/status` | Full JSON | `{"samples":1200,"pack_voltage":24.35,...}` |
| `bms/soc` | Number | `67.4` |
| `bms/pack_voltage` | Number | `24.350` |
| `bms/pack_current` | Number | `-2.100` |
| `bms/cell/1` | Number | `4.058` |
| `bms/cell/2` | Number | `4.061` |
| `bms/cell/3` | Number | `4.055` |
| `bms/cell/4` | Number | `4.059` |
| `bms/cell/5` | Number | `4.060` |
| `bms/cell/6` | Number | `4.057` |
| `bms/charged_ah` | Number | `1.2340` |
| `bms/discharged_ah` | Number | `3.5670` |

---

## Command Topics (ESP32 subscribes)

Send a message to these topics to control the BMS remotely.

| Topic | Payload | Result |
|---|---|---|
| `bms/cmd/balance_auto` | `ON` | Enable automatic balancing |
| `bms/cmd/balance_auto` | `OFF` | Disable automatic balancing |
| `bms/cmd/balance_all` | `OFF` | Force all balance outputs off |
| `bms/cmd/balance_set` | `3:1` | Turn cell 3 balance on manually |
| `bms/cmd/balance_set` | `3:0` | Turn cell 3 balance off manually |

**Examples using `mosquitto_pub` from a terminal:**

```bash
# Enable auto balance
mosquitto_pub -h 192.168.1.100 -t "bms/cmd/balance_auto" -m "ON"

# Turn off all balancing
mosquitto_pub -h 192.168.1.100 -t "bms/cmd/balance_all" -m "OFF"

# Manually turn on cell 2
mosquitto_pub -h 192.168.1.100 -t "bms/cmd/balance_set" -m "2:1"

# Manually turn off cell 2
mosquitto_pub -h 192.168.1.100 -t "bms/cmd/balance_set" -m "2:0"
```

**Monitor all topics from a terminal:**

```bash
# Subscribe to everything under the bms/ prefix
mosquitto_sub -h 192.168.1.100 -t "bms/#" -v

# Watch SOC only
mosquitto_sub -h 192.168.1.100 -t "bms/soc"
```

---

## Home Assistant Integration

Add the following to your `configuration.yaml`:

```yaml
mqtt:
  sensor:
    - name: "BMS SOC"
      state_topic: "bms/soc"
      unit_of_measurement: "%"
      device_class: battery

    - name: "BMS Pack Voltage"
      state_topic: "bms/pack_voltage"
      unit_of_measurement: "V"
      device_class: voltage

    - name: "BMS Pack Current"
      state_topic: "bms/pack_current"
      unit_of_measurement: "A"
      device_class: current

    - name: "BMS Charged Ah"
      state_topic: "bms/charged_ah"
      unit_of_measurement: "Ah"

    - name: "BMS Discharged Ah"
      state_topic: "bms/discharged_ah"
      unit_of_measurement: "Ah"

    - name: "BMS Cell 1"
      state_topic: "bms/cell/1"
      unit_of_measurement: "V"

    - name: "BMS Cell 2"
      state_topic: "bms/cell/2"
      unit_of_measurement: "V"

    - name: "BMS Cell 3"
      state_topic: "bms/cell/3"
      unit_of_measurement: "V"

    - name: "BMS Cell 4"
      state_topic: "bms/cell/4"
      unit_of_measurement: "V"

    - name: "BMS Cell 5"
      state_topic: "bms/cell/5"
      unit_of_measurement: "V"

    - name: "BMS Cell 6"
      state_topic: "bms/cell/6"
      unit_of_measurement: "V"

  switch:
    - name: "BMS Auto Balance"
      command_topic: "bms/cmd/balance_auto"
      payload_on: "ON"
      payload_off: "OFF"
```

---

## Serial Monitor Output

MQTT connection status is printed alongside the normal data report:

```
[MQTT] Connecting to 192.168.1.100… connected.
[MQTT] Subscribed to cmd topics.
====================================================
Samples : 1547
Pack    : 24.350 V | -2.100 A
SOC (CC): 67.45 %  |  SOC (V): 65.20 %
...
```

If the connection drops, the firmware retries automatically every 10 seconds:

```
[MQTT] Connecting to 192.168.1.100… failed (rc=-2) — will retry.
```

---

## Suggested Repository Structure

```
esp32-bms-firmware/
│
├── esp32_bms_coulomb/          ← v1 — Web dashboard only
│   ├── esp32_bms_coulomb.cpp
│   └── README.md
│
└── esp32_bms_mqtt/             ← v2 — Web dashboard + MQTT
    ├── esp32_bms_coulomb_mqtt.cpp
    └── README.md               ← this file
