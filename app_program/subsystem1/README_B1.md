# Buddy 1 — set up, build, flash and test the WiFi & Telemetry subsystem

Written for **Windows + WSL (Ubuntu)**. Every `$` command runs in the
**Ubuntu** terminal unless it says *Windows*. Do Part A once; after that,
each test cycle is just Parts E → F → G → H.

```
 Laptop (Windows)                                   Robot car (Pico W)
 ┌───────────────────────────┐   same 2.4 GHz WiFi   ┌──────────────────────┐
 │ Mosquitto broker :1883    │◄─────────────────────►│ comm_task (S1)       │
 │ mosquitto_sub / monitor   │   MQTT topics         │ kernel WiFi task     │
 │ WSL Ubuntu: make → .uf2   │──── USB cable ───────►│ USB serial console   │
 └───────────────────────────┘                       └──────────────────────┘
```

---

## Part A — One-time setup of Ubuntu (WSL)

**A1. Install WSL.** Click **Start**, type `PowerShell`, right-click
**Windows PowerShell → Run as administrator**, then:

```powershell
wsl --install -d Ubuntu
```

Restart Windows when asked. An **Ubuntu** window opens by itself afterwards
(or open it from Start → *Ubuntu*). Choose a username and password. The
password does not show while typing; that is normal.

**A2. Install the build tools** (in the Ubuntu window):

```sh
sudo apt update
sudo apt install -y git make g++ gcc-arm-none-eabi libnewlib-arm-none-eabi python3-pip
arm-none-eabi-gcc --version        # must print a version number
```

**A3. Download the Pico C SDK, including lwIP.** The project README only
fetches `tinyusb` and `cyw43-driver`; MQTT also needs **`lib/lwip`**:

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

**A4. Tell the shell where the SDK is:**

```sh
echo 'export PICO_SDK_PATH="$HOME/pico/pico-sdk"' >> ~/.bashrc
source ~/.bashrc
ls $PICO_SDK_PATH/lib/lwip/src/core/init.c    # must exist
```

## Part B — Get the project

```sh
cd ~/pico
git clone https://github.com/notmufliha/INF2004-Project.git
cd INF2004-Project
git checkout -b b1-wifi-mqtt          # work on your own branch
```

Copy the Buddy 1 files into this folder (same paths as in the zip, see the
file list at the bottom), then check the build works **without** WiFi
credentials yet:

```sh
cd build_make
make -j8          # plain build: S1 compiles, network off ("DISABLED")
```

## Part C — Run an MQTT broker on the laptop (Windows)

The broker must run on **Windows**, not inside WSL: the Pico cannot reach a
broker inside WSL's private virtual network.

1. Download **Mosquitto** from <https://mosquitto.org/download/> → *Windows*
   → `mosquitto-2.x.x-install-windows-x64.exe`. Run it, keep the defaults
   (installs to `C:\Program Files\mosquitto`).
2. Mosquitto 2 only accepts connections from the laptop itself unless told
   otherwise. Open **Notepad as administrator** (Start → type `Notepad` →
   right-click → *Run as administrator*), paste:
   ```
   listener 1883 0.0.0.0
   allow_anonymous true
   ```
   **File → Save As** → folder `C:\Program Files\mosquitto` → *Save as type:
   All Files* → name `g25.conf` → **Save**.
3. Start it: Start → type `cmd` → right-click **Command Prompt → Run as
   administrator**:
   ```bat
   cd "C:\Program Files\mosquitto"
   net stop mosquitto
   mosquitto -c g25.conf -v
   ```
   (`net stop` just stops the background service the installer may have
   started, so port 1883 is free; an error there is fine.) Leave this window
   open; it prints every connection.
4. If Windows Firewall pops up, tick **Private networks** → **Allow access**.
   If no pop-up appears and the Pico can't connect later: Start → *Windows
   Defender Firewall* → *Advanced settings* → *Inbound Rules* → *New Rule…*
   → **Port** → TCP, **1883** → *Allow the connection* → tick all profiles →
   name it `Mosquitto 1883`.
5. Find the laptop's IP: in a normal Command Prompt run `ipconfig`, look
   under **Wireless LAN adapter Wi-Fi** → **IPv4 Address**, e.g.
   `192.168.43.120`. Write it down.

**Network rule:** the laptop and the Pico must be on the **same 2.4 GHz
network**. Easiest: both join your phone's hotspot (turn on *Maximise
compatibility* on iPhone, or *2.4 GHz band* on Android). School WiFi with a
login page will not work.

## Part D — Put your WiFi and broker details in

```sh
cd ~/pico/INF2004-Project
cp config/wifi_secrets.example.h config/wifi_secrets.h
nano config/wifi_secrets.h
```

Change `WIFI_SSID`, `WIFI_PASSWORD`, and `MQTT_BROKER_IP` (the IPv4 address
from C5). Save with **Ctrl-O**, **Enter**, exit with **Ctrl-X**.
`wifi_secrets.h` is in `.gitignore`, so your password is never committed.

> If the laptop's IP changes (new hotspot, new day), update `MQTT_BROKER_IP`
> and rebuild.

## Part E — Build

```sh
cd ~/pico/INF2004-Project/build_make
make NET=lwip MQTT=1 -j8
```

It ends with:
```
tools/elf2uf2 mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.elf mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
```
**Always give `NET=lwip MQTT=1`.** A plain `make` builds an image with no
network (S1 then only prints events to the console).

## Part F — Flash

1. Unplug the Pico. **Hold BOOTSEL**, plug in USB, release. A drive
   **RPI-RP2** appears in Windows.
2. In Ubuntu: `explorer.exe .` (in `build_make`) opens the folder in Windows.
3. Drag **`mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`** onto **RPI-RP2**. The
   Pico reboots by itself and the drive disappears. That's normal.

## Part G — Watch the Pico's console

1. Start → *Device Manager* → expand **Ports (COM & LPT)** → note **USB Serial
   Device (COMx)**.
2. Install **PuTTY** (<https://www.putty.org>). Open it → *Connection type*:
   **Serial** → *Serial line*: `COMx` → *Speed*: `115200` → **Open**.
3. Press the Pico's reset (or unplug/replug) to see boot. Expected (abridged):
   ```
   s1: WiFi & Telemetry task started (STANDALONE DEMO MODE)
   s1-demo: fake robot running.  Set S1_STANDALONE_DEMO to 0 before integration!
   wifi: joining "MyHotspot" ...
   s1: link INITIALISING -> CONNECTING_WIFI
   wifi: link up, ip=192.168.43.57
   mqtt: connecting to 192.168.43.120:1883 as "inf2004-g25-car" ...
   s1: link CONNECTING_WIFI -> CONNECTING_MQTT
   mqtt: connected to 192.168.43.120
   s1: link CONNECTING_MQTT -> CONNECTED
   mqtt: subscribed to inf2004/g25/car/cmd
   s1: heartbeat #1  link=CONNECTED  tx=1 drop=0 rx=0 rej=0
   s1: EVENT barcode 'A' -> LEFT
   ```
   The LED on GP19 (with a 330 Ω resistor to GND) goes: fast blink while
   joining WiFi → slow blink while connecting to the broker → **solid** once
   connected.

## Part H — See telemetry and send commands from the laptop

Since Change Requirement 1 the car sends **binary frames** (MQTT.md
section 2), so `mosquitto_sub` only shows unreadable bytes. Use the Python
tools, which decode them (install Python on Windows from
<https://www.python.org>, tick *Add python.exe to PATH*, then
`pip install paho-mqtt`). Run them from the project folder; reach it from
Windows at `\wsl$\Ubuntu\home\<you>\pico\INF2004-Project`.

**Watch everything, and type commands:**
```bat
python tools\mqtt_monitor.py
python tools\mqtt_monitor.py --raw          (also shows each frame in hex)
```
You should see `telemetry` twice a second and `status` every 5 s, plus
`event` lines when B4 sees a hump or B5 profiles an obstacle. Type `STOP`,
`SPEED 40`, `START`, `MODE LINE`, `PING` or `TELEM 300` and press Enter:
each is sent as a 3-byte frame and answered on `cmd/ack`.

**Send one command and wait for its ack** (exit status 0 = accepted):
```bat
python tools\mqtt_send_cmd.py STOP --seq 1
python tools\mqtt_send_cmd.py SPEED 40 --seq 2
python tools\mqtt_send_cmd.py --hex 2e 00           (2-byte frame: rejected, "frame is not 3 bytes")
```

**10-minute A1 test:**
```bat
python tools\mqtt_monitor.py --quiet-telemetry --log soak.jsonl
```
**Ctrl-C** prints how many messages were missing per topic.

**Host test of the wire format** (Ubuntu, no Pico needed):
```sh
sh testing/subsystem1/host/run_host_tests.sh
```

## Part I — Run the automated tests

```sh
cd ~/pico/INF2004-Project/build_make
make NET=lwip MQTT=1 TEST=1 -j8
```
Flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test.uf2` (Part F), keep the
broker running, watch PuTTY. Expected:
```
[TEST] S1-U01 PASS  LED cyclic handler is running
...
[TEST] S1-U09 PASS  WiFi + MQTT connected within 30 s
[TEST] S1-U10 PASS  telemetry published for 3 s with no drops
[TEST] subsystem1 SUMMARY 11 passed, 0 failed
```
Save the output (PuTTY: *Session → Logging → All session output*) into
`testing/subsystem1/logs/YYYY-MM-DD_unit.txt` and add rows to
`testing/subsystem1/TEST_LOG.md`. The manual tests (S1-U11…S1-U19, S1-U21, S1-U22) are listed
there too.

## Part J — Commit

```sh
cd ~/pico/INF2004-Project
git add app_program/subsystem1 lib/liblwip/sysdepend/pico_rp2040 config/wifi_secrets.example.h \
        TEAM.md testing/subsystem1 tools/mqtt_monitor.py tools/mqtt_send_cmd.py tools/s1_wire.py
git status                 # wifi_secrets.h must NOT be listed
git commit -m "S1: WiFi/MQTT telemetry, commands, reconnect"
git push -u origin b1-wifi-mqtt
```
Then open a pull request on GitHub. The `mqtt_utk.c`/`lwipopts.h` changes are
shared library code: TEAM.md section 3 says agree them with the team **and
the instructor** first.

---

## Troubleshooting

| Console says | Meaning / fix |
|---|---|
| `wifi: link status -> FAILED: SSID not found` | Wrong SSID, or the network is 5 GHz only. Turn on 2.4 GHz / *Maximise compatibility*. |
| `... authentication rejected` | Wrong password, or the AP is WPA3-only. Use WPA2. |
| `mqtt: connection lost/refused (status 256 or 257)` then `retry 1 in 1000ms`… | Broker not reachable: wrong `MQTT_BROKER_IP`, Mosquitto not running with `g25.conf`, firewall blocking 1883, or laptop on a different network. |
| `mqtt: broker unreachable, OFFLINE for 60s` | 6 retries failed. Fix the above; it tries again by itself. |
| `s1: link ... -> DISABLED` | You built without `NET=lwip MQTT=1`. |
| `make: lwIP not found` | Run the `git submodule update ... lib/lwip` line from A3. |
| `make: arm-none-eabi-gcc not found` | A2 not done, or open a new terminal. |
| No COM port in Device Manager | Use a data USB cable (not charge-only); check the `.uf2` name had `usb_cdc`. |
| Mosquitto window shows nothing when the Pico connects | Firewall (C4) or different networks (C5). Test from the phone with an MQTT app pointing at the laptop IP. |

## Before integration (checklist)

1. `S1_STANDALONE_DEMO` now defaults to `0`: the real subsystems feed the
   telemetry and the Vehicle Controller owns `mbf_remote_cmd`. Build with
   `S1_STANDALONE_DEMO=1` only for a bench test without them.
2. Connected: B2 (S1 polls `encoder_get_counts()`), B4 (`task_imu.c`),
   B5 (`task_obstacle.c`). Still to do: B3 calls `comm_set_line()` and
   `comm_report_barcode()`; the Vehicle Controller calls
   `comm_set_robot_state()`, reads `mbf_remote_cmd`, and takes over
   creating `flg_faults` from `subsystem1_init()`.
3. Every buddy sets/clears only their own bit with `fault_raise()` /
   `fault_clear()` (`app_program/fault_flags.h`) and adds error codes to
   their block in `app_program/error_codes.h` (and the names to
   `tools/s1_wire.py`).
4. Priorities: the design review puts WiFi & Telemetry **lowest**
   (NFR-B1-3), but the template's bands give S1 the *highest* app band
   (12–13), above B4 (18–19) and B5 (20). The task is light (sleeps
   > 95 % of the time, S1-U02), but the team should swap bands so S1 is
   numerically the largest.
