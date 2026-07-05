# ESP32-CAM Surveillance Car

Arduino firmware for an ESP32-CAM based camera car. This folder
(`source/ESP32CAM_Car/`) is the real, maintained firmware.

- `ESP32CAM_Car.ino` — main sketch (setup, WiFi, motor control, loop)
- `app_httpd.cpp` — HTTP server: web UI, camera stream, motor/LED endpoints
- `camera_index.h` — camera index page assets

> Note: There is a separate, stale PlatformIO project at the repo root
> (`../../src/` + `../../platformio.ini`). Do **not** flash that one — it holds
> an old copy with placeholder WiFi credentials and will just print dots
> forever. Always build and upload from this folder.

## How to run

1. Start the **Arduino IDE** and open this folder:
   `d:\gitlab-haojiesun\zyc0024\source\ESP32CAM_Car`
2. Select the board **ESP32 Wrover Module** and choose the detected serial
   port (often **COM5** — confirm in Device Manager, it can change).
3. **Turn off the car's batteries before uploading.** When the motors are
   powered they interfere with serial data transfer and the upload can fail.
   Upload with battery power off, then switch the batteries back on.

## Accessing the car

After boot, the serial console prints the URL. It's protected by a passcode
prefix (set by `PASSCODE` in `app_httpd.cpp`):

```
http://<device-ip>:8081/<passcode>
```

The MJPEG stream runs on the next port (`8081 + 1`).

## Serial console

- Baud rate: **115200**
- The firmware logs WiFi connection progress, motor actions, and a periodic
  heap report (every 60s) for leak/fragmentation diagnosis.

### Console commands

Type a command and press Enter. Available commands:

| Command   | Action                                                        |
|-----------|---------------------------------------------------------------|
| `heap`    | Print heap stats now (free / min_free / largest_block)        |
| `stop`    | Force all motors off                                          |
| `ip`      | Print the device IP and WiFi connection status                |
| `uptime`  | Print uptime in milliseconds                                  |
| `restart` | Reboot the ESP32                                              |
| `help`    | List the available commands                                   |

Unknown input returns a hint; lines longer than 64 characters are discarded.

## WiFi

Credentials are set in `ESP32CAM_Car.ino`. It tries the work network first,
then the home network. ESP32 supports **2.4 GHz only**.
