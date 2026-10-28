# Smart Shade Controller (C, ESP-IDF)

ESP32-C3 / ESP32-S3 + TMC2209 + NEMA 17 driving a bead-chain loop, controlled from a phone web app.
Same hardware, wiring, web app and REST API as the Arduino version, but written in plain C on ESP-IDF.

## What changed from the Arduino version

| Piece | Arduino version | C version |
|---|---|---|
| Step generation | FastAccelStepper library | `stepper.c`: 1 MHz hardware timer interrupt with an integer trapezoid ramp |
| WiFi setup | WiFiManager | Built-in setup portal (AP `SmartShade-Setup`, open `http://192.168.4.1`) |
| Web server | Arduino WebServer | `esp_http_server` |
| JSON | ArduinoJson | cJSON (ships with ESP-IDF) |
| Storage | Preferences | NVS |
| Web app | `webui.h` | `main/index.html`, embedded in the firmware at build time |

## Files

```
CMakeLists.txt
sdkconfig.defaults       keeps the step interrupt alive during flash writes
main/main.c              WiFi, HTTP API, schedules, NVS, button
main/stepper.c/.h        timer-driven step/dir driver with acceleration
main/sun.c/.h            sunrise/sunset calculation
main/index.html          phone web app
main/idf_component.yml   pulls in the mDNS component (shade.local)
```

## Wiring

Unchanged: STEP = GPIO 4, DIR = GPIO 5, EN = GPIO 6 (active low), button = GPIO 7 to GND.
12V to the TMC2209 VM with a 100 µF capacitor, common ground with the ESP32, current set to about 0.8 to 1.0 A.
Pins are `#define`s at the top of `main/main.c`.

## Build and flash

Requires ESP-IDF 5.2 or newer.

```
. $IDF_PATH/export.sh
idf.py set-target esp32c3        # or esp32s3
idf.py build flash monitor
```

The first build downloads the mDNS component automatically.

## First run

1. Join the `SmartShade-Setup` WiFi network on your phone, open `http://192.168.4.1`, pick your WiFi, enter the password. The controller reboots and joins your network.
   (There is no automatic captive-portal popup, so type the address in the browser.)
2. Open `http://shade.local` in Safari, then Share > Add to Home Screen.
3. Settings > Calibration: jog to fully open, press *Set open here*, jog to fully closed, press *Set closed here*.
4. Set your timezone (default US Eastern) and latitude/longitude for sunrise/sunset schedules, then add schedules.

## Notes

- Speed and acceleration are in steps/s and steps/s². With the TMC2209 default 1/8 microstepping, a 200-step motor needs 1600 steps per revolution. Default 3000 steps/s is about 2 turns per second. The maximum is 20000 steps/s.
- Reversing a move mid-travel brakes to a stop first, then turns around.
- The motor is released 0.5 s after each move, and the position is saved to flash at that moment.
- If the saved WiFi stops working (wrong password, new router), the controller falls back to the setup portal after about 25 seconds of failed attempts. After the first successful connection it keeps reconnecting forever, so a short router outage never triggers the portal.
- There is no login on the API yet. Fine on a home network; add authentication before exposing it outside the house.

## Testing status

The sunrise/sunset math and the acceleration ramp logic were checked on a PC (the ramp always lands exactly on target for short and long moves at several speeds). The full firmware has not been compiled or run on hardware yet, so expect to fix a typo or two on the first build.
