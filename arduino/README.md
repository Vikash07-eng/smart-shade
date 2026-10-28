# Smart Shade Controller

ESP32 + TMC2209 + NEMA 17 driving a bead-chain loop, controlled from a phone web app.

## Parts

| Part | Notes |
|---|---|
| ESP32-C3 or ESP32-S3 dev board | Any board with WiFi works |
| NEMA 17 stepper (0.4 to 0.5 N·m) | e.g. 17HS4401 |
| TMC2209 driver module | BigTreeTech or Watterott modules work; run in standalone STEP/DIR mode |
| 12V 2A DC supply | Powers the motor |
| Buck converter 12V to 5V | Powers the ESP32 (or use a 5V USB supply and a separate 12V supply) |
| 100 µF electrolytic capacitor | Across the TMC2209 VM/GND pins, close to the driver |
| Chain sprocket | Sized to your bead chain (measure bead diameter, commonly 4.5 or 6 mm) |
| Momentary button (optional) | Between GPIO 7 and GND |

## Wiring

```
ESP32 GPIO 4  -> TMC2209 STEP
ESP32 GPIO 5  -> TMC2209 DIR
ESP32 GPIO 6  -> TMC2209 EN
ESP32 GND     -> TMC2209 GND (logic side)
ESP32 3V3     -> TMC2209 VIO
Button        -> GPIO 7 and GND (internal pull-up, no resistor needed)

12V supply +  -> TMC2209 VM  (with 100 µF cap to GND right at the module)
12V supply -  -> TMC2209 GND (power side) and ESP32 GND (common ground)
Motor coils   -> TMC2209 2B 2A 1A 1B  (swap a coil pair if the direction is wrong, or use the Reverse switch in Settings)
```

Important:
- Never connect or disconnect the motor while the 12V is on, it can kill the driver.
- Set the driver current to about 0.8 to 1.0 A for a 1.5 A motor. On most TMC2209 modules this is the small VREF potentiometer. Start low and raise it only if the motor skips.
- With the MS1/MS2 pins left at default the TMC2209 uses 1/8 microstepping (1600 steps per revolution on a 200-step motor). The default speed of 3000 steps/sec is roughly 2 turns per second.

## Software setup

1. Install the Arduino IDE and add ESP32 board support (Boards Manager: "esp32" by Espressif).
2. Install libraries (Library Manager): **FastAccelStepper**, **ArduinoJson** (version 7), **WiFiManager** (by tzapu).
3. Put `smart_shade.ino` and `webui.h` together in a folder named `smart_shade`.
4. Select your board (ESP32C3 Dev Module or ESP32S3 Dev Module). For C3/S3 boards with native USB, enable "USB CDC On Boot" if you want serial output.
5. Upload.

PlatformIO works too: use the `platformio.ini` provided.

## First run

1. On first boot the ESP32 creates a WiFi hotspot called **SmartShade-Setup**. Join it on your phone and pick your home WiFi in the page that appears.
2. Open **http://shade.local** in Safari (or use the IP address printed on the serial monitor).
3. Safari > Share > **Add to Home Screen** to make it feel like an app.
4. Go to **Settings > Calibration**:
   1. Use the jog arrows to move the shade to fully **open**, press *Set open here*.
   2. Jog to fully **closed**, press *Set closed here*.
   3. If the shade moves the wrong way, flip *Reverse motor direction* and save (it restarts).
5. Pick your timezone and enter latitude/longitude if you want sunrise/sunset schedules (the default is US Eastern).
6. Add schedules in the **Schedules** tab. They run on the ESP32, so they work even if your phone is off.

## How it works

- Position is counted in motor steps: 0 = fully open, calibrated max = fully closed. It is saved when each move finishes, so it survives power loss. If the chain is moved by hand while the controller is off, just recalibrate.
- Schedules: up to 16, each with a time or sunrise/sunset (plus an offset), weekday selection, and a target percent open.
- The motor is released when idle (no holding current), so it stays cool and silent.
- Button: short press toggles open/closed, pressing while the shade is moving stops it.

## API (for later: Home Assistant, Shortcuts, remote access)

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/state` | position, percent open, moving, time, sunrise/sunset |
| POST | `/api/move?percent=0..100` | go to a percent open |
| POST | `/api/stop` | stop |
| POST | `/api/jog?steps=N` | relative move (calibration) |
| POST | `/api/cal/open`, `/api/cal/closed` | set limits |
| GET/POST | `/api/schedules` | list / replace schedules (JSON array) |
| GET/POST | `/api/settings` | settings |

Note: there is no login yet. That's fine on a home network, but we should add authentication before exposing it outside your house.

## Next steps

- Remote access from outside the house (Tailscale on your iPhone is the easiest and safest option, or MQTT/Home Assistant).
- HomeKit/Siri via Matter or a Home Assistant bridge.
- Light sensor automation, limit switches for self-homing.
