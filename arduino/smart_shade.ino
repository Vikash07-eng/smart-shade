/*
 * Smart Shade Controller
 * ESP32-C3 / ESP32-S3 + TMC2209 + NEMA 17 driving a bead-chain loop.
 *
 * Features
 *  - WiFi setup via captive portal (no passwords in code)
 *  - Phone web app served from the ESP32 (http://shade.local)
 *  - Open/close, set percentage, stop, jog + two-point calibration
 *  - Schedules stored on the device: fixed time, sunrise, or sunset (+/- offset), per weekday
 *  - NTP time with timezone, position saved across power loss
 *  - Optional physical button (short press = toggle open/close, press while moving = stop)
 *
 * Libraries (Library Manager): FastAccelStepper, ArduinoJson (v7), WiFiManager (tzapu)
 * Board: ESP32C3 Dev Module (or ESP32S3 Dev Module)
 *
 * Position convention: 0 steps = fully OPEN (top), maxSteps = fully CLOSED (bottom).
 * "percent" in the API/UI is percent OPEN: 100 = fully open, 0 = fully closed.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <FastAccelStepper.h>
#include <time.h>
#include <math.h>
#include "webui.h"

// ---------------------------------------------------------------- pins
#define PIN_STEP 4   // TMC2209 STEP
#define PIN_DIR  5   // TMC2209 DIR
#define PIN_EN   6   // TMC2209 EN (active LOW)
#define PIN_BTN  7   // optional momentary button to GND

#define HOSTNAME "shade"
#define MAX_SCHED 16

// ---------------------------------------------------------------- state
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;
WebServer server(80);
Preferences prefs;

struct Config {
  float lat = 0, lon = 0;            // needed for sunrise/sunset schedules
  bool invert = false;               // flip motor direction
  uint32_t speed = 3000;             // steps/sec
  uint32_t accel = 2000;             // steps/sec^2
  int32_t maxSteps = 0;              // 0 = not calibrated yet
  char tz[48] = "EST5EDT,M3.2.0,M11.1.0";
} cfg;

struct Sched {
  bool used = false;
  bool enabled = true;
  uint8_t type = 0;      // 0 = clock time, 1 = sunrise, 2 = sunset
  uint8_t hour = 7;
  uint8_t minute = 0;
  int16_t offset = 0;    // minutes, for sunrise/sunset
  uint8_t days = 0x7F;   // bit0 = Sunday ... bit6 = Saturday
  uint8_t target = 100;  // percent open
};
bool autoTz = true;                  // follow the phone's timezone (DST rules come with it)
Sched sched[MAX_SCHED];
int32_t lastFired[MAX_SCHED];

bool wasMoving = false;
uint32_t restartAt = 0;
uint32_t lastSchedCheck = 0;

// ---------------------------------------------------------------- persistence
void loadConfig() {
  prefs.begin("shade", true);
  cfg.lat = prefs.getFloat("lat", 0);
  cfg.lon = prefs.getFloat("lon", 0);
  cfg.invert = prefs.getBool("inv", false);
  cfg.speed = prefs.getUInt("spd", 3000);
  cfg.accel = prefs.getUInt("acc", 2000);
  cfg.maxSteps = prefs.getInt("max", 0);
  autoTz = prefs.getBool("atz", true);
  String tz = prefs.getString("tz", cfg.tz);
  tz.toCharArray(cfg.tz, sizeof(cfg.tz));
  size_t n = prefs.getBytesLength("sched");
  if (n == sizeof(sched)) prefs.getBytes("sched", sched, sizeof(sched));
  prefs.end();
}

void saveConfig() {
  prefs.begin("shade", false);
  prefs.putFloat("lat", cfg.lat);
  prefs.putFloat("lon", cfg.lon);
  prefs.putBool("inv", cfg.invert);
  prefs.putUInt("spd", cfg.speed);
  prefs.putUInt("acc", cfg.accel);
  prefs.putInt("max", cfg.maxSteps);
  prefs.putBool("atz", autoTz);
  prefs.putString("tz", cfg.tz);
  prefs.end();
}

void saveSchedules() {
  prefs.begin("shade", false);
  prefs.putBytes("sched", sched, sizeof(sched));
  prefs.end();
}

void savePosition() {
  prefs.begin("shade", false);
  prefs.putInt("pos", stepper->getCurrentPosition());
  prefs.end();
}

// ---------------------------------------------------------------- motion helpers
int percentFromSteps(int32_t steps) {
  if (cfg.maxSteps <= 0) return 0;
  long p = 100 - lround(100.0 * steps / cfg.maxSteps);
  return constrain(p, 0, 100);
}

int32_t stepsFromPercent(int percent) {
  percent = constrain(percent, 0, 100);
  return (int32_t)lround((100 - percent) / 100.0 * cfg.maxSteps);
}

void applyMotion() {
  stepper->setSpeedInHz(cfg.speed);
  stepper->setAcceleration(cfg.accel);
}

bool moveToPercent(int percent) {
  if (cfg.maxSteps <= 0) return false;
  applyMotion();
  stepper->moveTo(stepsFromPercent(percent));
  return true;
}

void stopMotor() {
  stepper->stopMove();
}

// ---------------------------------------------------------------- time + sun
bool timeValid() { return time(nullptr) > 1700000000; }

int tzOffsetMin(time_t now) {
  struct tm l, g;
  localtime_r(&now, &l);
  gmtime_r(&now, &g);
  g.tm_isdst = l.tm_isdst;
  return (int)((now - mktime(&g)) / 60);
}

// NOAA approximation. Returns local minutes after midnight, or -1 if unavailable.
int sunEvent(bool rise, const struct tm &lt, time_t now) {
  if (cfg.lat == 0 && cfg.lon == 0) return -1;
  double g = 2.0 * M_PI / 365.0 * lt.tm_yday;
  double eqt = 229.18 * (0.000075 + 0.001868 * cos(g) - 0.032077 * sin(g) -
                         0.014615 * cos(2 * g) - 0.040849 * sin(2 * g));
  double decl = 0.006918 - 0.399912 * cos(g) + 0.070257 * sin(g) -
                0.006758 * cos(2 * g) + 0.000907 * sin(2 * g) -
                0.002697 * cos(3 * g) + 0.00148 * sin(3 * g);
  double latr = cfg.lat * M_PI / 180.0;
  double cosha = cos(90.833 * M_PI / 180.0) / (cos(latr) * cos(decl)) - tan(latr) * tan(decl);
  if (cosha > 1 || cosha < -1) return -1;  // polar day/night
  double ha = acos(cosha) * 180.0 / M_PI;
  double utc = rise ? 720 - 4 * (cfg.lon + ha) - eqt : 720 - 4 * (cfg.lon - ha) - eqt;
  int loc = (int)lround(utc) + tzOffsetMin(now);
  return ((loc % 1440) + 1440) % 1440;
}

void checkSchedules() {
  static int32_t lastDay = -1;
  static int lastMin = -1;

  if (!timeValid() || cfg.maxSteps <= 0) return;
  time_t now = time(nullptr);
  struct tm lt;
  localtime_r(&now, &lt);
  int minNow = lt.tm_hour * 60 + lt.tm_min;
  int32_t day = lt.tm_year * 366 + lt.tm_yday;

  // Look at every minute since the last check, not just this one. When clocks spring forward
  // (02:00 -> 03:00) a 02:30 schedule sits in the skipped hour and would never fire; now it
  // fires right at 03:00. When clocks fall back, minNow goes backwards and the repeated hour
  // is ignored, so nothing fires twice.
  int from = minNow;
  if (day == lastDay && lastMin >= 0 && minNow - lastMin > 1 && minNow - lastMin <= 120) from = lastMin + 1;
  lastDay = day;
  lastMin = minNow;

  for (int i = 0; i < MAX_SCHED; i++) {
    Sched &s = sched[i];
    if (!s.used || !s.enabled) continue;
    if (!(s.days & (1 << lt.tm_wday))) continue;

    int trig;
    if (s.type == 0) {
      trig = s.hour * 60 + s.minute;
    } else {
      int sun = sunEvent(s.type == 1, lt, now);
      if (sun < 0) continue;
      trig = sun + s.offset;
    }
    if (trig < 0 || trig >= 1440) continue;

    int32_t key = day * 1440 + trig;
    if (trig >= from && trig <= minNow && lastFired[i] != key) {
      lastFired[i] = key;
      Serial.printf("Schedule %d fired -> %d%%\n", i, s.target);
      moveToPercent(s.target);
    }
  }
}

// ---------------------------------------------------------------- HTTP helpers
void sendJson(int code, JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json", out);
}

void sendError(int code, const char *msg) {
  JsonDocument d;
  d["error"] = msg;
  sendJson(code, d);
}

bool readBody(JsonDocument &doc) {
  if (!server.hasArg("plain")) return false;
  return !deserializeJson(doc, server.arg("plain"));
}

// ---------------------------------------------------------------- HTTP handlers
void handleState() {
  JsonDocument d;
  int32_t pos = stepper->getCurrentPosition();
  d["calibrated"] = cfg.maxSteps > 0;
  d["maxSteps"] = cfg.maxSteps;
  d["pos"] = pos;
  d["percent"] = percentFromSteps(pos);
  d["target"] = percentFromSteps(stepper->targetPos());
  d["moving"] = stepper->isRunning();
  d["timeOk"] = timeValid();
  if (timeValid()) {
    time_t now = time(nullptr);
    struct tm lt;
    localtime_r(&now, &lt);
    char buf[8];
    snprintf(buf, sizeof(buf), "%02d:%02d", lt.tm_hour, lt.tm_min);
    d["time"] = buf;
    d["sunrise"] = sunEvent(true, lt, now);
    d["sunset"] = sunEvent(false, lt, now);
  }
  sendJson(200, d);
}

void handleMove() {
  if (!server.hasArg("percent")) return sendError(400, "percent required");
  if (!moveToPercent(server.arg("percent").toInt())) return sendError(409, "not calibrated");
  JsonDocument d;
  d["ok"] = true;
  sendJson(200, d);
}

void handleStop() {
  stopMotor();
  JsonDocument d;
  d["ok"] = true;
  sendJson(200, d);
}

// Relative move, ignores limits (for calibration)
void handleJog() {
  int32_t steps = server.arg("steps").toInt();
  if (steps == 0) return sendError(400, "steps required");
  applyMotion();
  stepper->move(steps);
  JsonDocument d;
  d["ok"] = true;
  sendJson(200, d);
}

void handleCalOpen() {
  if (stepper->isRunning()) return sendError(409, "wait for motor to stop");
  int32_t oldPos = stepper->getCurrentPosition();
  stepper->setCurrentPosition(0);
  if (cfg.maxSteps > 0) {
    // keep the closed limit relative to the new open point
    cfg.maxSteps = max<int32_t>(cfg.maxSteps - oldPos, 0);
    saveConfig();
  }
  savePosition();
  JsonDocument d;
  d["ok"] = true;
  sendJson(200, d);
}

void handleCalClosed() {
  if (stepper->isRunning()) return sendError(409, "wait for motor to stop");
  int32_t pos = stepper->getCurrentPosition();
  if (pos < 200) return sendError(400, "closed point must be well below the open point");
  cfg.maxSteps = pos;
  saveConfig();
  savePosition();
  JsonDocument d;
  d["ok"] = true;
  sendJson(200, d);
}

void handleSchedulesGet() {
  JsonDocument d;
  JsonArray arr = d.to<JsonArray>();
  for (int i = 0; i < MAX_SCHED; i++) {
    if (!sched[i].used) continue;
    JsonObject o = arr.add<JsonObject>();
    o["enabled"] = sched[i].enabled;
    o["type"] = sched[i].type;
    o["hour"] = sched[i].hour;
    o["minute"] = sched[i].minute;
    o["offset"] = sched[i].offset;
    o["days"] = sched[i].days;
    o["target"] = sched[i].target;
  }
  sendJson(200, d);
}

void handleSchedulesPost() {
  JsonDocument d;
  if (!readBody(d) || !d.is<JsonArray>()) return sendError(400, "expected JSON array");
  JsonArray arr = d.as<JsonArray>();
  if (arr.size() > MAX_SCHED) return sendError(400, "too many schedules");
  for (int i = 0; i < MAX_SCHED; i++) {
    sched[i] = Sched();
    lastFired[i] = -1;
  }
  int i = 0;
  for (JsonObject o : arr) {
    Sched &s = sched[i++];
    s.used = true;
    s.enabled = o["enabled"] | true;
    s.type = constrain((int)(o["type"] | 0), 0, 2);
    s.hour = constrain((int)(o["hour"] | 7), 0, 23);
    s.minute = constrain((int)(o["minute"] | 0), 0, 59);
    s.offset = constrain((int)(o["offset"] | 0), -240, 240);
    s.days = (uint8_t)((int)(o["days"] | 127) & 0x7F);
    s.target = constrain((int)(o["target"] | 100), 0, 100);
  }
  saveSchedules();
  JsonDocument r;
  r["ok"] = true;
  sendJson(200, r);
}

void handleSettingsGet() {
  JsonDocument d;
  d["lat"] = cfg.lat;
  d["lon"] = cfg.lon;
  d["invert"] = cfg.invert;
  d["speed"] = cfg.speed;
  d["accel"] = cfg.accel;
  d["tz"] = cfg.tz;
  d["autoTz"] = autoTz;
  sendJson(200, d);
}

void handleSettingsPost() {
  JsonDocument d;
  if (!readBody(d)) return sendError(400, "bad JSON");
  bool oldInvert = cfg.invert;
  String oldTz = cfg.tz;
  cfg.lat = d["lat"] | cfg.lat;
  cfg.lon = d["lon"] | cfg.lon;
  cfg.invert = d["invert"] | cfg.invert;
  autoTz = d["autoTz"] | autoTz;
  cfg.speed = constrain((int)(d["speed"] | (int)cfg.speed), 200, 20000);
  cfg.accel = constrain((int)(d["accel"] | (int)cfg.accel), 100, 50000);
  if (d["tz"].is<const char *>()) strlcpy(cfg.tz, d["tz"], sizeof(cfg.tz));
  saveConfig();
  if (oldTz != cfg.tz) configTzTime(cfg.tz, "pool.ntp.org", "time.nist.gov");
  if (oldInvert != cfg.invert) restartAt = millis() + 800;  // direction pin polarity applied at boot
  JsonDocument r;
  r["ok"] = true;
  r["restarting"] = oldInvert != cfg.invert;
  sendJson(200, r);
}

void handleResetWifi() {
  JsonDocument r;
  r["ok"] = true;
  sendJson(200, r);
  delay(300);
  WiFiManager wm;
  wm.resetSettings();
  ESP.restart();
}

void setupRoutes() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/move", HTTP_POST, handleMove);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/jog", HTTP_POST, handleJog);
  server.on("/api/cal/open", HTTP_POST, handleCalOpen);
  server.on("/api/cal/closed", HTTP_POST, handleCalClosed);
  server.on("/api/schedules", HTTP_GET, handleSchedulesGet);
  server.on("/api/schedules", HTTP_POST, handleSchedulesPost);
  server.on("/api/settings", HTTP_GET, handleSettingsGet);
  server.on("/api/settings", HTTP_POST, handleSettingsPost);
  server.on("/api/wifi/reset", HTTP_POST, handleResetWifi);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
}

// ---------------------------------------------------------------- button
void handleButton() {
  static bool last = true;
  static uint32_t lastChange = 0;
  bool now = digitalRead(PIN_BTN);
  if (now != last && millis() - lastChange > 40) {
    lastChange = millis();
    last = now;
    if (!now) {  // pressed
      if (stepper->isRunning()) {
        stopMotor();
      } else if (cfg.maxSteps > 0) {
        moveToPercent(percentFromSteps(stepper->getCurrentPosition()) > 50 ? 0 : 100);
      }
    }
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  for (int i = 0; i < MAX_SCHED; i++) lastFired[i] = -1;
  loadConfig();

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (!stepper) {
    Serial.println("Stepper init failed");
    while (true) delay(1000);
  }
  stepper->setDirectionPin(PIN_DIR, !cfg.invert);
  stepper->setEnablePin(PIN_EN);  // active low (TMC2209)
  stepper->setAutoEnable(true);   // motor released when idle: cooler and silent
  applyMotion();

  prefs.begin("shade", true);
  stepper->setCurrentPosition(prefs.getInt("pos", 0));
  prefs.end();

  WiFi.setHostname(HOSTNAME);
  WiFiManager wm;
  wm.setConnectTimeout(20);
  wm.setConfigPortalTimeout(300);
  if (!wm.autoConnect("SmartShade-Setup")) {
    Serial.println("WiFi setup timed out, restarting");
    ESP.restart();
  }
  Serial.print("Connected, IP: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
  configTzTime(cfg.tz, "pool.ntp.org", "time.nist.gov");
  setupRoutes();
}

void loop() {
  server.handleClient();
  handleButton();

  bool moving = stepper->isRunning();
  if (wasMoving && !moving) savePosition();  // store position once a move finishes
  wasMoving = moving;

  if (millis() - lastSchedCheck > 1000) {
    lastSchedCheck = millis();
    checkSchedules();
  }

  if (restartAt && millis() > restartAt) ESP.restart();
}
