/*
 * Smart Shade Controller (ESP-IDF, C)
 * ESP32-C3 / ESP32-S3 + TMC2209 + NEMA 17 driving a bead-chain loop.
 *
 *  - WiFi setup portal on first boot (AP "SmartShade-Setup", open http://192.168.4.1)
 *  - Phone web app served from the ESP32 (http://shade.local)
 *  - Open/close, percentage, stop, jog + two-point calibration
 *  - 16 on-device schedules: clock time, sunrise or sunset (+/- offset), per weekday
 *  - SNTP time with timezone, position saved to flash after every move
 *  - Optional button: press = toggle open/close, press while moving = stop
 *
 * Position convention: 0 steps = fully OPEN, max_steps = fully CLOSED.
 * "percent" in the API is percent OPEN (100 = open, 0 = closed).
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "stepper.h"
#include "sun.h"

#define PIN_STEP 4
#define PIN_DIR  5
#define PIN_EN   6
#define PIN_BTN  7

#define HOSTNAME  "shade"
#define AP_SSID   "SmartShade-Setup"
#define MAX_SCHED 16
#define NVS_NS    "shade"

static const char *TAG = "shade";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

/* ------------------------------------------------------------------ config */
typedef struct {
    uint32_t magic;
    float lat, lon;
    uint8_t invert;
    uint32_t speed;
    uint32_t accel;
    int32_t max_steps; /* 0 = not calibrated */
    char tz[48];
} config_t;

#define CFG_MAGIC 0x53484431

typedef struct {
    uint8_t used, enabled, type; /* type: 0 clock, 1 sunrise, 2 sunset */
    uint8_t hour, minute;
    int16_t offset;
    uint8_t days;   /* bit0 = Sunday */
    uint8_t target; /* percent open */
} sched_t;

static config_t g_cfg;
static sched_t g_sched[MAX_SCHED];
static int32_t g_last_fired[MAX_SCHED];

static void config_defaults(void)
{
    memset(&g_cfg, 0, sizeof g_cfg);
    g_cfg.magic = CFG_MAGIC;
    g_cfg.speed = 3000;
    g_cfg.accel = 2000;
    strcpy(g_cfg.tz, "EST5EDT,M3.2.0,M11.1.0");
}

/* ------------------------------------------------------------------ NVS */
static void nvs_save_blob(const char *key, const void *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, key, data, len);
    nvs_commit(h);
    nvs_close(h);
}

static bool nvs_load_blob(const char *key, void *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t n = len;
    esp_err_t e = nvs_get_blob(h, key, data, &n);
    nvs_close(h);
    return e == ESP_OK && n == len;
}

static void save_config(void) { nvs_save_blob("cfg", &g_cfg, sizeof g_cfg); }
static void save_sched(void) { nvs_save_blob("sched", g_sched, sizeof g_sched); }

static void save_position(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "pos", stepper_pos());
    nvs_commit(h);
    nvs_close(h);
}

static void load_all(void)
{
    config_defaults();
    config_t tmp;
    if (nvs_load_blob("cfg", &tmp, sizeof tmp) && tmp.magic == CFG_MAGIC) g_cfg = tmp;
    nvs_load_blob("sched", g_sched, sizeof g_sched);
    for (int i = 0; i < MAX_SCHED; i++) g_last_fired[i] = -1;
}

/* ------------------------------------------------------------------ motion helpers */
static int pct_from_steps(int32_t s)
{
    if (g_cfg.max_steps <= 0) return 0;
    int64_t p = 100 - ((int64_t)100 * s + g_cfg.max_steps / 2) / g_cfg.max_steps;
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return (int)p;
}

static int32_t steps_from_pct(int p)
{
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return (int32_t)(((int64_t)(100 - p) * g_cfg.max_steps + 50) / 100);
}

static bool move_to_percent(int p)
{
    if (g_cfg.max_steps <= 0) return false;
    stepper_set_speed(g_cfg.speed, g_cfg.accel);
    stepper_move_to(steps_from_pct(p));
    return true;
}

/* ------------------------------------------------------------------ time + schedules */
static bool time_valid(void) { return time(NULL) > 1700000000; }

static int tz_offset_min(time_t now)
{
    struct tm l, g;
    localtime_r(&now, &l);
    gmtime_r(&now, &g);
    g.tm_isdst = l.tm_isdst;
    return (int)((now - mktime(&g)) / 60);
}

static void apply_timezone(void)
{
    setenv("TZ", g_cfg.tz, 1);
    tzset();
}

static void check_schedules(void)
{
    if (!time_valid() || g_cfg.max_steps <= 0) return;
    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    int min_now = lt.tm_hour * 60 + lt.tm_min;
    int32_t key = (lt.tm_year * 366 + lt.tm_yday) * 1440 + min_now;

    for (int i = 0; i < MAX_SCHED; i++) {
        sched_t *s = &g_sched[i];
        if (!s->used || !s->enabled) continue;
        if (!(s->days & (1 << lt.tm_wday))) continue;

        int trig;
        if (s->type == 0) {
            trig = s->hour * 60 + s->minute;
        } else {
            int sun = sun_event_min(s->type == 1, lt.tm_yday, g_cfg.lat, g_cfg.lon, tz_offset_min(now));
            if (sun < 0) continue;
            trig = sun + s->offset;
        }
        if (trig < 0 || trig >= 1440) continue;

        if (trig == min_now && g_last_fired[i] != key) {
            g_last_fired[i] = key;
            ESP_LOGI(TAG, "schedule %d fired -> %d%%", i, s->target);
            move_to_percent(s->target);
        }
    }
}

/* ------------------------------------------------------------------ HTTP helpers */
static const char *status_str(int c)
{
    switch (c) {
    case 200: return "200 OK";
    case 400: return "400 Bad Request";
    case 409: return "409 Conflict";
    default: return "500 Internal Server Error";
    }
}

static esp_err_t send_json(httpd_req_t *r, int code, cJSON *j)
{
    char *s = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);
    httpd_resp_set_status(r, status_str(code));
    httpd_resp_set_type(r, "application/json");
    esp_err_t e = httpd_resp_send(r, s ? s : "{}", HTTPD_RESP_USE_STRLEN);
    free(s);
    return e;
}

static esp_err_t send_err(httpd_req_t *r, int code, const char *msg)
{
    cJSON *j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "error", msg);
    return send_json(r, code, j);
}

static esp_err_t send_ok(httpd_req_t *r)
{
    cJSON *j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "ok", true);
    return send_json(r, 200, j);
}

static char *read_body(httpd_req_t *r)
{
    int n = r->content_len;
    if (n <= 0 || n > 4096) return NULL;
    char *b = malloc(n + 1);
    if (!b) return NULL;
    int got = 0;
    while (got < n) {
        int k = httpd_req_recv(r, b + got, n - got);
        if (k <= 0) {
            free(b);
            return NULL;
        }
        got += k;
    }
    b[n] = 0;
    return b;
}

static bool query_int(httpd_req_t *r, const char *key, int *out)
{
    char q[64], v[24];
    if (httpd_req_get_url_query_str(r, q, sizeof q) != ESP_OK) return false;
    if (httpd_query_key_value(q, key, v, sizeof v) != ESP_OK) return false;
    *out = atoi(v);
    return true;
}

static void restart_cb(void *arg)
{
    if (arg) { /* forget WiFi credentials */
        nvs_handle_t h;
        if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_key(h, "ssid");
            nvs_erase_key(h, "pass");
            nvs_commit(h);
            nvs_close(h);
        }
    }
    esp_restart();
}

static void restart_later(bool wipe_wifi)
{
    const esp_timer_create_args_t a = {.callback = restart_cb, .arg = (void *)(intptr_t)wipe_wifi, .name = "rst"};
    esp_timer_handle_t t;
    if (esp_timer_create(&a, &t) == ESP_OK) esp_timer_start_once(t, 800000);
}

/* ------------------------------------------------------------------ HTTP handlers */
static esp_err_t h_index(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html");
    return httpd_resp_send(r, index_html_start, index_html_end - index_html_start);
}

static esp_err_t h_state(httpd_req_t *r)
{
    cJSON *j = cJSON_CreateObject();
    int32_t pos = stepper_pos();
    cJSON_AddBoolToObject(j, "calibrated", g_cfg.max_steps > 0);
    cJSON_AddNumberToObject(j, "maxSteps", g_cfg.max_steps);
    cJSON_AddNumberToObject(j, "pos", pos);
    cJSON_AddNumberToObject(j, "percent", pct_from_steps(pos));
    cJSON_AddNumberToObject(j, "target", pct_from_steps(stepper_target()));
    cJSON_AddBoolToObject(j, "moving", stepper_running());
    cJSON_AddBoolToObject(j, "timeOk", time_valid());
    if (time_valid()) {
        time_t now = time(NULL);
        struct tm lt;
        localtime_r(&now, &lt);
        char buf[8];
        snprintf(buf, sizeof buf, "%02d:%02d", lt.tm_hour, lt.tm_min);
        cJSON_AddStringToObject(j, "time", buf);
        int off = tz_offset_min(now);
        cJSON_AddNumberToObject(j, "sunrise", sun_event_min(true, lt.tm_yday, g_cfg.lat, g_cfg.lon, off));
        cJSON_AddNumberToObject(j, "sunset", sun_event_min(false, lt.tm_yday, g_cfg.lat, g_cfg.lon, off));
    }
    return send_json(r, 200, j);
}

static esp_err_t h_move(httpd_req_t *r)
{
    int p;
    if (!query_int(r, "percent", &p)) return send_err(r, 400, "percent required");
    if (!move_to_percent(p)) return send_err(r, 409, "not calibrated");
    return send_ok(r);
}

static esp_err_t h_stop(httpd_req_t *r)
{
    stepper_stop();
    return send_ok(r);
}

static esp_err_t h_jog(httpd_req_t *r)
{
    int steps;
    if (!query_int(r, "steps", &steps) || steps == 0) return send_err(r, 400, "steps required");
    stepper_set_speed(g_cfg.speed, g_cfg.accel);
    stepper_move_rel(steps); /* ignores limits: used for calibration */
    return send_ok(r);
}

static esp_err_t h_cal_open(httpd_req_t *r)
{
    if (stepper_running()) return send_err(r, 409, "wait for motor to stop");
    int32_t old = stepper_pos();
    stepper_set_pos(0);
    if (g_cfg.max_steps > 0) { /* keep the closed limit relative to the new open point */
        g_cfg.max_steps -= old;
        if (g_cfg.max_steps < 0) g_cfg.max_steps = 0;
        save_config();
    }
    save_position();
    return send_ok(r);
}

static esp_err_t h_cal_closed(httpd_req_t *r)
{
    if (stepper_running()) return send_err(r, 409, "wait for motor to stop");
    int32_t pos = stepper_pos();
    if (pos < 200) return send_err(r, 400, "closed point must be well below the open point");
    g_cfg.max_steps = pos;
    save_config();
    save_position();
    return send_ok(r);
}

static esp_err_t h_sched_get(httpd_req_t *r)
{
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < MAX_SCHED; i++) {
        if (!g_sched[i].used) continue;
        cJSON *o = cJSON_CreateObject();
        cJSON_AddBoolToObject(o, "enabled", g_sched[i].enabled);
        cJSON_AddNumberToObject(o, "type", g_sched[i].type);
        cJSON_AddNumberToObject(o, "hour", g_sched[i].hour);
        cJSON_AddNumberToObject(o, "minute", g_sched[i].minute);
        cJSON_AddNumberToObject(o, "offset", g_sched[i].offset);
        cJSON_AddNumberToObject(o, "days", g_sched[i].days);
        cJSON_AddNumberToObject(o, "target", g_sched[i].target);
        cJSON_AddItemToArray(arr, o);
    }
    return send_json(r, 200, arr);
}

static int clampi(double v, int lo, int hi)
{
    int x = (int)v;
    return x < lo ? lo : (x > hi ? hi : x);
}

static double num_or(cJSON *o, const char *key, double dflt)
{
    cJSON *it = cJSON_GetObjectItem(o, key);
    return cJSON_IsNumber(it) ? it->valuedouble : dflt;
}

static esp_err_t h_sched_post(httpd_req_t *r)
{
    char *body = read_body(r);
    if (!body) return send_err(r, 400, "bad body");
    cJSON *arr = cJSON_Parse(body);
    free(body);
    if (!cJSON_IsArray(arr)) {
        cJSON_Delete(arr);
        return send_err(r, 400, "expected JSON array");
    }
    if (cJSON_GetArraySize(arr) > MAX_SCHED) {
        cJSON_Delete(arr);
        return send_err(r, 400, "too many schedules");
    }
    memset(g_sched, 0, sizeof g_sched);
    for (int i = 0; i < MAX_SCHED; i++) g_last_fired[i] = -1;

    int i = 0;
    cJSON *o;
    cJSON_ArrayForEach(o, arr)
    {
        sched_t *s = &g_sched[i++];
        cJSON *en = cJSON_GetObjectItem(o, "enabled");
        s->used = 1;
        s->enabled = cJSON_IsBool(en) ? cJSON_IsTrue(en) : 1;
        s->type = clampi(num_or(o, "type", 0), 0, 2);
        s->hour = clampi(num_or(o, "hour", 7), 0, 23);
        s->minute = clampi(num_or(o, "minute", 0), 0, 59);
        s->offset = clampi(num_or(o, "offset", 0), -240, 240);
        s->days = clampi(num_or(o, "days", 127), 0, 127);
        s->target = clampi(num_or(o, "target", 100), 0, 100);
    }
    cJSON_Delete(arr);
    save_sched();
    return send_ok(r);
}

static esp_err_t h_settings_get(httpd_req_t *r)
{
    cJSON *j = cJSON_CreateObject();
    cJSON_AddNumberToObject(j, "lat", g_cfg.lat);
    cJSON_AddNumberToObject(j, "lon", g_cfg.lon);
    cJSON_AddBoolToObject(j, "invert", g_cfg.invert);
    cJSON_AddNumberToObject(j, "speed", g_cfg.speed);
    cJSON_AddNumberToObject(j, "accel", g_cfg.accel);
    cJSON_AddStringToObject(j, "tz", g_cfg.tz);
    return send_json(r, 200, j);
}

static esp_err_t h_settings_post(httpd_req_t *r)
{
    char *body = read_body(r);
    if (!body) return send_err(r, 400, "bad body");
    cJSON *j = cJSON_Parse(body);
    free(body);
    if (!j) return send_err(r, 400, "bad JSON");

    bool old_invert = g_cfg.invert;
    g_cfg.lat = (float)num_or(j, "lat", g_cfg.lat);
    g_cfg.lon = (float)num_or(j, "lon", g_cfg.lon);
    cJSON *inv = cJSON_GetObjectItem(j, "invert");
    if (cJSON_IsBool(inv)) g_cfg.invert = cJSON_IsTrue(inv);
    g_cfg.speed = clampi(num_or(j, "speed", g_cfg.speed), 200, 20000);
    g_cfg.accel = clampi(num_or(j, "accel", g_cfg.accel), 100, 50000);
    cJSON *tz = cJSON_GetObjectItem(j, "tz");
    if (cJSON_IsString(tz) && strlen(tz->valuestring) < sizeof g_cfg.tz) {
        strcpy(g_cfg.tz, tz->valuestring);
        apply_timezone();
    }
    cJSON_Delete(j);
    save_config();

    bool restart = old_invert != (bool)g_cfg.invert; /* DIR polarity applied at boot */
    if (restart) restart_later(false);

    cJSON *res = cJSON_CreateObject();
    cJSON_AddBoolToObject(res, "ok", true);
    cJSON_AddBoolToObject(res, "restarting", restart);
    return send_json(r, 200, res);
}

static esp_err_t h_wifi_reset(httpd_req_t *r)
{
    esp_err_t e = send_ok(r);
    restart_later(true);
    return e;
}

static httpd_handle_t start_main_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 20;
    cfg.stack_size = 8192;
    httpd_handle_t srv = NULL;
    if (httpd_start(&srv, &cfg) != ESP_OK) return NULL;

    static const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = h_index},
        {.uri = "/api/state", .method = HTTP_GET, .handler = h_state},
        {.uri = "/api/move", .method = HTTP_POST, .handler = h_move},
        {.uri = "/api/stop", .method = HTTP_POST, .handler = h_stop},
        {.uri = "/api/jog", .method = HTTP_POST, .handler = h_jog},
        {.uri = "/api/cal/open", .method = HTTP_POST, .handler = h_cal_open},
        {.uri = "/api/cal/closed", .method = HTTP_POST, .handler = h_cal_closed},
        {.uri = "/api/schedules", .method = HTTP_GET, .handler = h_sched_get},
        {.uri = "/api/schedules", .method = HTTP_POST, .handler = h_sched_post},
        {.uri = "/api/settings", .method = HTTP_GET, .handler = h_settings_get},
        {.uri = "/api/settings", .method = HTTP_POST, .handler = h_settings_post},
        {.uri = "/api/wifi/reset", .method = HTTP_POST, .handler = h_wifi_reset},
    };
    for (size_t i = 0; i < sizeof routes / sizeof routes[0]; i++) httpd_register_uri_handler(srv, &routes[i]);
    return srv;
}

/* ------------------------------------------------------------------ WiFi */
#define BIT_OK   BIT0
#define BIT_FAIL BIT1

static EventGroupHandle_t s_wifi_eg;
static int s_retry;
static bool s_connected_once, s_portal;

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (s_portal) return;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_connected_once || s_retry++ < 5) {
            esp_wifi_connect(); /* after the first success, keep reconnecting forever */
        } else {
            xEventGroupSetBits(s_wifi_eg, BIT_FAIL);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_connected_once = true;
        s_retry = 0;
        xEventGroupSetBits(s_wifi_eg, BIT_OK);
    }
}

static bool load_wifi_creds(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = nvs_get_str(h, "ssid", ssid, &ssid_len) == ESP_OK;
    if (ok && nvs_get_str(h, "pass", pass, &pass_len) != ESP_OK) pass[0] = 0;
    nvs_close(h);
    return ok && ssid[0];
}

static bool connect_sta(const char *ssid, const char *pass)
{
    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, ssid, sizeof wc.sta.ssid);
    strlcpy((char *)wc.sta.password, pass, sizeof wc.sta.password);
    wc.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    EventBits_t b = xEventGroupWaitBits(s_wifi_eg, BIT_OK | BIT_FAIL, pdFALSE, pdFALSE, pdMS_TO_TICKS(25000));
    return b & BIT_OK;
}

/* ---- setup portal (first boot / wrong password) ---- */
static const char PORTAL_HTML[] =
    "<!doctype html><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Shade WiFi</title>"
    "<body style='font:16px sans-serif;max-width:420px;margin:24px auto;padding:0 16px'>"
    "<h2>Smart Shade WiFi setup</h2>"
    "<select id='s' style='width:100%;padding:10px'></select>"
    "<input id='p' type='password' placeholder='Password' style='width:100%;padding:10px;margin:10px 0'>"
    "<button onclick='go()' style='padding:12px 20px'>Connect</button><p id='m'>Scanning...</p>"
    "<script>"
    "fetch('/scan').then(r=>r.json()).then(a=>{a.forEach(x=>s.add(new Option(x)));m.textContent=''});"
    "function go(){m.textContent='Saving...';"
    "fetch('/wifi',{method:'POST',body:JSON.stringify({ssid:s.value,pass:p.value})})"
    ".then(()=>m.textContent='Saved. The shade is restarting. Rejoin your home WiFi, then open http://shade.local')}"
    "</script>";

static esp_err_t h_portal_index(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html");
    return httpd_resp_send(r, PORTAL_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_portal_scan(httpd_req_t *r)
{
    wifi_scan_config_t sc = {0};
    esp_wifi_scan_start(&sc, true);
    uint16_t n = 20;
    wifi_ap_record_t recs[20];
    esp_wifi_scan_get_ap_records(&n, recs);
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < n; i++) {
        if (recs[i].ssid[0]) cJSON_AddItemToArray(arr, cJSON_CreateString((char *)recs[i].ssid));
    }
    return send_json(r, 200, arr);
}

static esp_err_t h_portal_save(httpd_req_t *r)
{
    char *body = read_body(r);
    cJSON *j = body ? cJSON_Parse(body) : NULL;
    free(body);
    cJSON *ssid = j ? cJSON_GetObjectItem(j, "ssid") : NULL;
    cJSON *pass = j ? cJSON_GetObjectItem(j, "pass") : NULL;
    if (!cJSON_IsString(ssid) || !ssid->valuestring[0]) {
        cJSON_Delete(j);
        return send_err(r, 400, "ssid required");
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "ssid", ssid->valuestring);
        nvs_set_str(h, "pass", cJSON_IsString(pass) ? pass->valuestring : "");
        nvs_commit(h);
        nvs_close(h);
    }
    cJSON_Delete(j);
    esp_err_t e = send_ok(r);
    restart_later(false);
    return e;
}

static void run_portal(void)
{
    ESP_LOGW(TAG, "starting WiFi setup portal: join '%s', open http://192.168.4.1", AP_SSID);
    s_portal = true;
    esp_wifi_stop();
    wifi_config_t ap = {0};
    strlcpy((char *)ap.ap.ssid, AP_SSID, sizeof ap.ap.ssid);
    ap.ap.ssid_len = strlen(AP_SSID);
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    httpd_handle_t srv;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));
    static const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = h_portal_index},
        {.uri = "/scan", .method = HTTP_GET, .handler = h_portal_scan},
        {.uri = "/wifi", .method = HTTP_POST, .handler = h_portal_save},
    };
    for (size_t i = 0; i < sizeof routes / sizeof routes[0]; i++) httpd_register_uri_handler(srv, &routes[i]);

    for (int i = 0; i < 600; i++) vTaskDelay(pdMS_TO_TICKS(1000)); /* 10 min, then retry normal boot */
    esp_restart();
}

static void wifi_start_or_portal(void)
{
    s_wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(sta, HOSTNAME);

    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL));

    char ssid[33] = "", pass[65] = "";
    if (load_wifi_creds(ssid, sizeof ssid, pass, sizeof pass) && connect_sta(ssid, pass)) {
        ESP_LOGI(TAG, "WiFi connected");
        return;
    }
    run_portal(); /* never returns */
}

/* ------------------------------------------------------------------ button */
static void button_task(void *arg)
{
    gpio_config_t io = {.pin_bit_mask = 1ull << PIN_BTN, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io);
    int stable = 1, count = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        int lvl = gpio_get_level(PIN_BTN);
        if (lvl == stable) {
            count = 0;
            continue;
        }
        if (++count < 4) continue; /* 40 ms debounce */
        stable = lvl;
        count = 0;
        if (stable == 0) { /* pressed */
            if (stepper_running()) {
                stepper_stop();
            } else if (g_cfg.max_steps > 0) {
                move_to_percent(pct_from_steps(stepper_pos()) > 50 ? 0 : 100);
            }
        }
    }
}

/* ------------------------------------------------------------------ main */
void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    load_all();
    apply_timezone();

    stepper_init(PIN_STEP, PIN_DIR, PIN_EN, g_cfg.invert);
    stepper_set_speed(g_cfg.speed, g_cfg.accel);
    stepper_set_idle_cb(save_position);
    nvs_handle_t h;
    int32_t pos = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i32(h, "pos", &pos);
        nvs_close(h);
    }
    stepper_set_pos(pos);

    wifi_start_or_portal(); /* returns only when connected */

    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_init();

    ESP_ERROR_CHECK(mdns_init());
    mdns_hostname_set(HOSTNAME);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);

    start_main_server();
    xTaskCreate(button_task, "button", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "ready: http://%s.local", HOSTNAME);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        check_schedules();
    }
}
