#include "stepper.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VMIN 150u /* start/stop speed, steps/s */

static gptimer_handle_t s_timer;
static int s_pin_step, s_pin_dir, s_pin_en;
static bool s_invert;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile int32_t s_pos, s_target, s_pending;
static volatile bool s_has_pending, s_running;
static volatile int32_t s_dir = 1;
/* speed is kept as steps/s * 256 so the ramp can use integer math only */
static volatile uint32_t s_v256, s_vmax256 = 3000u * 256u, s_accel = 2000;
static bool s_powered;
static void (*s_idle_cb)(void);

static inline void IRAM_ATTR set_dir_pin(int32_t d)
{
    gpio_set_level(s_pin_dir, ((d > 0) ? 1 : 0) ^ (s_invert ? 1 : 0));
}

/* ISR: one call = one step. Re-arms itself until the target is reached. */
static bool IRAM_ATTR on_alarm(gptimer_handle_t t, const gptimer_alarm_event_data_t *e, void *ctx)
{
    uint32_t period;

    portENTER_CRITICAL_ISR(&s_mux);
    int32_t rem = (s_target - s_pos) * s_dir;

    if (rem <= 0) {
        if (s_has_pending && s_pending != s_pos) {
            /* finished decelerating toward a reversal: turn around */
            s_target = s_pending;
            s_has_pending = false;
            s_dir = (s_target > s_pos) ? 1 : -1;
            set_dir_pin(s_dir);
            s_v256 = VMIN * 256u;
            period = 3000; /* short pause so DIR settles */
        } else {
            s_has_pending = false;
            s_running = false;
            portEXIT_CRITICAL_ISR(&s_mux);
            return false; /* no re-arm: timer idles */
        }
    } else {
        gpio_set_level(s_pin_step, 1);
        esp_rom_delay_us(2);
        gpio_set_level(s_pin_step, 0);
        s_pos += s_dir;
        rem--;

        uint32_t v = s_v256;
        uint32_t a = s_accel;
        uint32_t dv = (uint32_t)(((uint64_t)a << 16) / v);               /* a/v in x256 units */
        uint64_t stop = ((uint64_t)v * v) / (131072ull * (uint64_t)a);   /* v^2 / 2a steps */

        if ((uint64_t)rem <= stop) {
            v = (v > dv + VMIN * 256u) ? (v - dv) : (VMIN * 256u);
        } else if (v < s_vmax256) {
            v += dv;
            if (v > s_vmax256) v = s_vmax256;
        } else if (v > s_vmax256) {
            v = s_vmax256;
        }
        s_v256 = v;
        period = 256000000u / v;
    }
    portEXIT_CRITICAL_ISR(&s_mux);

    uint64_t now;
    gptimer_get_raw_count(t, &now);
    uint64_t next = e->alarm_value + period;
    if (next < now + 10) next = now + 10; /* never schedule in the past */
    gptimer_alarm_config_t cfg = {.alarm_count = next, .reload_count = 0};
    cfg.flags.auto_reload_on_alarm = false;
    gptimer_set_alarm_action(t, &cfg);
    return false;
}

/* Arm the first step. Call with s_mux held. */
static void arm_first_step(void)
{
    uint64_t now;
    gptimer_get_raw_count(s_timer, &now);
    gptimer_alarm_config_t cfg = {.alarm_count = now + 256000000u / s_v256, .reload_count = 0};
    cfg.flags.auto_reload_on_alarm = false;
    gptimer_set_alarm_action(s_timer, &cfg);
}

/* Steps needed to stop from the current speed (+1 margin). */
static int32_t stop_distance(void)
{
    uint64_t v = s_v256;
    return (int32_t)((v * v) / (131072ull * (uint64_t)s_accel)) + 1;
}

static void idle_task(void *arg)
{
    bool prev = false;
    int64_t t_stop = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        bool r = s_running;
        if (prev && !r) t_stop = esp_timer_get_time();
        prev = r;
        if (!r && s_powered && (esp_timer_get_time() - t_stop) > 500000) {
            gpio_set_level(s_pin_en, 1); /* release the motor: cool and silent */
            s_powered = false;
            if (s_idle_cb) s_idle_cb();
        }
    }
}

void stepper_init(int pin_step, int pin_dir, int pin_en, bool invert_dir)
{
    s_pin_step = pin_step;
    s_pin_dir = pin_dir;
    s_pin_en = pin_en;
    s_invert = invert_dir;

    gpio_config_t io = {
        .pin_bit_mask = (1ull << pin_step) | (1ull << pin_dir) | (1ull << pin_en),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(pin_step, 0);
    gpio_set_level(pin_dir, 0);
    gpio_set_level(pin_en, 1); /* TMC2209 EN is active low: start disabled */

    gptimer_config_t tc = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tc, &s_timer));
    gptimer_event_callbacks_t cbs = {.on_alarm = on_alarm};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_timer));
    ESP_ERROR_CHECK(gptimer_start(s_timer));

    xTaskCreate(idle_task, "stepper_idle", 3072, NULL, 5, NULL);
}

void stepper_set_speed(uint32_t speed, uint32_t accel)
{
    if (speed < VMIN) speed = VMIN;
    if (accel < 50) accel = 50;
    s_vmax256 = speed * 256u;
    s_accel = accel;
}

void stepper_move_to(int32_t t)
{
    if (!s_powered) {
        gpio_set_level(s_pin_en, 0);
        s_powered = true;
        esp_rom_delay_us(1000); /* let the driver wake up */
    }

    portENTER_CRITICAL(&s_mux);
    if (!s_running) {
        if (t != s_pos) {
            s_target = t;
            s_dir = (t > s_pos) ? 1 : -1;
            set_dir_pin(s_dir);
            s_v256 = VMIN * 256u;
            s_has_pending = false;
            s_running = true;
            arm_first_step();
        }
    } else {
        int32_t sd = stop_distance();
        int32_t fwd = (t - s_pos) * s_dir; /* >0: still ahead of us */
        if (fwd >= sd) {
            s_target = t; /* same direction, far enough to stop in time */
            s_has_pending = false;
        } else {
            /* reversal, or too close to stop in time: brake first, then go */
            s_target = s_pos + s_dir * sd;
            s_has_pending = (t != s_target);
            s_pending = t;
        }
    }
    portEXIT_CRITICAL(&s_mux);
}

void stepper_move_rel(int32_t delta)
{
    int32_t base = s_running ? s_target : s_pos;
    stepper_move_to(base + delta);
}

void stepper_stop(void)
{
    portENTER_CRITICAL(&s_mux);
    if (s_running) {
        s_target = s_pos + s_dir * stop_distance();
        s_has_pending = false;
    }
    portEXIT_CRITICAL(&s_mux);
}

bool stepper_running(void) { return s_running; }
int32_t stepper_pos(void) { return s_pos; }
int32_t stepper_target(void) { return s_running ? s_target : s_pos; }

void stepper_set_pos(int32_t pos)
{
    if (!s_running) s_pos = pos;
}

void stepper_set_idle_cb(void (*cb)(void)) { s_idle_cb = cb; }
