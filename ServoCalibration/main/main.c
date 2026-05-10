#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "mdns.h"

static const char *TAG = "SERVO_TUNER";

// --- PIN ASSIGNMENTS ---
#define PIN_SERVO_LEFT_ELEVATOR  13
#define PIN_SERVO_RIGHT_ELEVATOR 14
#define PIN_SERVO_RUDDER         8
#define PIN_SERVO_LEFT_AILERON   5
#define PIN_SERVO_RIGHT_AILERON  16

static mcpwm_cmpr_handle_t cmpr_le = NULL;
static mcpwm_cmpr_handle_t cmpr_re = NULL;
static mcpwm_cmpr_handle_t cmpr_rud = NULL;
static mcpwm_cmpr_handle_t cmpr_la = NULL;
static mcpwm_cmpr_handle_t cmpr_ra = NULL;

// --- WEB INTERFACE ---
static const char* tuner_html = 
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>"
    "body{font-family:sans-serif; margin:20px; background:#1e1e1e; color:#fff;}"
    ".card{background:#333; padding:15px; border-radius:8px; margin-bottom:15px;}"
    ".row{display:flex; align-items:center; gap:10px; margin-top:10px;}"
    "input[type=number]{width:75px; padding:8px; background:#222; color:#00adb5; font-weight:bold; border:1px solid #555; border-radius:4px; text-align:center;}"
    "input[type=range]{flex-grow:1; accent-color:#00adb5; height:20px;}"
    "span{color:#00adb5; font-weight:bold;}"
    "label{font-size: 1.1em;}"
    "</style></head><body>"
    "<h2>Plane Servo Tuner</h2>"
    "<div id='ui'></div>"
    "<script>"
    "const servos = ["
    "  {id:'le', name:'Left Elevator'}, {id:'re', name:'Right Elevator'},"
    "  {id:'rud', name:'Rudder'},"
    "  {id:'la', name:'Left Aileron'}, {id:'ra', name:'Right Aileron'}"
    "];"
    "let html = '';"
    "servos.forEach(s => {"
    "  html += `<div class='card'>` +"
    "  `<label>${s.name}: <span id='${s.id}_val'>1500</span> us</label>` +"
    "  `<div class='row'>` +"
    "  `<input type='number' id='${s.id}_min' value='800' title='Min Limit' onchange='updateBounds(\"${s.id}\")'>` +"
    "  `<input type='range' id='${s.id}_slider' min='800' max='2200' value='1500' oninput='update(\"${s.id}\", this.value)'>` +"
    "  `<input type='number' id='${s.id}_max' value='2200' title='Max Limit' onchange='updateBounds(\"${s.id}\")'>` +"
    "  `</div></div>`;"
    "});"
    "document.getElementById('ui').innerHTML = html;"
    
    "function updateBounds(id) {"
    "  let minVal = document.getElementById(id+'_min').value;"
    "  let maxVal = document.getElementById(id+'_max').value;"
    "  let slider = document.getElementById(id+'_slider');"
    "  slider.min = minVal;"
    "  slider.max = maxVal;"
    "}"
    
    "function update(id, val) {"
    "  document.getElementById(id+'_val').innerText = val;"
    "  fetch(`/set?${id}=${val}`);" 
    "}"
    "</script></body></html>";

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, tuner_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_get_handler(httpd_req_t *req) {
    char buf[128];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val_str[16];
        if (httpd_query_key_value(buf, "le", val_str, sizeof(val_str)) == ESP_OK) mcpwm_comparator_set_compare_value(cmpr_le, atoi(val_str));
        if (httpd_query_key_value(buf, "re", val_str, sizeof(val_str)) == ESP_OK) mcpwm_comparator_set_compare_value(cmpr_re, atoi(val_str));
        if (httpd_query_key_value(buf, "rud", val_str, sizeof(val_str)) == ESP_OK) mcpwm_comparator_set_compare_value(cmpr_rud, atoi(val_str));
        if (httpd_query_key_value(buf, "la", val_str, sizeof(val_str)) == ESP_OK) mcpwm_comparator_set_compare_value(cmpr_la, atoi(val_str));
        if (httpd_query_key_value(buf, "ra", val_str, sizeof(val_str)) == ESP_OK) mcpwm_comparator_set_compare_value(cmpr_ra, atoi(val_str));
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// --- HARDWARE SETUP ---
static void setup_servo(int group_id, mcpwm_timer_handle_t timer, int gpio_num, mcpwm_cmpr_handle_t *cmpr) {
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t op_cfg = { .group_id = group_id }; 
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, cmpr));

    mcpwm_gen_handle_t gen;
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = gpio_num };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*cmpr, 1500)); // Start centered
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *cmpr, MCPWM_GEN_ACTION_LOW)));
}

void init_pwm() {
    mcpwm_timer_handle_t timer0 = NULL, timer1 = NULL;
    mcpwm_timer_config_t t_cfg0 = { .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, .resolution_hz = 1000000, .period_ticks = 20000, .count_mode = MCPWM_TIMER_COUNT_MODE_UP };
    mcpwm_timer_config_t t_cfg1 = { .group_id = 1, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, .resolution_hz = 1000000, .period_ticks = 20000, .count_mode = MCPWM_TIMER_COUNT_MODE_UP };
    
    ESP_ERROR_CHECK(mcpwm_new_timer(&t_cfg0, &timer0));
    ESP_ERROR_CHECK(mcpwm_new_timer(&t_cfg1, &timer1));

    setup_servo(0, timer0, PIN_SERVO_LEFT_ELEVATOR, &cmpr_le);
    setup_servo(0, timer0, PIN_SERVO_RIGHT_ELEVATOR, &cmpr_re);
    setup_servo(0, timer0, PIN_SERVO_RUDDER, &cmpr_rud);
    setup_servo(1, timer1, PIN_SERVO_LEFT_AILERON, &cmpr_la);
    setup_servo(1, timer1, PIN_SERVO_RIGHT_AILERON, &cmpr_ra);

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer0));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer1));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Web Servo Tuner...");
    
    // Initialize NVS and PWM
    nvs_flash_init();
    init_pwm();

    // Start Wi-Fi
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Hos Therese IoT",
            .password = "S7jodi4n",
        },
    };
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start(); 
    esp_wifi_connect();

    // Start mDNS
    mdns_init();
    mdns_hostname_set("servo-tuner");

    // Start Web Server
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &server_config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t uri_set = { .uri = "/set", .method = HTTP_GET, .handler = set_get_handler };
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_set);
        ESP_LOGI(TAG, "Web server running! Go to http://servo-tuner.local");
    }
}