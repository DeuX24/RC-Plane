#include "remote_ota.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "mdns.h"

static const char *TAG = "REMOTE_OTA";

static const char* upload_html = 
    "<html><body><h2>RC Remote OTA Update</h2>"
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='update' accept='.bin'>"
    "<input type='submit' value='Flash Remote'></form></body></html>";

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, upload_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t update_post_handler(httpd_req_t *req) {
    esp_ota_handle_t ota_handle;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    
    if (update_partition == NULL) return ESP_FAIL;

    esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);

    char buf[1024];
    int remaining = req->content_len;

    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf, (remaining > sizeof(buf)) ? sizeof(buf) : remaining);
        if (recv_len <= 0) {
            esp_ota_end(ota_handle);
            return ESP_FAIL;
        }
        esp_ota_write(ota_handle, buf, recv_len);
        remaining -= recv_len;
    }

    if (esp_ota_end(ota_handle) == ESP_OK) {
        esp_ota_set_boot_partition(update_partition);
        httpd_resp_sendstr(req, "Remote Updated! Rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    return ESP_OK;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "OTA Ready! Go to http://" IPSTR " in your browser.", IP2STR(&event->ip_info.ip));
    }
}

void start_home_ota_mode(void) {
    ESP_LOGI(TAG, "Entering Maintenance Mode. Connecting to home Wi-Fi...");
    
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Hos Therese IoT",
            .password = "S7jodi4n",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Unlock the channel so it can find home router
    esp_wifi_set_channel(0, WIFI_SECOND_CHAN_NONE);
    
    esp_wifi_start(); 
    esp_wifi_connect();

    // Initialize mDNS so we can connect via hostname instead of IP
    if (mdns_init() == ESP_OK) {
        mdns_hostname_set("rc-remote");
        mdns_instance_name_set("RC Transmitter OTA Server");
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t uri_update = { .uri = "/update", .method = HTTP_POST, .handler = update_post_handler };
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_update);
    }
}