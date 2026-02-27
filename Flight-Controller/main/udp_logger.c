#include "udp_logger.h"
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

static int udp_socket = -1;
static struct sockaddr_in dest_addr;
static const char* TAG = "UDP_LOG";

// FreeRTOS event group to signal when we are connected
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Intercepts ESP_LOG statements
int udp_logging_vprintf(const char *fmt, va_list args) {
    char buffer[512]; // Increased to 512 to handle long error paths/color codes
    
    // Create a copy of the arguments for the first use
    va_list args_copy;
    va_copy(args_copy, args);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args_copy);
    va_end(args_copy); // Always clean up the copy
    
    if (udp_socket >= 0 && len > 0) {
        // Use the length returned by vsnprintf, capped at buffer size
        int send_len = (len >= sizeof(buffer)) ? sizeof(buffer) - 1 : len;
        sendto(udp_socket, buffer, send_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }

    // Now the original 'args' is still fresh for the standard vprintf
    return vprintf(fmt, args);
}

// Event handler for Wi-Fi connection and IP assignment
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected or failed. Retrying...");
        esp_wifi_connect(); // Keep retrying infinitely
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Signal that we are ready
    }
}

// Internal Wi-Fi setup
static void wifi_init_sta(const char* ssid, const char* password) {
    s_wifi_event_group = xEventGroupCreate();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Register our event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID: %s...", ssid);
    
    // Completely freeze the boot process until the WIFI_CONNECTED_BIT is set
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

// The single public function called from main.c
void init_wifi_and_udp_logger(const char* ssid, const char* password, const char* target_ip, int target_port) {
    // 1. Connect to Wi-Fi (This will now block until an IP is acquired)
    wifi_init_sta(ssid, password);

    // 2. Setup UDP Socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        return;
    }

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(target_port);
    inet_pton(AF_INET, target_ip, &dest_addr.sin_addr.s_addr);

    // 3. Hijack ESP_LOG
    esp_log_set_vprintf(udp_logging_vprintf);
    ESP_LOGI(TAG, "UDP Logging Initialized! Routing to %s:%d", target_ip, target_port);
}