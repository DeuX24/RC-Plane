#ifndef UDP_LOGGER_H
#define UDP_LOGGER_H

// Initializes Wi-Fi, connects, sets up the UDP socket, and redirects ESP_LOG
void init_wifi_and_udp_logger(const char* ssid, const char* password, const char* target_ip, int target_port);

#endif