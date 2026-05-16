// WiFi-CSI Tx node (ESP-IDF).
//
// Joins the laptop's WiFi AP, then continuously broadcasts ESP-NOW packets
// (~500 Hz). These packets are what the three Rx nodes will receive in
// promiscuous mode and extract CSI from. Each broadcast is a fixed-length
// management/action frame with a stable source MAC, so the Rx can filter.
//
// Build/flash (with ESP-IDF environment configured):
//   idf.py set-target esp32
//   idf.py menuconfig          # (optional, defaults from sdkconfig.defaults)
//   idf.py build flash monitor
//
// Edit WIFI_SSID / WIFI_PASSWORD below before flashing. On first boot the
// node prints its MAC; copy that into the Rx firmware's TX_MAC constant.

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

// Internal API used by espressif/esp-csi reference: forces a fixed PHY rate so
// every packet trains CSI with identical preamble/modulation. Massively
// improves CSI consistency vs. letting the rate adapter swing.
#include "esp_private/wifi.h"

// ---------- EDIT FOR YOUR SETUP ----------
#define WIFI_SSID       "LAPTOP-E9GMR8ST"
#define WIFI_PASSWORD   "1234567891"
#define TX_INTERVAL_MS  2          // ~500 Hz packet rate
// -----------------------------------------

static const char *TAG = "TX";
static uint8_t bcast_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP, joining AP complete");
    }
}

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char*)wc.sta.ssid,     WIFI_SSID,     sizeof(wc.sta.ssid));
    strncpy((char*)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    // Disable PS so we transmit on a strict schedule.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Max TX power (units of 0.25 dBm; 80 -> 20 dBm = legal max). Stronger
    // signal means better SNR per CSI sample at all three Rx.
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));

    // Force a stable PHY rate so every packet has the same preamble timing
    // and the Rx side gets coherent LLTF+HT-LTF symbols. MCS0 + Long GI is
    // the standard CSI-friendly rate used in espressif/esp-csi.
    ESP_ERROR_CHECK(
        esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, WIFI_PHY_RATE_MCS0_LGI));
}

static void esp_now_init_tx(void) {
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, bcast_mac, 6);
    peer.channel = 0;                       // use the channel of the connected AP
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_sta();

    // Wait until we have IP (best-effort - 5 s)
    vTaskDelay(pdMS_TO_TICKS(5000));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Tx MAC = %02X:%02X:%02X:%02X:%02X:%02X  --  paste into rx_node TX_MAC",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_now_init_tx();

    uint8_t payload[20] = { 'C', 'S', 'I', '-', 'T', 'X' };
    uint32_t cnt        = 0;
    uint32_t sent_ok    = 0;
    uint32_t sent_fail  = 0;
    uint32_t last_log_t = 0;

    uint8_t cur_chan = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&cur_chan, &sec);
    ESP_LOGI(TAG, "Sending on channel %u, broadcast MAC ff:ff:ff:ff:ff:ff", (unsigned)cur_chan);

    while (1) {
        // Counter in the first 4 bytes makes packets unique on the air
        // (so radios don't drop them as duplicates).
        payload[8]  = (cnt >> 24) & 0xff;
        payload[9]  = (cnt >> 16) & 0xff;
        payload[10] = (cnt >>  8) & 0xff;
        payload[11] = (cnt      ) & 0xff;
        cnt++;

        esp_err_t err = esp_now_send(bcast_mac, payload, sizeof(payload));
        if (err == ESP_OK) sent_ok++;
        else               sent_fail++;

        // Every 2 s, log a heartbeat so we can confirm the Tx is alive AND
        // confirm what rate is actually leaving the chip (failures, channel).
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now_ms - last_log_t > 2000) {
            esp_wifi_get_channel(&cur_chan, &sec);
            ESP_LOGI(TAG, "tx ok=%lu fail=%lu cnt=%lu chan=%u",
                     (unsigned long)sent_ok, (unsigned long)sent_fail,
                     (unsigned long)cnt, (unsigned)cur_chan);
            sent_ok = 0;
            sent_fail = 0;
            last_log_t = now_ms;
        }

        // pdMS_TO_TICKS(2) is 0 on a 100Hz tick rate, causing a watchdog crash!
        // We use vTaskDelay(1) to guarantee we wait exactly 1 OS tick (~10ms / 100Hz).
        vTaskDelay(1);
    }
}
