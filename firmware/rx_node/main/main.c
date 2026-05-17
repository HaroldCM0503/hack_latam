// WiFi-CSI Rx node (ESP-IDF) — raw CSI dump via UDP.
//
// This is espressif/esp-csi's get-started/csi_recv_router example, ported into
// our tree with two modifications only:
//
//   1. SSID/password are hardcoded here instead of pulled from the
//      protocol_examples_common menuconfig (so the project is self-contained).
//   2. Each captured CSI frame is shipped as a JSON line over UDP to the
//      laptop on port 5005 — replacing csi_recv_router's CSV-over-UART dump.
//
// The CSI capture path is otherwise IDENTICAL to upstream:
//   - Associate to the AP via standard STA mode.
//   - Filter the CSI callback on the AP's BSSID (so only frames from the AP
//     produce callbacks — same as upstream).
//   - Ping the gateway at 100 Hz to give the AP a steady reason to send
//     reply traffic (gives the CSI callback a steady cadence).
//   - esp_wifi_set_csi_config + esp_wifi_set_csi_rx_cb + esp_wifi_set_csi(true).
//
// JSON schema emitted per frame:
//   {"rx":N, "t":<esp_log_timestamp_ms>, "rssi":<dBm>, "ch":<channel>,
//    "len":<csi_buf_bytes>, "amp":[a0, a1, ..., a_{n_subc-1}]}
//
// Python side (fusion/receiver.py) computes the motion score from `amp`.

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "ping/ping_sock.h"

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define CONFIG_GAIN_CONTROL 1
#include "esp_csi_gain_ctrl.h"
#endif

// ---------- EDIT FOR YOUR SETUP ----------
#define WIFI_SSID         "debri_net"
#define WIFI_PASSWORD     "12345678901"
#define LAPTOP_IP         "192.168.137.1"
#define LAPTOP_PORT       5005
#define PING_FREQUENCY_HZ 100

#ifndef RX_ID
#define RX_ID 1
#endif
// -----------------------------------------

static const char *TAG = "RX";

#define MAX_SUBC          128
static int  udp_sock = -1;
static struct sockaddr_in laptop_addr;

// Diagnostics counters (logged every 2 s, then reset).
static volatile uint32_t csi_cb_total  = 0;
static volatile uint32_t csi_cb_match  = 0;
static volatile uint32_t udp_ok        = 0;
static volatile uint32_t udp_fail      = 0;
static volatile bool     wifi_up       = false;
static          uint8_t  ap_bssid[6]   = { 0 };

// ---------- Wi-Fi STA bring-up ----------
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_up = false;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        wifi_up = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Got IP from %s", WIFI_SSID);
    }
}

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));

    // Block until we actually have an IP. The CSI callback registration
    // needs the AP BSSID, which only exists after association.
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

// ---------- UDP socket ----------
static void udp_init(void) {
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "socket failed");
        return;
    }
    memset(&laptop_addr, 0, sizeof(laptop_addr));
    laptop_addr.sin_family = AF_INET;
    laptop_addr.sin_port   = htons(LAPTOP_PORT);
    inet_aton(LAPTOP_IP, &laptop_addr.sin_addr);
}

// ---------- CSI callback (same shape as upstream csi_recv_router) ----------
// `ctx` is the AP BSSID we registered with esp_wifi_set_csi_rx_cb (so we
// reject frames from any other MAC). CSI buffer layout: pairs of int8
// (imag, real) per subcarrier — same on every ESP32 family we support.
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    csi_cb_total++;

    if (!info || !info->buf) {
        return;
    }
    if (memcmp(info->mac, ctx, 6) != 0) {
        return;
    }
    csi_cb_match++;

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;

    // Optional gain compensation for newer chips (lifted from upstream).
    float compensate_gain = 1.0f;
#if CONFIG_GAIN_CONTROL
    static int s_count = 0;
    static uint8_t agc_gain_baseline = 0;
    static int8_t  fft_gain_baseline = 0;
    uint8_t agc_gain = 0;
    int8_t  fft_gain = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    if (s_count < 100) {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    } else if (s_count == 100) {
        esp_csi_gain_ctrl_get_rx_gain_baseline(&agc_gain_baseline, &fft_gain_baseline);
    }
    esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);
    s_count++;
#endif

    // Compute per-subcarrier amplitude from int8 (imag, real) pairs.
    int n_subc = info->len / 2;
    if (n_subc > MAX_SUBC) n_subc = MAX_SUBC;
    float amp[MAX_SUBC];
    for (int i = 0; i < n_subc; i++) {
        int8_t imag = (int8_t)info->buf[i * 2];
        int8_t real = (int8_t)info->buf[i * 2 + 1];
        float r = compensate_gain * (float)real;
        float m = compensate_gain * (float)imag;
        amp[i] = sqrtf(r * r + m * m);
    }

    if (udp_sock < 0) return;

    // Build the JSON. Sized for 128 subcarriers worst case (each amp <= ~7 chars).
    char buf[1600];
    int  n = snprintf(buf, sizeof(buf),
                      "{\"rx\":%d,\"t\":%lu,\"rssi\":%d,\"ch\":%u,\"len\":%d,\"amp\":[",
                      (int)RX_ID,
                      (unsigned long)esp_log_timestamp(),
                      (int)rx_ctrl->rssi,
                      (unsigned)rx_ctrl->channel,
                      (int)info->len);
    for (int i = 0; i < n_subc && n < (int)sizeof(buf) - 32; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%s%.1f", i ? "," : "", amp[i]);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");
    if (n <= 0 || n >= (int)sizeof(buf)) return;

    int r = sendto(udp_sock, buf, n, 0,
                   (struct sockaddr *)&laptop_addr, sizeof(laptop_addr));
    if (r > 0) udp_ok++; else udp_fail++;
}

// ---------- CSI bring-up (upstream wifi_csi_init, verbatim) ----------
static void wifi_csi_init(void) {
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable                   = true,
        .acquire_csi_legacy       = true,
        .acquire_csi_force_lltf   = 0,
        .acquire_csi_ht20         = true,
        .acquire_csi_ht40         = true,
        .acquire_csi_vht          = false,
        .acquire_csi_su           = false,
        .acquire_csi_mu           = false,
        .acquire_csi_dcm          = false,
        .acquire_csi_beamformed   = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg            = 0,
        .dump_ack_en              = false,
        .reserved                 = false,
    };
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable                 = true,
        .acquire_csi_legacy     = true,
        .acquire_csi_ht20       = true,
        .acquire_csi_ht40       = true,
        .acquire_csi_su         = false,
        .acquire_csi_mu         = false,
        .acquire_csi_dcm        = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc    = 2,
        .val_scale_cfg          = false,
        .dump_ack_en            = false,
        .reserved               = false,
    };
#else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = false,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = true,
        .shift             = true,
    };
#endif

    static wifi_ap_record_t s_ap_info = { 0 };
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&s_ap_info));
    memcpy(ap_bssid, s_ap_info.bssid, 6);

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    // Pass AP BSSID as ctx so the callback filters on it.
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, s_ap_info.bssid));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG,
        "CSI capture started, filter=AP BSSID %02X:%02X:%02X:%02X:%02X:%02X, "
        "streaming JSON to %s:%d",
        ap_bssid[0], ap_bssid[1], ap_bssid[2],
        ap_bssid[3], ap_bssid[4], ap_bssid[5],
        LAPTOP_IP, LAPTOP_PORT);
}

// ---------- Gateway ping (drives the CSI rate) ----------
static void ping_router_start(uint32_t freq_hz) {
    static esp_ping_handle_t ping_handle = NULL;

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.count            = 0;
    cfg.interval_ms      = 1000 / freq_hz;
    cfg.task_stack_size  = 3072;
    cfg.data_size        = 1;

    esp_netif_ip_info_t local_ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
                          &local_ip);
    cfg.target_addr.u_addr.ip4.addr = ip4_addr_get_u32(&local_ip.gw);
    cfg.target_addr.type            = ESP_IPADDR_TYPE_V4;

    esp_ping_callbacks_t cbs = { 0 };
    esp_ping_new_session(&cfg, &cbs, &ping_handle);
    esp_ping_start(ping_handle);

    ESP_LOGI(TAG, "ping: target=" IPSTR " interval=%lu ms (~%lu Hz)",
             IP2STR((esp_ip4_addr_t *)&local_ip.gw),
             (unsigned long)cfg.interval_ms, (unsigned long)freq_hz);
}

// ---------- Diagnostics task ----------
static void stats_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG,
            "rx_id=%d  csi total=%lu match=%lu (~%lu Hz)  "
            "udp ok=%lu fail=%lu  wifi=%s",
            (int)RX_ID,
            (unsigned long)csi_cb_total,
            (unsigned long)csi_cb_match,
            (unsigned long)(csi_cb_match / 2),
            (unsigned long)udp_ok,
            (unsigned long)udp_fail,
            wifi_up ? "UP" : "DOWN");
        csi_cb_total = 0;
        csi_cb_match = 0;
        udp_ok = 0;
        udp_fail = 0;
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Rx node #%d starting ===", (int)RX_ID);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_sta();           // blocks until we have an IP
    udp_init();
    wifi_csi_init();
    ping_router_start(PING_FREQUENCY_HZ);

    xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, 5, NULL, 0);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
