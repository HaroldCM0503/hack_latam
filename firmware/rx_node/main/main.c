// WiFi-CSI Rx node (ESP-IDF).
//
// Joins the laptop's WiFi AP and runs in promiscuous mode in parallel. For
// each received packet from the designated Tx MAC, extracts CSI, computes a
// motion score against a rolling per-Rx baseline, and sends one JSON line
// over UDP to the laptop.
//
// Each of the three boards is the same firmware with a different RX_ID
// (1, 2, or 3). Set it either with a build flag (-DRX_ID=2) or by editing
// the #define below.
//
// Build/flash (per board, with ESP-IDF environment configured):
//   idf.py set-target esp32
//   idf.py build flash monitor
//
// Edit WIFI_SSID / WIFI_PASSWORD / LAPTOP_IP / TX_MAC below before flashing.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

// ---------- EDIT FOR YOUR SETUP ----------
#define WIFI_SSID       "LAPTOP-E9GMR8ST"
#define WIFI_PASSWORD   "1234567891"
#define LAPTOP_IP       "192.168.137.1"     // Windows hotspot default
#define LAPTOP_PORT     5005

// Tx MAC: read it from the Tx node's serial on first boot, paste here.
static const uint8_t TX_MAC[6] = { 0xCC, 0x8D, 0xA2, 0xED, 0xB6, 0xF0 };

// 1, 2, or 3 - MUST DIFFER between the three Rx boards.
#ifndef RX_ID
#define RX_ID 1
#endif
// -----------------------------------------

static const char *TAG = "RX";

#define MAX_SUBC       64
#define BASELINE_LEN   30

static int  udp_sock = -1;
static struct sockaddr_in laptop_addr;

// Rolling per-subcarrier amplitude baseline (EWMA).
static float baseline_amp[MAX_SUBC] = { 0 };
static int   baseline_count         = 0;

// ---- Diagnostics counters (logged every 2 s) ----
// csi_total          : every CSI callback (any source MAC, any state)
// csi_first_invalid  : frames the radio flagged as decode-failures
// csi_mac_match      : passed our TX_MAC filter
// udp_ok / udp_fail  : UDP transmission outcome
// last_channel       : channel of the most recent matched CSI frame
// other_mac_count    : non-Tx MACs seen (helps spot AP/STA frames vs the
//                      Tx node, and detect MAC typos)
static volatile uint32_t csi_total         = 0;
static volatile uint32_t csi_first_invalid = 0;
static volatile uint32_t csi_mac_match     = 0;
static volatile uint32_t udp_ok            = 0;
static volatile uint32_t udp_fail          = 0;
static volatile uint8_t  last_channel      = 0;
static volatile uint8_t  last_other_mac[6] = { 0 };
static volatile uint32_t other_mac_count   = 0;

// ---------- WiFi STA bring-up ----------
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP");
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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Push UDP back to the laptop at max legal power; helps when the
    // promiscuous radio is busy demodulating Tx packets simultaneously.
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));
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

// ---------- CSI callback ----------
// The Tx broadcasts ESP-NOW (action management frames) - source MAC = TX_MAC.
// For each received packet from that MAC we compute amplitude per subcarrier,
// derive a motion-score against a rolling baseline, and ship it.
static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    csi_total++;

    // Track non-Tx-MAC sources so we can sanity-check the filter at runtime.
    if (memcmp(info->mac, TX_MAC, 6) != 0) {
        other_mac_count++;
        memcpy((void *)last_other_mac, info->mac, 6);
        return;
    }
    if (info->len < 4 || info->len > MAX_SUBC * 2) return;

    // Drop frames the radio flagged as decode-failures - the first CSI word
    // is the leading subcarrier; if invalid the whole vector is garbage and
    // would pollute the rolling baseline. This is the esp-csi reference's
    // first-line-of-defence sanity check.
    if (info->first_word_invalid) {
        csi_first_invalid++;
        return;
    }
    csi_mac_match++;
    last_channel = info->rx_ctrl.channel;

    int n_subc = info->len / 2;
    float amp[MAX_SUBC];
    int8_t raw_real[MAX_SUBC];
    int8_t raw_imag[MAX_SUBC];
    for (int i = 0; i < n_subc; i++) {
        int8_t imag = info->buf[i * 2];
        int8_t real = info->buf[i * 2 + 1];
        raw_real[i] = real;
        raw_imag[i] = imag;
        amp[i] = sqrtf((float)real * real + (float)imag * imag);
    }

    // ---- Motion score against rolling baseline ----
    float score = 0.0f;
    if (baseline_count >= BASELINE_LEN) {
        float diff_sq = 0.0f, base_sq = 0.0f;
        for (int i = 0; i < n_subc; i++) {
            float d = amp[i] - baseline_amp[i];
            diff_sq += d * d;
            base_sq += baseline_amp[i] * baseline_amp[i];
        }
        if (base_sq > 1.0f) {
            score = sqrtf(diff_sq / base_sq);
        }
    }
    // EWMA baseline update
    float alpha = (baseline_count < BASELINE_LEN)
                    ? 1.0f / (float)(baseline_count + 1)
                    : 1.0f / (float)BASELINE_LEN;
    for (int i = 0; i < n_subc; i++) {
        baseline_amp[i] += alpha * (amp[i] - baseline_amp[i]);
    }
    if (baseline_count < BASELINE_LEN) baseline_count++;

    // ---- Build JSON ----
    // Compact - only include `amp` array if you need the per-subcarrier waterfall
    // visualisation downstream. Comment out the amp section to save bandwidth.
    char buf[2048];
    int  n = snprintf(buf, sizeof(buf),
                      "{\"rx\":%d,\"t\":%lu,\"rssi\":%d,\"score\":%.4f,\"amp\":[",
                      (int)RX_ID,
                      (unsigned long)info->rx_ctrl.timestamp,
                      (int)info->rx_ctrl.rssi,
                      score);
    for (int i = 0; i < n_subc && n < (int)sizeof(buf) - 128; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%s%.1f", (i ? "," : ""), amp[i]);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "],\"real\":[");
    for (int i = 0; i < n_subc && n < (int)sizeof(buf) - 128; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%s%d", (i ? "," : ""), (int)raw_real[i]);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "],\"imag\":[");
    for (int i = 0; i < n_subc && n < (int)sizeof(buf) - 128; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%s%d", (i ? "," : ""), (int)raw_imag[i]);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");

    if (udp_sock >= 0) {
        int r = sendto(udp_sock, buf, n, 0,
                       (struct sockaddr *)&laptop_addr, sizeof(laptop_addr));
        if (r > 0) udp_ok++; else udp_fail++;
    }
}

// ---------- Diagnostics task ----------
// Logs counters every 2 s and resets them, so you can see live rates.
static void stats_task(void *arg) {
    uint8_t tx_chan = 0;
    wifi_second_chan_t sec;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_wifi_get_channel(&tx_chan, &sec);
        ESP_LOGI(TAG,
            "csi_total=%lu match=%lu first_invalid=%lu other_mac=%lu "
            "udp_ok=%lu udp_fail=%lu  | our_chan=%u rx_chan=%u",
            (unsigned long)csi_total,
            (unsigned long)csi_mac_match,
            (unsigned long)csi_first_invalid,
            (unsigned long)other_mac_count,
            (unsigned long)udp_ok,
            (unsigned long)udp_fail,
            (unsigned)tx_chan,
            (unsigned)last_channel);
        if (other_mac_count && csi_mac_match == 0) {
            ESP_LOGW(TAG,
                "Seeing CSI but never matching TX_MAC. Last other MAC: "
                "%02X:%02X:%02X:%02X:%02X:%02X  -- typo in TX_MAC?",
                last_other_mac[0], last_other_mac[1], last_other_mac[2],
                last_other_mac[3], last_other_mac[4], last_other_mac[5]);
        }
        csi_total = 0;
        csi_mac_match = 0;
        csi_first_invalid = 0;
        other_mac_count = 0;
        udp_ok = 0;
        udp_fail = 0;
    }
}

// ---------- Promiscuous + CSI enable ----------
static void start_csi_capture(void) {
    wifi_promiscuous_filter_t filt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    // Capture both the legacy LTF and HT-LTF training fields and let the
    // radio average them ("LTF merge"). This is the configuration used by
    // espressif/esp-csi's `csi_recv` example - it produces the cleanest CSI
    // vectors out of the box. channel_filter_en applies the radio's matched
    // filter and reduces broadband noise floor.
    wifi_csi_config_t csi_cfg = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "=== Rx node #%d starting ===", (int)RX_ID);

    wifi_init_sta();
    // Wait for IP
    vTaskDelay(pdMS_TO_TICKS(5000));

    udp_init();
    start_csi_capture();

    ESP_LOGI(TAG, "Capturing CSI, streaming to %s:%d", LAPTOP_IP, LAPTOP_PORT);

    xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, 5, NULL, 0);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
