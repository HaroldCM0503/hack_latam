// WiFi-CSI Tx node (ESP-IDF).
//
// Adopts the espressif/esp-csi `csi_send` pattern:
//   - Pins a fixed MAC (0x1a:00:00:00:00:00) so every Rx can filter on it
//     without pasting per-board MACs.
//   - Locks the ESP-NOW PHY rate to MCS0 + Long GI via esp_now_set_peer_rate_config
//     for the most CSI-friendly, deterministic preamble training on the Rx side.
//   - Broadcasts a tiny ESP-NOW payload (a 32-bit sequence counter) at a fixed
//     rate using usleep, which works regardless of FreeRTOS tick rate.
//
// We additionally associate to the laptop hotspot so the Tx channel automatically
// matches whatever channel the AP picked — every Rx is on the same channel by
// virtue of all four nodes joining the same SSID.
//
// Build/flash:
//   cd firmware/tx_node
//   idf.py set-target esp32
//   idf.py build flash monitor

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

// ---------- EDIT FOR YOUR SETUP ----------
#define WIFI_SSID       "debri_net"
#define WIFI_PASSWORD   "12345678901"
// Hz, target ESP-NOW broadcast rate.
// 100 Hz is the espressif/esp-csi csi_send default — comfortably below the
// radio's TX-buffer drain rate for ESP-NOW broadcasts. Raising this further
// causes ieee80211_alloc_tx_buf to busy-spin (IDLE TWDT trips). 100 Hz x
// ~100 ms gate-crossing time still gives 10+ CSI samples per transit, which
// is plenty for bistatic-Fresnel fitting.
#define SEND_FREQUENCY  100
// -----------------------------------------

// Fixed Tx MAC — Rx firmware filters on exactly this value.
// Matches espressif/esp-csi csi_send convention.
static const uint8_t TX_MAC[6] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};

static const uint8_t BCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static const char *TAG = "TX";

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

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Pin our STA MAC BEFORE start, so association + every ESP-NOW frame uses it.
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, TX_MAC));

    wifi_config_t wc = { 0 };
    strncpy((char*)wc.sta.ssid,     WIFI_SSID,     sizeof(wc.sta.ssid));
    strncpy((char*)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

    // Power save off — we transmit on a strict cadence.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Note on bandwidth: we don't force HT40 here. The Windows mobile hotspot
    // advertises BW20, and the STA negotiates down to that anyway. The peer
    // rate config below uses HT20 to match. If you switch to an HT40-capable
    // AP, raise both the peer rate's phymode AND uncomment a
    // esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40) call here.

    ESP_ERROR_CHECK(esp_wifi_start());

    // Max legal Tx power (units of 0.25 dBm). Higher SNR at every Rx.
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));
}

static void wifi_esp_now_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, BCAST_MAC, 6);
    peer.channel = 0;                       // follow the connected AP's channel
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // Lock the peer rate. WIFI_PHY_RATE_MCS0_LGI is the espressif/esp-csi
    // canonical "CSI-friendly" rate: shortest packets, deterministic timing,
    // every frame trains LLTF+HT-LTF with identical modulation.
    //
    // phymode MUST match the radio's actual bandwidth, which is determined by
    // the AP we associated to (Windows mobile hotspot is BW20 by default).
    // Setting HT40 here while the radio is BW20 fails with
    // ESP_ERR_ESPNOW_ARG ("invalid chanel info, need change second channel
    // to 40"). HT20 works on both BW20 and BW40 APs.
    esp_now_rate_config_t rate = {
        .phymode = WIFI_PHY_MODE_HT20,
        .rate    = WIFI_PHY_RATE_MCS0_LGI,
        .ersu    = false,
        .dcm     = false,
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(BCAST_MAC, &rate));
}

// ---------- TX loop ----------
// Runs in its own task pinned to CPU1, so IDLE0 always has CPU0 to run on
// and reset the task watchdog — even if our task happens to busy-wait inside
// the Wi-Fi stack's TX-buffer allocator.
static void tx_task(void *arg) {
    // Period in FreeRTOS ticks. pdMS_TO_TICKS(10) is at least 1 tick on every
    // tick rate ESP-IDF ships with (10 @ 1 kHz, 1 @ 100 Hz), so vTaskDelay
    // never degenerates to a non-sleeping yield.
    const TickType_t period_ticks  = pdMS_TO_TICKS(1000 / SEND_FREQUENCY);
    // On send-failure (typically ESP_ERR_ESPNOW_NO_MEM == TX buffer pool full)
    // we wait noticeably longer before retrying, so the Wi-Fi driver can
    // drain its queue instead of being slammed again immediately.
    const TickType_t backoff_ticks = pdMS_TO_TICKS(5);

    uint32_t sent_ok   = 0;
    uint32_t sent_fail = 0;
    uint32_t last_log  = 0;

    ESP_LOGI(TAG,
        "tx_task on CPU%d  configTICK_RATE_HZ=%lu  period_ticks=%lu (~%lu ms)",
        (int)xPortGetCoreID(),
        (unsigned long)configTICK_RATE_HZ,
        (unsigned long)period_ticks,
        (unsigned long)(period_ticks * portTICK_PERIOD_MS));

    for (uint32_t count = 0; ; ++count) {
        esp_err_t r = esp_now_send(BCAST_MAC, (const uint8_t *)&count, sizeof(count));
        if (r == ESP_OK) {
            sent_ok++;
            vTaskDelay(period_ticks);
        } else {
            sent_fail++;
            vTaskDelay(backoff_ticks);
        }

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now_ms - last_log > 2000) {
            uint8_t cur_chan = 0;
            wifi_second_chan_t sec;
            esp_wifi_get_channel(&cur_chan, &sec);
            ESP_LOGI(TAG, "tx ok=%lu fail=%lu cnt=%lu chan=%u rate~%lu Hz",
                     (unsigned long)sent_ok, (unsigned long)sent_fail,
                     (unsigned long)count, (unsigned)cur_chan,
                     (unsigned long)(sent_ok / 2));
            sent_ok = 0;
            sent_fail = 0;
            last_log = now_ms;
        }
    }
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_sta();

    // Best-effort wait for STA to associate so channel/peer-rate apply cleanly.
    vTaskDelay(pdMS_TO_TICKS(5000));

    wifi_esp_now_init();

    uint8_t cur_chan = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&cur_chan, &sec);
    ESP_LOGI(TAG,
        "================ CSI SEND ================\n"
        "  TX_MAC = %02X:%02X:%02X:%02X:%02X:%02X (matches Rx filter)\n"
        "  channel = %u, target rate = %d Hz, MCS0_LGI",
        TX_MAC[0], TX_MAC[1], TX_MAC[2], TX_MAC[3], TX_MAC[4], TX_MAC[5],
        (unsigned)cur_chan, SEND_FREQUENCY);

    // Pin the TX loop to CPU1. Leaves CPU0 free for IDLE0 + Wi-Fi/lwip work,
    // which avoids the task-watchdog trip that fires when our send loop
    // monopolises CPU0 (whether through busy-wait inside esp_now_send's
    // buffer allocator, or because pdMS_TO_TICKS rounded to 0 ticks).
    xTaskCreatePinnedToCore(tx_task, "tx_loop", 4096, NULL, 5, NULL, 1);
}
