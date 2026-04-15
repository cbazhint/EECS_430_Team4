/*
 * ESP32-S3 CSI Capture and UART Forwarding
 * Phase-Coherent Direction of Arrival (DoA) System
 *
 * Packet format sent to Nucleo (11 bytes):
 *   [0]      SYNC byte       0xAA
 *   [1]      Node ID         0x00 to 0x07
 *   [2..5]   Timestamp       uint32_t microseconds (little-endian)
 *   [6..9]   Phase           float32 radians (little-endian)
 *   [10]     Checksum        XOR of bytes [0..9]
 */

#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_timer.h"
#include <math.h>

/* ============================================================
   CONFIGURATION - adjust per node before flashing
   ============================================================ */
#define NODE_ID             0
#define UART_TX_PIN         17
#define UART_RX_PIN         18
#define UART_BAUD           921600
#define CSI_SUBCARRIER_IDX  10   // 0 is DC (always zero); 1-28 are valid LLTF subcarriers
#define SYNC_BYTE           0xAA
#define PACKET_SIZE         11
#define WIFI_CHANNEL        6    /* fixed channel — must match across all nodes */

/* ============================================================
   Globals
   ============================================================ */
static volatile bool     csi_data_ready = false;
static volatile float    csi_phase      = 0.0f;
static volatile float    csi_amplitude  = 0.0f;
static volatile int8_t   csi_rssi       = 0;
static volatile uint32_t csi_timestamp  = 0;

// Diagnostic counters — incremented in callbacks, printed from loop()
static volatile uint32_t dbg_promisc   = 0;  // raw promiscuous packets seen
static volatile uint32_t dbg_cb_total  = 0;  // CSI callback entries
static volatile uint32_t dbg_cb_null   = 0;  // dropped: null/short buf
static volatile uint32_t dbg_cb_bounds = 0;  // dropped: subcarrier out of range
static volatile uint32_t dbg_cb_zero   = 0;  // dropped: I=0 Q=0
static volatile uint32_t dbg_cb_ok     = 0;  // passed all checks → phase sent

/* ============================================================
   Promiscuous RX callback — counts raw packets, confirms traffic visible
   ============================================================ */
void IRAM_ATTR promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    (void)buf;
    (void)type;
    dbg_promisc++;
}

/* ============================================================
   CSI Callback - called from WiFi task context
   ============================================================ */
void IRAM_ATTR csi_rx_callback(void *ctx, wifi_csi_info_t *info)
{
    dbg_cb_total++;

    if (info == NULL || info->buf == NULL || info->len < 2)
    {
        dbg_cb_null++;
        return;
    }

    uint32_t ts = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);

    int idx = CSI_SUBCARRIER_IDX * 2;
    if (idx + 1 >= info->len)
    {
        dbg_cb_bounds++;
        return;
    }

    int8_t imag      = (int8_t)info->buf[idx];
    int8_t real_part = (int8_t)info->buf[idx + 1];

    if (imag == 0 && real_part == 0)
    {
        dbg_cb_zero++;
        return;
    }

    csi_phase      = atan2f((float)imag, (float)real_part);
    csi_amplitude  = sqrtf((float)imag * imag + (float)real_part * real_part);
    csi_rssi       = info->rx_ctrl.rssi;
    csi_timestamp  = ts;
    csi_data_ready = true;
    dbg_cb_ok++;
}

/* ============================================================
   WiFi + CSI Initialization
   ============================================================ */
void setup_wifi_promiscuous()
{
    esp_err_t err;

    Serial.printf("[Node %d] WiFi.mode(WIFI_STA)...\n", NODE_ID);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    Serial.printf("[Node %d] WiFi mode set\n", NODE_ID);

    err = esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);
    Serial.printf("[Node %d] set_promiscuous_rx_cb: %s\n", NODE_ID, esp_err_to_name(err));

    err = esp_wifi_set_promiscuous(true);
    Serial.printf("[Node %d] set_promiscuous(true): %s\n", NODE_ID, esp_err_to_name(err));

    err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[Node %d] set_channel(%d):       %s\n", NODE_ID, WIFI_CHANNEL, esp_err_to_name(err));

    wifi_promiscuous_filter_t filter = {};
    filter.filter_mask = WIFI_PROMIS_FILTER_MASK_ALL;
    err = esp_wifi_set_promiscuous_filter(&filter);
    Serial.printf("[Node %d] set_promisc_filter:    %s\n", NODE_ID, esp_err_to_name(err));

    wifi_csi_config_t csi_config = {};
    csi_config.lltf_en           = true;
    csi_config.htltf_en          = true;
    csi_config.stbc_htltf2_en    = true;
    csi_config.ltf_merge_en      = true;
    csi_config.channel_filter_en = false;
    csi_config.manu_scale        = false;
    csi_config.shift             = 0;

    err = esp_wifi_set_csi_config(&csi_config);
    Serial.printf("[Node %d] set_csi_config:        %s\n", NODE_ID, esp_err_to_name(err));

    err = esp_wifi_set_csi_rx_cb(csi_rx_callback, NULL);
    Serial.printf("[Node %d] set_csi_rx_cb:         %s\n", NODE_ID, esp_err_to_name(err));

    err = esp_wifi_set_csi(true);
    Serial.printf("[Node %d] set_csi(true):         %s\n", NODE_ID, esp_err_to_name(err));

    Serial.flush();
}

/* ============================================================
   UART Packet Transmission
   ============================================================ */
void send_csi_packet(float phase, uint32_t timestamp)
{
    uint8_t packet[PACKET_SIZE];

    packet[0] = SYNC_BYTE;
    packet[1] = NODE_ID;

    packet[2] = (uint8_t)(timestamp         & 0xFF);
    packet[3] = (uint8_t)((timestamp >>  8) & 0xFF);
    packet[4] = (uint8_t)((timestamp >> 16) & 0xFF);
    packet[5] = (uint8_t)((timestamp >> 24) & 0xFF);

    uint8_t *phase_bytes = (uint8_t *)&phase;
    packet[6] = phase_bytes[0];
    packet[7] = phase_bytes[1];
    packet[8] = phase_bytes[2];
    packet[9] = phase_bytes[3];

    uint8_t chk = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++)
        chk ^= packet[i];
    packet[10] = chk;

    Serial1.write(packet, PACKET_SIZE);
}

/* ============================================================
   Arduino Setup
   ============================================================ */
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.printf("[Node %d] Booting...\n", NODE_ID);

    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[Node %d] UART1 TX=GPIO%d baud=%d\n", NODE_ID, UART_TX_PIN, UART_BAUD);

    setup_wifi_promiscuous();

    Serial.printf("[Node %d] Init complete — ch=%d subcarrier=%d\n",
                  NODE_ID, WIFI_CHANNEL, CSI_SUBCARRIER_IDX);
}

/* ============================================================
   Arduino Main Loop
   ============================================================ */
void loop()
{
    if (csi_data_ready)
    {
        float    phase = csi_phase;
        float    amp   = csi_amplitude;
        int8_t   rssi  = csi_rssi;
        uint32_t ts    = csi_timestamp;
        csi_data_ready = false;

        send_csi_packet(phase, ts);

        Serial.printf("[Node %d] phase=%.4f rad  amp=%.1f  rssi=%d dBm\n",
                      NODE_ID, phase, amp, rssi);
    }

    // Diagnostic dump every 2 seconds
    static uint32_t last_diag = 0;
    if (millis() - last_diag >= 2000)
    {
        last_diag = millis();
        Serial.printf("[Node %d] diag | promisc=%lu | csi total=%lu null=%lu bounds=%lu zero=%lu ok=%lu\n",
                      NODE_ID,
                      dbg_promisc,
                      dbg_cb_total, dbg_cb_null, dbg_cb_bounds, dbg_cb_zero, dbg_cb_ok);
    }

    delay(1);
}
