/*
 * ESP32-S3 CSI Capture and UART Forwarding
 * Phase-Coherent Direction of Arrival (DoA) System
 *
 * Each ESP32-S3 node:
 *   1. Initializes WiFi in promiscuous (monitor) mode
 *   2. Enables CSI collection on all received packets
 *   3. Extracts I/Q data and computes phase and timestamp
 *   4. Sends a compact binary packet over UART1 to the Nucleo-H753ZI
 *
 * Channel behavior:
 *   - CHANNEL_MODE_FIXED:  stays on WIFI_CHANNEL (use when you know the channel)
 *   - CHANNEL_MODE_HOP:    cycles through channels 1-13, dwells on each for
 *                           CHANNEL_DWELL_MS. Keeps hopping continuously.
 *   - CHANNEL_MODE_LOCK:   hops until packets are found, then locks permanently.
 *                           This is the default - finds traffic automatically
 *                           and stays put so all 8 nodes converge on the same
 *                           channel without reflashing.
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
#define CSI_SUBCARRIER_IDX  0
#define SYNC_BYTE           0xAA
#define PACKET_SIZE         11

/* Channel configuration */
#define CHANNEL_MODE_FIXED  0
#define CHANNEL_MODE_HOP    1
#define CHANNEL_MODE_LOCK   2

#define CHANNEL_MODE        CHANNEL_MODE_LOCK
#define WIFI_CHANNEL        6        /* used only in FIXED mode */
#define CHANNEL_MIN         1
#define CHANNEL_MAX         13
#define CHANNEL_DWELL_MS    200      /* ms to listen per channel before hopping */

/* ============================================================
   Globals
   ============================================================ */
static volatile bool     csi_data_ready = false;
static volatile float    csi_phase      = 0.0f;
static volatile uint32_t csi_timestamp  = 0;

static uint8_t  current_channel    = CHANNEL_MIN;
static bool     channel_locked     = false;
static uint32_t last_hop_time      = 0;
static uint32_t packets_on_channel = 0;

/* ============================================================
   CSI Callback - called from WiFi task context
   ============================================================ */
void IRAM_ATTR csi_rx_callback(void *ctx, wifi_csi_info_t *info)
{
    if (info == NULL || info->buf == NULL || info->len < 2)
    {
        return;
    }

    uint32_t ts = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);

    int idx = CSI_SUBCARRIER_IDX * 2;
    if (idx + 1 >= info->len)
    {
        return;
    }

    int8_t imag = (int8_t)info->buf[idx];
    int8_t real_part = (int8_t)info->buf[idx + 1];

    if (imag == 0 && real_part == 0)
    {
        return;
    }

    float phase = atan2f((float)imag, (float)real_part);

    csi_phase      = phase;
    csi_timestamp  = ts;
    csi_data_ready = true;
    packets_on_channel++;
}

/* ============================================================
   Channel Management
   ============================================================ */
void set_channel(uint8_t ch)
{
    current_channel = ch;
    packets_on_channel = 0;
    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
}

void hop_to_next_channel()
{
    uint8_t next = current_channel + 1;
    if (next > CHANNEL_MAX)
    {
        next = CHANNEL_MIN;
    }
    set_channel(next);
    Serial.printf("[Node %d] Hopped to channel %d\n", NODE_ID, current_channel);
}

void update_channel()
{
    if (CHANNEL_MODE == CHANNEL_MODE_FIXED || channel_locked)
    {
        return;
    }

    uint32_t now = millis();

    /* Lock mode: if we got packets on this channel, lock here */
    if (CHANNEL_MODE == CHANNEL_MODE_LOCK && packets_on_channel > 0)
    {
        channel_locked = true;
        Serial.printf("[Node %d] Locked to channel %d (%lu packets)\n",
                      NODE_ID, current_channel, packets_on_channel);
        return;
    }

    /* Dwell time elapsed with no packets - hop */
    if (now - last_hop_time >= CHANNEL_DWELL_MS)
    {
        last_hop_time = now;
        hop_to_next_channel();
    }
}

/* ============================================================
   WiFi + CSI Initialization
   ============================================================ */
void setup_wifi_promiscuous()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    esp_wifi_set_promiscuous(true);

    /* Set initial channel */
    if (CHANNEL_MODE == CHANNEL_MODE_FIXED)
    {
        set_channel(WIFI_CHANNEL);
    }
    else
    {
        set_channel(CHANNEL_MIN);
        last_hop_time = millis();
    }

    wifi_csi_config_t csi_config;
    csi_config.lltf_en           = true;
    csi_config.htltf_en          = true;
    csi_config.stbc_htltf2_en    = true;
    csi_config.ltf_merge_en      = true;
    csi_config.channel_filter_en = false;
    csi_config.manu_scale        = false;
    csi_config.shift             = false;

    esp_wifi_set_csi_config(&csi_config);
    esp_wifi_set_csi_rx_cb(csi_rx_callback, NULL);
    esp_wifi_set_csi(true);
}

/* ============================================================
   UART Packet Transmission
   ============================================================ */
void send_csi_packet(float phase, uint32_t timestamp)
{
    uint8_t packet[PACKET_SIZE];

    packet[0] = SYNC_BYTE;
    packet[1] = NODE_ID;

    packet[2] = (uint8_t)(timestamp & 0xFF);
    packet[3] = (uint8_t)((timestamp >> 8) & 0xFF);
    packet[4] = (uint8_t)((timestamp >> 16) & 0xFF);
    packet[5] = (uint8_t)((timestamp >> 24) & 0xFF);

    uint8_t *phase_bytes = (uint8_t *)&phase;
    packet[6] = phase_bytes[0];
    packet[7] = phase_bytes[1];
    packet[8] = phase_bytes[2];
    packet[9] = phase_bytes[3];

    uint8_t chk = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++)
    {
        chk ^= packet[i];
    }
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

    if (CHANNEL_MODE == CHANNEL_MODE_FIXED)
    {
        Serial.printf("[Node %d] Fixed on channel %d\n", NODE_ID, WIFI_CHANNEL);
    }
    else if (CHANNEL_MODE == CHANNEL_MODE_HOP)
    {
        Serial.printf("[Node %d] Hopping channels %d-%d, dwell %dms\n",
                      NODE_ID, CHANNEL_MIN, CHANNEL_MAX, CHANNEL_DWELL_MS);
    }
    else
    {
        Serial.printf("[Node %d] Scanning for traffic, will lock on first active channel\n", NODE_ID);
    }

    Serial.printf("[Node %d] CSI capture running\n", NODE_ID);
}

/* ============================================================
   Arduino Main Loop
   ============================================================ */
void loop()
{
    /* Handle channel hopping if enabled */
    update_channel();

    if (csi_data_ready)
    {
        float    phase = csi_phase;
        uint32_t ts    = csi_timestamp;
        csi_data_ready = false;

        send_csi_packet(phase, ts);

        Serial.printf("[Node %d] ch=%d t=%lu phase=%.4f rad\n",
                      NODE_ID, current_channel, ts, phase);
    }

    delay(1);
}