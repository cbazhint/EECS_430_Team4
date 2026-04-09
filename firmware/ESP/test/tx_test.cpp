/*
 * ESP32-S3-DevKitC-1U — Raw 802.11 Transmitter
 * Receiver only needs promiscuous mode — no ESP-NOW stack required.
 */

#include <WiFi.h>
#include "esp_wifi.h"

#define NODE_ID    0
#define WIFI_CHAN  6
#define MAGIC      0xDE  // filter byte so receiver ignores unrelated frames

// Minimal 802.11 data frame header (24 bytes)
static const uint8_t frame_header[] = {
    0x08, 0x00,                          // Frame Control: data frame
    0x00, 0x00,                          // Duration
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // Destination: broadcast
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, // Source MAC (placeholder)
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // BSSID: broadcast
    0x00, 0x00                           // Sequence control
};

typedef struct __attribute__((packed)) {
    uint8_t  magic;          // 0xDE — lets receiver filter quickly
    uint8_t  node_id;
    uint32_t timestamp_us;
    float    phase_rad;
    uint8_t  checksum;       // XOR of preceding bytes
} Payload;

void build_and_send()
{
    Payload p;
    p.magic        = MAGIC;
    p.node_id      = NODE_ID;
    p.timestamp_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
    p.phase_rad    = 1.234f;   // replace with real CSI phase

    // Checksum over everything except the checksum byte itself
    uint8_t chk = 0;
    uint8_t *raw = (uint8_t *)&p;
    for (size_t i = 0; i < sizeof(p) - 1; i++) chk ^= raw[i];
    p.checksum = chk;

    // Assemble full frame: header + payload
    uint8_t frame[sizeof(frame_header) + sizeof(p)];
    memcpy(frame, frame_header, sizeof(frame_header));
    memcpy(frame + sizeof(frame_header), &p, sizeof(p));

    esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), false);
}

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[TX] Raw frame transmitter ready on ch%d\n", WIFI_CHAN);
}

void loop()
{
    build_and_send();
    Serial.println("[TX] Frame sent");
    delay(100);
}