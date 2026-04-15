/*
 * ESP32-S3 — ESP-NOW Beacon Transmitter
 *
 * Sends broadcast ESP-NOW frames on a fixed channel.
 * ESP-NOW uses properly formatted 802.11 Action frames which trigger
 * real CSI capture on the receiving nodes (unlike raw esp_wifi_80211_tx
 * which doesn't reliably produce non-zero I/Q values).
 *
 * Receiver only needs promiscuous mode + CSI enabled — no ESP-NOW
 * stack required on the receiver side.
 *
 * Control:
 *   The Nucleo controls when the cal signal reaches sensor nodes by toggling
 *   the BGS12WN6 RF switches (PF_1).  This transmitter runs at fixed 100 Hz
 *   continuously — the switches handle the gating, not this firmware.
 */

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

#define WIFI_CHAN       6
#define TX_INTERVAL_MS  10   // 100 Hz — fast enough for good CSI snapshots

static uint8_t broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup()
{
    Serial.begin(115200);
    delay(500);   // brief settle — do NOT block on !Serial (no monitor = hang)

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    esp_wifi_set_max_tx_power(78);  // 78 = ~19.5 dBm (hardware maximum)

    // Lock to the correct channel before ESP-NOW init
    esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[TX] ERROR: ESP-NOW init failed");
        return;
    }

    // Register broadcast peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_addr, 6);
    peer.channel = WIFI_CHAN;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK)
    {
        Serial.println("[TX] ERROR: add peer failed");
        return;
    }

    Serial.printf("[TX] ESP-NOW beacon on ch%d @ %dHz  max TX power set\n",
                  WIFI_CHAN, 1000 / TX_INTERVAL_MS);
}

void loop()
{
    static uint32_t seq = 0;

    // Payload just needs to exist — content doesn't matter for CSI
    uint8_t data[4];
    data[0] = 0xDE;               // magic
    data[1] = 0xAD;
    data[2] = (seq >> 8) & 0xFF;
    data[3] =  seq       & 0xFF;
    seq++;

    esp_err_t result = esp_now_send(broadcast_addr, data, sizeof(data));

    // Only print errors and a heartbeat every 100 packets (~1 s at 100 Hz)
    if (result != ESP_OK)
        Serial.printf("[TX] send error: %s\n", esp_err_to_name(result));
    else if (seq % 100 == 0)
        Serial.printf("[TX] alive — seq=%lu\n", seq);

    delay(TX_INTERVAL_MS);
}
