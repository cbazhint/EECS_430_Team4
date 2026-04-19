/*
 * ESP32-S3 — Gated Calibration Transmitter
 *
 * Sends broadcast ESP-NOW frames on a fixed channel, but ONLY while
 * the ENABLE pin is HIGH.  The Nucleo drives the same GPIO line that
 * controls the relay/RF switch; this ESP32 reads that line so both the
 * switch and the cal transmitter turn on and off together.
 *
 * Wiring:
 *   Nucleo D8  ──┬──  relay/switch CTRL
 *                └──  ESP32 GPIO CAL_EN_PIN   (share the line)
 *   Common GND between Nucleo and cal ESP32 required.
 *
 * CTRL LOW  (cal mode) → relay switches to cal path AND this ESP32 starts beaconing
 * CTRL HIGH (measure)  → relay switches to antenna path AND this ESP32 goes silent
 */

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

#define WIFI_CHAN        6
#define TX_INTERVAL_MS  5       // 200 Hz when enabled
#define CAL_EN_PIN      4       // GPIO connected to Nucleo D8 CTRL line

static uint8_t broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool    esp_now_ready    = false;

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(CAL_EN_PIN, INPUT);   // reads Nucleo D8 — no pull needed (Nucleo drives it)

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Disable 802.11b (DSSS) — DSSS frames have no OFDM subcarriers so the
    // sensor nodes' CSI engine ignores them.  11g/n forces OFDM from the start.
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    esp_wifi_set_max_tx_power(78);   // ~19.5 dBm
    esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[TX] ERROR: ESP-NOW init failed");
        return;
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_addr, 6);
    peer.channel = WIFI_CHAN;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[TX] ERROR: add peer failed");
        return;
    }

    // Force OFDM rate so sensor nodes can extract LLTF CSI.
    // Default ESP-NOW broadcast uses 1 Mbps DSSS (802.11b) which has no
    // OFDM subcarriers — the CSI callback never fires for DSSS frames.
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);

    esp_now_ready = true;
    Serial.printf("[TX] Ready — ch%d @ %dHz when EN pin (GPIO%d) is HIGH\n",
                  WIFI_CHAN, 1000 / TX_INTERVAL_MS, CAL_EN_PIN);
}

void loop()
{
    if (!esp_now_ready) { delay(100); return; }

    bool enabled = digitalRead(CAL_EN_PIN) == LOW;   // LOW = cal mode

    if (enabled) {
        static uint32_t seq = 0;
        uint8_t data[4] = { 0xDE, 0xAD,
                            (uint8_t)((seq >> 8) & 0xFF),
                            (uint8_t)( seq       & 0xFF) };
        seq++;

        esp_err_t result = esp_now_send(broadcast_addr, data, sizeof(data));

        if (result != ESP_OK)
            Serial.printf("[TX] send error: %s\n", esp_err_to_name(result));
        else if (seq % 100 == 0)
            Serial.printf("[TX] cal active — seq=%lu\n", seq);

        delay(TX_INTERVAL_MS);
    } else {
        // Silent during measurement — only print a heartbeat every 5 s
        static uint32_t lastIdle = 0;
        if (millis() - lastIdle >= 5000) {
            lastIdle = millis();
            Serial.println("[TX] idle (EN LOW — measurement mode)");
        }
        delay(10);
    }
}
