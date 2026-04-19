/*
 * ESP32-S3 — Gated Calibration Transmitter
 *
 * Injects raw 802.11 data frames (NOT ESP-NOW) so that the sensor nodes'
 * CSI engine fires on every packet.  ESP-NOW uses management action frames
 * which are always sent at 1 Mbps DSSS — DSSS has no OFDM subcarriers so
 * the CSI callback never triggers.  Raw data frames respect the protocol
 * rate setting; with 802.11b disabled the minimum rate is 6 Mbps OFDM.
 *
 * Transmits only while CAL_EN_PIN is LOW (Nucleo D8 pulled low during cal).
 *
 * Wiring:
 *   Nucleo D8  ──┬──  relay/switch CTRL
 *                └──  ESP32 GPIO CAL_EN_PIN   (share the line)
 *   Common GND between Nucleo and cal ESP32 required.
 *
 * CTRL LOW  (cal mode) → RF switches to cal path AND this ESP32 beacons
 * CTRL HIGH (measure)  → RF switches to antenna path AND this ESP32 silent
 */

#include <WiFi.h>
#include "esp_wifi.h"

#define WIFI_CHAN        6
#define TX_INTERVAL_MS  5       // 200 Hz when enabled
#define CAL_EN_PIN      4       // GPIO connected to Nucleo D8 CTRL line

// ── Minimal broadcast 802.11 data frame ──────────────────────────────────
// 24-byte MAC header + 4-byte payload = 28 bytes; hardware appends FCS.
// Addr1 = broadcast (FF:FF:...) so all promiscuous receivers accept it.
// Addr2 = our MAC (filled at runtime).
// Addr3 = broadcast BSSID (ad-hoc style).
// ─────────────────────────────────────────────────────────────────────────
static uint8_t cal_frame[28] = {
    // Frame Control: Type=Data(2<<2), Subtype=Data(0), no flags
    0x08, 0x00,
    // Duration
    0x00, 0x00,
    // Addr1 — Receiver (broadcast)
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    // Addr2 — Transmitter (our MAC, filled in setup)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // Addr3 — BSSID (broadcast / ad-hoc)
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    // Sequence Control (updated per frame)
    0x00, 0x00,
    // Payload: 0xCA 0x1B + 2-byte counter
    0xCA, 0x1B, 0x00, 0x00
};

static uint16_t tx_seq = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(CAL_EN_PIN, INPUT);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Disable 802.11b entirely — forces all TX to use OFDM (min 6 Mbps).
    // Without this, even data frames may fall back to 1 Mbps DSSS on an
    // unassociated STA, and DSSS frames carry no OFDM training fields.
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    esp_wifi_set_max_tx_power(78);   // ~19.5 dBm
    esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE);

    // Stamp our MAC into Addr2 of the frame template
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(&cal_frame[10], mac, 6);

    Serial.printf("[TX] Ready — ch%d @ %dHz when EN pin (GPIO%d) is LOW\n",
                  WIFI_CHAN, 1000 / TX_INTERVAL_MS, CAL_EN_PIN);
    Serial.printf("[TX] MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop()
{
    bool enabled = digitalRead(CAL_EN_PIN) == LOW;   // LOW = cal mode

    if (enabled) {
        // Update sequence control field (bits 7:4 of byte 22 = seq bits 3:0,
        // byte 23 = seq bits 11:4; fragment = 0)
        cal_frame[22] = (uint8_t)((tx_seq & 0x0F) << 4);
        cal_frame[23] = (uint8_t)((tx_seq >> 4) & 0xFF);
        // Update 2-byte counter in payload
        cal_frame[26] = (uint8_t)((tx_seq >> 8) & 0xFF);
        cal_frame[27] = (uint8_t)( tx_seq        & 0xFF);
        tx_seq = (tx_seq + 1) & 0x0FFF;   // 12-bit wrap

        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, cal_frame, sizeof(cal_frame), false);

        if (err != ESP_OK)
            Serial.printf("[TX] inject error: %s\n", esp_err_to_name(err));
        else if (tx_seq % 200 == 0)
            Serial.printf("[TX] cal active — seq=%u\n", tx_seq);

        delay(TX_INTERVAL_MS);
    } else {
        static uint32_t lastIdle = 0;
        if (millis() - lastIdle >= 5000) {
            lastIdle = millis();
            Serial.println("[TX] idle (EN HIGH — measurement mode)");
        }
        delay(10);
    }
}
