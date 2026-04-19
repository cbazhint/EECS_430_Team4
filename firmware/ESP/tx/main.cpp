/*
 * ESP32-S3 — Gated Calibration Transmitter
 *
 * Sends UDP broadcast packets from a SoftAP — same approach as the beacon
 * (test/main.cpp) so that sensor nodes see 802.11n HT data frames and the
 * CSI callback fires.  Only transmits while CAL_EN_PIN is LOW.
 *
 * Wiring:
 *   Nucleo D8 → relay/switch CTRL and → ESP32 GPIO CAL_EN_PIN
 *   Common GND required.
 *
 * CTRL LOW  → RF switches to cal coax path AND this ESP32 beacons
 * CTRL HIGH → RF switches to antenna path  AND this ESP32 is silent
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"

#define WIFI_CHAN        6
#define TX_INTERVAL_MS  5       // 200 Hz when enabled
#define CAL_EN_PIN      4
#define UDP_PORT        12346   // different port from beacon to avoid confusion

static WiFiUDP   udp;
static IPAddress bcast(192, 168, 4, 255);
static uint32_t  seq = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(CAL_EN_PIN, INPUT);

    WiFi.mode(WIFI_AP);
    WiFi.softAP("CALREF", nullptr, WIFI_CHAN, 1, 4);   // hidden SSID
    delay(200);

    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_max_tx_power(78);

    udp.begin(UDP_PORT);

    Serial.printf("[CAL TX] SoftAP ready — ch%d @ %d Hz when GPIO%d LOW\n",
                  WIFI_CHAN, 1000 / TX_INTERVAL_MS, CAL_EN_PIN);
}

void loop()
{
    bool enabled = digitalRead(CAL_EN_PIN) == LOW;

    if (enabled) {
        uint8_t payload[4] = { 0xCA, 0x1B,
                               (uint8_t)((seq >> 8) & 0xFF),
                               (uint8_t)( seq       & 0xFF) };
        seq++;

        udp.beginPacket(bcast, UDP_PORT);
        udp.write(payload, sizeof(payload));
        udp.endPacket();

        if (seq % 200 == 0)
            Serial.printf("[CAL TX] cal active — seq=%lu\n", seq);

        delay(TX_INTERVAL_MS);
    } else {
        static uint32_t lastIdle = 0;
        if (millis() - lastIdle >= 5000) {
            lastIdle = millis();
            Serial.println("[CAL TX] idle (EN HIGH — measurement mode)");
        }
        delay(10);
    }
}
