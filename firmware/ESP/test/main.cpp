/*
 * ESP32-S3 — CSI Beacon Transmitter
 *
 * Sends UDP broadcast packets from a SoftAP so that sensor nodes see
 * proper 802.11n HT data frames.  ESP-NOW uses management action frames
 * which always use non-HT preamble — the CSI engine ignores them.
 * UDP data frames from an HT-capable AP carry HTLTF in the preamble,
 * which is what triggers the CSI callback on the receiving nodes.
 *
 * No association required on the receiver — promiscuous mode captures
 * all frames on the channel regardless of BSSID.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"

#define WIFI_CHAN        6
#define TX_INTERVAL_MS  10      // 100 Hz
#define UDP_PORT        12345

static WiFiUDP    udp;
static IPAddress  bcast(192, 168, 4, 255);
static uint32_t   seq = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);

    // SoftAP establishes an HT BSS — data frames from this AP use 802.11n
    // HT mixed-mode preamble which carries HTLTF for CSI extraction.
    WiFi.mode(WIFI_AP);
    WiFi.softAP("BEACON", nullptr, WIFI_CHAN, 0, 4);
    delay(200);

    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_max_tx_power(78);   // ~19.5 dBm

    udp.begin(UDP_PORT);

    Serial.printf("[TX] UDP beacon ready — ch%d @ %d Hz  IP=%s\n",
                  WIFI_CHAN, 1000 / TX_INTERVAL_MS,
                  WiFi.softAPIP().toString().c_str());
}

void loop()
{
    uint8_t payload[4] = { 0xDE, 0xAD,
                           (uint8_t)((seq >> 8) & 0xFF),
                           (uint8_t)( seq       & 0xFF) };
    seq++;

    udp.beginPacket(bcast, UDP_PORT);
    udp.write(payload, sizeof(payload));
    udp.endPacket();

    if (seq % 500 == 0)
        Serial.printf("[TX] alive — seq=%lu\n", seq);

    delay(TX_INTERVAL_MS);
}
