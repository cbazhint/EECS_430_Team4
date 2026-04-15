/*
 * ESP32-S3 Calibration Reference Transmitter
 * Phase-Coherent Direction of Arrival (DoA) System
 *
 * Replaces the nRF24L01+ as the CW reference source.
 * Transmits standard 802.11 frames on channel 6 so that the 8 sensor
 * ESP32s (running in promiscuous/CSI mode) can extract phase measurements.
 *
 * Control:
 *   CAL_TRIGGER_PIN (GPIO 4) is wired to Nucleo PF_1 — the same line that
 *   drives all 8 BGS12WN6 RF switch CTRL pins.
 *
 *   PF_1 LOW  → switches on antenna path,  this node at IDLE rate (~10 Hz)
 *   PF_1 HIGH → switches on cal path,      this node at CAL  rate (~100 Hz)
 *
 * No UART, no SPI — purely a WiFi transmitter.
 */

#include <WiFi.h>
#include "esp_wifi.h"
#include <Arduino.h>

/* ============================================================
   CONFIGURATION
   ============================================================ */
#define CAL_TRIGGER_PIN  4       // GPIO wired to Nucleo PF_1 / BGS12WN6 CTRL net
#define WIFI_CHANNEL     6       // must match sensor node WIFI_CHANNEL
#define AP_SSID          "DoA-REF"

#define CAL_TX_RATE_MS   10      // 100 Hz during calibration window
#define IDLE_TX_RATE_MS  100     // 10 Hz at idle (beacons already at ~10 Hz)

/* ============================================================
   Raw 802.11 null-data frame template
   Injected via esp_wifi_80211_tx() — hardware appends FCS.
   DA = broadcast so every sensor in promiscuous mode captures it.
   SA / BSSID = fixed MAC for the DoA reference node.
   ============================================================ */
static uint8_t raw_frame[] = {
    0x48, 0x00,                            // Frame Control: data, subtype=null
    0x00, 0x00,                            // Duration
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,    // DA: broadcast
    0x24, 0x6F, 0x28, 0xCA, 0x11, 0x00,   // SA:    DoA-REF fixed MAC
    0x24, 0x6F, 0x28, 0xCA, 0x11, 0x00,   // BSSID: DoA-REF fixed MAC
    0x00, 0x00,                            // Sequence Control
};

/* ============================================================
   Globals
   ============================================================ */
static volatile bool cal_active = false;

/* ============================================================
   Calibration trigger ISR
   ============================================================ */
void IRAM_ATTR cal_isr()
{
    cal_active = (digitalRead(CAL_TRIGGER_PIN) == HIGH);
}

/* ============================================================
   Setup
   ============================================================ */
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("[CAL-TX] Booting...");

    // SoftAP provides automatic beacon frames at ~10 Hz
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, NULL, WIFI_CHANNEL, 0 /*hidden=false*/, 1 /*max clients*/);
    esp_wifi_set_max_tx_power(78);  // 78 = ~19.5 dBm (hardware maximum)

    Serial.printf("[CAL-TX] SoftAP '%s' on ch%d  max TX power set\n",
                  AP_SSID, WIFI_CHANNEL);

    // Cal trigger: shared with RF switch CTRL net
    pinMode(CAL_TRIGGER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(CAL_TRIGGER_PIN), cal_isr, CHANGE);
    cal_active = (digitalRead(CAL_TRIGGER_PIN) == HIGH);

    Serial.printf("[CAL-TX] Cal trigger on GPIO%d — initial state: %s\n",
                  CAL_TRIGGER_PIN, cal_active ? "CAL (HIGH)" : "IDLE (LOW)");
    Serial.println("[CAL-TX] Ready.");
}

/* ============================================================
   Main Loop — inject frames at rate determined by cal trigger
   ============================================================ */
void loop()
{
    static uint32_t last_tx   = 0;
    static uint32_t last_diag = 0;
    static uint32_t tx_count  = 0;

    uint32_t now      = millis();
    uint32_t interval = cal_active ? CAL_TX_RATE_MS : IDLE_TX_RATE_MS;

    if (now - last_tx >= interval)
    {
        last_tx = now;
        esp_wifi_80211_tx(WIFI_IF_AP, raw_frame, sizeof(raw_frame), false);
        tx_count++;
    }

    // Status print every 5 s
    if (now - last_diag >= 5000)
    {
        last_diag = now;
        Serial.printf("[CAL-TX] mode=%s  frames_sent=%lu\n",
                      cal_active ? "CAL" : "IDLE", tx_count);
    }
}
