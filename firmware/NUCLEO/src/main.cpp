/**
 * ==========================================================
 *  Phase-Coherent DoA System — Nucleo-H753ZI Boot Firmware
 *  Arduino Framework (PlatformIO)
 * ==========================================================
 *
 *  System overview:
 *    - 8x ESP32-S3 sensor nodes (UART)
 *    - 1x nRF24L01+ CW reference transmitter (SPI)
 *    - 8x BGS12WN6 RF switches (shared CTRL GPIO)
 *    - Boot-time diagnostics over ST-Link VCP
 *
 *  Calibration sequence (runs once at boot):
 *    1. RF switches → reference path (nRF trace)
 *    2. nRF24 transmits packets at 2437 MHz (= WiFi ch 6)
 *       through the Wilkinson trace to every ESP32 antenna input
 *    3. Each ESP32 measures the phase of that arriving signal
 *       and reports it back over UART
 *    4. Nucleo averages CAL_SAMPLES readings per node (circular mean)
 *    5. Correction vector built: correction[i] = phase[0] - phase[i]
 *    6. RF switches → antenna path, normal operation resumes
 *
 *  Pin assignments: see pinconfig.h
 */

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "pinconfig.h"

// ──────────────────────────────────────────────────────────
//  nRF24L01+ radio  (CE pin, CSN pin)
// ──────────────────────────────────────────────────────────
RF24 radio(NRF24_CE_PIN, NRF24_CS_PIN);

// ──────────────────────────────────────────────────────────
//  Calibration configuration
// ──────────────────────────────────────────────────────────
#define CAL_SAMPLES      20     // phase measurements averaged per node
#define CAL_TIMEOUT_MS  500     // max wait for one packet per sample attempt (ms)

// Calibration results — populated once by runCalibration()
// Apply to every measurement: corrected_phase = raw_phase + phaseCorrection[node]
static float phaseCorrection[NUM_ESP_NODES] = {0.0f};
static bool  calDone = false;

// ──────────────────────────────────────────────────────────
//  Forward declarations
// ──────────────────────────────────────────────────────────
void initDebugSerial();
void initRFSwitchGPIO();
void initNRF24();
void initESP32UARTs();
bool testNRF24Link();
void setRFSwitch(bool toReference);
bool readCSIPacket(HardwareSerial* ser, uint8_t* nodeId, float* phase, uint32_t timeoutMs);
void runCalibration();
void printBootBanner();
void printBootDiagnostics();

// ──────────────────────────────────────────────────────────
//  UART handles for the 8 ESP32 nodes
// ──────────────────────────────────────────────────────────
HardwareSerial ESP_Serial1(ESP1_UART_RX, ESP1_UART_TX);
HardwareSerial ESP_Serial2(ESP2_UART_RX, ESP2_UART_TX);
HardwareSerial ESP_Serial3(ESP3_UART_RX, ESP3_UART_TX);
HardwareSerial ESP_Serial4(ESP4_UART_RX, ESP4_UART_TX);
HardwareSerial ESP_Serial5(ESP5_UART_RX, ESP5_UART_TX);
HardwareSerial ESP_Serial6(ESP6_UART_RX, ESP6_UART_TX);
HardwareSerial ESP_Serial7(ESP7_UART_RX, ESP7_UART_TX);
HardwareSerial ESP_Serial8(ESP8_UART_RX, ESP8_UART_TX);

HardwareSerial* espSerials[NUM_ESP_NODES] = {
    &ESP_Serial1, &ESP_Serial2, &ESP_Serial3, &ESP_Serial4,
    &ESP_Serial5, &ESP_Serial6, &ESP_Serial7, &ESP_Serial8
};

// ──────────────────────────────────────────────────────────
//  Boot state tracking
// ──────────────────────────────────────────────────────────
struct BootStatus {
    bool debugSerial;
    bool rfSwitch;
    bool nrf24;
    bool nrf24Link;
    bool espUarts[NUM_ESP_NODES];
    uint32_t bootTimeMs;
} bootStatus;

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
    memset(&bootStatus, 0, sizeof(bootStatus));
    uint32_t bootStart = millis();

    // 1. Debug serial
    initDebugSerial();
    printBootBanner();

    // 2. RF switch GPIO
    Serial.println("[BOOT] Initializing RF switch CTRL...");
    initRFSwitchGPIO();

    // 3. nRF24L01+ via RF24 library
    Serial.println("[BOOT] Initializing nRF24L01+...");
    initNRF24();

    // 4. nRF24 link test
    Serial.println("[BOOT] Testing nRF24L01+ link...");
    bootStatus.nrf24Link = testNRF24Link();

    // 5. ESP32 UART channels
    Serial.println("[BOOT] Initializing ESP32 UART channels...");
    initESP32UARTs();

    // 6. Default RF path: antenna
    setRFSwitch(false);
    Serial.println("[BOOT] RF switches → ANTENNA path");

    // 7. One-shot calibration via nRF reference trace
    runCalibration();

    bootStatus.bootTimeMs = millis() - bootStart;
    printBootDiagnostics();
    Serial.println("[BOOT] === System ready ===\n");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════
void loop() {
    // Echo ESP32 UART traffic to debug console
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        if (espSerials[i]->available()) {
            Serial.print("[ESP");
            Serial.print(i + 1);
            Serial.print("] ");
            while (espSerials[i]->available()) {
                Serial.write(espSerials[i]->read());
            }
            Serial.println();
        }
    }

    // Debug serial command parser
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "status") {
            printBootDiagnostics();
        } else if (cmd == "cal") {
            runCalibration();
        } else if (cmd == "ant") {
            setRFSwitch(false);
            Serial.println("[CMD] RF → ANTENNA path");
        } else if (cmd == "nrf") {
            Serial.print("[CMD] nRF24 link: ");
            Serial.println(testNRF24Link() ? "PASS" : "FAIL");
        } else if (cmd.length() > 0) {
            Serial.print("[CMD] Unknown: ");
            Serial.println(cmd);
            Serial.println("  Commands: status, cal, ant, nrf");
        }
    }
}

// ──────────────────────────────────────────────────────────
//  Init helpers
// ──────────────────────────────────────────────────────────

void initDebugSerial() {
    Serial.begin(DEBUG_BAUD);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 2000)) {}
    bootStatus.debugSerial = true;
}

void initRFSwitchGPIO() {
    // Single GPIO drives all 8 BGS12WN6 CTRL pins simultaneously —
    // guarantees phase-coherent simultaneous switching across all nodes.
    // CTRL=LOW  → RF1 (antenna path)
    // CTRL=HIGH → RF2 (reference/calibration path via Wilkinson combiner)
    pinMode(RF_SWITCH_CTRL_PIN, OUTPUT);
    digitalWrite(RF_SWITCH_CTRL_PIN, LOW);
    bootStatus.rfSwitch = true;
}

void initNRF24() {
    if (!radio.begin()) {
        Serial.println("[WARN] nRF24: radio.begin() failed — check wiring");
        bootStatus.nrf24 = false;
        return;
    }
    // Match ESP32 WiFi channel: nRF channel N = (2400+N) MHz
    // WiFi ch 6 = 2437 MHz → nRF channel 37
    radio.setPALevel(RF24_PA_MAX);       // maximum power through trace
    radio.setDataRate(RF24_250KBPS);     // lowest rate = cleanest signal
    radio.setChannel(NRF24_WIFI_CHANNEL);
    radio.stopListening();               // TX mode
    bootStatus.nrf24 = true;
    Serial.printf("  nRF24 ready: ch=%d (%.0f MHz), rate=250kbps, PA=MAX\n",
                  NRF24_WIFI_CHANNEL, 2400.0f + NRF24_WIFI_CHANNEL);
}

bool testNRF24Link() {
    bool ok = radio.isChipConnected();
    if (!ok) Serial.println("[WARN] nRF24: isChipConnected() = false");
    return ok;
}

void initESP32UARTs() {
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        espSerials[i]->begin(ESP_BAUD);
        bootStatus.espUarts[i] = true;
        Serial.printf("  UART %d ... OK\n", i + 1);
    }
}

void setRFSwitch(bool toReference) {
    digitalWrite(RF_SWITCH_CTRL_PIN, toReference ? HIGH : LOW);
}

// ──────────────────────────────────────────────────────────
//  CSI packet parser
// ──────────────────────────────────────────────────────────

// Reads one 11-byte binary CSI packet from a node UART.
// Hunts for the 0xAA sync byte, reads the remaining 10 bytes,
// validates the XOR checksum, extracts nodeId and phase.
// Returns true on success, false on timeout or bad checksum.
bool readCSIPacket(HardwareSerial* ser, uint8_t* nodeId, float* phase, uint32_t timeoutMs)
{
    uint32_t deadline = millis() + timeoutMs;

    while (millis() < deadline)
    {
        if (!ser->available()) continue;
        if (ser->read() != SYNC_BYTE) continue;  // hunt for sync

        // Read remaining 10 bytes with a short inner timeout
        uint8_t buf[PACKET_SIZE - 1];
        uint32_t inner = millis() + 50;
        int n = 0;
        while (n < PACKET_SIZE - 1 && millis() < inner) {
            if (ser->available()) buf[n++] = ser->read();
        }
        if (n < PACKET_SIZE - 1) continue;  // incomplete — resync

        // XOR of all 11 bytes (incl. sync byte already consumed) must be 0
        uint8_t chk = SYNC_BYTE;
        for (int i = 0; i < PACKET_SIZE - 2; i++) chk ^= buf[i];
        if (chk != buf[PACKET_SIZE - 2]) continue;  // bad checksum — resync

        *nodeId = buf[0];
        // buf[1..4] = timestamp (not used during calibration)
        memcpy(phase, &buf[5], sizeof(float));
        return true;
    }
    return false;
}

// ──────────────────────────────────────────────────────────
//  Calibration
// ──────────────────────────────────────────────────────────

// Runs once at boot (and again if 'cal' is typed over serial).
//
// The nRF24 transmits a 2.4 GHz signal through the Wilkinson trace
// to every ESP32 antenna input simultaneously.  Each node measures
// the phase of that arriving signal and reports it over UART.
// Because all nodes receive the SAME physical signal through a FIXED
// trace, any phase difference between node i and node 0 is purely
// a hardware offset (trace length, component tolerances) that must
// be removed before MUSIC can work correctly.
//
// Correction applied during normal operation:
//   corrected_phase[i] = raw_phase[i] + phaseCorrection[i]
void runCalibration()
{
    Serial.println("[CAL] ── Calibration start ──────────────────");

    if (!bootStatus.nrf24) {
        Serial.println("[CAL] ERROR: nRF24 not ready — skipping");
        return;
    }

    // Switch all RF paths to the reference trace
    Serial.println("[CAL] RF switches → REFERENCE path (nRF trace)");
    setRFSwitch(true);
    delay(50);  // switch + nRF PLL settle time

    // Open TX pipe and ensure radio is transmitting
    radio.openWritingPipe((const uint8_t*)NRF24_CAL_ADDRESS);
    radio.stopListening();

    // Calibration payload — content is irrelevant, just needs to be
    // non-zero so the nRF hardware actually modulates the carrier
    uint8_t calPayload[4] = {0xCA, 0x1B, 0x00, 0x00};

    // Circular-mean accumulators per node
    // Uses sum-of-sin / sum-of-cos rather than arithmetic mean to
    // correctly handle phase wrap-around at ±π
    double sinSum[NUM_ESP_NODES] = {};
    double cosSum[NUM_ESP_NODES] = {};
    int    count[NUM_ESP_NODES]  = {};

    Serial.printf("[CAL] Collecting %d samples per node...\n", CAL_SAMPLES);

    for (int node = 0; node < NUM_ESP_NODES; node++)
    {
        Serial.printf("[CAL]   Node %d: ", node);

        for (int s = 0; s < CAL_SAMPLES; s++)
        {
            // Fire one reference packet through the trace before each read
            calPayload[2] = (uint8_t)node;
            calPayload[3] = (uint8_t)s;
            radio.write(calPayload, sizeof(calPayload));

            uint8_t rxId;
            float   rxPhase;
            if (readCSIPacket(espSerials[node], &rxId, &rxPhase, CAL_TIMEOUT_MS))
            {
                sinSum[node] += sin((double)rxPhase);
                cosSum[node] += cos((double)rxPhase);
                count[node]++;
                Serial.print(".");
            }
            else
            {
                Serial.print("x");  // timeout — node not responding
            }
        }
        Serial.printf("  (%d/%d)\n", count[node], CAL_SAMPLES);
    }

    // Done transmitting — power down nRF and return to antenna path
    radio.powerDown();
    setRFSwitch(false);
    Serial.println("[CAL] nRF powered down, RF switches → ANTENNA path");

    // Compute circular mean phase per node
    float meanPhase[NUM_ESP_NODES] = {};
    for (int node = 0; node < NUM_ESP_NODES; node++)
    {
        if (count[node] > 0) {
            meanPhase[node] = (float)atan2(
                sinSum[node] / count[node],
                cosSum[node] / count[node]);
        } else {
            Serial.printf("[CAL] WARN: Node %d no data — using node 0 phase\n", node);
            meanPhase[node] = meanPhase[0];
        }
    }

    // Build correction vector: correction[i] = phase[0] - phase[i]
    // After applying correction, all nodes are phase-aligned to node 0.
    // Node 0 correction is always 0 by definition.
    Serial.println("[CAL] Correction vector (node 0 = reference):");
    for (int node = 0; node < NUM_ESP_NODES; node++)
    {
        phaseCorrection[node] = meanPhase[0] - meanPhase[node];
        Serial.printf("[CAL]   Node %d  mean=% .4f rad  correction=% .4f rad\n",
                      node, meanPhase[node], phaseCorrection[node]);
    }

    calDone = true;
    Serial.println("[CAL] ── Calibration complete ───────────────\n");
}

// ──────────────────────────────────────────────────────────
//  Diagnostics
// ──────────────────────────────────────────────────────────

void printBootBanner() {
    Serial.println();
    Serial.println("=============================================");
    Serial.println("  Phase-Coherent DoA System");
    Serial.println("  Nucleo-H753ZI Coordinator");
    Serial.println("  Build: " __DATE__ " " __TIME__);
    Serial.println("=============================================");
    Serial.println();
}

void printBootDiagnostics() {
    Serial.println();
    Serial.println("─── Boot Diagnostics ───────────────────────");
    Serial.print("  Debug Serial (USART3 VCP) : "); Serial.println(bootStatus.debugSerial ? "OK" : "FAIL");
    Serial.print("  RF Switch CTRL GPIO       : "); Serial.println(bootStatus.rfSwitch    ? "OK" : "FAIL");
    Serial.print("  nRF24L01+                 : "); Serial.println(bootStatus.nrf24       ? "OK" : "FAIL");
    Serial.print("  nRF24L01+ link test       : "); Serial.println(bootStatus.nrf24Link   ? "PASS" : "FAIL");
    Serial.print("  Calibration               : "); Serial.println(calDone                ? "DONE" : "NOT RUN");
    if (calDone) {
        Serial.println("  Correction vector:");
        for (int i = 0; i < NUM_ESP_NODES; i++) {
            Serial.printf("    Node %d: % .4f rad\n", i, phaseCorrection[i]);
        }
    }
    Serial.println("  ESP32 UART channels:");
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        Serial.printf("    Node %d : %s\n", i + 1, bootStatus.espUarts[i] ? "OK" : "FAIL");
    }
    Serial.printf("  Boot time: %lu ms\n", bootStatus.bootTimeMs);
    Serial.println("────────────────────────────────────────────");
    Serial.println();
}
