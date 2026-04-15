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
 *    - Boot-time diagnostics over ST-Link VCP (USART3)
 *
 *  Boot sequence:
 *    1. Initialize debug serial, RF switch, nRF24, ESP32 UARTs
 *    2. Run one-shot calibration (nRF reference → phase correction vector)
 *    3. Enter command loop
 *
 *  Serial commands (type over ST-Link VCP, 115200 baud):
 *    run    - start streaming binary host packets to PC
 *    stop   - stop streaming, return to text debug mode
 *    cal    - re-run calibration (stops/restarts streaming automatically)
 *    status - print boot diagnostics and correction vector
 *    ant    - force RF switches to antenna path
 *    nrf    - test nRF24 chip connection
 *
 *  Host packet format (41 bytes, binary, sent during "run" mode):
 *    [0]      SYNC_A     0xBB
 *    [1]      SYNC_B     0xCC
 *    [2..3]   seq        uint16 LE  (wraps at 65535)
 *    [4..7]   timestamp  uint32 LE  (Nucleo millis())
 *    [8..39]  phases[8]  float32 LE each  (corrected phase, radians)
 *    [40]     checksum   XOR of bytes [0..39]
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
//  SPI pins are remapped to SPI3 in initNRF24() via SPI.setMOSI/MISO/SCLK
//  STM32 core auto-selects SPI3 peripheral from pin AF lookup
// ──────────────────────────────────────────────────────────
RF24 radio(NRF24_CE_PIN, NRF24_CS_PIN);

// ──────────────────────────────────────────────────────────
//  Calibration configuration
// ──────────────────────────────────────────────────────────
#define CAL_SAMPLES          20    // phase measurements averaged per node
#define CAL_TIMEOUT_MS      500    // max wait for one packet per sample attempt (ms)
#define CAL_REFERENCE_NODE    1    // preferred reference node (falls back if no data)
#define CAL_MIN_SAMPLES       5    // minimum samples to consider a node calibrated

static float   phaseCorrection[NUM_ESP_NODES] = {0.0f};
static uint8_t calValid = 0x00;    // bitmask: bit i set = node i is calibrated
static bool    calDone  = false;

// ──────────────────────────────────────────────────────────
//  Non-blocking per-node ESP32 UART parser
//
//  Each node streams 11-byte packets at up to ~100 Hz.
//  The parser accumulates bytes one at a time so the main loop
//  never blocks waiting for a full packet.
// ──────────────────────────────────────────────────────────
struct NodeParser {
    uint8_t  buf[PACKET_SIZE];
    int      count;     // bytes buffered (0 = hunting for sync)
    bool     has_data;  // true when a validated packet is ready
    float    phase;     // most recent validated phase (raw, uncorrected)
};
static NodeParser parsers[NUM_ESP_NODES];

// ──────────────────────────────────────────────────────────
//  Snapshot / streaming state
//
//  A "snapshot" is one phase reading per node assembled into a
//  single host packet.  We send one snapshot per collection window:
//  either when all 8 nodes have reported fresh data, or after
//  SNAP_TIMEOUT_MS from the previous snapshot (whichever comes first).
// ──────────────────────────────────────────────────────────
static float    snapPhase[NUM_ESP_NODES];   // corrected phases for current snapshot
static bool     snapFresh[NUM_ESP_NODES];   // which nodes have new data this window
static uint32_t lastSnapTime = 0;
static uint16_t snapSeq      = 0;
static bool     streaming    = false;

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
void verifyCalibration();
void feedParsers();
void tryEmitSnapshot();
void sendHostPacket();
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
    memset(parsers,     0, sizeof(parsers));
    memset(snapFresh,   0, sizeof(snapFresh));
    memset(snapPhase,   0, sizeof(snapPhase));
    uint32_t bootStart = millis();

    initDebugSerial();
    printBootBanner();

    Serial.println("[BOOT] Initializing RF switch CTRL...");
    initRFSwitchGPIO();

    Serial.println("[BOOT] Initializing nRF24L01+...");
    initNRF24();

    Serial.println("[BOOT] Testing nRF24L01+ link...");
    bootStatus.nrf24Link = testNRF24Link();

    Serial.println("[BOOT] Initializing ESP32 UART channels...");
    initESP32UARTs();

    setRFSwitch(false);
    Serial.println("[BOOT] RF switches → ANTENNA path");

    //runCalibration();

    bootStatus.bootTimeMs = millis() - bootStart;
    printBootDiagnostics();
    Serial.println("[BOOT] === System ready ===");
    Serial.println("  Type 'run' to begin streaming, 'stop' to halt, 'help' for all commands.\n");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════
void loop() {
    // ── 1. Feed bytes from all ESP32 UARTs into per-node parsers ──────────────
    feedParsers();

    // ── 2. Snapshot assembly + host packet emission (streaming mode) ──────────
    if (streaming) {
        tryEmitSnapshot();
    }

    // ── 3. Serial command parser ──────────────────────────────────────────────
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "run") {
            memset(parsers,   0, sizeof(parsers));
            memset(snapFresh, 0, sizeof(snapFresh));
            lastSnapTime = millis();
            streaming = true;
            Serial.println("[CMD] Streaming ON — sending binary host packets");

        } else if (cmd == "stop") {
            streaming = false;
            Serial.println("[CMD] Streaming OFF");

        } else if (cmd == "cal") {
            bool wasStreaming = streaming;
            streaming = false;
            runCalibration();
            if (wasStreaming) {
                memset(parsers,   0, sizeof(parsers));
                memset(snapFresh, 0, sizeof(snapFresh));
                lastSnapTime = millis();
                streaming = true;
                Serial.println("[CMD] Streaming resumed after cal");
            }

        } else if (cmd == "status") {
            printBootDiagnostics();

        } else if (cmd == "verify") {
            verifyCalibration();

        } else if (cmd == "ant") {
            setRFSwitch(false);
            Serial.println("[CMD] RF → ANTENNA path");

        } else if (cmd == "nrf") {
            Serial.print("[CMD] nRF24 link: ");
            Serial.println(testNRF24Link() ? "PASS" : "FAIL");

        } else if (cmd == "help" || cmd == "?") {
            Serial.println("Commands: run | stop | cal | verify | status | ant | nrf");

        } else if (cmd.length() > 0) {
            Serial.print("[CMD] Unknown: '");
            Serial.print(cmd);
            Serial.println("'  — type 'help' for commands");
        }
    }
}

// ──────────────────────────────────────────────────────────
//  feedParsers — non-blocking per-node byte accumulator
//
//  Reads all bytes currently available on each ESP32 UART.
//  Hunts for the 0xAA sync byte, then accumulates PACKET_SIZE-1
//  more bytes.  On a complete packet, validates the XOR checksum
//  and stores the phase in parsers[i].phase / .has_data.
//
//  Never blocks — returns immediately if no bytes are waiting.
// ──────────────────────────────────────────────────────────
void feedParsers()
{
    static uint32_t lastDiag  = 0;
    static uint32_t bytesRx[NUM_ESP_NODES] = {};

    for (int i = 0; i < NUM_ESP_NODES; i++) {
        while (espSerials[i]->available()) {
            uint8_t b = (uint8_t)espSerials[i]->read();
            bytesRx[i]++;
            NodeParser& p = parsers[i];

            if (p.count == 0) {
                // Hunting for sync byte
                if (b == SYNC_BYTE) {
                    p.buf[0] = b;
                    p.count  = 1;
                }
            } else {
                p.buf[p.count++] = b;
                if (p.count == PACKET_SIZE) {
                    p.count = 0;  // ready for next packet regardless of outcome

                    // Validate XOR checksum over all 11 bytes (last byte is chk)
                    uint8_t chk = 0;
                    for (int j = 0; j < PACKET_SIZE - 1; j++) chk ^= p.buf[j];
                    if (chk == p.buf[PACKET_SIZE - 1]) {
                        // Extract phase from bytes [6..9]
                        memcpy(&p.phase, &p.buf[6], sizeof(float));
                        p.has_data = true;
                    }
                    // Bad checksum: silently drop and re-hunt
                }
            }
        }
    }

    // Print byte counts every 5 s so we can confirm wiring without a scope
    if (millis() - lastDiag >= 5000) {
        lastDiag = millis();
        Serial.print("[UART rx bytes]");
        for (int i = 0; i < NUM_ESP_NODES; i++) {
            Serial.printf("  U%d:%lu", i + 1, bytesRx[i]);
        }
        Serial.println();
    }
}

// ──────────────────────────────────────────────────────────
//  tryEmitSnapshot — assemble one host packet per window
//
//  Harvests fresh phases from completed parsers, applies
//  calibration correction, then emits a host packet when either:
//    a) all 8 nodes have fresh data, or
//    b) SNAP_TIMEOUT_MS has elapsed and at least one node reported.
//
//  Nodes that did not report in this window keep their previous
//  corrected phase value (stale but better than NaN for MUSIC).
// ──────────────────────────────────────────────────────────
void tryEmitSnapshot()
{
    // Harvest any fresh parser results
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        if (parsers[i].has_data) {
            snapPhase[i]        = parsers[i].phase + phaseCorrection[i];
            snapFresh[i]        = true;
            parsers[i].has_data = false;
        }
    }

    bool allFresh = true;
    bool anyFresh = false;
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        if (!snapFresh[i]) allFresh = false;
        if (snapFresh[i])  anyFresh = true;
    }

    bool timedOut = (millis() - lastSnapTime) >= SNAP_TIMEOUT_MS;

    // Send when all nodes fresh, or timeout with partial data, or keepalive
    // when no ESP32s are connected (anyFresh=false) so Python pipeline can
    // be tested before hardware is fully wired.
    if (allFresh || timedOut) {
        sendHostPacket();
        memset(snapFresh, false, sizeof(snapFresh));
        lastSnapTime = millis();
    }
}

// ──────────────────────────────────────────────────────────
//  sendHostPacket — emit one 41-byte binary frame over Serial
//
//  Format:
//    [0]      0xBB        sync A
//    [1]      0xCC        sync B
//    [2..3]   seq         uint16 LE
//    [4..7]   timestamp   uint32 LE  (Nucleo millis)
//    [8..39]  phases[8]   float32 LE each (calibration-corrected)
//    [40]     checksum    XOR of bytes [0..39]
// ──────────────────────────────────────────────────────────
void sendHostPacket()
{
    uint8_t  pkt[HOST_PKT_SIZE];
    uint32_t ts = millis();

    pkt[0] = HOST_SYNC_A;
    pkt[1] = HOST_SYNC_B;
    pkt[2] = (uint8_t)( snapSeq        & 0xFF);
    pkt[3] = (uint8_t)((snapSeq >> 8)  & 0xFF);
    pkt[4] = (uint8_t)( ts             & 0xFF);
    pkt[5] = (uint8_t)((ts >>  8)      & 0xFF);
    pkt[6] = (uint8_t)((ts >> 16)      & 0xFF);
    pkt[7] = (uint8_t)((ts >> 24)      & 0xFF);

    for (int i = 0; i < NUM_ESP_NODES; i++) {
        uint8_t* fb = (uint8_t*)&snapPhase[i];
        pkt[8 + i*4 + 0] = fb[0];
        pkt[8 + i*4 + 1] = fb[1];
        pkt[8 + i*4 + 2] = fb[2];
        pkt[8 + i*4 + 3] = fb[3];
    }

    pkt[40] = calValid;   // calibration validity bitmask — bit i = node i calibrated

    uint8_t chk = 0;
    for (int i = 0; i < HOST_PKT_SIZE - 1; i++) chk ^= pkt[i];
    pkt[HOST_PKT_SIZE - 1] = chk;

    Serial.write(pkt, HOST_PKT_SIZE);
    snapSeq++;
}

// ══════════════════════════════════════════════════════════
//  Init helpers
// ══════════════════════════════════════════════════════════

void initDebugSerial() {
    Serial.begin(DEBUG_BAUD);
    delay(100);   // let UART settle; no blocking wait on !Serial (USART3, not USB-CDC)
    bootStatus.debugSerial = true;
}

void initRFSwitchGPIO() {
    // Single GPIO drives all 8 BGS12WN6 CTRL pins simultaneously —
    // guarantees phase-coherent simultaneous switching.
    // CTRL=LOW  → RF1 (antenna path)
    // CTRL=HIGH → RF2 (calibration path via Wilkinson combiner)
    pinMode(RF_SWITCH_CTRL_PIN, OUTPUT);
    digitalWrite(RF_SWITCH_CTRL_PIN, LOW);
    bootStatus.rfSwitch = true;
}

void initNRF24() {
    // nRF24L01+ replaced by cal ESP32 (DoA-REF) — no SPI init needed.
    // Cal ESP32 transmits 802.11 frames on ch6; triggered via RF_SWITCH_CTRL_PIN.
    bootStatus.nrf24 = true;   // always ready — cal ESP32 is always-on
    Serial.println("  Cal ESP32 (DoA-REF) replaces nRF24 — no SPI required");
}

bool testNRF24Link() {
    // Cal ESP32 is always-on; no link test needed.
    return true;
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
//  readCSIPacket — blocking single-packet read (used only during calibration)
// ──────────────────────────────────────────────────────────
bool readCSIPacket(HardwareSerial* ser, uint8_t* nodeId, float* phase, uint32_t timeoutMs)
{
    uint32_t deadline = millis() + timeoutMs;

    while (millis() < deadline)
    {
        if (!ser->available()) continue;
        if (ser->read() != SYNC_BYTE) continue;

        uint8_t buf[PACKET_SIZE - 1];
        uint32_t inner = millis() + 50;
        int n = 0;
        while (n < PACKET_SIZE - 1 && millis() < inner)
            if (ser->available()) buf[n++] = ser->read();
        if (n < PACKET_SIZE - 1) continue;

        uint8_t chk = SYNC_BYTE;
        for (int i = 0; i < PACKET_SIZE - 2; i++) chk ^= buf[i];
        if (chk != buf[PACKET_SIZE - 2]) continue;

        *nodeId = buf[0];
        memcpy(phase, &buf[5], sizeof(float));
        return true;
    }
    return false;
}

// ──────────────────────────────────────────────────────────
//  runCalibration
//
//  Transmits nRF reference packets through the Wilkinson trace
//  to all ESP32 antenna inputs simultaneously.  Each node
//  measures phase of the arriving signal.  Circular mean per node
//  avoids wrap-around artefacts at ±π.
//
//  Result: phaseCorrection[i] = meanPhase[0] - meanPhase[i]
//  Apply during normal operation: corrected = raw + phaseCorrection[i]
// ──────────────────────────────────────────────────────────
void runCalibration()
{
    Serial.println("[CAL] ── Calibration start ──────────────────");

    // Assert PF_1 HIGH: simultaneously flips all RF switches to reference path
    // AND signals the cal ESP32 (DoA-REF) to ramp to 100 Hz frame injection
    Serial.println("[CAL] RF switches → REFERENCE path (cal ESP32 transmitting)");
    setRFSwitch(true);
    delay(100);   // let TX ESP32 ramp up and sensors lock on

    // Flush stale bytes from all UARTs before collecting samples
    for (int i = 0; i < NUM_ESP_NODES; i++)
        while (espSerials[i]->available()) espSerials[i]->read();

    double sinSum[NUM_ESP_NODES] = {};
    double cosSum[NUM_ESP_NODES] = {};
    int    count[NUM_ESP_NODES]  = {};

    // Poll all nodes simultaneously so no UART FIFO starves while waiting
    // for another node.  At 100 Hz cal rate, CAL_SAMPLES×10ms per node = 200ms
    // of useful data; give 3× headroom for slow nodes.
    uint32_t calDeadline = millis() + (CAL_SAMPLES * 30UL * NUM_ESP_NODES);

    // Local per-node packet parsers (independent of main feedParsers state)
    struct { uint8_t buf[PACKET_SIZE]; int count; } cp[NUM_ESP_NODES];
    memset(cp, 0, sizeof(cp));

    Serial.printf("[CAL] Collecting %d samples per node (round-robin)...\n", CAL_SAMPLES);

    while (millis() < calDeadline)
    {
        bool allDone = true;
        for (int i = 0; i < NUM_ESP_NODES; i++)
            if (count[i] < CAL_SAMPLES) { allDone = false; break; }
        if (allDone) break;

        for (int i = 0; i < NUM_ESP_NODES; i++)
        {
            if (count[i] >= CAL_SAMPLES) continue;
            while (espSerials[i]->available())
            {
                uint8_t b = (uint8_t)espSerials[i]->read();
                if (cp[i].count == 0) {
                    if (b == SYNC_BYTE) { cp[i].buf[0] = b; cp[i].count = 1; }
                } else {
                    cp[i].buf[cp[i].count++] = b;
                    if (cp[i].count == PACKET_SIZE) {
                        cp[i].count = 0;
                        uint8_t chk = 0;
                        for (int j = 0; j < PACKET_SIZE - 1; j++) chk ^= cp[i].buf[j];
                        if (chk == cp[i].buf[PACKET_SIZE - 1]) {
                            float raw;
                            memcpy(&raw, &cp[i].buf[6], sizeof(float));
                            sinSum[i] += sin((double)raw);
                            cosSum[i] += cos((double)raw);
                            count[i]++;
                        }
                    }
                }
            }
        }
    }

    for (int node = 0; node < NUM_ESP_NODES; node++)
        Serial.printf("[CAL]   Node %d: %d/%d\n", node, count[node], CAL_SAMPLES);

    // ── Retry pass for nodes below minimum threshold ──────────────────────────
    // RF switches are still HIGH — give struggling nodes one more window.
    int failedCount = 0;
    for (int i = 0; i < NUM_ESP_NODES; i++)
        if (count[i] < CAL_MIN_SAMPLES) failedCount++;

    if (failedCount > 0) {
        Serial.printf("[CAL] %d node(s) below threshold (%d samples) — retry (2 s)...\n",
                      failedCount, CAL_MIN_SAMPLES);

        // Flush then collect for 2 more seconds, only targeting failed nodes
        for (int i = 0; i < NUM_ESP_NODES; i++)
            while (espSerials[i]->available()) espSerials[i]->read();
        memset(cp, 0, sizeof(cp));

        uint32_t retryDeadline = millis() + 2000;
        while (millis() < retryDeadline) {
            for (int i = 0; i < NUM_ESP_NODES; i++) {
                if (count[i] >= CAL_MIN_SAMPLES) continue;
                while (espSerials[i]->available()) {
                    uint8_t b = (uint8_t)espSerials[i]->read();
                    if (cp[i].count == 0) {
                        if (b == SYNC_BYTE) { cp[i].buf[0] = b; cp[i].count = 1; }
                    } else {
                        cp[i].buf[cp[i].count++] = b;
                        if (cp[i].count == PACKET_SIZE) {
                            cp[i].count = 0;
                            uint8_t chk = 0;
                            for (int j = 0; j < PACKET_SIZE - 1; j++) chk ^= cp[i].buf[j];
                            if (chk == cp[i].buf[PACKET_SIZE - 1]) {
                                float raw;
                                memcpy(&raw, &cp[i].buf[6], sizeof(float));
                                sinSum[i] += sin((double)raw);
                                cosSum[i] += cos((double)raw);
                                count[i]++;
                            }
                        }
                    }
                }
            }
        }
        Serial.println("[CAL] Retry results:");
        for (int node = 0; node < NUM_ESP_NODES; node++)
            if (count[node] < CAL_MIN_SAMPLES)
                Serial.printf("[CAL]   Node %d: %d/%d  (still below threshold)\n",
                              node, count[node], CAL_MIN_SAMPLES);
    }

    // De-assert PF_1: RF switches back to antenna path
    setRFSwitch(false);
    Serial.println("[CAL] RF switches → ANTENNA path");

    // Pass 1: compute circular mean for nodes that received data
    float meanPhase[NUM_ESP_NODES] = {};
    for (int node = 0; node < NUM_ESP_NODES; node++) {
        if (count[node] > 0) {
            meanPhase[node] = (float)atan2(
                sinSum[node] / count[node],
                cosSum[node] / count[node]);
        }
    }

    // Pass 2: determine actual reference — prefer CAL_REFERENCE_NODE, fall back
    // to the first node that received data if the preferred one has none
    int refNode = -1;
    if (count[CAL_REFERENCE_NODE] > 0) {
        refNode = CAL_REFERENCE_NODE;
    } else {
        for (int node = 0; node < NUM_ESP_NODES; node++) {
            if (count[node] > 0) { refNode = node; break; }
        }
    }
    if (refNode < 0) {
        Serial.println("[CAL] ERROR: No nodes received data — calibration aborted");
        return;
    }
    if (refNode != CAL_REFERENCE_NODE) {
        Serial.printf("[CAL] WARN: Preferred reference node %d has no data — using node %d instead\n",
                      CAL_REFERENCE_NODE, refNode);
    }

    // Build calValid bitmask and fill in missing nodes
    calValid = 0x00;
    for (int node = 0; node < NUM_ESP_NODES; node++) {
        if (count[node] >= CAL_MIN_SAMPLES) {
            calValid |= (1 << node);
        } else {
            Serial.printf("[CAL] WARN: Node %d uncalibrated (%d samples) — excluded from array\n",
                          node, count[node]);
            meanPhase[node] = meanPhase[refNode];  // correction = 0, will be masked in Python
        }
    }

    Serial.printf("[CAL] Correction vector (node %d = reference):\n", refNode);
    for (int node = 0; node < NUM_ESP_NODES; node++) {
        phaseCorrection[node] = meanPhase[refNode] - meanPhase[node];
        Serial.printf("[CAL]   Node %d  mean=% .4f rad  correction=% .4f rad  %s\n",
                      node, meanPhase[node], phaseCorrection[node],
                      (calValid >> node) & 1 ? "CAL" : "UNCAL");
    }
    Serial.printf("[CAL] Valid node mask: 0x%02X\n", calValid);

    calDone = true;
    Serial.println("[CAL] ── Calibration complete ───────────────\n");
    verifyCalibration();
}

// ──────────────────────────────────────────────────────────
//  verifyCalibration — non-blocking round-robin verification
//
//  Re-enables the cal path briefly, polls all 8 UARTs simultaneously
//  for 2 seconds (same round-robin pattern as feedParsers so no UART
//  starves or overflows), applies phaseCorrection[], then checks that
//  all nodes' corrected phases converge to within 0.2 rad of node 0.
//
//  Call after runCalibration() has set phaseCorrection[].
//  Also available as the "verify" serial command.
// ──────────────────────────────────────────────────────────
void verifyCalibration()
{
    if (!calDone) {
        Serial.println("[VER] No calibration data — run 'cal' first");
        return;
    }

    Serial.println("[VER] ── Verification start (2 s) ───────────");
    setRFSwitch(true);
    delay(100);   // let cal ESP32 ramp up

    // Flush stale bytes
    for (int i = 0; i < NUM_ESP_NODES; i++)
        while (espSerials[i]->available()) espSerials[i]->read();

    // Per-node accumulators
    double sinV[NUM_ESP_NODES] = {};
    double cosV[NUM_ESP_NODES] = {};
    int    cntV[NUM_ESP_NODES] = {};

    // Local parsers — independent of the main feedParsers() state
    struct { uint8_t buf[PACKET_SIZE]; int count; } vp[NUM_ESP_NODES];
    memset(vp, 0, sizeof(vp));

    uint32_t deadline = millis() + 2000;
    while (millis() < deadline)
    {
        for (int i = 0; i < NUM_ESP_NODES; i++)
        {
            while (espSerials[i]->available())
            {
                uint8_t b = (uint8_t)espSerials[i]->read();
                if (vp[i].count == 0) {
                    if (b == SYNC_BYTE) { vp[i].buf[0] = b; vp[i].count = 1; }
                } else {
                    vp[i].buf[vp[i].count++] = b;
                    if (vp[i].count == PACKET_SIZE) {
                        vp[i].count = 0;
                        uint8_t chk = 0;
                        for (int j = 0; j < PACKET_SIZE - 1; j++) chk ^= vp[i].buf[j];
                        if (chk == vp[i].buf[PACKET_SIZE - 1]) {
                            float raw;
                            memcpy(&raw, &vp[i].buf[6], sizeof(float));
                            float corrected = raw + phaseCorrection[i];
                            sinV[i] += sin((double)corrected);
                            cosV[i] += cos((double)corrected);
                            cntV[i]++;
                        }
                    }
                }
            }
        }
    }

    setRFSwitch(false);
    Serial.println("[VER] RF switches → ANTENNA path");

    // Compute corrected means; find first node with data to use as residual reference
    float verMean[NUM_ESP_NODES] = {};
    for (int i = 0; i < NUM_ESP_NODES; i++)
        if (cntV[i] > 0)
            verMean[i] = (float)atan2(sinV[i] / cntV[i], cosV[i] / cntV[i]);

    int verRef = -1;
    for (int i = 0; i < NUM_ESP_NODES; i++)
        if (cntV[i] > 0) { verRef = i; break; }

    if (verRef < 0) {
        Serial.println("[VER] No nodes received data during verification");
        setRFSwitch(false);
        return;
    }

    Serial.println("[VER] Results (all corrected phases should agree):");
    for (int node = 0; node < NUM_ESP_NODES; node++) {
        if (cntV[node] == 0) {
            Serial.printf("[VER]   Node %d  NO DATA\n", node);
            continue;
        }
        float residual = fabsf(verMean[node] - verMean[verRef]);
        if (residual > (float)M_PI) residual = 2.0f * (float)M_PI - residual;
        Serial.printf("[VER]   Node %d  corrected=% .4f rad  residual=%.4f rad  %s  (n=%d)\n",
                      node, verMean[node], residual,
                      residual < 0.2f ? "PASS" : "WARN — check coax/switch",
                      cntV[node]);
    }
    Serial.println("[VER] ── Verification complete ───────────────\n");
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
    Serial.print("  Debug Serial (USART3 VCP) : "); Serial.println(bootStatus.debugSerial ? "OK"   : "FAIL");
    Serial.print("  RF Switch CTRL GPIO       : "); Serial.println(bootStatus.rfSwitch    ? "OK"   : "FAIL");
    Serial.print("  nRF24L01+                 : "); Serial.println(bootStatus.nrf24       ? "OK"   : "FAIL");
    Serial.print("  nRF24L01+ link test       : "); Serial.println(bootStatus.nrf24Link   ? "PASS" : "FAIL");
    Serial.print("  Calibration               : "); Serial.println(calDone                ? "DONE" : "NOT RUN");
    Serial.print("  Streaming                 : "); Serial.println(streaming              ? "ON"   : "OFF");
    if (calDone) {
        Serial.println("  Correction vector:");
        for (int i = 0; i < NUM_ESP_NODES; i++)
            Serial.printf("    Node %d: % .4f rad\n", i, phaseCorrection[i]);
    }
    Serial.println("  ESP32 UART channels:");
    for (int i = 0; i < NUM_ESP_NODES; i++)
        Serial.printf("    Node %d : %s\n", i + 1, bootStatus.espUarts[i] ? "OK" : "FAIL");
    Serial.printf("  Boot time: %lu ms\n", bootStatus.bootTimeMs);
    Serial.println("────────────────────────────────────────────");
    Serial.println();
}
