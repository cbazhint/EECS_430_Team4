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
#define CAL_SAMPLES      20     // phase measurements averaged per node
#define CAL_TIMEOUT_MS  500     // max wait for one packet per sample attempt (ms)

static float phaseCorrection[NUM_ESP_NODES] = {0.0f};
static bool  calDone = false;

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

        } else if (cmd == "ant") {
            setRFSwitch(false);
            Serial.println("[CMD] RF → ANTENNA path");

        } else if (cmd == "nrf") {
            Serial.print("[CMD] nRF24 link: ");
            Serial.println(testNRF24Link() ? "PASS" : "FAIL");

        } else if (cmd == "help" || cmd == "?") {
            Serial.println("Commands: run | stop | cal | status | ant | nrf");

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
    Serial.println("I AM INITIALIZING!!!");
    // Remap SPI1 to PB3/PB4/PB5 — avoids PA7 Ethernet conflict and PC10/PC11 SDMMC pull-downs
    SPI.setMOSI(NRF24_MOSI_PIN);  // PB_5 → SPI1 MOSI
    SPI.setMISO(NRF24_MISO_PIN);  // PB_4 → SPI1 MISO
    SPI.setSCLK(NRF24_SCK_PIN);   // PB_3 → SPI1 SCK

    // ── Raw SPI diagnostic — read nRF24 STATUS register directly ─────────────
    // STATUS reg (addr 0x07) resets to 0x0E on a working chip.
    // 0xFF = MISO floating (not connected)
    // 0x00 = MISO pulled low (short or wrong pin)
    // 0x0E = SPI working, chip alive
    SPI.begin();
    Serial.println("[nRF24] SPI.begin() returned (SPI3 pins)");
    Serial.flush();
    pinMode(NRF24_CS_PIN, OUTPUT);
    digitalWrite(NRF24_CS_PIN, HIGH);
    delay(5);
    digitalWrite(NRF24_CS_PIN, LOW);
    uint8_t spiStatus = SPI.transfer(0xFF);  // NOP command returns STATUS byte
    digitalWrite(NRF24_CS_PIN, HIGH);
    Serial.printf("[nRF24] Raw STATUS via SPI3: 0x%02X  (expect 0x0E)\n", spiStatus);
    Serial.flush();
    // ─────────────────────────────────────────────────────────────────────────

    if (!radio.begin()) {
        Serial.println("[WARN] nRF24: radio.begin() failed — check wiring");
        bootStatus.nrf24 = false;
        return;
    }
    // nRF channel N = (2400+N) MHz  →  WiFi ch 6 (2437 MHz) = nRF ch 37
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(NRF24_WIFI_CHANNEL);
    radio.stopListening();
    bootStatus.nrf24 = true;
    Serial.printf("  nRF24 ready: ch=%d (%.0f MHz), 250kbps, PA=MAX\n",
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

    if (!bootStatus.nrf24) {
        Serial.println("[CAL] ERROR: nRF24 not ready — skipping calibration");
        return;
    }

    Serial.println("[CAL] RF switches → REFERENCE path (nRF trace)");
    setRFSwitch(true);
    delay(50);

    radio.openWritingPipe((const uint8_t*)NRF24_CAL_ADDRESS);
    radio.stopListening();

    uint8_t calPayload[4] = {0xCA, 0x1B, 0x00, 0x00};

    double sinSum[NUM_ESP_NODES] = {};
    double cosSum[NUM_ESP_NODES] = {};
    int    count[NUM_ESP_NODES]  = {};

    Serial.printf("[CAL] Collecting %d samples per node...\n", CAL_SAMPLES);

    for (int node = 0; node < NUM_ESP_NODES; node++)
    {
        Serial.printf("[CAL]   Node %d: ", node);

        for (int s = 0; s < CAL_SAMPLES; s++)
        {
            calPayload[2] = (uint8_t)node;
            calPayload[3] = (uint8_t)s;
            radio.write(calPayload, sizeof(calPayload));

            uint8_t rxId;
            float   rxPhase;
            if (readCSIPacket(espSerials[node], &rxId, &rxPhase, CAL_TIMEOUT_MS)) {
                sinSum[node] += sin((double)rxPhase);
                cosSum[node] += cos((double)rxPhase);
                count[node]++;
                Serial.print(".");
            } else {
                Serial.print("x");
            }
        }
        Serial.printf("  (%d/%d)\n", count[node], CAL_SAMPLES);
    }

    radio.powerDown();
    setRFSwitch(false);
    Serial.println("[CAL] nRF powered down, RF switches → ANTENNA path");

    float meanPhase[NUM_ESP_NODES] = {};
    for (int node = 0; node < NUM_ESP_NODES; node++) {
        if (count[node] > 0) {
            meanPhase[node] = (float)atan2(
                sinSum[node] / count[node],
                cosSum[node] / count[node]);
        } else {
            Serial.printf("[CAL] WARN: Node %d no data — using node 0 phase\n", node);
            meanPhase[node] = meanPhase[0];
        }
    }

    Serial.println("[CAL] Correction vector (node 0 = reference):");
    for (int node = 0; node < NUM_ESP_NODES; node++) {
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
