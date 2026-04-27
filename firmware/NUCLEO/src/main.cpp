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
 *  Host packet format (42 bytes, binary, sent during "run" mode):
 *    [0]      SYNC_A     0xBB
 *    [1]      SYNC_B     0xCC
 *    [2..3]   seq        uint16 LE  (wraps at 65535)
 *    [4..7]   timestamp  uint32 LE  (Nucleo millis())
 *    [8..39]  phases[8]  float32 LE each  (corrected phase, radians)
 *    [40]     cal_mask   uint8   bit i = node i has valid calibration
 *    [41]     checksum   XOR of bytes [0..40]
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
#define CAL_SAMPLES          100    // phase measurements averaged per node
#define CAL_TIMEOUT_MS      500    // max wait for one packet per sample attempt (ms)
#define CAL_REFERENCE_NODE    1    // preferred reference node (falls back if no data)
#define CAL_MIN_SAMPLES       3    // minimum samples to consider a node calibrated

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
    int      count;      // bytes buffered (0 = hunting for sync)
    bool     has_data;   // true when a validated packet is ready
    float    phase;      // most recent validated phase (raw, uncorrected)
    uint32_t recv_time;  // Nucleo millis() when has_data was set
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
static float    snapPhase[NUM_ESP_NODES];     // corrected phases for current snapshot
static bool     snapFresh[NUM_ESP_NODES];     // which nodes have new data this window
static uint32_t snapFreshTime[NUM_ESP_NODES]; // when each node's fresh data arrived
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
void sendHostPacket(uint8_t fresh_mask);
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
    memset(&bootStatus,  0, sizeof(bootStatus));
    memset(parsers,      0, sizeof(parsers));
    memset(snapFresh,    0, sizeof(snapFresh));
    memset(snapFreshTime,0, sizeof(snapFreshTime));
    memset(snapPhase,    0, sizeof(snapPhase));
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
            memset(parsers,       0, sizeof(parsers));
            memset(snapFresh,     0, sizeof(snapFresh));
            memset(snapFreshTime, 0, sizeof(snapFreshTime));
            memset(snapPhase,     0, sizeof(snapPhase));
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
                memset(parsers,       0, sizeof(parsers));
                memset(snapFresh,     0, sizeof(snapFresh));
                memset(snapFreshTime, 0, sizeof(snapFreshTime));
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

        } else if (cmd == "pintest") {
            // Cycle through candidate free GPIOs one at a time (2 s each).
            // Watch the RF switch LED — when it toggles, that is your CTRL pin.
            // All UART/SPI/LED/debug pins are excluded from the list.
            const PinName candidates[] = {
                PF_0, PF_1, PF_2, PF_3, PF_4, PF_5,
                PG_0, PG_1, PG_2, PG_3, PG_4, PG_5, PG_6, PG_7,
                PE_2, PE_3, PE_4, PE_5, PE_6,
                PD_0, PD_1, PD_3, PD_4, PD_5, PD_6, PD_7,
                PC_8, PC_9, PC_10, PC_11, PC_12,
                PB_0, PB_1, PB_2, PB_8, PB_9, PB_10, PB_11,
                NC   // sentinel
            };
            bool wasStreaming = streaming;
            streaming = false;
            Serial.println("[PINTEST] Toggling candidate pins LOW→HIGH→LOW (2 s each).");
            Serial.println("          Watch the RF switch LED. Type any key to stop early.");
            for (int ci = 0; candidates[ci] != NC; ci++) {
                if (Serial.available()) { Serial.read(); break; }
                // Print the Arduino pin name via a lookup table
                char pinName[8];
                int port = (candidates[ci] >> 4) & 0xF;
                int num  = candidates[ci] & 0xF;
                snprintf(pinName, sizeof(pinName), "P%c_%d",
                         'A' + port, num);
                Serial.printf("[PINTEST] Testing %-6s ...", pinName);
                pinMode(candidates[ci], OUTPUT);
                digitalWrite(candidates[ci], LOW);
                delay(800);
                digitalWrite(candidates[ci], HIGH);
                delay(800);
                digitalWrite(candidates[ci], LOW);
                delay(400);
                pinMode(candidates[ci], INPUT);   // release pin after test
                Serial.println(" done");
            }
            // Restore correct CTRL pin state — antenna path (HIGH)
            pinMode(RF_SWITCH_CTRL_PIN, OUTPUT);
            digitalWrite(RF_SWITCH_CTRL_PIN, HIGH);
            streaming = wasStreaming;
            Serial.println("[PINTEST] Complete. Update RF_SWITCH_CTRL_PIN in pinconfig.h");

        } else if (cmd == "hi") {
            // Direct GPIO test — force CTRL pin HIGH right now, no cal involved
            pinMode(RF_SWITCH_CTRL_PIN, OUTPUT);
            digitalWrite(RF_SWITCH_CTRL_PIN, HIGH);
            Serial.printf("[GPIO] PF_1 forced HIGH — measure with multimeter now\n");

        } else if (cmd == "lo") {
            // Direct GPIO test — force CTRL pin LOW
            pinMode(RF_SWITCH_CTRL_PIN, OUTPUT);
            digitalWrite(RF_SWITCH_CTRL_PIN, LOW);
            Serial.printf("[GPIO] PF_1 forced LOW\n");

        } else if (cmd == "help" || cmd == "?") {
            Serial.println("Commands: run | stop | cal | verify | status | ant | nrf | pintest | hi | lo");

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
                        p.recv_time = (uint32_t)millis();
                        p.has_data  = true;
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
//  tryEmitSnapshot — assemble one coherent host packet per window
//
//  Harvests fresh phases from completed parsers, applies calibration
//  correction, records per-node arrival times, then emits when either:
//    a) all nodes have fresh data (fast path — same-frame guaranteed), or
//    b) SNAP_TIMEOUT_MS has elapsed and at least one node reported.
//
//  COHERENCE FILTER (applied on the timeout path):
//  WiFi CSI phase = hardware_offset + φ_c, where φ_c is a random carrier
//  phase that changes every transmitted frame.  φ_c is the SAME for all
//  nodes receiving the same beacon frame, so it cancels in inter-element
//  differences — but only if every node in the snapshot came from the
//  SAME frame.  Nodes whose most recent data arrived more than
//  SNAP_SYNC_MS before the freshest node are from an earlier beacon frame
//  (different φ_c) and are excluded from the emitted snapshot.
// ──────────────────────────────────────────────────────────
void tryEmitSnapshot()
{
    // Harvest fresh parser results, recording per-node Nucleo arrival time.
    // Wrap corrected phase to [-π, π]: raw ∈ [-π,π] + correction ∈ [-π,π]
    // can reach ±2π.  exp(j*x) is 2π-periodic so MUSIC is unaffected, but
    // direct scalar comparisons (verify residual, debug prints) need [-π, π].
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        if (parsers[i].has_data) {
            float ph             = parsers[i].phase + phaseCorrection[i];
            snapPhase[i]         = atan2f(sinf(ph), cosf(ph));   // wrap to [-π, π]
            snapFreshTime[i]     = parsers[i].recv_time;
            snapFresh[i]         = true;
            parsers[i].has_data  = false;
        }
    }

    bool allFresh = true;
    bool anyFresh = false;
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        if (!snapFresh[i]) allFresh = false;
        if (snapFresh[i])  anyFresh = true;
    }

    bool timedOut = (millis() - lastSnapTime) >= SNAP_TIMEOUT_MS;

    if (!allFresh && !(timedOut && anyFresh)) {
        if (timedOut) lastSnapTime = millis();   // nothing to send, just reset
        return;
    }

    // Find the most recent arrival among fresh nodes
    uint32_t maxTime = 0;
    for (int i = 0; i < NUM_ESP_NODES; i++)
        if (snapFresh[i] && snapFreshTime[i] > maxTime)
            maxTime = snapFreshTime[i];

    // Build coherent fresh mask: exclude nodes whose data arrived more than
    // SNAP_SYNC_MS before the newest arrival (they're from an older beacon
    // frame with a different random carrier phase φ_c).
    // Fast path (allFresh): all nodes fresh together — skip filter, use 0xFF.
    uint8_t fresh_bits = 0;
    if (allFresh) {
        fresh_bits = 0xFF;
    } else {
        for (int i = 0; i < NUM_ESP_NODES; i++) {
            if (snapFresh[i] && (maxTime - snapFreshTime[i]) <= SNAP_SYNC_MS)
                fresh_bits |= (1 << i);
        }
    }

    if (fresh_bits != 0)
        sendHostPacket(fresh_bits);

    memset(snapFresh, false, sizeof(snapFresh));
    lastSnapTime = millis();
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
void sendHostPacket(uint8_t fresh_mask)
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

    // When calibration has run: only pass nodes that are calibrated AND fresh.
    // When calibration hasn't run yet: pass all fresh nodes with raw phases
    // (phaseCorrection is all-zero, so "uncorrected" = same as "corrected with 0").
    // Without this fallback, calValid=0 zeros every node → MUSIC sees a zero
    // covariance matrix → peak is completely random (causes observed hopping).
    pkt[40] = calDone ? (calValid & fresh_mask) : fresh_mask;

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
    // CTRL=LOW  → calibration path  (relay closed, cal ESP32 beaconing)
    // CTRL=HIGH → antenna path      (relay open,   cal ESP32 silent)
    pinMode(RF_SWITCH_CTRL_PIN, OUTPUT);
    digitalWrite(RF_SWITCH_CTRL_PIN, HIGH);   // start on antenna path
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
    // CTRL=LOW  → cal path    (toReference=true)
    // CTRL=HIGH → antenna     (toReference=false)
    digitalWrite(RF_SWITCH_CTRL_PIN, toReference ? LOW : HIGH);
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

    // Assert CTRL LOW: relay switches to cal path AND cal ESP32 starts beaconing
    Serial.println("[CAL] CTRL → LOW  (relay settling, cal ESP32 starting...)");
    setRFSwitch(true);

    // Relay mechanical settle (~5-20 ms) + cal ESP32 first frame latency (~10 ms)
    // + CSI pipeline flush on each node (~30 ms).  500 ms is generous headroom.
    delay(500);
    Serial.println("[CAL] Relay settled — flushing UART buffers");

    // Flush everything that arrived during the settling window — those measurements
    // were taken while the RF path was still transitioning.
    for (int i = 0; i < NUM_ESP_NODES; i++)
        while (espSerials[i]->available()) espSerials[i]->read();

    // Discard the first 3 valid packets per node after the flush.  The CSI
    // pipeline may still be mid-acquisition on the first frame post-transition.
    Serial.println("[CAL] Discarding first 3 packets per node (pipeline warm-up)...");
    {
        int discarded[NUM_ESP_NODES] = {};
        struct { uint8_t buf[PACKET_SIZE]; int count; } dp[NUM_ESP_NODES];
        memset(dp, 0, sizeof(dp));
        uint32_t warmDeadline = millis() + 500;   // 500 ms max for discard pass
        while (millis() < warmDeadline) {
            bool allWarmedUp = true;
            for (int i = 0; i < NUM_ESP_NODES; i++) {
                if (discarded[i] < 3) allWarmedUp = false;
                while (espSerials[i]->available()) {
                    uint8_t b = (uint8_t)espSerials[i]->read();
                    if (dp[i].count == 0) {
                        if (b == SYNC_BYTE) { dp[i].buf[0] = b; dp[i].count = 1; }
                    } else {
                        dp[i].buf[dp[i].count++] = b;
                        if (dp[i].count == PACKET_SIZE) {
                            dp[i].count = 0;
                            uint8_t chk = 0;
                            for (int j = 0; j < PACKET_SIZE - 1; j++) chk ^= dp[i].buf[j];
                            if (chk == dp[i].buf[PACKET_SIZE - 1])
                                discarded[i]++;
                        }
                    }
                }
            }
            if (allWarmedUp) break;
        }
        Serial.print("[CAL] Warm-up discarded:");
        for (int i = 0; i < NUM_ESP_NODES; i++)
            Serial.printf("  N%d:%d", i, discarded[i]);
        Serial.println();
    }

    // Accumulate per-pair relative phases: delta_i = phase_i - phase_ref.
    // Carrier phase is common to all nodes on the same frame and cancels
    // in the difference, leaving only the hardware offset.
    //
    // Strategy: when the reference node gets a fresh packet, pair it with
    // every other node that also has a fresh packet within SNAP_SYNC_MS.
    // Dead nodes (nodes 0, 7, etc.) are simply never fresh and never
    // accumulate samples — they don't block calibration of alive nodes.
    double sinDiff[NUM_ESP_NODES] = {};
    double cosDiff[NUM_ESP_NODES] = {};
    int    count[NUM_ESP_NODES]   = {};
    bool   alive[NUM_ESP_NODES]   = {};   // set on first valid packet

    struct {
        uint8_t  buf[PACKET_SIZE];
        int      count;
        float    phase;
        bool     fresh;
        uint32_t time;
    } cp[NUM_ESP_NODES];
    memset(cp, 0, sizeof(cp));

    uint32_t calDeadline = millis() + (CAL_SAMPLES * 30UL * NUM_ESP_NODES);

    Serial.printf("[CAL] Collecting %d samples per node (ref=node %d, pair-wise sync)...\n",
                  CAL_SAMPLES, CAL_REFERENCE_NODE);

    while (millis() < calDeadline)
    {
        // Early exit once all alive nodes have enough samples
        if (alive[CAL_REFERENCE_NODE]) {
            bool allDone = true;
            for (int i = 0; i < NUM_ESP_NODES; i++)
                if (alive[i] && count[i] < CAL_SAMPLES) { allDone = false; break; }
            if (allDone) break;
        }

        // Feed bytes into local parsers for all nodes simultaneously
        for (int i = 0; i < NUM_ESP_NODES; i++)
        {
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
                            memcpy(&cp[i].phase, &cp[i].buf[6], sizeof(float));
                            cp[i].time  = (uint32_t)millis();
                            cp[i].fresh = true;
                            alive[i]    = true;
                        }
                    }
                }
            }
        }

        // When reference is fresh, pair it with every other fresh node
        // that arrived within SNAP_SYNC_MS (same beacon frame).
        // Dead nodes are never fresh, so they never block this.
        if (!cp[CAL_REFERENCE_NODE].fresh) continue;

        uint32_t refTime  = cp[CAL_REFERENCE_NODE].time;
        float    refPhase = cp[CAL_REFERENCE_NODE].phase;

        for (int i = 0; i < NUM_ESP_NODES; i++) {
            if (i == CAL_REFERENCE_NODE) continue;
            if (!cp[i].fresh) continue;

            uint32_t dt = cp[i].time > refTime ? cp[i].time - refTime
                                               : refTime  - cp[i].time;
            if (dt > SNAP_SYNC_MS) {
                cp[i].fresh = false;  // from a different frame, discard
                continue;
            }
            if (count[i] < CAL_SAMPLES) {
                float delta = cp[i].phase - refPhase;
                sinDiff[i] += sin((double)delta);
                cosDiff[i] += cos((double)delta);
                count[i]++;
            }
            cp[i].fresh = false;
        }
        // Count this reference packet and release it
        if (count[CAL_REFERENCE_NODE] < CAL_SAMPLES)
            count[CAL_REFERENCE_NODE]++;
        cp[CAL_REFERENCE_NODE].fresh = false;
    }

    for (int node = 0; node < NUM_ESP_NODES; node++)
        Serial.printf("[CAL]   Node %d: %d/%d\n", node, count[node], CAL_SAMPLES);

    // ── Retry pass for nodes below minimum threshold ──────────────────────────
    // RF switches are still HIGH — give struggling nodes one more window.
    int failedCount = 0;
    for (int i = 0; i < NUM_ESP_NODES; i++)
        if (count[i] < CAL_MIN_SAMPLES) failedCount++;

    if (failedCount > 0) {
        Serial.printf("[CAL] %d node(s) below threshold — retry (2 s)...\n", failedCount);
        for (int i = 0; i < NUM_ESP_NODES; i++)
            while (espSerials[i]->available()) espSerials[i]->read();
        memset(cp, 0, sizeof(cp));

        uint32_t retryDeadline = millis() + 2000;
        while (millis() < retryDeadline) {
            for (int i = 0; i < NUM_ESP_NODES; i++) {
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
                                memcpy(&cp[i].phase, &cp[i].buf[6], sizeof(float));
                                cp[i].time  = (uint32_t)millis();
                                cp[i].fresh = true;
                                alive[i]    = true;
                            }
                        }
                    }
                }
            }
            if (!cp[CAL_REFERENCE_NODE].fresh) continue;
            uint32_t refTime  = cp[CAL_REFERENCE_NODE].time;
            float    refPhase = cp[CAL_REFERENCE_NODE].phase;
            for (int i = 0; i < NUM_ESP_NODES; i++) {
                if (i == CAL_REFERENCE_NODE) continue;
                if (!cp[i].fresh) continue;
                uint32_t dt = cp[i].time > refTime ? cp[i].time - refTime
                                                   : refTime  - cp[i].time;
                if (dt > SNAP_SYNC_MS) { cp[i].fresh = false; continue; }
                if (count[i] < CAL_MIN_SAMPLES) {
                    float delta = cp[i].phase - refPhase;
                    sinDiff[i] += sin((double)delta);
                    cosDiff[i] += cos((double)delta);
                    count[i]++;
                }
                cp[i].fresh = false;
            }
            if (count[CAL_REFERENCE_NODE] < CAL_MIN_SAMPLES)
                count[CAL_REFERENCE_NODE]++;
            cp[CAL_REFERENCE_NODE].fresh = false;
        }
        Serial.println("[CAL] Retry results:");
        for (int node = 0; node < NUM_ESP_NODES; node++)
            if (count[node] < CAL_MIN_SAMPLES)
                Serial.printf("[CAL]   Node %d: %d/%d  (still below threshold)\n",
                              node, count[node], CAL_MIN_SAMPLES);
    }

    // De-assert CTRL: relay switches back to antenna path, cal ESP32 goes silent
    setRFSwitch(false);
    delay(500);   // relay settle before measurement resumes
    Serial.println("[CAL] Relay settled — back on antenna path");
    Serial.println("[CAL] RF switches → ANTENNA path");

    // Compute phaseCorrection[i] = -mean(delta_i) = -mean(phase_i - phase_ref).
    // Applying this during streaming: corrected_i = raw_i + correction_i
    //   = (hardware_i + carrier) + (-(hardware_i - hardware_ref))
    //   = hardware_ref + carrier   ← same for all nodes → relative phases zero out
    //
    // Reference node always gets correction = 0 (delta_ref = 0 by construction).
    calValid = 0x00;
    for (int node = 0; node < NUM_ESP_NODES; node++) {
        if (count[node] >= CAL_MIN_SAMPLES) {
            calValid |= (1 << node);
            float meanDelta = (float)atan2(sinDiff[node] / count[node],
                                           cosDiff[node] / count[node]);
            phaseCorrection[node] = -meanDelta;
        } else {
            phaseCorrection[node] = 0.0f;  // no correction — will be masked in Python
            Serial.printf("[CAL] WARN: Node %d uncalibrated (%d sync-snapshots) — excluded\n",
                          node, count[node]);
        }
    }
    // Reference node correction is exactly zero by definition
    phaseCorrection[CAL_REFERENCE_NODE] = 0.0f;

    Serial.printf("[CAL] Correction vector (node %d = reference):\n", CAL_REFERENCE_NODE);
    for (int node = 0; node < NUM_ESP_NODES; node++) {
        Serial.printf("[CAL]   Node %d  snapshots=%d  correction=% .4f rad  %s\n",
                      node, count[node], phaseCorrection[node],
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
    delay(300);   // RF path + CSI pipeline settling (same as runCalibration)

    // Flush settling-window data
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
