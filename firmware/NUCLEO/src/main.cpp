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
 *  Pin assignments: see pin_config.h
 */

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "pinconfig.h"

// ──────────────────────────────────────────────────────────
//  Calibration configuration
// ──────────────────────────────────────────────────────────
#define CAL_SAMPLES       20      // phase measurements averaged per node
#define CAL_TIMEOUT_MS   500      // max wait for one packet from a node (ms)

// ──────────────────────────────────────────────────────────
//  Calibration state  (populated once at boot by runCalibration)
// ──────────────────────────────────────────────────────────
static float phaseCorrection[NUM_ESP_NODES] = {0.0f};  // correction[i] = phase[0] - phase[i]
static bool  calDone = false;

// ──────────────────────────────────────────────────────────
//  Forward declarations
// ──────────────────────────────────────────────────────────
void initDebugSerial();
void initRFSwitchGPIO();
void initESP32UARTs();
void initNRF24SPI();
bool testNRF24Link();
void printBootBanner();
void printBootDiagnostics();
void setRFSwitch(bool toReference);
bool readCSIPacket(HardwareSerial* ser, uint8_t* nodeId, float* phase, uint32_t timeoutMs);
void runCalibration();

// ──────────────────────────────────────────────────────────
//  UART handles for the 8 ESP32 nodes
//  On STM32 Arduino core, HardwareSerial takes UART instance
// ──────────────────────────────────────────────────────────
// Serial  = USART3 (ST-Link VCP, debug) — built-in, don't redeclare
// The rest we declare explicitly with their RX/TX pins

HardwareSerial ESP_Serial1(ESP1_UART_RX, ESP1_UART_TX);  // ESP32 node 1
HardwareSerial ESP_Serial2(ESP2_UART_RX, ESP2_UART_TX);  // ESP32 node 2
HardwareSerial ESP_Serial3(ESP3_UART_RX, ESP3_UART_TX);  // ESP32 node 3
HardwareSerial ESP_Serial4(ESP4_UART_RX, ESP4_UART_TX);  // ESP32 node 4
HardwareSerial ESP_Serial5(ESP5_UART_RX, ESP5_UART_TX);  // ESP32 node 5
HardwareSerial ESP_Serial6(ESP6_UART_RX, ESP6_UART_TX);  // ESP32 node 6
HardwareSerial ESP_Serial7(ESP7_UART_RX, ESP7_UART_TX);  // ESP32 node 7
HardwareSerial ESP_Serial8(ESP8_UART_RX, ESP8_UART_TX);  // ESP32 node 8

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
    bool nrf24SPI;
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

    // ── 1. Debug serial (USART3 via ST-Link VCP) ────────
    initDebugSerial();
    printBootBanner();

    // ── 2. RF switch GPIO (shared CTRL for all BGS12WN6) ─
    Serial.println("[BOOT] Initializing RF switch CTRL...");
    initRFSwitchGPIO();

    // ── 3. SPI for nRF24L01+ ────────────────────────────
    Serial.println("[BOOT] Initializing nRF24L01+ SPI bus...");
    initNRF24SPI();

    // ── 4. Test nRF24L01+ communication ─────────────────
    Serial.println("[BOOT] Testing nRF24L01+ link...");
    bootStatus.nrf24Link = testNRF24Link();

    // ── 5. ESP32 UART channels ──────────────────────────
    Serial.println("[BOOT] Initializing ESP32 UART channels...");
    initESP32UARTs();

    // ── 6. Default RF switch state: antenna path ────────
    //    CTRL=0 → RFIN-RF1 (antenna), CTRL=1 → RFIN-RF2 (reference)
    setRFSwitch(false);
    Serial.println("[BOOT] RF switches set to ANTENNA path (CTRL=LOW)");

    // ── 7. One-shot calibration via nRF reference path ──
    runCalibration();

    // ── Done ────────────────────────────────────────────
    bootStatus.bootTimeMs = millis() - bootStart;
    printBootDiagnostics();
    Serial.println("[BOOT] === System ready ===\n");
}

// ══════════════════════════════════════════════════════════
//  LOOP — placeholder for main state machine
// ══════════════════════════════════════════════════════════
void loop() {
    // Future: main operational state machine
    //   STATE_IDLE → STATE_CALIBRATE → STATE_CAPTURE → STATE_PROCESS
    //
    // For i, echo anything received from any ESP32 to debug serial
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

    // Echo debug serial commands back (future: command parser)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "status") {
            printBootDiagnostics();
        } 
        else if (cmd == "cal") {
            Serial.println("[CMD] Switching RF to REFERENCE path...");
            setRFSwitch(true);
            Serial.println("[CMD] Reference path active. Send 'ant' to return.");
        }
        else if (cmd == "ant") {
            Serial.println("[CMD] Switching RF to ANTENNA path...");
            setRFSwitch(false);
            Serial.println("[CMD] Antenna path active.");
        }
        else if (cmd == "nrf") {
            Serial.print("[CMD] nRF24L01+ link test: ");
            Serial.println(testNRF24Link() ? "PASS" : "FAIL");
        }
        else if (cmd.length() > 0) {
            Serial.print("[CMD] Unknown: ");
            Serial.println(cmd);
            Serial.println("  Commands: status, cal, ant, nrf");
        }
    }
}

// ──────────────────────────────────────────────────────────
//  Init functions
// ──────────────────────────────────────────────────────────

void initDebugSerial() {
    Serial.begin(DEBUG_BAUD);
    // Wait up to 2 seconds for USB serial to connect (host side)
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 2000)) {
        // spin
    }
    bootStatus.debugSerial = true;
}

void initRFSwitchGPIO() {
    // Single GPIO controls all 8 BGS12WN6 CTRL pins simultaneously.
    // This is intentional — individual GPIOs per switch would cause
    // non-simultaneous channel snapshots and phase errors.
    //
    // BGS12WN6 truth table:
    //   CTRL=0 → RFIN routed to RF1 (antenna path)
    //   CTRL=1 → RFIN routed to RF2 (reference/calibration path)
    pinMode(RF_SWITCH_CTRL_PIN, OUTPUT);
    digitalWrite(RF_SWITCH_CTRL_PIN, LOW);  // Default: antenna path
    bootStatus.rfSwitch = true;
}

void initNRF24SPI() {
    // nRF24L01+ SPI: up to 10 MHz clock
    // Using hardware SPI bus with explicit CS pin
    pinMode(NRF24_CS_PIN, OUTPUT);
    digitalWrite(NRF24_CS_PIN, HIGH);  // Deselect
    
    pinMode(NRF24_CE_PIN, OUTPUT);
    digitalWrite(NRF24_CE_PIN, LOW);   // Standby mode
    
    SPI.begin();  // Uses default SPI pins from pin_config.h
    // SPI settings: 8 MHz, MSB first, SPI Mode 0 (CPOL=0, CPHA=0)
    // Applied per-transaction in nRF24 read/write functions
    
    bootStatus.nrf24SPI = true;
}

bool testNRF24Link() {
    // Quick sanity check: read the nRF24L01+ STATUS register (0x07)
    // After reset it should be 0x0E (RX_DR=0, TX_DS=0, MAX_RT=0,
    //                                 RX_P_NO=111, TX_FULL=0)
    //
    // Also try writing/reading the TX_ADDR register to verify
    // bidirectional SPI communication.
    
    SPI.beginTransaction(SPISettings(NRF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
    
    // Read STATUS register
    digitalWrite(NRF24_CS_PIN, LOW);
    uint8_t status = SPI.transfer(0x07);  // NOP command returns STATUS
    digitalWrite(NRF24_CS_PIN, HIGH);
    
    // Write a test pattern to TX_ADDR (register 0x10, 5 bytes)
    uint8_t testAddr[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01};
    uint8_t readBack[5] = {0};
    
    // Write
    digitalWrite(NRF24_CS_PIN, LOW);
    SPI.transfer(0x20 | 0x10);  // W_REGISTER | TX_ADDR
    for (int i = 0; i < 5; i++) {
        SPI.transfer(testAddr[i]);
    }
    digitalWrite(NRF24_CS_PIN, HIGH);
    
    delayMicroseconds(10);
    
    // Read back
    digitalWrite(NRF24_CS_PIN, LOW);
    SPI.transfer(0x10);  // R_REGISTER | TX_ADDR
    for (int i = 0; i < 5; i++) {
        readBack[i] = SPI.transfer(0xFF);
    }
    digitalWrite(NRF24_CS_PIN, HIGH);
    
    SPI.endTransaction();
    
    // Verify
    bool match = true;
    for (int i = 0; i < 5; i++) {
        if (testAddr[i] != readBack[i]) {
            match = false;
            break;
        }
    }
    
    if (!match) {
        Serial.print("[WARN] nRF24 TX_ADDR readback mismatch. Got: ");
        for (int i = 0; i < 5; i++) {
            Serial.print("0x");
            if (readBack[i] < 0x10) Serial.print("0");
            Serial.print(readBack[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    // STATUS should have bits [3:1] = 111 (no RX data) after reset
    // but we mainly care that SPI is alive, so check readback
    return match;
}

void initESP32UARTs() {
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        espSerials[i]->begin(ESP_BAUD);
        bootStatus.espUarts[i] = true;  // Opened successfully
        
        Serial.print("  UART ");
        Serial.print(i + 1);
        Serial.println(" ... OK");
    }
}

// ──────────────────────────────────────────────────────────
//  RF switch control
// ──────────────────────────────────────────────────────────

void setRFSwitch(bool toReference) {
    // CTRL=LOW  → RFIN-RF1 (antenna path, normal operation)
    // CTRL=HIGH → RFIN-RF2 (reference/calibration path from Wilkinson)
    digitalWrite(RF_SWITCH_CTRL_PIN, toReference ? HIGH : LOW);
}

// ──────────────────────────────────────────────────────────
//  CSI packet parser
// ──────────────────────────────────────────────────────────

// Reads one 11-byte binary packet from a node's UART.
// Hunts for the 0xAA sync byte, then reads the remaining 10 bytes,
// validates the XOR checksum, and extracts nodeId + phase.
// Returns true on success, false on timeout or checksum failure.
bool readCSIPacket(HardwareSerial* ser, uint8_t* nodeId, float* phase, uint32_t timeoutMs)
{
    uint32_t deadline = millis() + timeoutMs;

    while (millis() < deadline)
    {
        if (!ser->available()) continue;

        // Hunt for sync byte
        if (ser->read() != SYNC_BYTE) continue;

        // Read remaining 10 bytes with a short inner timeout
        uint8_t buf[PACKET_SIZE - 1];
        uint32_t innerDeadline = millis() + 50;
        int n = 0;
        while (n < (PACKET_SIZE - 1) && millis() < innerDeadline)
        {
            if (ser->available()) buf[n++] = ser->read();
        }
        if (n < (PACKET_SIZE - 1)) continue;  // incomplete — resync

        // Validate XOR checksum: XOR of all 11 bytes (including sync) must be 0
        uint8_t chk = SYNC_BYTE;
        for (int i = 0; i < PACKET_SIZE - 2; i++) chk ^= buf[i];
        if (chk != buf[PACKET_SIZE - 2]) continue;  // bad checksum — resync

        // Extract fields
        *nodeId = buf[0];
        // buf[1..4] = timestamp (not needed for calibration)
        memcpy(phase, &buf[5], sizeof(float));
        return true;
    }
    return false;  // timed out
}

// ──────────────────────────────────────────────────────────
//  Calibration
// ──────────────────────────────────────────────────────────

// Runs once at boot:
//   1. Switches all RF paths to the nRF reference signal
//   2. Collects CAL_SAMPLES phase measurements from each node
//   3. Computes a circular mean phase per node (handles ±π wrap correctly)
//   4. Normalises to node 0: correction[i] = meanPhase[0] - meanPhase[i]
//   5. Switches RF paths back to antenna
//
// The correction vector is stored in phaseCorrection[] and applied
// to every measurement taken during normal operation.
void runCalibration()
{
    Serial.println("[CAL] ── Calibration start ──────────────────");
    Serial.println("[CAL] Switching RF to REFERENCE path (nRF signal)");
    setRFSwitch(true);
    delay(50);  // let RF switch and nRF settle

    // Circular mean accumulators: average phases with sin/cos sums
    // to handle the ±π wrap-around correctly
    double sinSum[NUM_ESP_NODES] = {};
    double cosSum[NUM_ESP_NODES] = {};
    int    count[NUM_ESP_NODES]  = {};

    Serial.printf("[CAL] Collecting %d samples per node...\n", CAL_SAMPLES);

    for (int node = 0; node < NUM_ESP_NODES; node++)
    {
        Serial.printf("[CAL]   Node %d: ", node);

        for (int s = 0; s < CAL_SAMPLES; s++)
        {
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
                Serial.print("x");  // timed out — node not responding
            }
        }

        Serial.printf("  (%d/%d)\n", count[node], CAL_SAMPLES);
    }

    // Compute circular mean phase per node, then correction relative to node 0
    float meanPhase[NUM_ESP_NODES] = {};

    for (int node = 0; node < NUM_ESP_NODES; node++)
    {
        if (count[node] > 0)
        {
            meanPhase[node] = (float)atan2(
                sinSum[node] / count[node],
                cosSum[node] / count[node]
            );
        }
        else
        {
            Serial.printf("[CAL] WARN: Node %d returned no data — correction = 0\n", node);
            meanPhase[node] = 0.0f;
        }
    }

    Serial.println("[CAL] Correction vector (node 0 = reference):");
    for (int node = 0; node < NUM_ESP_NODES; node++)
    {
        phaseCorrection[node] = meanPhase[0] - meanPhase[node];
        Serial.printf("[CAL]   Node %d  mean=% .4f rad  correction=% .4f rad\n",
                      node, meanPhase[node], phaseCorrection[node]);
    }

    setRFSwitch(false);
    calDone = true;
    Serial.println("[CAL] RF switch returned to ANTENNA path");
    Serial.println("[CAL] ── Calibration complete ───────────────");
    Serial.println();
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
    
    Serial.print("  Debug Serial (USART3 VCP) : ");
    Serial.println(bootStatus.debugSerial ? "OK" : "FAIL");
    
    Serial.print("  RF Switch CTRL GPIO       : ");
    Serial.println(bootStatus.rfSwitch ? "OK" : "FAIL");
    
    Serial.print("  nRF24L01+ SPI bus         : ");
    Serial.println(bootStatus.nrf24SPI ? "OK" : "FAIL");
    
    Serial.print("  nRF24L01+ link test       : ");
    Serial.println(bootStatus.nrf24Link ? "PASS" : "FAIL");
    
    Serial.println("  ESP32 UART channels:");
    for (int i = 0; i < NUM_ESP_NODES; i++) {
        Serial.print("    Node ");
        Serial.print(i + 1);
        Serial.print("                  : ");
        Serial.println(bootStatus.espUarts[i] ? "OK" : "FAIL");
    }
    
    Serial.print("  Total boot time           : ");
    Serial.print(bootStatus.bootTimeMs);
    Serial.println(" ms");
    
    Serial.println("────────────────────────────────────────────");
    Serial.println();
}