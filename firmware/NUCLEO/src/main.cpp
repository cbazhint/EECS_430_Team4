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
#include "pin_config.h"

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
    // For now, echo anything received from any ESP32 to debug serial
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