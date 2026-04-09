/**
 * ==========================================================
 *  Pin Configuration — Nucleo-H753ZI DoA System
 * ==========================================================
 *
 *  !! IMPORTANT !!
 *  Fill in pin names from your CubeMX assignment.
 *  STM32 Arduino core uses PX_N notation (e.g., PA_9, PD_1).
 *
 *  Cross-reference:
 *    MCU pin (e.g., PD1) → UM2407 Table 22 → morpho connector location
 *    BGS12WN6 CTRL truth table: 0=RF1(antenna), 1=RF2(reference)
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ──────────────────────────────────────────────────────────
//  System constants
// ──────────────────────────────────────────────────────────
#define NUM_ESP_NODES       8
#define DEBUG_BAUD          115200
#define ESP_BAUD            921600   // High speed for CSI data throughput
#define NRF24_SPI_SPEED     8000000  // 8 MHz SPI clock for nRF24L01+

// ──────────────────────────────────────────────────────────
//  Debug Serial — USART3 via ST-Link VCP
//  PD8 (TX) / PD9 (RX) — hardwired on Nucleo, handled by Serial
//  No pin definition needed, just use Serial.begin()
// ──────────────────────────────────────────────────────────

// ──────────────────────────────────────────────────────────
//  nRF24L01+ SPI — CW reference transmitter
//  Using SPI1 (default Arduino SPI bus on Nucleo-144)
//    SCK  = PA5  (CN7 pin 10, D13)
//    MISO = PA6  (CN7 pin 12, D12)
//    MOSI = PA7  (CN7 pin 14, D11)  ** check SB31 for Ethernet conflict **
//  CS and CE are generic GPIOs — pick two free pins
// ──────────────────────────────────────────────────────────
#define NRF24_CS_PIN        PB_6    // TODO: confirm from your CubeMX
#define NRF24_CE_PIN        PB_7    // TODO: confirm from your CubeMX

// ──────────────────────────────────────────────────────────
//  RF Switch — BGS12WN6 shared CTRL
//  Single GPIO drives all 8 switch CTRL pins in parallel.
//  Do NOT use separate GPIOs — simultaneous switching is
//  required for phase calibration integrity.
//
//  Pick any free GPIO with no alternate-function conflict.
// ──────────────────────────────────────────────────────────
#define RF_SWITCH_CTRL_PIN  PC_6    // TODO: confirm from your CubeMX

// ──────────────────────────────────────────────────────────
//  ESP32 Node UARTs
//
//  Fill in from your CubeMX pin assignments.
//  Format: PX_N  (e.g., PD_1 for PD1)
//
//  Reminder of known constraints:
//    - USART3 (PD8/PD9) reserved for ST-Link VCP
//    - Ethernet pins (PA1,PA2,PA7,PB13,PC1,PC4,PC5,PG11,PG13) if enabled
//    - USB OTG (PA9-PA12)
//    - SWD debug (PA13, PA14)
//    - PE1 (UART8_TX) hardwired to LD2 yellow LED on PCB trace
//
//  Candidate UART instances (from our earlier CubeMX work):
//    USART1, USART2, USART6, UART4, UART5, UART7, UART8, LPUART1
//  UART8 status depends on your PE1 resolution.
// ──────────────────────────────────────────────────────────

// ESP32 Node 1 — USART1
#define ESP1_UART_TX        PA_9    // TODO: verify — may conflict with USB OTG
#define ESP1_UART_RX        PA_10   // TODO: verify

// ESP32 Node 2 — USART2
#define ESP2_UART_TX        PD_5    // TODO: verify
#define ESP2_UART_RX        PD_6    // TODO: verify

// ESP32 Node 3 — USART6
#define ESP3_UART_TX        PC_6    // !! CONFLICT if also used for RF_SWITCH_CTRL !!
#define ESP3_UART_RX        PC_7    // TODO: verify

// ESP32 Node 4 — UART4
#define ESP4_UART_TX        PD_1    // TODO: verify
#define ESP4_UART_RX        PD_0    // TODO: verify

// ESP32 Node 5 — UART5
#define ESP5_UART_TX        PC_12   // TODO: verify
#define ESP5_UART_RX        PD_2    // TODO: verify

// ESP32 Node 6 — UART7
#define ESP6_UART_TX        PE_8    // TODO: verify
#define ESP6_UART_RX        PE_7    // TODO: verify

// ESP32 Node 7 — UART8
//  ** PE1 (only TX option) is hardwired to LD2 on Nucleo **
//  If you resolved this, fill in. Otherwise use alt peripheral.
#define ESP7_UART_TX        PE_1    // TODO: confirm PE1 conflict resolved
#define ESP7_UART_RX        PE_0    // TODO: verify

// ESP32 Node 8 — LPUART1
#define ESP8_UART_TX        PB_6    // !! CONFLICT if also used for NRF24_CS !!
#define ESP8_UART_RX        PB_7    // !! CONFLICT if also used for NRF24_CE !!

// ──────────────────────────────────────────────────────────
//  Sync pulse GPIO (optional)
//  If using a dedicated GPIO for "take measurement now" sync
//  to all ESP32s (separate from nRF24 channel), define here.
// ──────────────────────────────────────────────────────────
// #define SYNC_PULSE_PIN   PX_N   // TODO: define if needed

#endif // PIN_CONFIG_H