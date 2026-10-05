#pragma once
#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Radio configuration (SX1276 / RFM95)
// Verify 433 vs 868 MHz for Tunisia before final deployment.
// ---------------------------------------------------------------------------
#define LORA_FREQ_MHZ     868.0
#define LORA_BW_KHZ       125.0
#define LORA_SF           7
#define LORA_CR           5       // 4/5
#define LORA_SYNC_WORD    0x12
#define LORA_TX_DBM       14
#define LORA_PREAMBLE     8

// Packet types (first byte on the wire)
#define PKT_BEACON   0x01
#define PKT_LOG      0x02
#define PKT_MISSION  0x03

// Event types
#define EVT_NONE     0
#define EVT_VICTIM   1
#define EVT_GAS      2
#define EVT_BLOCKED  3

// Aging constants (receiver-side)
constexpr float TAU_GAS_S    = 600.0f;
constexpr float TAU_VICTIM_S = 1800.0f;
constexpr float STALE_CONF   = 0.3f;

// Mission cap (243 B on air, under the SX1276 255 B FIFO limit)
#define MISSION_MAX_N 48

// ---------------------------------------------------------------------------
// Wire formats (all little-endian, packed)
// ---------------------------------------------------------------------------
// BEACON  : [type:1][BeaconMsg:14]                     -> 15 B on air
// LOG     : [type:1][BeaconMsg:14][parentId:1]         -> 16 B on air
// MISSION : [type:1][targetEvent:1][n:1][entries:5n]   -> 3 + 5n B on air
// ---------------------------------------------------------------------------

struct __attribute__((packed)) BeaconMsg {
  uint8_t  version;      // 1
  uint8_t  beaconId;     // 1
  uint8_t  eventType;    // 1  EVT_*
  uint8_t  nextHopId;    // 1  0xFF = none
  int16_t  x_cm;         // 2
  int16_t  y_cm;         // 2
  int16_t  bearingDeg;   // 2  Writer heading at drop time
  uint32_t ageAtWriteS;  // 4  re-stamped at every TX
};                       // = 14 B

struct __attribute__((packed)) LogMsg {
  BeaconMsg b;           // 14
  uint8_t   parentId;    // 1  beacon it was reached from
};                       // = 15 B

struct __attribute__((packed)) MissionEntry {
  uint8_t beaconId;      // 1
  int16_t x_cm;          // 2
  int16_t y_cm;          // 2
};                       // = 5 B

struct __attribute__((packed)) MissionMsg {
  uint8_t      targetEvent;             // 1
  uint8_t      n;                       // 1
  MissionEntry entries[MISSION_MAX_N];  // 5 * n used
};                                      // = 2 + 5n B payload

static inline size_t missionWireSize(uint8_t n) { return 3u + 5u * n; }

// Compile-time size checks: these must stay in sync with livingmap/beacon.py (BEACON 15 B, LOG 16 B on air).
static_assert(sizeof(BeaconMsg) == 14, "BeaconMsg must be 14 B");
static_assert(sizeof(LogMsg) == 15, "LogMsg must be 15 B");
static_assert(sizeof(MissionEntry) == 5, "MissionEntry must be 5 B");
