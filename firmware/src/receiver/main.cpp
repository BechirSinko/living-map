#include <Arduino.h>
#include <RadioLib.h>
#include <math.h>
#include "beacon_msg.h"

#define LORA_CS   5
#define LORA_DIO0 26
#define LORA_RST  14
#define LORA_DIO1 33

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);

static float confidence(uint8_t eventType, uint32_t ageS) {
  if (eventType == EVT_NONE) return 1.0f;
  float tau = (eventType == EVT_GAS) ? TAU_GAS_S : TAU_VICTIM_S;
  return expf(-(float)ageS / tau);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("radio init failed: %d\n", st);
    while (true) delay(1000);
  }
  radio.setCRC(true);
}

void loop() {
  uint8_t buf[255];
  int st = radio.receive(buf, sizeof(buf));
  if (st != RADIOLIB_ERR_NONE) return;

  size_t len = radio.getPacketLength();
  if (len < 1) return;

  uint8_t type = buf[0];

  if (type == PKT_BEACON && len == 1 + sizeof(BeaconMsg)) {
    BeaconMsg m;
    memcpy(&m, &buf[1], sizeof(m));
    Serial.print("raw:");
    for (size_t i = 0; i < len; i++) Serial.printf(" %02X", buf[i]);
    Serial.println();
    float c = confidence(m.eventType, m.ageAtWriteS);
    Serial.printf("BEACON id=%u evt=%u next=%u (%d,%d)cm brg=%d age=%lus conf=%.2f RSSI=%.1f SNR=%.1f%s\n",
                  m.beaconId, m.eventType, m.nextHopId,
                  m.x_cm, m.y_cm, m.bearingDeg,
                  (unsigned long)m.ageAtWriteS, c,
                  radio.getRSSI(), radio.getSNR(),
                  (c < STALE_CONF ? "  [STALE]" : ""));
  }
  else if (type == PKT_LOG && len == 1 + sizeof(LogMsg)) {
    LogMsg l;
    memcpy(&l, &buf[1], sizeof(l));
    Serial.printf("LOG id=%u evt=%u parent=%u (%d,%d)cm age=%lus RSSI=%.1f\n",
                  l.b.beaconId, l.b.eventType, l.parentId,
                  l.b.x_cm, l.b.y_cm,
                  (unsigned long)l.b.ageAtWriteS, radio.getRSSI());
  }
  else if (type == PKT_MISSION && len >= 3) {
    MissionMsg mm;
    uint8_t n = buf[2];
    if (n > MISSION_MAX_N) n = MISSION_MAX_N;
    if (len != 3u + 5u * n) { Serial.println("MISSION bad length"); return; }
    mm.targetEvent = buf[1];
    mm.n = n;
    memcpy(mm.entries, &buf[3], 5u * n);
    Serial.printf("MISSION target=%u n=%u\n", mm.targetEvent, mm.n);
    for (uint8_t i = 0; i < mm.n; i++) {
      Serial.printf("  wp[%u] id=%u (%d,%d)cm\n", i,
                    mm.entries[i].beaconId,
                    mm.entries[i].x_cm, mm.entries[i].y_cm);
    }
  }
  else {
    Serial.printf("unknown frame type=%u len=%u\n", type, (unsigned)len);
  }
}