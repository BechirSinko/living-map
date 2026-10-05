#include <Arduino.h>
#include <RadioLib.h>
#include "beacon_msg.h"

#ifndef BEACON_ID
#define BEACON_ID 1
#endif

#define LORA_CS   5
#define LORA_DIO0 26
#define LORA_RST  14
#define LORA_DIO1 33

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);

BeaconMsg fact = {
  1,           // version
  BEACON_ID,   // beaconId
  EVT_NONE,    // eventType
  0xFF,        // nextHopId
  0, 0,        // x_cm, y_cm
  0,           // bearingDeg
  0            // ageAtWriteS (re-stamped each TX)
};

uint32_t writtenAtMs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("Beacon %u booting\n", BEACON_ID);

  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("radio init failed: %d\n", st);
    while (true) delay(1000);
  }
  radio.setCRC(true);

  writtenAtMs = millis();
}

void loop() {
  fact.ageAtWriteS = (millis() - writtenAtMs) / 1000;

  uint8_t frame[1 + sizeof(BeaconMsg)];
  frame[0] = PKT_BEACON;
  memcpy(&frame[1], &fact, sizeof(BeaconMsg));

  int st = radio.transmit(frame, sizeof(frame));
  Serial.printf("TX beacon %u type=BEACON %u B status=%d age=%lus\n",
                BEACON_ID, (unsigned)sizeof(frame), st,
                (unsigned long)fact.ageAtWriteS);

  delay(5500 + random(0, 2000));  // 5.5 s + jitter
}