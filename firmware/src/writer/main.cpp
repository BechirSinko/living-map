#include <Arduino.h>
#include <RadioLib.h>
#include <ESP32Servo.h>
#include "beacon_msg.h"

#define LORA_CS   5
#define LORA_DIO0 26
#define LORA_RST  14
#define LORA_DIO1 33
#define SERVO_PIN 18

// Set to the ID of the last beacon the Writer will drop.
// Left at 0xFF: the Writer does not know the mission length.
// nextHopId is a fallback; real navigation comes from the MISSION briefing.
#define LAST_BEACON_ID  0xFF

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
Servo dropServo;

static int16_t x_cm = 0;
static int16_t y_cm = 0;
static int16_t headingDeg = 0;
static uint8_t nextBeaconId = 1;
static uint8_t prevBeaconId = 0xFF;

static void dropBeacon(uint8_t eventType) {
  dropServo.write(90);
  delay(500);
  dropServo.write(0);

  // nextHopId = the next beacon in drop order.
  // 0xFF if this is the last beacon (chain terminator).
  uint8_t nextHop = (uint8_t)(nextBeaconId + 1);
  if (LAST_BEACON_ID != 0xFF && nextBeaconId == LAST_BEACON_ID) {
    nextHop = 0xFF;
  }

  BeaconMsg m = {
    1,             // version
    nextBeaconId,  // beaconId
    eventType,     // eventType
    nextHop,       // nextHopId  <-- forward pointer, not the parent
    x_cm, y_cm, headingDeg,
    0              // ageAtWriteS (beacon re-stamps at each TX)
  };

  uint8_t frame[1 + sizeof(BeaconMsg)];
  frame[0] = PKT_BEACON;
  memcpy(&frame[1], &m, sizeof(BeaconMsg));

  int st = radio.transmit(frame, sizeof(frame));
  Serial.printf("Dropped beacon %u evt=%u next=%u status=%d\n",
                nextBeaconId, eventType, nextHop, st);

  // LOG frame keeps the parent (the beacon it was reached from).
  LogMsg lm;
  lm.b = m;
  lm.parentId = prevBeaconId;
  uint8_t logFrame[1 + sizeof(LogMsg)];
  logFrame[0] = PKT_LOG;
  memcpy(&logFrame[1], &lm, sizeof(LogMsg));
  radio.transmit(logFrame, sizeof(logFrame));

  prevBeaconId = nextBeaconId;
  nextBeaconId++;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  dropServo.attach(SERVO_PIN);
  dropServo.write(0);

  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("radio init failed: %d\n", st);
    while (true) delay(1000);
  }
  radio.setCRC(true);

  Serial.println("Writer ready. Press 'd' waypoint, 'v' victim, 'g' gas, 'b' blocked.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'd') dropBeacon(EVT_NONE);
    else if (c == 'v') dropBeacon(EVT_VICTIM);
    else if (c == 'g') dropBeacon(EVT_GAS);
    else if (c == 'b') dropBeacon(EVT_BLOCKED);
  }
}