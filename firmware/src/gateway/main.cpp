#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <RadioLib.h>
#include "beacon_msg.h"
#include "secrets.h"

#define LORA_CS   5
#define LORA_DIO0 26
#define LORA_RST  14
#define LORA_DIO1 33

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// HEADING_DEG = compass bearing of the local +x axis, clockwise from true north.
// Same maths as livingmap/frames.py (tested): E = x sin(psi) - y cos(psi), N = x cos(psi) + y sin(psi).
static void localToLatLon(double x_m, double y_m,
                          double& lat, double& lon) {
  double psi = HEADING_DEG * PI / 180.0;
  double east  = x_m * sin(psi) - y_m * cos(psi);
  double north = x_m * cos(psi) + y_m * sin(psi);
  double phi = LAT0_DEG * PI / 180.0;
  double mLat = 111132.92 - 559.82 * cos(2 * phi) + 1.175 * cos(4 * phi);
  double mLon = 111412.84 * cos(phi) - 93.5 * cos(3 * phi);
  lat = LAT0_DEG + north / mLat;
  lon = LON0_DEG + east  / mLon;
}

static void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " ok" : " FAILED");
}

static void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  while (!mqtt.connected()) {
    Serial.print("MQTT...");
    if (mqtt.connect("lora-gateway")) Serial.println("ok");
    else { Serial.println("retry"); delay(2000); }
  }
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

  connectWiFi();
  connectMQTT();
}

void loop() {
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  uint8_t buf[255];
  int st = radio.receive(buf, sizeof(buf));
  if (st != RADIOLIB_ERR_NONE) return;

  size_t len = radio.getPacketLength();
  if (len < 1) return;
  uint8_t type = buf[0];

  char json[320];

  if (type == PKT_BEACON && len == 1 + sizeof(BeaconMsg)) {
    BeaconMsg m;
    memcpy(&m, &buf[1], sizeof(m));
    double lat, lon;
    localToLatLon(m.x_cm / 100.0, m.y_cm / 100.0, lat, lon);
    snprintf(json, sizeof(json),
      "{\"type\":\"beacon\",\"id\":%u,\"evt\":%u,\"next\":%u,"
      "\"x_cm\":%d,\"y_cm\":%d,\"brg\":%d,\"age\":%lu,"
      "\"lat\":%.6f,\"lon\":%.6f,\"rssi\":%.1f,\"snr\":%.1f}",
      m.beaconId, m.eventType, m.nextHopId, m.x_cm, m.y_cm, m.bearingDeg,
      (unsigned long)m.ageAtWriteS, lat, lon,
      radio.getRSSI(), radio.getSNR());
    mqtt.publish(MQTT_TOPIC, json);
    Serial.println(json);
  }
  else if (type == PKT_LOG && len == 1 + sizeof(LogMsg)) {
    LogMsg l;
    memcpy(&l, &buf[1], sizeof(l));
    double lat, lon;
    localToLatLon(l.b.x_cm / 100.0, l.b.y_cm / 100.0, lat, lon);
    snprintf(json, sizeof(json),
      "{\"type\":\"log\",\"id\":%u,\"evt\":%u,\"parent\":%u,"
      "\"x_cm\":%d,\"y_cm\":%d,\"age\":%lu,\"lat\":%.6f,\"lon\":%.6f}",
      l.b.beaconId, l.b.eventType, l.parentId,
      l.b.x_cm, l.b.y_cm, (unsigned long)l.b.ageAtWriteS, lat, lon);
    mqtt.publish(MQTT_TOPIC, json);
    Serial.println(json);
  }
  else if (type == PKT_MISSION && len >= 3) {
    Serial.printf("MISSION received on LoRa (unexpected) len=%u\n",
                  (unsigned)len);
  }
}