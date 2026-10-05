#pragma once

// Copy this file to secrets.h and fill in your own values.
// secrets.h is gitignored; never commit real credentials.

#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASS     "YOUR_WIFI_PASSWORD"
#define MQTT_HOST     "192.168.1.10"
#define MQTT_PORT     1883
#define MQTT_TOPIC    "livingmap/beacons"

// Tunnel entrance anchor (used for local (x,y) -> lat/lon translation)
#define LAT0_DEG      36.8065
#define LON0_DEG      10.1815
// Compass bearing of the local +x axis, degrees clockwise from true north
#define HEADING_DEG   0.0