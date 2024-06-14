#pragma once

#include <stdbool.h>

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_wpa2.h"

// TROOPERS WiFi settings.
#define WIFI_TROOPERS_SSID     "trp-badge"
#define WIFI_TROOPERS_PASSWORD "TroopersBadge24"
#define WIFI_TROOPERS_AUTH     WIFI_AUTH_WPA2_PSK

bool wifi_set_defaults();
bool wifi_check_configured();
