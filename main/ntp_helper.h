#pragma once

#include <stdbool.h>
#include <stdlib.h>
#include "sys/time.h"
#include "sntp.h"
#include <esp_log.h>

extern bool ntp_synced;

bool sync_ntp();