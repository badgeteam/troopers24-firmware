#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

void menu_nfcreader(xQueueHandle button_queue);
