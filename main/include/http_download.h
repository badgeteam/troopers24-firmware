#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

bool download_file(const char* url, const char* path);
bool download_file_retries(const char* url, const char* path, int retries);

bool download_ram(const char* url, uint8_t** ptr, size_t* size);
bool download_ram_retries(const char* url, uint8_t** ptr, size_t* size, int retries);
