#include "http_download.h"
#include "ntp_helper.h"
#include "pax_gfx.h"
#include "system_wrapper.h"

bool rename_or_replace(const char* TAG, const char* old, const char* new);
bool file_exists(const char* filename);
bool load_file(const char* TAG, const char* filename, char** buf, size_t* len);