#include "utils.h"

bool rename_or_replace(const char* TAG, const char* old, const char* new) {
    if (access(new, F_OK) == 0) {
        ESP_LOGD(TAG, "Destination file exists, deleting...");
        // File exists, try to delete
        if (remove(new) != 0) {
            ESP_LOGD(TAG, "Destination file could not be deleted");
            // Deleting failed
            return false;
        }
    }

    return rename(old, new) != 0;
}

bool file_exists(const char* filename) {
    return access(filename, F_OK) == 0;
}

bool load_file(const char* TAG, const char* filename, char** buf, size_t* len) {
    FILE* fd = fopen(filename, "r");
    if (fd == NULL) {
        ESP_LOGE(TAG, "Unable to open file: %s", filename);
        return false;
    }

    /* Go to the end of the file. */
    if (fseek(fd, 0L, SEEK_END) == 0) {
        /* Get the size of the file. */
        *len = ftell(fd);

        if (*buf != NULL) {
            free(*buf);
        }

        /* Allocate our buffer to that size. */
        *buf = malloc(*len);

        /* Go back to the start of the file. */
        if (fseek(fd, 0L, SEEK_SET) != 0) {
            free(*buf);
            *len = 0;
            ESP_LOGE(TAG, "Failed to seek to start");
            return false;
        }

        /* Read the entire file into memory. */
        fread(*buf, 1, *len, fd);
        int err = ferror(fd);
        if (err != 0) {
            free(*buf);
            *len = 0;
            ESP_LOGE(TAG, "Failed to read file: %d", err);
            return false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to seek to end");
        return false;
    }
    fclose(fd);
    return true;
}