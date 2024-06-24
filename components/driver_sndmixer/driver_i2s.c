#include "driver_i2s.h"

#include <esp_log.h>
#include <sdkconfig.h>

struct Config {
    uint8_t volume;
} config;

static QueueHandle_t soundQueue;
static int           soundRunning = 0;

void driver_i2s_sound_start(i2s_pin_config_t *pin_config) {
    config.volume = 128;

    i2s_config_t cfg = {.mode                 = I2S_MODE_MASTER | I2S_MODE_TX,
                        .sample_rate          = 44100,
                        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
                        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
                        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
                        .dma_buf_count        = 8,
                        .dma_buf_len          = 256,
                        .intr_alloc_flags     = 0,
                        .use_apll             = false,
                        .bits_per_chan        = I2S_BITS_PER_SAMPLE_16BIT};

    i2s_driver_install(0, &cfg, 4, &soundQueue);
    i2s_set_pin(0, pin_config);
    soundRunning = 1;
}

void driver_i2s_sound_stop() { i2s_driver_uninstall(0); }

#define SND_CHUNKSZ 32
void driver_i2s_sound_push(int16_t *buf, int len, int stereo_input) {
    int16_t tmpb[SND_CHUNKSZ * 2];
    int     i = 0;
    while (i < len) {
        int plen = len - i;
        if (plen > SND_CHUNKSZ) {
            plen = SND_CHUNKSZ;
        }
        for (int sample = 0; sample < plen; sample++) {
            int32_t s[2] = {0, 0};
            if (stereo_input) {
                s[0] = buf[(i + sample) * 2 + 0];
                s[1] = buf[(i + sample) * 2 + 1];
            } else {
                s[0] = s[1] = buf[i + sample];
            }

            // Multiply with volume/volume_max, resulting in signed integers with range [INT16_MIN:INT16_MAX]
//            s[0] = (s[0] * config.volume / 255);
//            s[1] = (s[1] * config.volume / 255);

//#ifdef CONFIG_DRIVER_SNDMIXER_I2S_DATA_FORMAT_UNSIGNED
            // Offset to [0:UINT16_MAX] store as unsigned integers
//            s[0] -= INT16_MIN;
//            s[1] -= INT16_MIN;
//#endif
            tmpb[(i + sample) * 2 + 0] = s[0];
            tmpb[(i + sample) * 2 + 1] = s[1];

        }
        size_t bytes_written;
        i2s_write(0, (char *) tmpb, plen * 2 * sizeof(tmpb[0]), &bytes_written, portMAX_DELAY);
        i += plen;
    }
}

void driver_i2s_set_volume(uint8_t new_volume) {
    // xSemaphoreTake(configMux, portMAX_DELAY);
    config.volume = new_volume;
    // xSemaphoreGive(configMux);
}

uint8_t driver_i2s_get_volume() { return config.volume; }

void driver_i2s_sound_mute(int doMute) {
    if (doMute) {
        dac_i2s_disable();
    } else {
        dac_i2s_enable();
    }
}
