#include "audio.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#include "hardware.h"
#include "driver/i2s.h"
#include "driver/rtc_io.h"
#include "esp_system.h"

void _audio_init(int i2s_num) {
    i2s_config_t i2s_config = {.mode                 = I2S_MODE_MASTER | I2S_MODE_TX,
                               .sample_rate          = 44100,
                               .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
                               .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
                               .communication_format = I2S_COMM_FORMAT_STAND_I2S,
                               .dma_buf_count        = 8,
                               .dma_buf_len          = 256,
                               .intr_alloc_flags     = 0,
                               .use_apll             = false,
                               .bits_per_chan        = I2S_BITS_PER_SAMPLE_16BIT};

    i2s_driver_install(i2s_num, &i2s_config, 0, NULL);

    i2s_pin_config_t pin_config = {.mck_io_num = -1, .bck_io_num = GPIO_I2S_BCLK, .ws_io_num = GPIO_I2S_WS, .data_out_num = GPIO_I2S_DATA, .data_in_num = I2S_PIN_NO_CHANGE};

    i2s_set_pin(i2s_num, &pin_config);
}

typedef struct _audio_player_cfg {
    uint8_t* buffer;
    size_t   size;
    bool     free_buffer;
} audio_player_cfg_t;

void audio_player_task(void* arg) {
    audio_player_cfg_t* config        = (audio_player_cfg_t*) arg;
    size_t              sample_length = config->size;
    uint8_t*            sample_buffer = config->buffer;

    size_t count;
    size_t position = 0;

    while (position < sample_length) {
        size_t length = sample_length - position;
        if (length > 256) length = 256;
        uint8_t buffer[256];
        memcpy(buffer, &sample_buffer[position], length);
        for (size_t l = 0; l < length; l += 2) {
            int16_t* sample = (int16_t*) &buffer[l];
            *sample *= 0.55;
        }
        i2s_write(0, buffer, length, &count, portMAX_DELAY);
        if (count != length) {
            printf("i2s_write_bytes: count (%d) != length (%d)\n", count, length);
            abort();
        }
        position += length;
    }

    i2s_zero_dma_buffer(0);  // Fill buffer with silence
    if (config->free_buffer) free(sample_buffer);
    vTaskDelete(NULL);  // Tell FreeRTOS that the task is done
}

void audio_init() { _audio_init(0); }

extern const uint8_t boot_snd_start[] asm("_binary_boot_pcm_start");
extern const uint8_t boot_snd_end[] asm("_binary_boot_pcm_end");

audio_player_cfg_t bootsound;

void play_bootsound() {
    TaskHandle_t handle;

    bootsound.buffer      = (uint8_t*) (boot_snd_start);
    bootsound.size        = boot_snd_end - boot_snd_start;
    bootsound.free_buffer = false;

    xTaskCreate(&audio_player_task, "Audio player", 4096, (void*) &bootsound, 10, &handle);
}
