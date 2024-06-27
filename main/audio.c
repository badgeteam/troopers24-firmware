#include "audio.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#include "hardware.h"
#include "driver/i2s.h"
#include "driver/rtc_io.h"
#include "esp_system.h"
#include "esp_log.h"

static const char* TAG = "audio";

static xSemaphoreHandle audio_mutex;
bool audio_playing = false;

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
    audio_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(audio_mutex);
}

typedef struct _audio_player_cfg {
    uint8_t* buffer;
    size_t   current;
    size_t   size;
} audio_player_cfg_t;


extern const uint8_t boot_mp3_start[] asm("_binary_boot_mp3_start");
extern const uint8_t boot_mp3_end[] asm("_binary_boot_mp3_end");

//extern const uint8_t happy_snd_start[] asm("_binary_happy_pcm_start");
//extern const uint8_t happy_snd_end[] asm("_binary_happy_pcm_end");

extern const uint8_t happy_mp3_start[] asm("_binary_happy_mp3_start");
extern const uint8_t happy_mp3_end[] asm("_binary_happy_mp3_end");

static ssize_t play_from_resource(void* config, void* buf, size_t len) {
    audio_player_cfg_t* cfg = (audio_player_cfg_t*) config;
    size_t remaining = cfg->size - cfg->current;
    size_t read = remaining < len ? remaining : len;
    memcpy(buf, cfg->buffer + cfg->current, read);
    cfg->current += read;
    return (ssize_t) read;
}

static ssize_t seek_from_resource(void* config, size_t pos, size_t unknown) {
    audio_player_cfg_t* cfg = (audio_player_cfg_t*) config;
    cfg->current = pos;
    return 0;
}

static ssize_t audio_ended(void* handle, size_t a, size_t b) {
    audio_playing = false;
    return 0;
}

int play_from_resources(audio_player_cfg_t* cfg, const uint8_t* start, const uint8_t* end) {
    cfg->buffer    = (uint8_t*) (start);
    cfg->size      = end - start;
    cfg->current   = 0;

    int id = sndmixer_queue_mp3_stream(play_from_resource, seek_from_resource, (void*) cfg);
    sndmixer_set_volume(id, 128);
    sndmixer_play(id);
    return id;
}

audio_player_cfg_t cfg_boot;
audio_player_cfg_t cfg_happy;

void play_bootsound() {
    play_from_resources(&cfg_boot, boot_mp3_start, boot_mp3_end);
}

void play_happy_birthday(bool connected) {
    if (!connected || audio_playing) {
        return;
    }
    audio_playing = true;
    int id = play_from_resources(&cfg_happy, happy_mp3_start, happy_mp3_end);
    sndmixer_set_callback(id, audio_ended, NULL);
}

void audio_init() {
//    _audio_init(0);
    i2s_pin_config_t pin_config = {
        .mck_io_num = -1,
        .bck_io_num = GPIO_I2S_BCLK,
        .ws_io_num = GPIO_I2S_WS,
        .data_out_num = GPIO_I2S_DATA,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    sndmixer_init(1, false, &pin_config);
    set_sao_callback_tr24(&play_happy_birthday);
}