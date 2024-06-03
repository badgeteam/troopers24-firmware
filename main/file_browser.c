#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <string.h>

#include "appfs.h"
#include "appfs_wrapper.h"
#include "bootscreen.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "hardware.h"
#include "ili9341.h"
#include "menu.h"
#include "pax_gfx.h"
#include "system_wrapper.h"

static const char* TAG = "file browser";

void list_files_in_folder(const char* path) {
    DIR* dir = opendir(path);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory %s", path);
        return;
    }

    struct dirent* ent;
    char           type;
    char           size[12];
    char           tpath[255];
    char           tbuffer[80];
    struct stat    sb;
    struct tm*     tm_info;
    char*          lpath = NULL;
    int            statok;

    uint64_t total  = 0;
    int      nfiles = 0;
    printf("T  Size      Date/Time         Name\n");
    printf("-----------------------------------\n");
    while ((ent = readdir(dir)) != NULL) {
        sprintf(tpath, path);
        if (path[strlen(path) - 1] != '/') {
            strcat(tpath, "/");
        }
        strcat(tpath, ent->d_name);
        tbuffer[0] = '\0';

        // Get file stat
        statok = stat(tpath, &sb);

        if (statok == 0) {
            tm_info = localtime(&sb.st_mtime);
            strftime(tbuffer, 80, "%d/%m/%Y %R", tm_info);
        } else {
            sprintf(tbuffer, "                ");
        }

        if (ent->d_type == DT_REG) {
            type = 'f';
            nfiles++;
            if (statok) {
                strcpy(size, "       ?");
            } else {
                total += sb.st_size;
                if (sb.st_size < (1024 * 1024))
                    sprintf(size, "%8d", (int) sb.st_size);
                else if ((sb.st_size / 1024) < (1024 * 1024))
                    sprintf(size, "%6dKB", (int) (sb.st_size / 1024));
                else
                    sprintf(size, "%6dMB", (int) (sb.st_size / (1024 * 1024)));
            }
        } else {
            type = 'd';
            strcpy(size, "       -");
        }

        printf("%c  %s  %s  %s\r\n", type, size, tbuffer, ent->d_name);
    }

    printf("-----------------------------------\n");
    if (total < (1024 * 1024))
        printf("   %8d", (int) total);
    else if ((total / 1024) < (1024 * 1024))
        printf("   %6dKB", (int) (total / 1024));
    else
        printf("   %6dMB", (int) (total / (1024 * 1024)));
    printf(" in %d file(s)\n", nfiles);
    printf("-----------------------------------\n");

    closedir(dir);
    free(lpath);
}

typedef struct _file_browser_menu_args {
    char type;
    char path[512];
    char label[512];
} file_browser_menu_args_t;

void find_parent_dir(char* path, char* parent) {
    size_t last_separator = 0;
    for (size_t index = 0; index < strlen(path); index++) {
        if (path[index] == '/') last_separator = index;
    }

    strcpy(parent, path);
    parent[last_separator] = '\0';
}

static bool is_esp32_binary(FILE* fd) {
    if (get_file_size(fd) < 1) return false;
    fseek(fd, 0, SEEK_SET);
    uint8_t magic_value = 0;
    fread(&magic_value, 1, 1, fd);
    fseek(fd, 0, SEEK_SET);
    return (magic_value == 0xE9);
}

static void file_browser_open_file(xQueueHandle button_queue, const char* filename, const char* label) {
    pax_buf_t*        pax_buffer = get_pax_buffer();
    const pax_font_t* font       = pax_font_saira_regular;
    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xFFFFFF);

    char* path = strdup(filename);
    if (path == NULL) return;
    for (ssize_t position = strlen(path) - 1; position >= 0; position--) {
        if (path[position] == '/') {
            path[position] = '\0';
            break;
        }
    }

    FILE* fd = fopen(filename, "rb");
    if (fd == NULL) {
        pax_draw_text(pax_buffer, 0xFFFF0000, font, 18, 0, 0, "Failed to open file\n\nPress A or B to go back");
        display_flush();
        ESP_LOGE(TAG, "Failed to open file");
        wait_for_button();
        free(path);
        return;
    }

    if (is_esp32_binary(fd)) {
        size_t file_size = get_file_size(fd);
        fclose(fd);
        size_t appfs_free = appfsGetFreeMem();
        if (file_size <= appfs_free) {
            pax_draw_text(pax_buffer, 0xFF000000, font, 18, 0, 0, "ESP32 application\n\nPress A to install\nPress B to go back");
            display_flush();
            if (wait_for_button()) {
                appfs_store_app(button_queue, filename, label, label, 0xFFFF);
            }
        } else {
            char buffer[128];
            snprintf(buffer, sizeof(buffer),
                     "ESP32 application\nSize: %u KB\nFree: %u KB\nNot enough free space\nplease free up space\n\nPress A or B to go back", file_size / 1024,
                     appfs_free / 1024);
            pax_draw_text(pax_buffer, 0xFFFF0000, font, 18, 0, 0, buffer);
            display_flush();
            wait_for_button();
        }
        free(path);
        return;
    } else if (false) {
        // TODO: Open mp3 in media player?
    } else {
        fclose(fd);
        pax_draw_text(pax_buffer, 0xFFFF0000, font, 18, 0, 0, "Unsupported file type\n\nPress A or B to go back");
        display_flush();
        ESP_LOGE(TAG, "Failed to open file");
        wait_for_button();
    }
    free(path);
    return;
}

static void file_browser_delete_file(file_browser_menu_args_t* menuArgs) {
    pax_buf_t*        pax_buffer = get_pax_buffer();
    const pax_font_t* font       = pax_font_saira_regular;
    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xFFFFFF);

    if (menuArgs->type == 'f') {
        remove(menuArgs->path);
    } else {
        pax_draw_text(pax_buffer, 0xFFFF0000, font, 18, 0, 0, "Currently cannot delete directory\n\nPress A or B to go back");
        display_flush();
        ESP_LOGE(TAG, "Currently cannot delete directory");
        wait_for_button();
    }
}

void file_browser(xQueueHandle button_queue, const char* initial_path) {
    pax_buf_t* pax_buffer = get_pax_buffer();
    display_boot_screen("Please wait...");
    char path[513] = {0};
    strncpy(path, initial_path, sizeof(path) - 1);
    while (true) {
        menu_t* menu = menu_alloc(path, 20, 18);
        DIR*    dir  = opendir(path);
        if (dir == NULL) {
            if (path[0] != 0) {
                ESP_LOGE(TAG, "Failed to open directory %s", path);
                display_boot_screen("Failed to open directory");
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            return;
        }
        struct dirent*            ent;
        file_browser_menu_args_t* pd_args = malloc(sizeof(file_browser_menu_args_t));
        pd_args->type                     = 'd';
        find_parent_dir(path, pd_args->path);
        menu_insert_item(menu, "../", NULL, pd_args, -1);

        while ((ent = readdir(dir)) != NULL) {
            file_browser_menu_args_t* args = malloc(sizeof(file_browser_menu_args_t));
            sprintf(args->path, path);
            if (path[strlen(path) - 1] != '/') {
                strcat(args->path, "/");
            }
            strcat(args->path, ent->d_name);

            if (ent->d_type == DT_REG) {
                args->type = 'f';
            } else {
                args->type = 'd';
            }

            snprintf(args->label, sizeof(args->label), "%s%s", ent->d_name, (args->type == 'd') ? "/" : "");
            menu_insert_item(menu, args->label, NULL, args, -1);
        }
        closedir(dir);

        bool                      render   = true;
        bool                      renderbg = true;
        bool                      exit     = false;
        file_browser_menu_args_t* menuArgs = NULL;
        bool delete = false;

        while (1) {
            keyboard_input_message_t buttonMessage = {0};
            if (xQueueReceive(button_queue, &buttonMessage, 16 / portTICK_PERIOD_MS) == pdTRUE) {
                uint8_t pin   = buttonMessage.input;
                bool    value = buttonMessage.state;
                switch (pin) {
                    case JOYSTICK_DOWN:
                        if (value) {
                            menu_navigate_next(menu);
                            render = true;
                        }
                        break;
                    case JOYSTICK_UP:
                        if (value) {
                            menu_navigate_previous(menu);
                            render = true;
                        }
                        break;
                    case BUTTON_BACK:
                        if (value) {
                            menuArgs = pd_args;
                        }
                        break;
                    case BUTTON_ACCEPT:
                        if (value) {
                            menuArgs = menu_get_callback_args(menu, menu_get_position(menu));
                        }
                        break;
                    case BUTTON_SELECT:
                    case JOYSTICK_PUSH:
                        if (value && menu_get_position(menu) > 0) {
                            menuArgs = menu_get_callback_args(menu, menu_get_position(menu));
                            delete = true;
                        }
                        break;
                    case BUTTON_START:
                        if (value) exit = true;
                        break;
                    default:
                        break;
                }
            }

            if (renderbg) {
                pax_background(pax_buffer, 0xFFFFFF);
                pax_noclip(pax_buffer);
                const pax_font_t* font = pax_font_saira_regular;
                pax_draw_text(pax_buffer, 0xFF000000, font, 18, 5, 240 - 19, "🅰 install  🅱 back 🅴 delete");
                renderbg = false;
            }

            if (render) {
                menu_render(pax_buffer, menu, 0, 0, 320, 220);
                display_flush();
                render = false;
            }

            if (menuArgs != NULL) {
                if (delete) {
                    ESP_LOGD(TAG, "File selected: %c -> %s\n", menuArgs->type, menuArgs->path);
                    file_browser_delete_file(menuArgs);
                    break;
                } else {
                    if (menuArgs->type == 'd') {
                        strcpy(path, menuArgs->path);
                        break;
                    } else {
                        printf("File selected: %s\n", menuArgs->path);
                        file_browser_open_file(button_queue, menuArgs->path, menuArgs->label);
                    }
                    menuArgs = NULL;
                    delete   = false;
                    render   = true;
                    renderbg = true;
                }
            }

            if (exit) {
                break;
            }
        }

        for (size_t index = 0; index < menu_get_length(menu); index++) {
            free(menu_get_callback_args(menu, index));
        }

        menu_free(menu);
    }
}
