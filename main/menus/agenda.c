#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "app_management.h"
#include "graphics_wrapper.h"
#include "hardware.h"
#include "http_download.h"
#include "ntp_helper.h"
#include "menu.h"
#include "pax_codecs.h"
#include "pax_gfx.h"
#include "system_wrapper.h"
#include "wifi_connect.h"
#include "utils.h"

static const char* TAG = "agenda";

static const char* BOOKMARKED = "bookmarked";

#define DEBUG_INFRA 0

static const char* last_update_path = "/internal/apps/agenda/last_update";
static const char* day1_path        = "/internal/apps/agenda/0.json";
static const char* day1_path_tmp    = "/internal/apps/agenda/0.json.tmp";
static const char* day2_path        = "/internal/apps/agenda/1.json";
static const char* day2_path_tmp    = "/internal/apps/agenda/1.json.tmp";
static const char* my_agenda_path   = "/internal/apps/agenda/my.json";

#if DEBUG_INFRA
static const char* last_update_url  = "http://con.troopers.de/agenda/last_update";
static const char* day1_url         = "http://con.troopers.de/agenda/0.json";
static const char* day2_url         = "http://con.troopers.de/agenda/1.json";
#else
static const char* last_update_url  = "https://con.troopers.de/agenda/last_update";
static const char* day1_url         = "https://con.troopers.de/agenda/0.json";
static const char* day2_url         = "https://con.troopers.de/agenda/1.json";
#endif

static const char* DEFAULT_MY = "{\"day1\": [], \"day2\": []}";
static const char* DEFAULT_DAY1 = "{\"tracks\": [{\"title\": \"Attack & Research\", \"talks\": [{\"id\": 1141, \"title\": \"Keynote\", \"time\": \"09:00\", \"start\": 32400, \"end\": \"10:00\", \"duration\": 3600, \"special\": false, \"ts\": 1687942800, \"speakers\": [\"Mikko Hypp\\u00f6nen\"]}, {\"id\": 1126, \"title\": \"Coffee Break\", \"time\": \"10:00\", \"start\": 36000, \"end\": \"10:30\", \"duration\": 1800, \"special\": true, \"ts\": 1687946400, \"speakers\": []}, {\"id\": 1206, \"title\": \"OopsSec - The bad, the worst and the ugly  of APT\\u2019s operations security\", \"time\": \"10:30\", \"start\": 37800, \"end\": \"11:30\", \"duration\": 3600, \"special\": false, \"ts\": 1687948200, \"speakers\": [\"Tomer Bar\"]}, {\"id\": 1146, \"title\": \"Spooky authentication at a distance\", \"time\": \"11:30\", \"start\": 41400, \"end\": \"12:30\", \"duration\": 3600, \"special\": false, \"ts\": 1687951800, \"speakers\": [\"Tamas Jos\"]}, {\"id\": 1136, \"title\": \"Lunch Break\", \"time\": \"12:30\", \"start\": 45000, \"end\": \"13:00\", \"duration\": 1800, \"special\": true, \"ts\": 1687955400, \"speakers\": []}, {\"id\": 1283, \"title\": \"Attacking Ultra-Wideband: Security Analysis of UWB Applications in Smartphones\", \"time\": \"13:45\", \"start\": 49500, \"end\": \"14:45\", \"duration\": 3600, \"special\": false, \"ts\": 1687959900, \"speakers\": [\"Alexander Heinrich\", \"Jiska Classen\"]}, {\"id\": 1181, \"title\": \"Fact Based Post Exploitation - Office365 Edition\", \"time\": \"14:45\", \"start\": 53100, \"end\": \"15:45\", \"duration\": 3600, \"special\": false, \"ts\": 1687963500, \"speakers\": [\"Melvin Langvik\"]}, {\"id\": 1125, \"title\": \"Coffee Break\", \"time\": \"15:45\", \"start\": 56700, \"end\": \"16:15\", \"duration\": 1800, \"special\": true, \"ts\": 1687967100, \"speakers\": []}, {\"id\": 1288, \"title\": \"Forensic Examination of Ceph\", \"time\": \"16:15\", \"start\": 58500, \"end\": \"17:15\", \"duration\": 3600, \"special\": false, \"ts\": 1687968900, \"speakers\": [\"Florian Bausch\"]}, {\"id\": 1290, \"title\": \"All your parcel are belong to us\", \"time\": \"17:15\", \"start\": 62100, \"end\": \"17:45\", \"duration\": 1800, \"special\": false, \"ts\": 1687972500, \"speakers\": [\"Dennis Kniel\", \"Klaus Kiehne\"]}, {\"id\": 1211, \"title\": \"Stay fit: Hack a Jump Rope\", \"time\": \"17:45\", \"start\": 63900, \"end\": \"18:15\", \"duration\": 1800, \"special\": false, \"ts\": 1687974300, \"speakers\": [\"Axelle Apvrille\"]}]}, {\"title\": \"Defense & Management\", \"talks\": [{\"id\": 1129, \"title\": \"Coffee Break\", \"time\": \"10:00\", \"start\": 36000, \"end\": \"10:30\", \"duration\": 1800, \"special\": true, \"ts\": 1687946400, \"speakers\": []}, {\"id\": 1249, \"title\": \"Das IT-Security-Lagebild aus Heise-Sicht\", \"time\": \"10:30\", \"start\": 37800, \"end\": \"11:30\", \"duration\": 3600, \"special\": false, \"ts\": 1687948200, \"speakers\": [\"J\\u00fcrgen Schmidt aka ju\"]}, {\"id\": 1173, \"title\": \"Cat & Mouse - or chess?\", \"time\": \"11:30\", \"start\": 41400, \"end\": \"12:30\", \"duration\": 3600, \"special\": false, \"ts\": 1687951800, \"speakers\": [\"Fabian Mosch\"]}, {\"id\": 1137, \"title\": \"Lunch Break\", \"time\": \"12:30\", \"start\": 45000, \"end\": \"13:30\", \"duration\": 3600, \"special\": true, \"ts\": 1687955400, \"speakers\": []}, {\"id\": 1197, \"title\": \"Real world detection engineering in a multi-cloud environment\", \"time\": \"13:45\", \"start\": 49500, \"end\": \"14:45\", \"duration\": 3600, \"special\": false, \"ts\": 1687959900, \"speakers\": [\"Aaron Jewitt\"]}, {\"id\": 1246, \"title\": \"Security Heroes versus the Power of Privacy\", \"time\": \"14:45\", \"start\": 53100, \"end\": \"15:45\", \"duration\": 3600, \"special\": false, \"ts\": 1687963500, \"speakers\": [\"Avi D\", \"Kim Wuyts\"]}, {\"id\": 1128, \"title\": \"Coffee Break\", \"time\": \"15:45\", \"start\": 56700, \"end\": \"16:15\", \"duration\": 1800, \"special\": true, \"ts\": 1687967100, \"speakers\": []}, {\"id\": 1236, \"title\": \"SAP (Anti-)Forensics: Detecting White-Collar Cyber-Crime\", \"time\": \"16:15\", \"start\": 58500, \"end\": \"17:15\", \"duration\": 3600, \"special\": false, \"ts\": 1687968900, \"speakers\": [\"Yvan Genuer\"]}, {\"id\": 1171, \"title\": \"Jupysec: Auditing Jupyter to Improve AI Security\", \"time\": \"17:15\", \"start\": 62100, \"end\": \"18:15\", \"duration\": 3600, \"special\": false, \"ts\": 1687972500, \"speakers\": [\"Joe Lucas\"]}]}, {\"title\": \"Active Directory & Azure Security\", \"talks\": [{\"id\": 1132, \"title\": \"Coffee Break\", \"time\": \"10:00\", \"start\": 36000, \"end\": \"10:30\", \"duration\": 1800, \"special\": true, \"ts\": 1687946400, \"speakers\": []}, {\"id\": 1278, \"title\": \"Dumping NTHashes from Azure AD\", \"time\": \"10:30\", \"start\": 37800, \"end\": \"11:30\", \"duration\": 3600, \"special\": false, \"ts\": 1687948200, \"speakers\": [\"Nestori Syynimaa\"]}, {\"id\": 1213, \"title\": \"Hidden Pathways: Exploring the Anatomy of ACL-Based Active Directory Attacks and Building Strong Defenses\", \"time\": \"11:30\", \"start\": 41400, \"end\": \"12:30\", \"duration\": 3600, \"special\": false, \"ts\": 1687951800, \"speakers\": [\"Jonas B\\u00fclow Knudsen\", \"Alexander Schmitt\"]}, {\"id\": 1140, \"title\": \"Lunch Break\", \"time\": \"12:30\", \"start\": 45000, \"end\": \"13:30\", \"duration\": 3600, \"special\": true, \"ts\": 1687955400, \"speakers\": []}, {\"id\": 1224, \"title\": \"Priority for Effective Action - A Practical Model for quantifying the Risk of Active Directory Attacks\", \"time\": \"13:45\", \"start\": 49500, \"end\": \"14:45\", \"duration\": 3600, \"special\": false, \"ts\": 1687959900, \"speakers\": [\"Mars Cheng\", \"Dexter Chen\"]}, {\"id\": 1210, \"title\": \"(Windows) Hello from the other side\", \"time\": \"14:45\", \"start\": 53100, \"end\": \"15:45\", \"duration\": 3600, \"special\": false, \"ts\": 1687963500, \"speakers\": [\"Dirk-jan Mollema\"]}, {\"id\": 1134, \"title\": \"Coffee Break\", \"time\": \"15:45\", \"start\": 56700, \"end\": \"16:15\", \"duration\": 1800, \"special\": true, \"ts\": 1687967100, \"speakers\": []}, {\"id\": 1229, \"title\": \"The Power of Coercion Techniques in Windows Environments\", \"time\": \"16:15\", \"start\": 58500, \"end\": \"17:15\", \"duration\": 3600, \"special\": false, \"ts\": 1687968900, \"speakers\": [\"Martin Grottenthaler\"]}, {\"id\": 1239, \"title\": \"So You Performed A Forest Recovery. How Do You Reconnect Your AD Again With Azure AD?\", \"time\": \"17:15\", \"start\": 62100, \"end\": \"18:15\", \"duration\": 3600, \"special\": false, \"ts\": 1687972500, \"speakers\": [\"Jorge de Almeida Pinto\"]}]}]}";
static const char* DEFAULT_DAY2 = "{\"tracks\": [{\"title\": \"Attack & Research\", \"talks\": [{\"id\": 1281, \"title\": \"Testing and Fuzzing the Kubernetes Admission Configuration\", \"time\": \"10:30\", \"start\": 37800, \"end\": \"11:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688034600, \"speakers\": [\"Benjamin Koltermann\", \"Maximilian Rademacher\"]}, {\"id\": 1299, \"title\": \"Surprise Talk\", \"time\": \"11:30\", \"start\": 41400, \"end\": \"12:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688038200, \"speakers\": []}, {\"id\": 1135, \"title\": \"Lunch Break\", \"time\": \"12:30\", \"start\": 45000, \"end\": \"13:30\", \"duration\": 3600, \"special\": true, \"ts\": 1688041800, \"speakers\": []}, {\"id\": 1196, \"title\": \"Monitoring Solutions: Attacking IT Infrastructure at its Core\", \"time\": \"13:30\", \"start\": 48600, \"end\": \"14:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688045400, \"speakers\": [\"Stefan Schiller\"]}, {\"id\": 1297, \"title\": \"The Anatomy of Windows Telemetry Part 2\", \"time\": \"14:30\", \"start\": 52200, \"end\": \"15:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688049000, \"speakers\": [\"Tillmann O\\u00dfwald\", \"Dominik Phillips\", \"Maximilian Winkler\"]}, {\"id\": 1124, \"title\": \"Coffee Break\", \"time\": \"15:30\", \"start\": 55800, \"end\": \"16:00\", \"duration\": 1800, \"special\": true, \"ts\": 1688052600, \"speakers\": []}, {\"id\": 1243, \"title\": \"Everyone knows SAP, everyone uses SAP, everyone uses RFC, no one knows RFC: From RFC to RCE 16 years later\", \"time\": \"16:00\", \"start\": 57600, \"end\": \"17:00\", \"duration\": 3600, \"special\": false, \"ts\": 1688054400, \"speakers\": [\"Fabian Hagg\"]}, {\"id\": 1298, \"title\": \"Closing\", \"time\": \"17:00\", \"start\": 61200, \"end\": \"18:00\", \"duration\": 3600, \"special\": false, \"ts\": 1688058000, \"speakers\": []}]}, {\"title\": \"Defense & Management\", \"talks\": [{\"id\": 1225, \"title\": \"OAuth and Proof of Possession - The long way round\", \"time\": \"10:30\", \"start\": 37800, \"end\": \"11:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688034600, \"speakers\": [\"Dominick Baier\"]}, {\"id\": 1273, \"title\": \"Detection And Blocking With BPF Via YAML\", \"time\": \"11:30\", \"start\": 41400, \"end\": \"12:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688038200, \"speakers\": [\"Kev Sheldrake\"]}, {\"id\": 1138, \"title\": \"Lunch Break\", \"time\": \"12:30\", \"start\": 45000, \"end\": \"13:30\", \"duration\": 3600, \"special\": true, \"ts\": 1688041800, \"speakers\": []}, {\"id\": 1212, \"title\": \"GPT-like Pre-Training on Unlabeled System Logs for Malware Detection\", \"time\": \"13:30\", \"start\": 48600, \"end\": \"14:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688045400, \"speakers\": [\"Dmitrijs Trizna\", \"Luca Demetrio\"]}, {\"id\": 1272, \"title\": \"Forensic analysis on real incidents inside Microsoft Remote Desktop Services\", \"time\": \"14:30\", \"start\": 52200, \"end\": \"15:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688049000, \"speakers\": [\"Catarina de Faria Cristas\"]}, {\"id\": 1127, \"title\": \"Coffee Break\", \"time\": \"15:30\", \"start\": 55800, \"end\": \"16:00\", \"duration\": 1800, \"special\": true, \"ts\": 1688052600, \"speakers\": []}, {\"id\": 1147, \"title\": \"Reportly - keep your head in the clouds. A new Azure visualization tool for analyzing user activities.\", \"time\": \"16:00\", \"start\": 57600, \"end\": \"16:30\", \"duration\": 1800, \"special\": false, \"ts\": 1688054400, \"speakers\": [\"Sapir Federovsky\"]}, {\"id\": 1254, \"title\": \"Homophonic Collisions: Hold me closer Tony Danza\", \"time\": \"16:30\", \"start\": 59400, \"end\": \"17:00\", \"duration\": 1800, \"special\": false, \"ts\": 1688056200, \"speakers\": [\"Justin Ibarra\", \"Reagan Short\"]}]}, {\"title\": \"Track 3\", \"talks\": [{\"id\": 1223, \"title\": \"Horror Stories from the Automotive Industry\", \"time\": \"10:30\", \"start\": 37800, \"end\": \"11:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688034600, \"speakers\": [\"Thomas Sermpinis\"]}, {\"id\": 1280, \"title\": \"Beyond Java: Obfuscating Android Apps with Purely Native Code\", \"time\": \"11:30\", \"start\": 41400, \"end\": \"12:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688038200, \"speakers\": [\"Laurie Kirk\"]}, {\"id\": 1139, \"title\": \"Lunch Break\", \"time\": \"12:30\", \"start\": 45000, \"end\": \"13:30\", \"duration\": 3600, \"special\": true, \"ts\": 1688041800, \"speakers\": []}, {\"id\": 1217, \"title\": \"Fault Injection Attacks on Secure Automotive Bootloaders\", \"time\": \"13:30\", \"start\": 48600, \"end\": \"14:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688045400, \"speakers\": [\"Nils Weiss\", \"Enrico Pozzobon\"]}, {\"id\": 1279, \"title\": \"Vulnerabilities in the TPM 2.0 reference implementation code\", \"time\": \"14:30\", \"start\": 52200, \"end\": \"15:30\", \"duration\": 3600, \"special\": false, \"ts\": 1688049000, \"speakers\": [\"Francisco Falcon\"]}, {\"id\": 1131, \"title\": \"Coffee Break\", \"time\": \"15:30\", \"start\": 55800, \"end\": \"16:00\", \"duration\": 1800, \"special\": true, \"ts\": 1688052600, \"speakers\": []}, {\"id\": 1248, \"title\": \"The Wire on Fire: The Spies Who Loved Telcos\", \"time\": \"16:00\", \"start\": 57600, \"end\": \"17:00\", \"duration\": 3600, \"special\": false, \"ts\": 1688054400, \"speakers\": [\"Aleksandar Milenkoski\"]}, {\"id\": 1300, \"title\": \"TR23 Badge - A RETROspective\", \"time\": \"16:30\", \"start\": 59400, \"end\": \"17:00\", \"duration\": 1800, \"special\": false, \"ts\": 1688056200, \"speakers\": [\"Malte Heinzelmann\", \"Jeff \\\"jeffmakes\\\" Gough\"]}]}]}";
static const char* DEFAULT_LAST_UPDATE = "1687853280";


extern const uint8_t agenda_png_start[] asm("_binary_calendar_png_start");
extern const uint8_t agenda_png_end[] asm("_binary_calendar_png_end");

extern const uint8_t clock_png_start[] asm("_binary_clock_png_start");
extern const uint8_t clock_png_end[] asm("_binary_clock_png_end");

extern const uint8_t bookmark_png_start[] asm("_binary_bookmark_png_start");
extern const uint8_t bookmark_png_end[] asm("_binary_bookmark_png_end");

typedef enum action {
    ACTION_NONE,
    ACTION_NEXT_UP,
    ACTION_MY_AGENDA,
    ACTION_WEDNESDAY,
    ACTION_THURSDAY
} menu_agenda_action_t;


static char*  data_day1 = NULL;
static size_t size_day1 = 0;
static cJSON* json_day1 = NULL;

static char*  data_day2 = NULL;
static size_t size_day2 = 0;
static cJSON* json_day2 = NULL;

static char*  data_my = NULL;
static size_t size_my = 0;
static cJSON* json_my = NULL;
static cJSON* json_my_day1 = NULL;
static cJSON* json_my_day2 = NULL;

static inline void do_init() {
    FILE* last_update_fd = fopen(last_update_path, "r");
    if (last_update_fd != NULL) {
        return;
    }
    // No last update -> init default data

    FILE* day1_fd = fopen(day1_path, "w");
    if (day1_fd == NULL) {
        ESP_LOGE(TAG, "Cannot init agenda data for day 1.");
        return;
    }
    fwrite(DEFAULT_DAY1, 1, strlen(DEFAULT_DAY1), day1_fd);
    fclose(day1_fd);

    FILE* day2_fd = fopen(day2_path, "w");
    if (day2_fd == NULL) {
        ESP_LOGE(TAG, "Cannot init agenda data for day 2.");
        return;
    }
    fwrite(DEFAULT_DAY2, 1, strlen(DEFAULT_DAY2), day2_fd);
    fclose(day2_fd);

    FILE* new_last_update_fd = fopen(last_update_path, "w");
    if (new_last_update_fd == NULL) {
        ESP_LOGE(TAG, "Cannot init agenda last_updated.");
        return;
    }
    fwrite(DEFAULT_LAST_UPDATE, 1, strlen(DEFAULT_LAST_UPDATE), new_last_update_fd);
    fclose(new_last_update_fd);

    FILE* my_fd = fopen(my_agenda_path, "w");
    if (my_fd == NULL) {
        ESP_LOGE(TAG, "Cannot init agenda last_updated.");
        return;
    }
    fwrite(DEFAULT_MY, 1, strlen(DEFAULT_MY), my_fd);
    fclose(my_fd);
    ESP_LOGI(TAG, "Successfully written initial agenda data");
}

static void agenda_render_background(pax_buf_t* pax_buffer) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xFF1E1E1E);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅰 Accept 🅱 Exit");
}

static void details_render_background(pax_buf_t* pax_buffer, bool tracks, bool days) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xFF1E1E1E);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    if (tracks) {
        pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 Back  ← → Track 🅴 Bookmark");
    } else if (days) {
        pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 Back  ← → Day 🅴 Bookmark");
    } else {
        pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 Back 🅴 Bookmark");
    }
}

static void render_topbar(pax_buf_t* pax_buffer, pax_buf_t* icon, const char* text) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_simple_rect(pax_buffer, 0xff131313, 0, 0, 320, 34);
    pax_draw_image(pax_buffer, icon, 1, 1);
    pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 34, 8, text);
}

static int render_track(pax_buf_t* pax_buffer, pax_buf_t* icon_top, pax_buf_t* icon_bookmarked, cJSON* tracks, int track, int talk, int render_talks, int slot_height) {
    const pax_font_t* font = pax_font_saira_regular;

    cJSON* track_data = cJSON_GetArrayItem(tracks, track);
    cJSON* title = cJSON_GetObjectItem(track_data, "title");
    cJSON* talks = cJSON_GetObjectItem(track_data, "talks");

    cJSON* talk_data;
    cJSON* time;
    cJSON* talk_title;
    cJSON* speakers;
    cJSON* speaker;

    int talk_count = cJSON_GetArraySize(talks);
    if (talk >= talk_count) {
        talk = talk_count - 1;
    }

    details_render_background(pax_buffer, true, false);
    render_topbar(pax_buffer, icon_top, title->valuestring);

    // First talk
    int start_talk = talk;
    int end_talk = talk + render_talks - 1;
    if (end_talk >= talk_count) {
        start_talk -= end_talk - talk_count + 1;
        end_talk = talk_count - 1;
    }
    if (start_talk < 0) {
        start_talk = 0;
    }

    for (int current = start_talk; current <= end_talk; current++) {
        int i = current - start_talk;
        int y = i * slot_height + 34;
        pax_col_t bg_color = (current == talk) ? 0xFFF1AA13 : 0xFF131313;
        pax_col_t fg_color = (current == talk) ? 0xFF131313 : 0xFFF1AA13;
        pax_simple_rect(pax_buffer, bg_color, 0, y, 320, slot_height);

        talk_data = cJSON_GetArrayItem(talks, current);
        time = cJSON_GetObjectItem(talk_data, "time");
        talk_title = cJSON_GetObjectItem(talk_data, "title");
        speakers = cJSON_GetObjectItem(talk_data, "speakers");

        pax_draw_text(pax_buffer, fg_color, font, 14, 5, y + 2, time->valuestring);
        if (cJSON_IsTrue(cJSON_GetObjectItem(talk_data, BOOKMARKED))) {
            pax_draw_image_sized(pax_buffer, icon_bookmarked, 302, y + 1, 16, 16);
        }

        pax_draw_text(pax_buffer, fg_color, font, 18, 5, y + 2 + 18, talk_title->valuestring);

        int max_len = 255;
        char speaker_str[max_len];
        speaker_str[max_len-1] = 0;
        uint offset = 1;
        for (int a = 0; a < cJSON_GetArraySize(speakers); a++) {
            speaker = cJSON_GetArrayItem(speakers, a);
            uint len = strlen(speaker->valuestring);
            strncat(speaker_str, speaker->valuestring, max_len - 1 - offset);
            offset += len;
            if (a < cJSON_GetArraySize(speakers) - 1) {
                strncat(speaker_str, ", ", max_len - 1 - offset);
                offset += 2;
            }
            if (offset >= max_len - 2) {
                break;
            }
        }

        pax_draw_text(pax_buffer, fg_color, font, 14, 15, y + 2 + 18 + 20, speaker_str);
    }

    display_flush();
    return talk_count;
}

static int render_bookmarks(pax_buf_t* pax_buffer, pax_buf_t* icon, cJSON* bookmarks, int day, int talk, int render_talks, int slot_height) {
    const pax_font_t* font = pax_font_saira_regular;

    int talk_count = cJSON_GetArraySize(bookmarks);
    if (talk >= talk_count) {
        talk = talk_count - 1;
    }

    details_render_background(pax_buffer, false, true);
    if (day == 0) {
        render_topbar(pax_buffer, icon, "Bookmarks - Wednesday");
    } else {
        render_topbar(pax_buffer, icon, "Bookmarks - Thursday");
    }

    // First talk
    int start_talk = talk;
    int end_talk = talk + render_talks - 1;
    if (end_talk >= talk_count) {
        start_talk -= end_talk - talk_count + 1;
        end_talk = talk_count - 1;
    }
    if (start_talk < 0) {
        start_talk = 0;
    }

    cJSON* bookmark;
    cJSON* track_data;
    cJSON* talk_data;
    cJSON* time;
    cJSON* track;
    cJSON* talk_title;
    cJSON* speakers;

    for (int current = start_talk; current <= end_talk; current++) {
        int i = current - start_talk;
        int y = i * slot_height + 34;
        pax_col_t bg_color = (current == talk) ? 0xFFF1AA13 : 0xFF131313;
        pax_col_t fg_color = (current == talk) ? 0xFF131313 : 0xFFF1AA13;
        pax_simple_rect(pax_buffer, bg_color, 0, y, 320, slot_height);

        bookmark = cJSON_GetArrayItem(bookmarks, current);
        track_data = cJSON_GetObjectItem(bookmark, "track");
        talk_data = cJSON_GetObjectItem(bookmark, "talk");

        track = cJSON_GetObjectItem(track_data, "title");
        time = cJSON_GetObjectItem(talk_data, "time");
        talk_title = cJSON_GetObjectItem(talk_data, "title");
        speakers = cJSON_GetObjectItem(talk_data, "speakers");

        uint len_time = strlen(time->valuestring);
        uint len_track = strlen(track->valuestring);
        char* time_title = malloc(len_time + len_track + 2);
        memcpy(time_title, time->valuestring, len_time);
        time_title[len_time] = ' ';
        memcpy(time_title + len_time + 1, track->valuestring, len_track);
        time_title[len_time + len_track + 1] = 0;
        pax_draw_text(pax_buffer, fg_color, font, 14, 5, y + 2, time_title);
        free(time_title);

        pax_draw_text(pax_buffer, fg_color, font, 18, 5, y + 2 + 18, talk_title->valuestring);

        int max_len = 255;
        char speaker_str[max_len];
        speaker_str[max_len-1] = 0;
        uint offset = 1;
        for (int a = 0; a < cJSON_GetArraySize(speakers); a++) {
            cJSON* speaker = cJSON_GetArrayItem(speakers, a);
            uint len = strlen(speaker->valuestring);
            strncat(speaker_str, speaker->valuestring, max_len - 1 - offset);
            offset += len;
            if (a < cJSON_GetArraySize(speakers) - 1) {
                strncat(speaker_str, ", ", max_len - 1 - offset);
                offset += 2;
            }
            if (offset >= max_len - 2) {
                break;
            }
        }

        pax_draw_text(pax_buffer, fg_color, font, 14, 15, y + 2 + 18 + 20, speaker_str);
    }

    display_flush();
    return talk_count;
}

static bool save_bookmarks() {
    char* json_string = cJSON_PrintUnformatted(json_my);
    if (json_string == NULL) {
        ESP_LOGE(TAG, "Cannot serialize bookmarks");
        return false;
    }

    printf("%s\n", json_string);

    FILE* my_fd = fopen(my_agenda_path, "w");
    if (my_fd == NULL) {
        ESP_LOGE(TAG, "Cannot init agenda last_updated.");
        return false;
    }
    fwrite(json_string, 1, strlen(json_string), my_fd);
    fclose(my_fd);
    return true;
}

static bool toggle_bookmark(cJSON* track, cJSON* talk, cJSON* my_day, cJSON* bookmarks) {
    if (talk == NULL) {
        ESP_LOGW(TAG, "Cannot toggle bookmark if no talk is given");
        return false;
    }
    bool added = false;

    if (!cJSON_HasObjectItem(talk, BOOKMARKED)) {
        added = true;
        cJSON_AddItemToObject(talk, BOOKMARKED, cJSON_CreateBool(true));
    } else {
        cJSON* bookmarked = cJSON_GetObjectItem(talk, BOOKMARKED);
        added = cJSON_IsFalse(bookmarked);
        cJSON_SetBoolValue(bookmarked, added);
    }

    int bookmarks_count = cJSON_GetArraySize(bookmarks);
    long id = (long) cJSON_GetNumberValue(cJSON_GetObjectItem(talk, "id"));
    long start = (long) cJSON_GetNumberValue(cJSON_GetObjectItem(talk, "start"));

    ESP_LOGD(TAG, "%s", cJSON_PrintUnformatted(bookmarks));
    ESP_LOGD(TAG, "%s", cJSON_PrintUnformatted(my_day));

    if (added) {
        cJSON* entry_id = cJSON_CreateNumber(id);
        cJSON* entry_list = cJSON_CreateObject();
        if (entry_list == NULL) {
            ESP_LOGE(TAG, "Failed to create bookmark entry for %ld", id);
            return false;
        }

        cJSON_AddItemReferenceToObject(entry_list, "track", track);
        cJSON_AddItemReferenceToObject(entry_list, "talk", talk);
        // Add to the list at correct position, expect the current list to be sorted
        cJSON* next = cJSON_GetArrayItem(bookmarks, 0);
        int i = 0;
        while(next != NULL && cJSON_GetNumberValue(cJSON_GetObjectItem(cJSON_GetObjectItem(next, "talk"), "start")) <= start) {
            i++;
            next = next->next;
        }

        cJSON_InsertItemInArray(my_day, i, entry_id);
        cJSON_InsertItemInArray(bookmarks, i, entry_list);
    } else {
        // Remove from the correct position
        cJSON* item;
        for (int i = 0; i < bookmarks_count; i++) {
            item = cJSON_GetArrayItem(bookmarks, i);
            if ((long) cJSON_GetNumberValue(cJSON_GetObjectItem(cJSON_GetObjectItem(item, "talk"), "id")) == id) {
                cJSON_DeleteItemFromArray(bookmarks, i);
                cJSON_DeleteItemFromArray(my_day, i);
                break;
            }
        }
    }

    save_bookmarks();

    return true;
}

static void details_day(pax_buf_t* pax_buffer, xQueueHandle button_queue, cJSON* data, cJSON* my_day, cJSON* bookmarks, pax_buf_t* icon_top, pax_buf_t* icon_bookmarked) {
    int track = 0;
    cJSON* tracks = cJSON_GetObjectItem(data, "tracks");
    int track_count = cJSON_GetArraySize(tracks);
    if (track_count == 0) {
        render_message("No tracks found");
        display_flush();
        wait_for_button();
        return;
    }

    bool render = true;
    bool exit = false;

    int    talk         = 0;
    int    render_talks = 3;
    int slot_height = 62;
    int talk_count;
    keyboard_input_message_t buttonMessage = {0};

    cJSON* talks = cJSON_GetObjectItem(cJSON_GetArrayItem(tracks, track), "talks");
    cJSON* talk_data;

    while(!exit) {
        if (render) {
            talk_count = render_track(pax_buffer, icon_top, icon_bookmarked, tracks, track, talk, render_talks, slot_height);
            render = false;
        }

        clear_keyboard_queue();
        if (xQueueReceive(button_queue, &buttonMessage, portMAX_DELAY) == pdTRUE) {
            if (buttonMessage.state) {
                switch (buttonMessage.input) {
                    case JOYSTICK_DOWN:
                        talk = (talk + 1) % talk_count;
                        render = true;
                        break;
                    case JOYSTICK_UP:
                        talk = (talk - 1 + talk_count) % talk_count;
                        render = true;
                        break;
                    case JOYSTICK_LEFT:
                        track = (track - 1 + track_count) % track_count;
                        // TODO: Do we need to reset the talk?
                        // talk = 0;
                        render = true;
                        break;
                    case JOYSTICK_RIGHT:
                        track = (track + 1) % track_count;
                        // TODO: Do we need to reset the talk?
                        // talk = 0;
                        render = true;
                        break;
                    case BUTTON_BACK:
                        exit = true;
                        break;
                    case BUTTON_SELECT:
                    case JOYSTICK_PUSH:
                        talk_data = cJSON_GetArrayItem(talks, talk);
                        if (cJSON_IsTrue(cJSON_GetObjectItem(talk_data, "special"))) {
                            // Don't allow adding breaks to bookmarks
                            break;
                        }
                        toggle_bookmark(cJSON_GetArrayItem(tracks, track), talk_data, my_day, bookmarks);
                        render = true;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

static void my_agenda(pax_buf_t* pax_buffer, xQueueHandle button_queue, cJSON* day1, cJSON* day2, pax_buf_t* icon) {
    if (day1 == NULL || day2 == NULL) {
        render_message("No talks found");
        display_flush();
        wait_for_button();
        return;
    }

    bool render = true;
    bool exit = false;

    int days = 2;
    int day = 0;
    int    talk         = 0;
    int    render_talks = 3;
    int slot_height = 62;
    int talk_count;
    keyboard_input_message_t buttonMessage = {0};


    cJSON* data = day1;
    cJSON* talk_data;
    cJSON* my_day;
    cJSON* bookmarks;

    while(!exit) {
        if (render) {
            talk_count = render_bookmarks(pax_buffer, icon, data, day, talk, render_talks, slot_height);
            render = false;
        }

        clear_keyboard_queue();
        if (xQueueReceive(button_queue, &buttonMessage, portMAX_DELAY) == pdTRUE) {
            if (buttonMessage.state) {
                switch (buttonMessage.input) {
                    case JOYSTICK_DOWN:
                        talk = (talk + 1) % talk_count;
                        render = true;
                        break;
                    case JOYSTICK_UP:
                        talk = (talk - 1 + talk_count) % talk_count;
                        render = true;
                        break;
                    case JOYSTICK_LEFT:
                        day = (day - 1 + days) % days;
                        data = day == 0 ? day1 : day2;
                        render = true;
                        break;
                    case JOYSTICK_RIGHT:
                        day = (day + 1) % days;
                        data = day == 0 ? day1 : day2;
                        render = true;
                        break;
                    case BUTTON_SELECT:
                    case JOYSTICK_PUSH:
                        talk_data = cJSON_GetArrayItem(data, talk % talk_count);
                        my_day = cJSON_GetObjectItem(json_my, (day == 0) ? "day1" : "day2");
                        bookmarks = (day == 0) ? json_my_day1 : json_my_day2;
                        toggle_bookmark(cJSON_GetObjectItem(talk_data, "track"), cJSON_GetObjectItem(talk_data, "talk"), my_day, bookmarks);
                        render = true;
                        break;
                    case BUTTON_BACK:
                        exit = true;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}


static void details_upcoming(pax_buf_t* pax_buffer, cJSON* data, pax_buf_t* icon) {
    const pax_font_t* font = pax_font_saira_regular;

    if (data == NULL) {
        render_message("No talks found");
        display_flush();
        wait_for_button();
        return;
    }

    cJSON* tracks = cJSON_GetObjectItem(data, "tracks");
    int track_count = cJSON_GetArraySize(tracks);
    if (track_count == 0) {
        render_message("No tracks found");
        display_flush();
        wait_for_button();
        return;
    }

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    details_render_background(pax_buffer, false, false);
    render_topbar(pax_buffer, icon, "Upcoming talks");

    // Remaining vertical space is 186px -> 62px per track
    for (int track = 0; track < track_count; track++) {
        cJSON* track_data = cJSON_GetArrayItem(tracks, track);
        cJSON* title = cJSON_GetObjectItem(track_data, "title");
        cJSON* talks = cJSON_GetObjectItem(track_data, "talks");

        int talk1, talk2 = -1;

        int talk_count = cJSON_GetArraySize(talks);
        for (int talk = 0; talk < talk_count; talk++) {
            cJSON* talk_data = cJSON_GetArrayItem(talks, talk);
            cJSON* start = cJSON_GetObjectItem(talk_data, "ts");
            long ts = (long) cJSON_GetNumberValue(start);
            if (ts > now) {
                talk2 = talk;
                break;
            }
        }

        if (talk2 == 0) {
            talk1 = 0;
            talk2 = 1;
        } else if (talk2 == talk_count - 1) {
            talk1 = talk2;
            talk2 = -1;
        } else {
            talk1 = talk2 - 1;
        }

        // Draw track title
        pax_simple_rect(pax_buffer, 0xFFF1AA13, 0, 34 + 62 * track, 320, 22);
        pax_draw_text(pax_buffer, 0xff131313, font, 18, 5, 34 + 62 * track + 2, title->valuestring);

        // First talk
        pax_simple_rect(pax_buffer, 0xff131313, 0, 34 + 62 * track + 22, 320, 20);
        if (talk1 >= 0) {
            cJSON* talk1_data = cJSON_GetArrayItem(talks, talk1);
            cJSON* talk1_title = cJSON_GetObjectItem(talk1_data, "title");
            cJSON* talk1_start = cJSON_GetObjectItem(talk1_data, "time");

            pax_vec1_t talk1_start_size = pax_text_size(font, 18, talk1_start->valuestring);
            pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 5, 34 + 62 * track + 22 + 2, talk1_start->valuestring);
            pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 5 + talk1_start_size.x + 5, 34 + 62 * track + 22 + 2, talk1_title->valuestring);
        }

        // Second talk
        pax_simple_rect(pax_buffer, 0xff131313, 0, 34 + 62 * track + 22 + 20, 320, 20);
        if (talk2 >= 0) {
            cJSON* talk2_data = cJSON_GetArrayItem(talks, talk2);
            cJSON* talk2_title = cJSON_GetObjectItem(talk2_data, "title");
            cJSON* talk2_start = cJSON_GetObjectItem(talk2_data, "time");

            pax_vec1_t talk2_start_size = pax_text_size(font, 18, talk2_start->valuestring);
            pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 5, 34 + 62 * track + 22 + 20 + 2, talk2_start->valuestring);
            pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 5 + talk2_start_size.x + 5, 34 + 62 * track + 22 + 20 + 2, talk2_title->valuestring);
        }
    }
    display_flush();
    wait_for_button();
}

static bool need_update(unsigned long *remote_last_update) {
    FILE* last_update_fd = fopen(last_update_path, "r");
    if (last_update_fd == NULL) {
        return true;
    }

    char* buf = malloc(11);
    buf[10] = '\0';
    fread(buf, 1, 10, last_update_fd);
    fclose(last_update_fd);

    unsigned long local_last_update = atol(buf);

    static char*  remote_buf = NULL;
    static size_t remote_buf_len = 0;
    bool success = download_ram(last_update_url, (uint8_t**) &remote_buf, &remote_buf_len);
    if (!success) return false; // Do not try to update if the first requests didn't work
    if (remote_buf_len > 10) return true;

    char* buf2 = malloc(11);
    buf2[10] = '\0';
    memcpy(buf2, remote_buf, 10);
    *remote_last_update = atol(buf2);

    ESP_LOGD(TAG, "local=%lu, remote=%lu", local_last_update, *remote_last_update);
    ESP_LOGD(TAG, "local=%s, remote=%s", buf, buf2);

    return *remote_last_update > local_last_update;
}

static bool test_load_data() {
    if (!load_file(TAG, day1_path_tmp, &data_day1, &size_day1)) return false;

    if (!load_file(TAG, day2_path_tmp, &data_day2, &size_day2)) return false;

    json_day1 = cJSON_ParseWithLength(data_day1, size_day1);
    if (json_day1 == NULL) {
        return false;
    }

    json_day2 = cJSON_ParseWithLength(data_day2, size_day2);
    if (json_day2 == NULL) {
        return false;
    }

    return true;
}

static uint find_correct_position(cJSON* my_day, long start) {
    cJSON* next = cJSON_GetArrayItem(my_day, 0);
    int i = 0;
    while(next != NULL && cJSON_GetNumberValue(cJSON_GetObjectItem(cJSON_GetObjectItem(next, "talk"), "start")) <= start) {
        i++;
        next = next->next;
    }
    return i;
}

static bool load_my_data(cJSON* day, cJSON* bookmarks, cJSON* results) {
    cJSON *bookmark;

    int bookmark_count = cJSON_GetArraySize(bookmarks);

    cJSON* tracks = cJSON_GetObjectItem(day, "tracks");
    if (tracks == NULL) {
        ESP_LOGE(TAG, "No tracks found");
        return false;
    }
    int tracks_count = cJSON_GetArraySize(tracks);
    int talks_count;

    cJSON* track;
    cJSON* talks;
    cJSON* talk;

    bool found = false;
    long searchId;
    long currentId;
    cJSON* result;

    for (int i = 0; i < bookmark_count; i++) {
        bookmark = cJSON_GetArrayItem(bookmarks, i);
        searchId = (long) cJSON_GetNumberValue(bookmark);
        found = false;

        for (int j = 0; j < tracks_count; j++) {
            track = cJSON_GetArrayItem(tracks, j);
            talks = cJSON_GetObjectItem(track, "talks");
            talks_count = cJSON_GetArraySize(talks);

            for (int k = 0; k < talks_count; k++) {
                talk = cJSON_GetArrayItem(talks, k);
                currentId = (long) cJSON_GetNumberValue(cJSON_GetObjectItem(talk, "id"));
                if (searchId == currentId) {
                    // Found the right talk, add it to the result array
                    found = true;
                    result = cJSON_CreateObject();
                    if (result == NULL) {
                        ESP_LOGE(TAG, "Failed to create bookmark entry for %ld", searchId);
                        return false;
                    }

                    cJSON_AddItemReferenceToObject(result, "track", track);
                    cJSON_AddItemReferenceToObject(result, "talk", talk);

                    // Set bookmarked
                    if (!cJSON_HasObjectItem(talk, BOOKMARKED)) {
                        cJSON_AddItemToObject(talk, BOOKMARKED, cJSON_CreateBool(true));
                    } else {
                        cJSON_SetBoolValue(cJSON_GetObjectItem(talk, BOOKMARKED), true);
                    }

                    // We sort here as well, bcz talks might be shifted -> IDs are not in correct order anymore
                    uint pos = find_correct_position(results, (long) cJSON_GetNumberValue(cJSON_GetObjectItem(talk, "start")));
                    cJSON_InsertItemInArray(results, pos, result);
                }
            }
        }

        if (!found) {
            ESP_LOGW(TAG, "Didn't find a talk for bookmark ID %ld", searchId);
        }
    }
    return true;
}

static bool load_data() {
    if (!load_file(TAG, day1_path, &data_day1, &size_day1)) {
        ESP_LOGE(TAG, "Failed to read agenda file: %s", day1_path);
        render_message("Failed to read agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    if (!load_file(TAG, day2_path, &data_day2, &size_day2)) {
        ESP_LOGE(TAG, "Failed to read agenda file: %s", day2_path);
        render_message("Failed to read agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    if (!load_file(TAG, my_agenda_path, &data_my, &size_my)) {
        ESP_LOGE(TAG, "Failed to read agenda file: %s", my_agenda_path);
        render_message("Failed to read agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    if (json_day1 != NULL) {
        cJSON_Delete(json_day1);
    }
    json_day1 = cJSON_ParseWithLength(data_day1, size_day1);
    if (json_day1 == NULL) {
        ESP_LOGE(TAG, "Failed to parse agenda file: %s", day1_path);
        render_message("Failed to parse agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    if (json_day2 != NULL) {
        cJSON_Delete(json_day2);
    }
    json_day2 = cJSON_ParseWithLength(data_day2, size_day2);
    if (json_day2 == NULL) {
        ESP_LOGE(TAG, "Failed to parse agenda file: %s", day2_path);
        render_message("Failed to parse agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    if (json_my != NULL) {
        cJSON_Delete(json_my);
    }
    json_my = cJSON_ParseWithLength(data_my, size_my);
    if (json_my == NULL) {
        ESP_LOGE(TAG, "Failed to parse agenda file: %s", my_agenda_path);
        render_message("Failed to parse agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    // A day in json_my contains a list of talk IDs. To print them fast, we look them up here and prepare the list

    json_my_day1 = cJSON_CreateArray();
    if (json_my_day1 == NULL) {
        ESP_LOGE(TAG, "Failed to create my_day1");
        render_message("Failed to parse agenda. 🅰 to retry.");
        display_flush();
        return false;
    }
    json_my_day2 = cJSON_CreateArray();
    if (json_my_day2 == NULL) {
        ESP_LOGE(TAG, "Failed to create my_day2");
        render_message("Failed to parse agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    load_my_data(json_day1, cJSON_GetObjectItem(json_my, "day1"), json_my_day1);
    load_my_data(json_day2, cJSON_GetObjectItem(json_my, "day2"), json_my_day2);

    return true;
}

static bool update_agenda(xQueueHandle button_queue, bool force) {
    render_message("Updating agenda...");
    display_flush();

    if (!wifi_connect_to_stored()) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        render_message("Failed to connect to WiFi");
        display_flush();
        return false;
    }

    unsigned long last_update = 0;
    if (!need_update(&last_update) && !force) {
        ESP_LOGI(TAG, "No update needed");
        wifi_disconnect_and_disable();
        return false;
    }

    ESP_LOGI(TAG, "Updating agenda");

    if (!download_file(day1_url, day1_path_tmp)) {
        ESP_LOGE(TAG, "Failed to download %s to %s", day1_url, day1_path);
        render_message("Failed to download file");
        display_flush();
        wifi_disconnect_and_disable();
        if (button_queue != NULL) wait_for_button();
        return false;
    }

    if (!download_file(day2_url, day2_path_tmp)) {
        ESP_LOGE(TAG, "Failed to download %s to %s", day2_url, day2_path);
        render_message("Failed to download file");
        display_flush();
        wifi_disconnect_and_disable();
        if (button_queue != NULL) wait_for_button();
        return false;
    }

    if (!test_load_data()) {
        ESP_LOGE(TAG, "Failed to load updated data, not replacing old");
        return false;
    }



    if (rename_or_replace(TAG, day1_path_tmp,day1_path)) {
        ESP_LOGE(TAG, "Failed to rename %s to %s", day1_path_tmp, day1_path);
        render_message("Failed to store file");
        display_flush();
        wifi_disconnect_and_disable();
        if (button_queue != NULL) wait_for_button();
        return true;
    }

    if (rename_or_replace(TAG, day2_path_tmp, day2_path)) {
        ESP_LOGE(TAG, "Failed to rename %s to %s", day2_path_tmp, day2_path);
        render_message("Failed to store file");
        display_flush();
        wifi_disconnect_and_disable();
        if (button_queue != NULL) wait_for_button();
        return true;
    }

    // Remember that we updated
    FILE* last_update_fd = fopen(last_update_path, "w");
    if (last_update_fd == NULL) {
        ESP_LOGE(TAG, "Unable to persist last update timestamp");
        wifi_disconnect_and_disable();
        return true;
    }

    char str[11];
    sprintf(str, "%lu", last_update);
    fwrite(str, 1, 10, last_update_fd);
    fclose(last_update_fd);
    wifi_disconnect_and_disable();
    return true;
}

static cJSON* get_current_day() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year != (2023 - 1900) || timeinfo.tm_mon != 5) {
        return NULL;
    }

    if (timeinfo.tm_mday == 28) return json_day1;
    if (timeinfo.tm_mday == 29) return json_day2;

    return NULL;
}

static bool try_update_or_load(xQueueHandle button_queue, bool first_attempt) {
    if (update_agenda(button_queue, !first_attempt)) {
        return true;
    }
    if (load_data()) {
        return true;
    }

    return false;
}


void menu_agenda(xQueueHandle button_queue) {
    pax_buf_t* pax_buffer = get_pax_buffer();

    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xFF131313);

    // Ensure directory exists
    if (!create_dir("/internal/apps")) {
        ESP_LOGE(TAG, "Failed to create directory in internal storage");
        render_message("Failed to create data dir");
        display_flush();
        wait_for_button();
        return;
    }
    if (!create_dir("/internal/apps/agenda")) {
        ESP_LOGE(TAG, "Failed to create directory in internal storage");
        render_message("Failed to create data dir");
        display_flush();
        wait_for_button();
        return;
    }

    do_init();

    bool first_attempt = true;
    do {
        // Try to update and load data
        if (try_update_or_load(button_queue, first_attempt)) break;
        // Only force download after the first attempt failed
        first_attempt = false;
        // If there was an error the user is asked if the processes should be retried
        clear_keyboard_queue();
        if (!wait_for_button()) {
            return;
        }
    } while (1);

    menu_t*    menu       = menu_alloc("TROOPERS24 - Agenda", 34, 18);

    menu->fgColor           = 0xFFF1AA13;
    menu->bgColor           = 0xFF131313;
    menu->bgTextColor       = 0xFF000000;
    menu->selectedItemColor = 0xFFF1AA13;
    menu->borderColor       = 0xFF1E1E1E;
    menu->titleColor        = 0xFFF1AA13;
    menu->titleBgColor      = 0xFF1E1E1E;
    menu->scrollbarBgColor  = 0xFFCCCCCC;
    menu->scrollbarFgColor  = 0xFF555555;

    pax_buf_t icon_agenda;
    pax_decode_png_buf(&icon_agenda, (void*) agenda_png_start, agenda_png_end - agenda_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_clock;
    pax_decode_png_buf(&icon_clock, (void*) clock_png_start, clock_png_end - clock_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_bookmark;
    pax_decode_png_buf(&icon_bookmark, (void*) bookmark_png_start, bookmark_png_end - bookmark_png_start, PAX_BUF_32_8888ARGB, 0);

    menu_set_icon(menu, &icon_agenda);
    menu_insert_item_icon(menu, "Bookmarks", NULL, (void*) ACTION_MY_AGENDA, -1, &icon_bookmark);
    if (ntp_synced) {
        menu_insert_item_icon(menu, "Next up", NULL, (void*) ACTION_NEXT_UP, -1, &icon_clock);
    }
    menu_insert_item_icon(menu, "Wednesday", NULL, (void*) ACTION_WEDNESDAY, -1, &icon_agenda);
    menu_insert_item_icon(menu, "Thursday", NULL, (void*) ACTION_THURSDAY, -1, &icon_agenda);

    bool                render = true;
    menu_agenda_action_t action = ACTION_NONE;

    bool full_redraw = true;
    bool exit = false;
    while (!exit) {
        if (render) {
            if (full_redraw) {
                agenda_render_background(pax_buffer);
            }

            if (full_redraw) {
                menu_render_grid(pax_buffer, menu, 0, 0, 320, 220);
                display_flush();
            } else {
                menu_render_grid_changes(pax_buffer, menu, 0, 0, 320, 220);
                display_flush();
            }

            render      = false;
            full_redraw = false;
        }

        clear_keyboard_queue();
        keyboard_input_message_t buttonMessage = {0};
        if (xQueueReceive(button_queue, &buttonMessage, portMAX_DELAY) == pdTRUE) {
            if (buttonMessage.state) {
                switch (buttonMessage.input) {
                    case JOYSTICK_DOWN:
                        menu_navigate_next_row(menu);
                        render = true;
                        full_redraw = true;
                        break;
                    case JOYSTICK_UP:
                        menu_navigate_previous_row(menu);
                        render = true;
                        full_redraw = true;
                        break;
                    case JOYSTICK_LEFT:
                        menu_navigate_previous(menu);
                        render = true;
                        break;
                    case JOYSTICK_RIGHT:
                        menu_navigate_next(menu);
                        render = true;
                        break;
                    case BUTTON_BACK:
                        exit = true;
                        break;
                    case BUTTON_ACCEPT:
                    case BUTTON_SELECT:
                        action = (menu_agenda_action_t) menu_get_callback_args(menu, menu_get_position(menu));
                        break;
                    default:
                        break;
                }
            }
        }

        if (action != ACTION_NONE) {
            if (action == ACTION_NEXT_UP) {
                details_upcoming(pax_buffer, get_current_day(), &icon_clock);
            } else if (action == ACTION_MY_AGENDA) {
                my_agenda(pax_buffer, button_queue, json_my_day1, json_my_day2, &icon_bookmark);
            } else if (action == ACTION_WEDNESDAY) {
                details_day(pax_buffer, button_queue, json_day1, cJSON_GetObjectItem(json_my, "day1"), json_my_day1, &icon_agenda, &icon_bookmark);
            } else if (action == ACTION_THURSDAY) {
                details_day(pax_buffer, button_queue, json_day2, cJSON_GetObjectItem(json_my, "day2"), json_my_day2, &icon_agenda, &icon_bookmark);
            }
            action      = ACTION_NONE;
            render      = true;
            full_redraw = true;
        }
    }

    menu_free(menu);

    cJSON_Delete(json_my_day1);
    json_my_day1 = NULL;

    cJSON_Delete(json_my_day2);
    json_my_day2 = NULL;

    cJSON_Delete(json_my);
    json_my = NULL;

    // Delete the data loaded from JSON
    cJSON_Delete(json_day1);
    json_day1 = NULL;
    cJSON_Delete(json_day2);
    json_day2 = NULL;

    pax_buf_destroy(&icon_agenda);
    pax_buf_destroy(&icon_clock);
}
