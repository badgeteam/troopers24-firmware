idf_component_register(
    SRCS "main.c"
         "appfs_wrapper.c"
         "settings.c"
         "system_wrapper.c"
         "wifi_ota.c"
         "audio.c"
         "bootscreen.c"
         "ntp_helper.c"
         "menus/utils.c"
         "menus/hatchery.c"
         "menus/settings.c"
         "menus/start.c"
         "menus/dev.c"
         "menus/wifi.c"
         "menus/sao.c"
         "menus/launcher.c"
         "menus/id.c"
         "menus/agenda.c"
         "menus/contacts.c"
         "menus/nfcreader.c"
         "nametag.c"
         "file_browser.c"
         "test_common.c"
         "factory_test.c"
         "button_test.c"
         "wifi_test.c"
         "sao_eeprom.c"
         "rtc_memory.c"
         "metadata.c"
         "wifi_defaults.c"
         "wifi_cert.c"
         "http_download.c"
         "filesystems.c"
         "app_management.c"
         "app_update.c"
         "terminal.c"
    INCLUDE_DIRS "."
                 "include"
                 "menus"
    EMBED_TXTFILES ${project_dir}/resources/isrgrootx1.pem
                   ${project_dir}/resources/custom_ota_cert.pem
    EMBED_FILES
        ${project_dir}/resources/happy.mp3
        ${project_dir}/resources/boot.mp3
        ${project_dir}/resources/boot.png
        ${project_dir}/resources/boot0.png
        ${project_dir}/resources/boot1.png
        ${project_dir}/resources/boot2.png
        ${project_dir}/resources/boot3.png
        ${project_dir}/resources/boot4.png
        ${project_dir}/resources/boot5.png
        ${project_dir}/resources/boot6.png
        ${project_dir}/resources/troopers1.png
        ${project_dir}/resources/icons/dev.png
        ${project_dir}/resources/icons/home.png
        ${project_dir}/resources/icons/settings.png
        ${project_dir}/resources/icons/apps.png
        ${project_dir}/resources/icons/hatchery.png
        ${project_dir}/resources/icons/tag.png
        ${project_dir}/resources/icons/bitstream.png
        ${project_dir}/resources/icons/python.png
        ${project_dir}/resources/icons/hourglass.png
        ${project_dir}/resources/icons/update.png
        ${project_dir}/resources/icons/sao.png
        ${project_dir}/resources/icons/calendar.png
        ${project_dir}/resources/icons/clock.png
        ${project_dir}/resources/icons/bookmark.png
        ${project_dir}/resources/icons/addressbook.png
        ${project_dir}/resources/icons/edit.png
        ${project_dir}/resources/icons/badge.png
        ${project_dir}/resources/icons/share.png
        ${project_dir}/resources/icons/receive.png
        ${project_dir}/resources/id/id_shield.png
        ${project_dir}/resources/id/id_15.png
        ${project_dir}/resources/id/id_glass.png
        ${project_dir}/resources/id/id_storytellers.png
        ${project_dir}/resources/id/id_ernw.png
        ${project_dir}/resources/nametag.png
        ${project_dir}/resources/tr23_nametag.png
)
