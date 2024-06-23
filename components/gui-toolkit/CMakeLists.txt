idf_component_register(
    SRCS "menu.c"
         "gui_element_header.c"
         "graphics_wrapper.c"
         "menu.c"
    INCLUDE_DIRS "." "include"
    REQUIRES
        "pax-graphics"
        "pax-codecs"
        "pax-keyboard"
        "troopers24-bsp"
)
