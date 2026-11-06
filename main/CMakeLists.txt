idf_component_register(SRCS "main.c"
                       REQUIRES esp32_p4_function_ev_board usb usb_host_uvc esp_driver_jpeg esp_driver_ppa esp_timer
                       INCLUDE_DIRS "")
