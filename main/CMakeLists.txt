idf_component_register(
    SRCS 
        "main.c"
        "audio_task.c"
        "event_queue.c"
        "failsafe.c"
        "lcd_display.c"
        "button_input.c"
        "joystick_input.c"
        "mode_manager.c"
        "state_machine.c"
        "espnow_comm.c"
    INCLUDE_DIRS 
        "."
    REQUIRES
        driver
        esp_system
        nvs_flash
        esp_wifi
        esp_adc
)


