add_executable(USBServoPlatformApp
        USBServoPlatformApp.c
        )

target_link_libraries(USBServoPlatformApp pico_stdlib)
pico_enable_stdio_usb(USBServoPlatformApp 1)
pico_enable_stdio_uart(USBServoPlatformApp 0)
pico_add_extra_outputs(USBServoPlatformApp)