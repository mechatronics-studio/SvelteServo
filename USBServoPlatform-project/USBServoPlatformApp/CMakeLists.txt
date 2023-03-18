add_executable(USBServoPlatformApp
        USBServoPlatformApp.c
        )

pico_generate_pio_header(USBServoPlatformApp ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_sources(USBServoPlatformApp PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Core0Commands.c
        ${CMAKE_CURRENT_LIST_DIR}/Core1Commands.c
        ${CMAKE_CURRENT_LIST_DIR}/USBCommunicationProtocol.c
        ${CMAKE_CURRENT_LIST_DIR}/CoreCommunication.c
        ${CMAKE_CURRENT_LIST_DIR}/SimulatedServoMotor.c
        ${CMAKE_CURRENT_LIST_DIR}/ClosedLoopServoController.c
        ${CMAKE_CURRENT_LIST_DIR}/LEDcontroller.c
        ${CMAKE_CURRENT_LIST_DIR}/RamVariablesCore0.c
        ${CMAKE_CURRENT_LIST_DIR}/RamVariablesCore1.c
        ${CMAKE_CURRENT_LIST_DIR}/BezierCurve.c
        ${CMAKE_CURRENT_LIST_DIR}/MT6701Encoder.c
        )

target_link_libraries(USBServoPlatformApp pico_stdlib pico_multicore hardware_pio hardware_pwm hardware_adc hardware_i2c)
pico_enable_stdio_usb(USBServoPlatformApp 1)
pico_enable_stdio_uart(USBServoPlatformApp 0)
pico_add_extra_outputs(USBServoPlatformApp)

pico_set_float_implementation(USBServoPlatformApp pico)

#pico_set_binary_type(USBServoPlatformApp copy_to_ram)