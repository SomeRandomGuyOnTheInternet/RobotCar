# Include the FreeRTOS import file at the top to make FreeRTOS-Kernel available
include("../../FreeRTOS_Kernel_import.cmake")

add_executable(line_following line_following.c)

target_include_directories(line_following PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${FREERTOS_KERNEL_PATH}  # FreeRTOS kernel include path
)

target_sources(line_following PRIVATE ${FREERTOS_KERNEL_PATH}/portable/MemMang/heap_4.c)

target_link_libraries(line_following
    pico_stdlib
    hardware_adc  
    hardware_pwm
    motor_lib
    FreeRTOS-Kernel
)

pico_enable_stdio_usb(line_following 1)
pico_add_extra_outputs(line_following)

add_library(line_following_lib
    line_following.c
    line_following.h
)

target_include_directories(line_following_lib PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${FREERTOS_KERNEL_PATH}  # FreeRTOS kernel include path
)

target_sources(line_following_lib PRIVATE ${FREERTOS_KERNEL_PATH}/portable/MemMang/heap_4.c)

target_link_libraries(line_following_lib
    pico_stdlib
    hardware_adc  
    hardware_pwm
    motor_lib
    FreeRTOS-Kernel
)

pico_enable_stdio_usb(line_following_lib 1)
