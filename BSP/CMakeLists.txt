# 创建 bsp 静态库
add_library(bsp STATIC
    DWT/dwt.c
    CAN/bsp_can.c
    UART/bsp_uart.c
    SPI/bsp_spi.c
    FLASH/bsp_flash.c
)

# 设置包含目录
target_include_directories(bsp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/DWT
        ${CMAKE_CURRENT_SOURCE_DIR}/CAN
        ${CMAKE_CURRENT_SOURCE_DIR}/UART
        ${CMAKE_CURRENT_SOURCE_DIR}/SPI
        ${CMAKE_CURRENT_SOURCE_DIR}/FLASH
)

# 链接必要的库
target_link_libraries(bsp stm32cubemx tools ThreadX)

# 将 segger 添加到主项目
target_link_libraries(${PROJECT_NAME} bsp)