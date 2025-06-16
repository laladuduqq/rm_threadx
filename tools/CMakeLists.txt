# 创建 tools 静态库
add_library(tools STATIC
    segger/SEGGER_RTT_printf.c 
    segger/SEGGER_RTT.c
    segger/SEGGER_RTT_ASM_ARMv7M.s
    easylogger/elog.c 
    easylogger/elog_port.c 
    easylogger/elog_utils.c
    CherryUSB/core/usbd_core.c 
    CherryUSB/class/cdc/usbd_cdc_acm.c 
    CherryUSB/port/dwc2/usb_dc_dwc2.c 
    CherryUSB/port/dwc2/usb_glue_st.c
)

# 设置包含目录
target_include_directories(tools
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/segger
        ${CMAKE_CURRENT_SOURCE_DIR}/easylogger
        ${CMAKE_CURRENT_SOURCE_DIR}/CherryUSB/core
        ${CMAKE_CURRENT_SOURCE_DIR}/CherryUSB/class/cdc
        ${CMAKE_CURRENT_SOURCE_DIR}/CherryUSB/common 
        ${CMAKE_CURRENT_SOURCE_DIR}/CherryUSB/port/dwc2 
        ${CMAKE_CURRENT_SOURCE_DIR}/CherryUSB/class/hub
)

set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/CherryUSB/core/usbh_core.h 
    PROPERTIES 
    COMPILE_FLAGS "-Wno-pedantic -Wno-unused-parameter"
)

# 链接必要的库
target_link_libraries(tools stm32cubemx ThreadX application modules)

# 将 segger 添加到主项目
target_link_libraries(${PROJECT_NAME} tools)