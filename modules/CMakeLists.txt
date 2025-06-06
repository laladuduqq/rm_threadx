# 创建 modules 静态库
add_library(modules STATIC
        RGB/RGB.c 
        algorithm/kalman_filter.c
        algorithm/controller.c 
        algorithm/LQR.c 
        algorithm/QuaternionEKF.c 
        algorithm/user_lib.c 
        BMI088/BMI088.c
        systemwatch/systemwatch.c
        offline/offline.c
        IMU/imu_task.c 
        IMU/imu.c
        REMOTE/sbus.c
        referee/crc_rm.c 
        referee/referee_UI.c 
        referee/referee.c 
)

# 设置包含目录
target_include_directories(modules
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/RGB
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/BMI088
        ${CMAKE_CURRENT_SOURCE_DIR}/systemwatch
        ${CMAKE_CURRENT_SOURCE_DIR}/offline
        ${CMAKE_CURRENT_SOURCE_DIR}/IMU
        ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE
        ${CMAKE_CURRENT_SOURCE_DIR}/referee
)

# 链接必要的库
target_link_libraries(modules stm32cubemx ThreadX tools bsp application)
target_link_libraries(modules -l:libarm_cortexM4lf_math.a) #链接dsp

# 将 modules 添加到主项目
target_link_libraries(${PROJECT_NAME} modules)