# tools文件夹说明

本文件夹下为一些开发调试工具，比如log日志，rtt打印等，后续会加上cm_backtrace,systemview等工具

## easylogger

[github地址](https://github.com/armink/EasyLogger)
移植时将buf缓冲和异步打印去除，目前只是纯log打印，当速度过快时会出现打印不全和打印丢失的问题。但是目前就主要观察error,正常使用时会将log_lvl提升到error级别，所以影响不大，如果有感兴趣的可以将buf缓冲和异步打印加入其中。
**注意： 目前log打印只在rtt通道，串口通道目前暂未加入，待后续增加切换宏**

## segger_rtt

目前主要的调试工具是jlink_ob，所以基本上所有的打印都走rtt通道。后续会考虑加入systemview,方便观察线程的实际运行情况

## 如何添加自己的tools

首先将对应的文件放入tools文件夹下。然后对tools文件下的CMakeLists.txt中修改

```cmake
add_library(tools STATIC
    segger/SEGGER_RTT_printf.c 
    segger/SEGGER_RTT.c
    segger/SEGGER_RTT_ASM_ARMv7M.s
    easylogger/elog.c 
    easylogger/elog_port.c 
    easylogger/elog_utils.c
)在这里加入对应的c文件或者.s文件

target_include_directories(tools
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/segger
        ${CMAKE_CURRENT_SOURCE_DIR}/easylogger
)在这里将头文件目录加入即可
然后会cmake自动刷新，自动加入文件
```
