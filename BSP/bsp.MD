# bsp

这里是底层适配层，主要对接stm32的外设与上层模块，方便不同板子下的调用。

## 如何添加bsp

将对应文件夹放入bsp文件夹下

```cmake
# 创建 bsp 静态库
add_library(bsp STATIC
    DWT/dwt.c
)这里放对应的.c .s文件

# 设置包含目录
target_include_directories(bsp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/DWT
) //这里放入对应的头文件
```
