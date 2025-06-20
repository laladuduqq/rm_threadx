/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USB_LOG_H
#define USB_LOG_H

#include "elog.h" // 添加EasyLogger头文件

// 日志级别映射
#define USB_DBG_ERROR   ELOG_LVL_ERROR
#define USB_DBG_WARNING ELOG_LVL_WARN
#define USB_DBG_INFO    ELOG_LVL_INFO
#define USB_DBG_LOG     ELOG_LVL_DEBUG

#ifndef USB_DBG_TAG
#define USB_DBG_TAG "USB"
#endif
/*
 * The color for terminal (foreground)
 * BLACK    30
 * RED      31
 * GREEN    32
 * YELLOW   33
 * BLUE     34
 * PURPLE   35
 * CYAN     36
 * WHITE    37
 */
#define _USB_DBG_COLOR(n)
#define _USB_DBG_LOG_HDR(lvl_name, color_n) \
    CONFIG_USB_PRINTF("[" lvl_name "/" USB_DBG_TAG "] ")
#define _USB_DBG_LOG_X_END

#define usb_dbg_log_line(lvl, color_n, fmt, ...) \
    do {                                         \
        _USB_DBG_LOG_HDR(lvl, color_n);          \
        CONFIG_USB_PRINTF(fmt, ##__VA_ARGS__);   \
        _USB_DBG_LOG_X_END;                      \
    } while (0)

#if (CONFIG_USB_DBG_LEVEL >= USB_DBG_LOG)
#define USB_LOG_DBG(fmt, ...) elog_d(USB_DBG_TAG, fmt, ##__VA_ARGS__)
#else
#define USB_LOG_DBG(...)
#endif

#if (CONFIG_USB_DBG_LEVEL >= USB_DBG_INFO)
#define USB_LOG_INFO(fmt, ...) elog_i(USB_DBG_TAG, fmt, ##__VA_ARGS__)
#else
#define USB_LOG_INFO(...)
#endif

#if (CONFIG_USB_DBG_LEVEL >= USB_DBG_WARNING)
#define USB_LOG_WRN(fmt, ...) elog_w(USB_DBG_TAG, fmt, ##__VA_ARGS__)
#else
#define USB_LOG_WRN(...)
#endif

#if (CONFIG_USB_DBG_LEVEL >= USB_DBG_ERROR)
#define USB_LOG_ERR(fmt, ...) elog_e(USB_DBG_TAG, fmt, ##__VA_ARGS__)
#else
#define USB_LOG_ERR(...)
#endif

#define USB_LOG_RAW(...) elog_raw(__VA_ARGS__)

#ifndef CONFIG_USB_ASSERT_DISABLE
#define USB_ASSERT(f)                                                            \
    do {                                                                         \
        if (!(f)) {                                                              \
            USB_LOG_ERR("ASSERT FAIL [%s] @ %s:%d\r\n", #f, __FILE__, __LINE__); \
            while (1) {                                                          \
            }                                                                    \
        }                                                                        \
    } while (false)

#define USB_ASSERT_MSG(f, fmt, ...)                                              \
    do {                                                                         \
        if (!(f)) {                                                              \
            USB_LOG_ERR("ASSERT FAIL [%s] @ %s:%d\r\n", #f, __FILE__, __LINE__); \
            USB_LOG_ERR(fmt "\r\n", ##__VA_ARGS__);                              \
            while (1) {                                                          \
            }                                                                    \
        }                                                                        \
    } while (false)
#else
#define USB_ASSERT(f) {}
#define USB_ASSERT_MSG(f, fmt, ...) {}
#endif

#define ___is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
static inline void usb_hexdump(const void *ptr, uint32_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    unsigned int i, j;

    (void)buf;

    for (i = 0; i < buflen; i += 16) {
        CONFIG_USB_PRINTF("%08x:", i);

        for (j = 0; j < 16; j++)
            if (i + j < buflen) {
                if ((j % 8) == 0) {
                    CONFIG_USB_PRINTF("  ");
                }

                CONFIG_USB_PRINTF("%02X ", buf[i + j]);
            } else
                CONFIG_USB_PRINTF("   ");
        CONFIG_USB_PRINTF(" ");

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                CONFIG_USB_PRINTF("%c", ___is_print(buf[i + j]) ? buf[i + j] : '.');
        CONFIG_USB_PRINTF("\n");
    }
}

#endif /* USB_LOG_H */
