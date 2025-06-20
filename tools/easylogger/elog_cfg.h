/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015-2016, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: It is the configure head file for this library.
 * Created on: 2015-07-30
 */

#ifndef _ELOG_CFG_H_
#define _ELOG_CFG_H_
#include "robot_config.h"
/*---------------------------------------------------------------------------*/
/* enable log output. */
#define ELOG_OUTPUT_ENABLE
/* setting static output log level. range: from ELOG_LVL_ASSERT to ELOG_LVL_VERBOSE */
#if ROBOT_CONFIG_DEBUG==1
#define ELOG_OUTPUT_LVL                          ELOG_LVL_VERBOSE
#else
#define ELOG_OUTPUT_LVL                          ELOG_LVL_ERROR
#endif
/* enable assert check */
#define ELOG_ASSERT_ENABLE
/* buffer size for every line's log */
#define ELOG_LINE_BUF_SIZE                       1024
/* output line number max length */
#define ELOG_LINE_NUM_MAX_LEN                    5
/* output filter's tag max length */
#define ELOG_FILTER_TAG_MAX_LEN                  30
/* output filter's keyword max length */
#define ELOG_FILTER_KW_MAX_LEN                   16
/* output filter's tag level max num */
#define ELOG_FILTER_TAG_LVL_MAX_NUM              5
/* output newline sign */
#define ELOG_NEWLINE_SIGN                        "\n"
/*---------------------------------------------------------------------------*/
/* enable log color */
#define ELOG_COLOR_ENABLE
/* change the some level logs to not default color if you want */
#define ELOG_COLOR_ASSERT                        (F_MAGENTA B_NULL S_NORMAL)
#define ELOG_COLOR_ERROR                         (F_RED B_NULL S_NORMAL)
#define ELOG_COLOR_WARN                          (F_YELLOW B_NULL S_NORMAL)
#define ELOG_COLOR_INFO                          (F_CYAN B_NULL S_NORMAL)
#define ELOG_COLOR_DEBUG                         (F_GREEN B_NULL S_NORMAL)
#define ELOG_COLOR_VERBOSE                       (F_BLUE B_NULL S_NORMAL)
/*---------------------------------------------------------------------------*/
/* enable log fmt */
/* comment it if you don't want to output them at all */
#define ELOG_FMT_USING_FUNC
//#define ELOG_FMT_USING_DIR
#define ELOG_FMT_USING_LINE

#endif /* _ELOG_CFG_H_ */

