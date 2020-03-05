/**************************************************************************
 * Copyright (C) 2018-2019 Junlon2006
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 **************************************************************************
 *
 * Description : uni_log.h
 * Author      : junlon2006@163.com
 * Date        : 2019.03.17
 *
 **************************************************************************/
#ifndef LOGGER_INC_UNI_LOG_H_
#define LOGGER_INC_UNI_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  N_LOG_NONE = -1,
  N_LOG_ERROR,
  N_LOG_WARN,
  N_LOG_TRACK,
  N_LOG_DEBUG,
  N_LOG_RAW,
  N_LOG_ALL
} LogLevel;

typedef struct {
  int      enable_time;
  int      enable_thread_id;
  int      enable_function_line;
  int      enable_color;
  int      enable_file;
  LogLevel set_level;
} LogConfig;

int LogInitialize(LogConfig logConfig);
int LogFinalize(void);
int LogLevelSet(LogLevel level);

int LogLevelValid(LogLevel level);
int LogWrite(LogLevel level, const char *tags, const char *function,
             int line, char *fmt, ...);

#define LOG(level, tag, fmt, ...) do { \
  if (LogLevelValid(level)) { \
    LogWrite(level, tag, __FUNCTION__, __LINE__, (char *)fmt, ##__VA_ARGS__); \
  } \
} while (0)

#define LOGD(tag, fmt, ...) LOG(N_LOG_DEBUG, tag, fmt, ##__VA_ARGS__)
#define LOGT(tag, fmt, ...) LOG(N_LOG_TRACK, tag, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...) LOG(N_LOG_WARN, tag, fmt, ##__VA_ARGS__)
#define LOGE(tag, fmt, ...) LOG(N_LOG_ERROR, tag, fmt, ##__VA_ARGS__)
#define LOGR(tag, fmt, ...) LOG(N_LOG_RAW, tag, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}   /* __cplusplus */
#endif
#endif  /* LOGGER_INC_UNI_LOG_H_ */
