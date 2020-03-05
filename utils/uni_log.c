/**************************************************************************
 * Copyright (C) 2018-2019  Junlon2006
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
 * Description : uni_log.c
 * Author      : junlon2006@163.com
 * Date        : 2019.03.17
 *
 **************************************************************************/
#include "uni_log.h"

#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>

#define LOG_BUFFER_LEN       (1024)
#define LOG_FILE_NAME        "app.log"
#define uni_min(x,y)         ({ \
                                typeof(x) _x = (x); \
                                typeof(y) _y = (y); \
                                (void)(&_x == &_y); \
                                _x < _y ? _x : _y;})
#define uni_max(x,y)         ({ \
                                typeof(x) _x = (x); \
                                typeof(y) _y = (y); \
                                (void)(&_x == &_y); \
                                _x > _y ? _x : _y;})

typedef struct {
  int             fd;
  pthread_mutex_t mutex;
} LogFile;

static LogConfig g_log_config = {1, 1, 1, 1, 0, N_LOG_ALL};
static LogFile   g_log_file;

static const char* _level_tostring(LogLevel level) {
  switch (level) {
    case N_LOG_ERROR: return g_log_config.enable_color ?
                             "\033[0m\033[41;33m[E]\033[0m" : "[E]";
    case N_LOG_DEBUG: return g_log_config.enable_color ?
                             "\033[0m\033[47;33m[D]\033[0m" : "[D]";
    case N_LOG_TRACK: return g_log_config.enable_color ?
                             "\033[0m\033[42;33m[T]\033[0m" : "[T]";
    case N_LOG_WARN:  return g_log_config.enable_color ?
                             "\033[0m\033[41;33m[W]\033[0m" : "[W]";
    default:          return "[N/A]";
  }
}

static void _get_now_str(char *buf, int len) {
  struct timeval tv;
  time_t s;
  struct tm local;
  gettimeofday(&tv, NULL);
  s = tv.tv_sec;
  localtime_r(&s, &local);
  snprintf(buf, len, "%02d:%02d:%02d.%06"PRId64" ", local.tm_hour,
           local.tm_min, local.tm_sec, (int64_t)tv.tv_usec);
}

static void _get_thread_id_str(char *buf, int len) {
  pthread_t thread_id = pthread_self();
  snprintf(buf, len, "%x", (unsigned int)thread_id);
}

static int _fill_log_level(LogLevel level, char *buf, int len) {
  int write_len = 0;
  switch (level) {
    case N_LOG_DEBUG:
      write_len = snprintf(buf, len, "%s ", _level_tostring(N_LOG_DEBUG));
      break;
    case N_LOG_TRACK:
      write_len = snprintf(buf, len, "%s ", _level_tostring(N_LOG_TRACK));
      break;
    case N_LOG_WARN:
      write_len = snprintf(buf, len, "%s ", _level_tostring(N_LOG_WARN));
      break;
    case N_LOG_ERROR:
      write_len = snprintf(buf, len, "%s ", _level_tostring(N_LOG_ERROR));
      break;
    default:
      break;
  }
  return uni_max(0, write_len);
}

static int _fill_tag(char *buf, int len, const char *tag) {
  return uni_max(0, snprintf(buf, len, "<%s>", tag));
}

static int _fill_time(char *buf, int len) {
  char now[64];
  if (!g_log_config.enable_time) {
    return 0;
  }
  _get_now_str(now, sizeof(now));
  return uni_max(0, snprintf(buf, len, "%s", now));
}

static int _fill_function_line(char *buf, int len, const char *function,
                               int line) {
  return (g_log_config.enable_function_line ?
          uni_max(0, snprintf(buf, len, "%s:%d->", function, line)) : 0);
}

static int _fill_thread_id(char *buf, int len) {
  char thread_id[32];
  if (!g_log_config.enable_thread_id) {
    return 0;
  }
  _get_thread_id_str(thread_id, sizeof(thread_id));
  return uni_max(0, snprintf(buf, len, "%s", thread_id));
}

static void _fill_customer_info(char *buf, int len, char *fmt, va_list args,
                                int append_feed_line) {
  int length, remain_len;
  length = vsnprintf(buf, len, fmt, args);
  length = uni_max(length, 0);
  length = uni_min(length, len);
  remain_len = len - length;
  if (0 == remain_len) {
    if (append_feed_line) {
      buf[len - 2] = '\n';
    }
    buf[len - 1] = '\0';
    return;
  }
  if (1 == remain_len) {
    if (append_feed_line) {
      buf[len - 2] = '\n';
    }
    return;
  }
  if (append_feed_line) {
    strncat(buf, "\n", remain_len);
  }
  return;
}

static void _save_log_2_file(char *buf, int len) {
  if (0 < g_log_file.fd && 0 < len) {
    pthread_mutex_lock(&g_log_file.mutex);
    write(g_log_file.fd, buf, len);
    pthread_mutex_unlock(&g_log_file.mutex);
  }
}

int LogLevelValid(LogLevel level) {
  return level <= g_log_config.set_level ? 1 : 0;
}

#define _log_assemble(buf, level, tags, function, line, fmt, args) do { \
  int len = 0; \
  if (level != N_LOG_RAW) { \
    len += _fill_log_level(level, buf + len, LOG_BUFFER_LEN - len); \
    len += _fill_time(buf + len, LOG_BUFFER_LEN - len); \
    len += _fill_thread_id(buf + len, LOG_BUFFER_LEN - len); \
    len += _fill_tag(buf + len, LOG_BUFFER_LEN - len, tags); \
    len += _fill_function_line(buf + len, LOG_BUFFER_LEN - len, \
                               function, line); \
  } \
  _fill_customer_info(buf + len, LOG_BUFFER_LEN - len, fmt, args, \
                      level != N_LOG_RAW); \
} while (0)

static int _sync_write_process(LogLevel level, const char *tags,
                               const char *function, int line,
                               char *fmt, va_list args) {
  char buf[LOG_BUFFER_LEN];
  _log_assemble(buf, level, tags, function, line, fmt, args);
  printf("%s", buf);
  _save_log_2_file(buf, strlen(buf) + 1);
  return 0;
}

int LogWrite(LogLevel level, const char *tags, const char *function, int line,
             char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  _sync_write_process(level, tags, function, line, fmt, args);
  va_end(args);
  return 0;
}

static void _destroy_all() {
  if (g_log_config.enable_file) {
    pthread_mutex_destroy(&g_log_file.mutex);
  }
}

int LogLevelSet(LogLevel level) {
  g_log_config.set_level = level;
  return 0;
}

static void _open_save_fd() {
  g_log_file.fd = open(LOG_FILE_NAME, O_WRONLY | O_CREAT, 0664);
  if (g_log_file.fd <= 0) {
    printf("%s%d: open save fd[%d] failed.\n", __FUNCTION__, __LINE__,
           g_log_file.fd);
  }
}

int LogInitialize(LogConfig logConfig) {
  g_log_config.enable_time = logConfig.enable_time;
  g_log_config.enable_thread_id = logConfig.enable_thread_id;
  g_log_config.enable_function_line = logConfig.enable_function_line;
  g_log_config.enable_color = logConfig.enable_color;
  g_log_config.enable_file = logConfig.enable_file;
  g_log_config.set_level = logConfig.set_level;
  if (g_log_config.enable_file) {
    pthread_mutex_init(&g_log_file.mutex, NULL);
    _open_save_fd();
  }
  return 0;
}

int LogFinalize(void) {
  _destroy_all();
  return 0;
}
