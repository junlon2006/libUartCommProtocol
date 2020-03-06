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
 * Description : uni_interruptable.c
 * Author      : junlon2006@163.com
 * Date        : 2019.03.17
 *
 **************************************************************************/
#include "uni_interruptable.h"

#include <sys/select.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#define uni_min(x,y)         (x > y ? y : x)
#define uni_max(x,y)         (x > y ? x : y)

typedef struct {
  int fd[2];
  int flag;
} Interruptable;

static int _socket_set_block_mode(int sockfd, int block) {
  int flags = fcntl(sockfd, F_GETFL);
  if (flags < 0) {
    return -1;
  }
  if (block) {
    if ((flags & O_NONBLOCK) != 0) {
      flags ^= O_NONBLOCK;
    }
  } else {
    flags |= O_NONBLOCK;
  }
  if (fcntl(sockfd, F_SETFL, flags) < 0) {
    return -1;
  }
  return 0;
}

static int _socket_set_nonblocking(int sockfd) {
  return _socket_set_block_mode(sockfd, 0);
}

static int _async_pipe(int fildes[2]) {
  if (0 != pipe(fildes)) {
    return -1;
  }
  if (0 != _socket_set_nonblocking(fildes[0])) {
    return -1;
  }
  if (0 != _socket_set_nonblocking(fildes[1])) {
    return -1;
  }
  return 0;
}

static int _select(int maxfd, fd_set *readfds, fd_set *writefds,
                   fd_set *errorfds, int timeout_ms) {
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  return select(maxfd + 1, readfds, writefds, errorfds,
                timeout_ms < 0 ? NULL : &tv);
}

static int _interruptable_select(Interruptable *interrupter, int maxfd,
                                 fd_set *readfds, fd_set *writefds,
                                 fd_set *errorfds, int timeout_ms) {
  unsigned char t[64];
  interrupter->flag = 0;
  maxfd = uni_max(maxfd, interrupter->fd[0]);
  FD_SET(interrupter->fd[0], readfds);
  if (_select(maxfd, readfds, writefds, errorfds, timeout_ms) < 0) {
    return -1;
  }
  if (FD_ISSET(interrupter->fd[0], readfds)) {
    if (0 > read(interrupter->fd[0], t, sizeof(t))) {
      return -1;
    }
    interrupter->flag = 1;
  }
  return 0;
}

InterruptHandle InterruptCreate() {
  Interruptable *interrupter = NULL;
  interrupter = (Interruptable *)malloc(sizeof(Interruptable));
  if (NULL == interrupter) {
    return NULL;
  }
  if (0 != _async_pipe(interrupter->fd)) {
    free(interrupter);
    return NULL;
  }
  interrupter->flag = 0;
  return (InterruptHandle)interrupter;
}

int InterruptDestroy(InterruptHandle handle) {
  Interruptable *interrupter = (Interruptable *)handle;
  close(interrupter->fd[0]);
  close(interrupter->fd[1]);
  free(interrupter);
  return 0;
}

int InterruptableSleep(InterruptHandle handle, int sleep_msec) {
  Interruptable *interrupter = (Interruptable *)handle;
  fd_set readfds;
  if (sleep_msec < 0) {
    printf("%s%d: invalid input %d", __FUNCTION__, __LINE__, sleep_msec);
    return -1;
  }
  FD_ZERO(&readfds);
  _interruptable_select(interrupter, 0, &readfds, NULL, NULL, sleep_msec);
  return interrupter->flag;
}

int InterruptableBreak(InterruptHandle handle) {
  Interruptable *interrupter = (Interruptable *)handle;
  char c[1] = {0x5A};
  return (write(interrupter->fd[1], c, sizeof(c)) == sizeof(c) ? 0 : -1);
}
