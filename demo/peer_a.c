#include "uni_communication.h"
#include "uni_log.h"

#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <pthread.h>

#define TAG "client"
#define FIFO_UART_MOCK_READ "/tmp/uart-mock-a"
#define FIFO_UART_MOCK_WRITE "/tmp/uart-mock-b"

static int fd_R = -1;
static int fd_T = -1;

static int _uart_write_mock_api(char *buf, int len) {
  LOGT(TAG, "send data...");
  for (int i = 0; i < len; i++) {
    LOGR(TAG, "0x%0x, ", (unsigned char)buf[i]);
  }
  LOGR(TAG, "\n");

  if (len != write(fd_T, buf, len)) {
    LOGE(TAG, "write failed");
  }

  return len;
}

static void _recv_comm_packet(CommPacket *packet) {
  LOGT(TAG, "recv frame... cmd=%d, len=%d", packet->cmd, packet->payload_len);
  return;
}

static int64_t _get_now_msec(void) {
  struct timeval t1;
  gettimeofday(&t1, NULL);
  return ((int64_t)t1.tv_sec * 1000 + t1.tv_usec/1000);
}

static void* __recv_task(void *args) {
  int read_len;
  unsigned char buf[1024];

  int64_t start_time = _get_now_msec();
  int64_t start, now;
  int64_t total_len = 0;
  start = start_time;

  while (1) {
    read_len = read(fd_R, buf, sizeof(buf));
    total_len += read_len;
    CommProtocolReceiveUartData(buf, read_len);

    now = _get_now_msec();
    if (now - start >= 1000) {
      LOGW(TAG, "total_len=%dKB, cost=%ds, avg_speed=%dKB/s", total_len / 1024,
           (now - start_time) / 1000, total_len / (now - start_time) * 1000 / 1024);
      start = now;
    }
  }

  return NULL;
}

int main() {
  LogLevelSet(N_LOG_WARN);

  mkfifo(FIFO_UART_MOCK_READ, 0644);
  mkfifo(FIFO_UART_MOCK_WRITE, 0644);

  fd_R = open(FIFO_UART_MOCK_READ, O_RDONLY);
  fd_T = open(FIFO_UART_MOCK_WRITE, O_WRONLY);

  signal(SIGPIPE, SIG_IGN);

  CommProtocolInit(_uart_write_mock_api, _recv_comm_packet);

  char buf[512];
  CommAttribute attr;
  attr.reliable = 1;

  pthread_t pid;
  pthread_create(&pid, NULL, __recv_task, NULL);

  LOGT(TAG, "peer a start...");
  while (1) {
    CommProtocolPacketAssembleAndSend(1, buf, sizeof(buf), &attr);
  }

  return 0;
}
