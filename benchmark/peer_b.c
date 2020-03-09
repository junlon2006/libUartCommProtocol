#include "uni_communication.h"
#include "uni_log.h"
#include "uni_interruptable.h"

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
#include <string.h>
#include <assert.h>

#define TAG                         "peer-b"
#define FIFO_UART_MOCK_READ         "/tmp/uart-mock-b"
#define FIFO_UART_MOCK_WRITE        "/tmp/uart-mock-a"
#define TRANSMISSION_ERROR_PER_BITS (1000000)
#define BAUD_RATE                   (921600)

typedef struct {
  int  seq;
  char buf[512];
} UserData;

static int fd_R = -1;
static int fd_T = -1;

static InterruptHandle interrupt_handle;

static int _uart_write_mock_api(char *buf, int len) {
  static int total_len = 0;
  static char clone[8192];
  unsigned int byte_idx;
  unsigned int bit_idx;
  int sleep_msec;

  memcpy(clone, buf, len);

  for (int i = 0; i < len; i++) {
    LOGR(TAG, "0x%0x, ", (unsigned char)clone[i]);
  }
  LOGR(TAG, "\n");

  total_len += (len << 3);
  if (total_len >= TRANSMISSION_ERROR_PER_BITS) {
    total_len = 0;

    LOGT(TAG, "random reverse one bit");

    byte_idx = rand() % len;
    bit_idx = rand() & 7;

    clone[byte_idx] ^= (1 << bit_idx);
  }

  if (len != write(fd_T, clone, len)) {
    LOGE(TAG, "write failed");
  }

  sleep_msec = len * 8 * 1000 / BAUD_RATE;
  sleep_msec += 1;
  InterruptableSleep(interrupt_handle, sleep_msec);

  return len;
}

static int64_t _get_now_msec(void) {
  struct timeval t1;
  gettimeofday(&t1, NULL);
  return ((int64_t)t1.tv_sec * 1000 + t1.tv_usec / 1000);
}

static void _recv_comm_packet(CommPacket *packet) {
  static int64_t start_time = _get_now_msec();
  static int64_t start = _get_now_msec();
  int64_t now, cost;
  float avg_speed;
  static int64_t total_len = 0;
  static int seq = 0;

  LOGT(TAG, "recv frame... cmd=%d, len=%d", packet->cmd, packet->payload_len);

  UserData *user_data = (UserData*)packet->payload;
  assert(user_data->seq == ++seq);
  total_len += packet->payload_len;
  now = _get_now_msec();
  if (now - start > 1000) {
    cost = (now - start_time) / 1000;
    avg_speed = total_len / (float)(now - start_time) * 1000 / 1024;
    LOGW(TAG, "[%d:ER%d] total=%dKB, cost=%d-%02d:%02d:%02d, speed=%.2fKB/s, BW RATIO=%.2f%%",
         BAUD_RATE,
         TRANSMISSION_ERROR_PER_BITS,
         total_len >> 10,
         cost / (3600 * 24),
         cost % (3600 * 24) / 3600,
         cost % (3600 * 24) % 3600 / 60,
         cost % (3600 * 24) % 3600 % 60,
         avg_speed, avg_speed / (BAUD_RATE >> 13) * 100);
    start = now;
  }
}

static void* __recv_task(void *args) {
  int read_len;
  unsigned char buf[1024];

  while (1) {
    read_len = read(fd_R, buf, sizeof(buf));
    CommProtocolReceiveUartData(buf, read_len);
  }

  return NULL;
}

int main() {
  LogLevelSet(N_LOG_WARN);

  interrupt_handle = InterruptCreate();

  mkfifo(FIFO_UART_MOCK_READ, 0644);
  mkfifo(FIFO_UART_MOCK_WRITE, 0644);

  fd_T = open(FIFO_UART_MOCK_WRITE, O_WRONLY);
  fd_R = open(FIFO_UART_MOCK_READ, O_RDONLY);

  signal(SIGPIPE, SIG_IGN);

  CommProtocolInit(_uart_write_mock_api, _recv_comm_packet);

  UserData user_data = {0};

  CommAttribute attr;
  attr.reliable = 1;

  pthread_t pid;
  pthread_create(&pid, NULL, __recv_task, NULL);

  LOGT(TAG, "peer b start...");

  while (1) {
    user_data.seq++;
    CommProtocolPacketAssembleAndSend(1, (char *)&user_data, sizeof(user_data), &attr);
  }

  return 0;
}
