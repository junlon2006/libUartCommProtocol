/**************************************************************************
 * Copyright (C) 2017-2017  Junlon2006
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
 * Description : uni_communication.c
 * Author      : junlon2006@163.com
 * Date        : 2019.12.27
 *
 **************************************************************************/
#include "uni_communication.h"

#include "uni_log.h"
#include "uni_crc16.h"
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/time.h>

#define UART_COMM_TAG                 "uart_comm"
#define UNI_COMM_SYNC_VALUE           (0xFF)

#define DEFAULT_PROTOCOL_BUF_SIZE     (16)
#define PROTOCOL_BUF_GC_TRIGGER_SIZE  (256)
#define PROTOCOL_BUF_SUPPORT_MAX_SIZE (8192)

#define WAIT_ACK_TIMEOUT_MSEC         (150)
/* make sure ONE_FRAME_BYTE_TIMEOUT_MSEC < WAIT_ACK_TIMEOUT_MSEC
 * otherwise resend cannot work, set
 * WAIT_ACK_TIMEOUT_MSEC = 1.5 * ONE_FRAME_BYTE_TIMEOUT_MSEC */
#define ONE_FRAME_BYTE_TIMEOUT_MSEC   (100)
#define TRY_RESEND_TIMES              (2)

#define uni_min(x, y)                 (x < y ? x : y)
#define uni_max(x, y)                 (x > y ? x : y)
#define uni_malloc                    malloc
#define uni_free                      free
#define false                         0
#define true                          1

/*--------------------------------------------------------------------------------*/
/*                   layout of uart communication app protocol                    */
/*--------------------------------------------------------------------------------*/
/*-1byte-|-1byte-|-1byte-|-1byte-|---2byte---|---2byte---|---2byte---|---N byte---*/
/* SYNC  |  seq  | type  |  ctrl |  command  | checksum  |payload len|   payload  */
/*--------------------------------------------------------------------------------*/

/*--------------------------------ack frame---------------------------------------*/
/*  0xFF | seqNum|  0x0  |  0x0  |    0x0    |   crc16   |    0x0    |    NULL    */
/*--------------------------------------------------------------------------------*/

/*---------------------------------*/
/*-------------control-------------*/
/*| 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 |*/
/*|RES|RES|RES|RES|RES|RES|RES|ACK|*/
/*---------------------------------*/

typedef unsigned short CommChecksum;
typedef unsigned char  CommSync;
typedef unsigned char  CommSequence;
typedef unsigned char  CommControl;

typedef enum {
  ACK = 0,
} Control;

typedef enum {
  LAYOUT_SYNC_IDX             = 0,
  LAYOUT_PAYLOAD_LEN_LOW_IDX  = 8,
  LAYOUT_PAYLOAD_LEN_HIGH_IDX = 9,
} CommLayoutIndex;

typedef struct {
  CommSync       sync;      /* must be UNI_COMM_SYNC_VALUE */
  CommSequence   sequence;  /* sequence number */
  CommType       type;      /* customer type */
  CommControl    control;   /* header ctrl */
  CommCmd        cmd;       /* command type, such as power on, power off etc */
  CommChecksum   checksum;      /* checksum of packet, use crc16 */
  CommPayloadLen payload_len;   /* the length of payload */
  char           payload[0];    /* the payload */
} PACKED CommProtocolPacket;

typedef struct {
  CommWriteHandler      on_write;
  CommRecvPacketHandler on_recv_frame;
  pthread_mutex_t       mutex;
  uni_bool              acked;
  CommSequence          sequence;
  char                  *protocol_buffer;
} CommProtocolBusiness;

static CommProtocolBusiness g_comm_protocol_business;

static void _register_write_handler(CommWriteHandler handler) {
  g_comm_protocol_business.on_write = handler;
}

static void _unregister_write_handler() {
  g_comm_protocol_business.on_write = NULL;
}

static void _sync_set(CommProtocolPacket *packet) {
  packet->sync = UNI_COMM_SYNC_VALUE;
}

static void _sequence_set(CommProtocolPacket *packet,
                          CommSequence seq,
                          uni_bool is_ack_packet) {
  if (is_ack_packet) {
    packet->sequence = seq;
  } else {
    packet->sequence = g_comm_protocol_business.sequence++;
  }
}

static CommSequence _current_sequence_get() {
  return g_comm_protocol_business.sequence - 1;
}

static void _product_type_set(CommProtocolPacket *packet, CommType type) {
  packet->type = type;
}

static void _bit_set(CommControl *control, int index) {
  *control |= (1 << index);
}

static uni_bool _is_bit_setted(CommControl control, int index) {
  return (control >> index) & 0x1;
}

static void _set_ack(CommProtocolPacket *packet) {
  _bit_set(&packet->control, ACK);
}

static uni_bool _is_ack_set(CommControl control) {
  return _is_bit_setted(control, ACK);
}

static void _control_set(CommProtocolPacket *packet, uni_bool reliable) {
  if (reliable) {
    _set_ack(packet);
  }
}

CommType uni_comm_protocol_product_type_get(CommProtocolPacket *packet,
                                            CommType type) {
  return packet->type;
}

static void _cmd_set(CommProtocolPacket *packet, CommCmd cmd) {
  packet->cmd = cmd;
}

static void _payload_len_set(CommProtocolPacket *packet,
                             CommPayloadLen payload_len) {
  packet->payload_len = payload_len;
}

static CommPayloadLen _payload_len_get(CommProtocolPacket *packet) {
  return packet->payload_len;
}

static void _payload_set(CommProtocolPacket *packet,
                         char *buf, CommPayloadLen len) {
  if (NULL != buf && 0 < len) {
    memcpy(packet->payload, buf, len);
  }
}

static char* _payload_get(CommProtocolPacket *packet) {
  return packet->payload;
}

static CommPayloadLen _packet_len_get(CommProtocolPacket *packet) {
  return sizeof(CommProtocolPacket) + packet->payload_len;
}

static void _checksum_calc(CommProtocolPacket *packet) {
  packet->checksum = 0; /* make sure the checksum be zero before calculate */
  packet->checksum = crc16((const char*)packet, _packet_len_get(packet));
}

static int _checksum_valid(CommProtocolPacket *packet) {
  CommChecksum checksum = packet->checksum; /* get the checksum from packet */
  _checksum_calc(packet); /* calc checksum again */
  return checksum == packet->checksum; /* check whether checksum valid or not */
}

static void _unset_acked_sync_flag() {
  g_comm_protocol_business.acked = false;
}

static void _set_acked_sync_flag() {
  g_comm_protocol_business.acked = true;
}

static uni_bool _is_acked_packet(CommProtocolPacket *protocol_packet) {
  return (protocol_packet->type == 0 &&
          protocol_packet->cmd == 0 &&
          protocol_packet->sequence == _current_sequence_get() &&
          protocol_packet->payload_len == 0);
}

static int _wait_ack(CommAttribute *attribute) {
  /* acked process */
  int timeout = WAIT_ACK_TIMEOUT_MSEC;
  if (NULL == attribute || !attribute->reliable) {
    return 0;
  }
  //TODO stupid timeout, use select perf???
  while (timeout > 0) {
    timeout -= 5;
    usleep(5 * 1000);
    if (g_comm_protocol_business.acked) {
      break;
    }
  }
  if (!g_comm_protocol_business.acked) {
    LOGW(UART_COMM_TAG, "wait uart ack timeout");
  }
  return g_comm_protocol_business.acked ? 0 : E_UNI_COMM_PAYLOAD_ACK_TIMEOUT;
}

static CommProtocolPacket* _packet_alloc(int payload_len) {
  return (CommProtocolPacket *)uni_malloc(sizeof(CommProtocolPacket) +
                                          payload_len);
}

static void _packet_free(CommProtocolPacket *packet) {
  uni_free(packet);
}

#define RESENDING  (1)
static int _resend_status(CommAttribute *attribute, int *resend_times) {
  int ret = _wait_ack(attribute);
  if (0 == ret) {
    return 0;
  }
  if (*resend_times > 0) {
    *resend_times = *resend_times - 1;
    return RESENDING;
  }
  return ret;
}

static int _write_uart(CommProtocolPacket *packet, CommAttribute *attribute) {
  int ret = 0;
  int resend_times = TRY_RESEND_TIMES;
  if (NULL != g_comm_protocol_business.on_write) {
    /* sync uart write, we use mutex lock */
    pthread_mutex_lock(&g_comm_protocol_business.mutex);
    _unset_acked_sync_flag();
    do {
      g_comm_protocol_business.on_write((char *)packet,
                                        (int)_packet_len_get(packet));
      LOGD(UART_COMM_TAG, "resend times=%d", resend_times);
      ret = _resend_status(attribute, &resend_times);
    } while (RESENDING == ret);
    pthread_mutex_unlock(&g_comm_protocol_business.mutex);
  }
  return ret;
}

static void _assmeble_packet(CommProtocolPacket *packet,
                             CommType type,
                             CommCmd cmd,
                             char *payload,
                             CommPayloadLen payload_len,
                             uni_bool reliable,
                             CommSequence seq,
                             uni_bool is_ack_packet) {
  _sync_set(packet);
  _sequence_set(packet, seq, is_ack_packet);
  _product_type_set(packet, type);
  _control_set(packet, reliable);
  _cmd_set(packet, cmd);
  _payload_set(packet, payload, payload_len);
  _payload_len_set(packet, payload_len);
  _checksum_calc(packet);
}

static uni_bool _is_protocol_buffer_overflow(int length) {
  return length >= PROTOCOL_BUF_SUPPORT_MAX_SIZE;
}

static int _assemble_and_send_frame(CommType type,
                                    CommCmd cmd,
                                    char *payload,
                                    CommPayloadLen payload_len,
                                    CommAttribute *attribute,
                                    CommSequence seq,
                                    uni_bool is_ack_packet) {
  int ret = 0;
  if (_is_protocol_buffer_overflow(sizeof(CommProtocolPacket) +
                                   payload_len)) {
    return E_UNI_COMM_PAYLOAD_TOO_LONG;
  }
  CommProtocolPacket *packet = _packet_alloc(payload_len);
  if (NULL == packet) {
    return E_UNI_COMM_ALLOC_FAILED;
  }
  _assmeble_packet(packet, type, cmd, payload, payload_len,
                   attribute && attribute->reliable,
                   seq, is_ack_packet);
   ret = _write_uart(packet, attribute);
  _packet_free(packet);
  return ret;
}

int CommProtocolPacketAssembleAndSend(CommType type, CommCmd cmd,
                                      char *payload,
                                      CommPayloadLen payload_len,
                                      CommAttribute *attribute) {
  return _assemble_and_send_frame(type, cmd, payload, payload_len,
                                  attribute, 0, false);
}

static CommPacket* _packet_disassemble(CommProtocolPacket *protocol_packet) {
  CommPacket *packet;
  if (!_checksum_valid(protocol_packet)) {
    LOGE(UART_COMM_TAG, "checksum failed");
    return NULL;
  }
  packet = uni_malloc(sizeof(CommPacket) + _payload_len_get(protocol_packet));
  if (NULL == packet) {
    LOGE(UART_COMM_TAG, "alloc memory failed");
    return NULL;
  }
  packet->cmd = protocol_packet->cmd;
  packet->payload_len = _payload_len_get(protocol_packet);
  memcpy(packet->payload, _payload_get(protocol_packet),
         _payload_len_get(protocol_packet));
  return packet;
}

static void _enlarge_protocol_buffer(char **orginal, int *orginal_len) {
  char *p;
  int new_length = *orginal_len * 2;
  p = uni_malloc(new_length);
  memcpy(p, *orginal, *orginal_len);
  uni_free(*orginal);
  *orginal = p;
  *orginal_len = new_length;
}

/* small heap memory stays alway, only garbage collection big bins*/
static void _try_garbage_collection_protocol_buffer(char **buffer,
                                                    int *length) {
  if (*length >= PROTOCOL_BUF_GC_TRIGGER_SIZE) {
    uni_free(*buffer);
    *buffer = NULL;
    *length = DEFAULT_PROTOCOL_BUF_SIZE;
    LOGT(UART_COMM_TAG, "free buffer=%p, len=%d", *buffer, *length);
  }
}

static void _reset_protocol_buffer_status(int *index, int *length) {
  *index = 0;
  *length = 0;
}

static void _protocol_buffer_alloc(char **buffer, int *length, int index) {
  if (NULL == *buffer) {
    *buffer = uni_malloc(*length);
    LOGD(UART_COMM_TAG, "init buffer=%p, len=%d", *buffer, *length);
    return;
  }
  if (*length <= index) {
    _enlarge_protocol_buffer(buffer, length);
    LOGD(UART_COMM_TAG, "protocol buffer enlarge. p=%p, new len=%d",
         *buffer, *length);
    return;
  }
}

static void _send_ack_frame(CommSequence seq) {
  _assemble_and_send_frame(0, 0, NULL, 0, NULL, seq, true);
  LOGD(UART_COMM_TAG, "send ack seq=%d", seq);
}

static void _do_ack(CommProtocolPacket *protocol_packet) {
  if (_is_ack_set(protocol_packet->control)) {
    _send_ack_frame(protocol_packet->sequence);
  }
}

static uni_bool _is_duplicate_frame(CommProtocolPacket *protocol_packet) {
  static int last_recv_packet_seq = -1;
  uni_bool duplicate;
  duplicate = (last_recv_packet_seq == (int)protocol_packet->sequence);
  last_recv_packet_seq = protocol_packet->sequence;
  LOGD(UART_COMM_TAG, "duplicate=%d", duplicate);
  return duplicate;
}

static void _one_protocol_frame_process(char *protocol_buffer) {
  CommProtocolPacket *protocol_packet = (CommProtocolPacket *)protocol_buffer;
  /* when application not register hook, ignore all*/
  if (NULL == g_comm_protocol_business.on_recv_frame) {
    LOGW(UART_COMM_TAG, "donot register recv_frame hook");
    return;
  }
  /* ack frame donnot notify application, ignore it now */
  if (_is_acked_packet(protocol_packet)) {
    LOGT(UART_COMM_TAG, "recv ack frame");
    _set_acked_sync_flag();
    return;
  }
  /* disassemble protocol buffer */
  CommPacket* packet = _packet_disassemble(protocol_packet);
  if (NULL == packet) {
    LOGW(UART_COMM_TAG, "disassemble packet failed");
    return;
  }
  /* ack automatically when ack attribute set */
  _do_ack(protocol_packet);
  /* notify application when not ack frame nor duplicate frame */
  if (!_is_duplicate_frame(protocol_packet)) {
    g_comm_protocol_business.on_recv_frame(packet);
  }
  uni_free(packet);
}

static long _get_clock_time_ms(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
    return (ts.tv_sec * 1000L + (ts.tv_nsec / 1000000));
  }
  return 0;
}

static uni_bool _bytes_coming_speed_too_slow(int index) {
  static long last_byte_coming_timestamp = 0;
  long now = _get_clock_time_ms();
  uni_bool timeout = false;
  if (now - last_byte_coming_timestamp > ONE_FRAME_BYTE_TIMEOUT_MSEC && /* lost one check when overflow */
      LAYOUT_SYNC_IDX != index) {
    timeout = true;
    LOGW(UART_COMM_TAG, "[%u->%u]", last_byte_coming_timestamp, now);
  }
  last_byte_coming_timestamp = now;
  return timeout;
}

static void _protocol_buffer_generate_byte_by_byte(char recv_c) {
  static int index = 0;
  static int length = 0;
  static int protocol_buffer_length = DEFAULT_PROTOCOL_BUF_SIZE;
  /* check timestamp to reset status when physical error */
  if (_bytes_coming_speed_too_slow(index)) {
    LOGT(UART_COMM_TAG, "reset protocol buffer automatically[%d]", index);
    _reset_protocol_buffer_status(&index, &length);
    _try_garbage_collection_protocol_buffer( \
        &g_comm_protocol_business.protocol_buffer, &protocol_buffer_length);
  }
  /* protect heap use, cannot alloc large than 8K now */
  if (_is_protocol_buffer_overflow(protocol_buffer_length)) {
    /* drop remain bytes of this frame*/
    if (length > 1) {
      length--;
      return;
    }
    _reset_protocol_buffer_status(&index, &length);
    _try_garbage_collection_protocol_buffer( \
        &g_comm_protocol_business.protocol_buffer, &protocol_buffer_length);
    LOGW(UART_COMM_TAG, "recv invalid frame, payload too long");
    return;
  }
  _protocol_buffer_alloc(&g_comm_protocol_business.protocol_buffer,
                         &protocol_buffer_length, index);
  /* get frame header sync byte */
  if (LAYOUT_SYNC_IDX == index) {
    if (UNI_COMM_SYNC_VALUE == (unsigned char)recv_c) {
      g_comm_protocol_business.protocol_buffer[index++] = recv_c;
    } else {
      LOGW(UART_COMM_TAG, "nonstandord sync byte, please check");
    }
    return;
  }
  /* get payload length (low 8 bit)*/
  if (LAYOUT_PAYLOAD_LEN_LOW_IDX == index) {
    length = recv_c;
    LOGD(UART_COMM_TAG, "len low=%d", length);
  }
  /* get payload length (high 8 bit)*/
  if (LAYOUT_PAYLOAD_LEN_HIGH_IDX == index) {
    length += (((unsigned short)recv_c) << 8);
    LOGD(UART_COMM_TAG, "length=%d", length);
  }
  /* set protocol header */
  if (index < sizeof(CommProtocolPacket)) {
    g_comm_protocol_business.protocol_buffer[index++] = recv_c;
    goto L_END;
  }
  /* set protocol payload */
  if (sizeof(CommProtocolPacket) <= index && 0 < length) {
    g_comm_protocol_business.protocol_buffer[index++] = recv_c;
    length--;
  }
L_END:
  /* callback protocol buffer */
  if (sizeof(CommProtocolPacket) <= index && 0 == length) {
    LOGD(UART_COMM_TAG, "assemble new frame, now callback");
    _one_protocol_frame_process(g_comm_protocol_business.protocol_buffer);
    _reset_protocol_buffer_status(&index, &length);
    _try_garbage_collection_protocol_buffer( \
        &g_comm_protocol_business.protocol_buffer, &protocol_buffer_length);
  }
}

void CommProtocolReceiveUartData(char *buf, int len) {
  for (int i = 0; i < len; i++) {
    _protocol_buffer_generate_byte_by_byte(buf[i]);
  }
}

static void _register_packet_receive_handler(CommRecvPacketHandler handler) {
  g_comm_protocol_business.on_recv_frame = handler;
}

static void _unregister_packet_receive_handler() {
  g_comm_protocol_business.on_recv_frame = NULL;
}

static void _protocol_business_init() {
  memset(&g_comm_protocol_business, 0, sizeof(g_comm_protocol_business));
  pthread_mutex_init(&g_comm_protocol_business.mutex, NULL);
}

static void _try_free_protocol_buffer() {
  if (NULL != g_comm_protocol_business.protocol_buffer) {
    uni_free(g_comm_protocol_business.protocol_buffer);
    g_comm_protocol_business.protocol_buffer = NULL;
  }
}

static void _protocol_business_final() {
  pthread_mutex_destroy(&g_comm_protocol_business.mutex);
  _try_free_protocol_buffer();
  memset(&g_comm_protocol_business, 0, sizeof(g_comm_protocol_business));
}

int CommProtocolInit(CommWriteHandler write_handler,
                     CommRecvPacketHandler recv_handler) {
  _protocol_business_init();
  _register_write_handler(write_handler);
  _register_packet_receive_handler(recv_handler);
  return 0;
}

void CommProtocolFinal() {
  _unregister_packet_receive_handler();
  _unregister_write_handler();
  _protocol_business_final();
}
