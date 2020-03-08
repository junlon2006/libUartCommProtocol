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
#include "uni_interruptable.h"

#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/time.h>

#define UART_COMM_TAG                 "uart_comm"

#define DEFAULT_PROTOCOL_BUF_SIZE     (16)
#define PROTOCOL_BUF_GC_TRIGGER_SIZE  (2048)
#define PROTOCOL_BUF_SUPPORT_MAX_SIZE (8192)

//TODO need refactor
#define WAIT_ACK_TIMEOUT_MSEC         (20)
/* make sure ONE_FRAME_BYTE_TIMEOUT_MSEC < WAIT_ACK_TIMEOUT_MSEC
 * otherwise resend cannot work, set
 * WAIT_ACK_TIMEOUT_MSEC = 1.5 * ONE_FRAME_BYTE_TIMEOUT_MSEC */
#define ONE_FRAME_BYTE_TIMEOUT_MSEC   (10)
#define TRY_RESEND_TIMES              (2)

#define uni_min(x, y)                 (x < y ? x : y)
#define uni_max(x, y)                 (x > y ? x : y)
#define uni_malloc                    malloc
#define uni_free                      free
#define false                         0
#define true                          1

/*-----------------------------------------------------------------*/
/*           layout of uart communication app protocol             */
/*-----------------------------------------------------------------*/
/*--6byte-|-1byte-|-1byte-|-2byte-|-2byte-|-2byte-|-2byte-|-N byte-*/
/*"uArTcP"|  seq  |  ctrl |  cmd  | crc16 |  len  |cs(len)|payload */
/*-----------------------------------------------------------------*/

/*---------------------------ack frame-----------------------------*/
/*"uArTcP"|  seq  |  0x0  |  0x0  | crc16 |  0x0  |  0x0  |  NULL  */
/*-----------------------------------------------------------------*/

/*------------------------------------*/
/*--------------control---------------*/
/*| 8 | 7 | 6 | 5 | 4 | 3  |  2  | 1 |*/
/*|RES|RES|RES|RES|RES|NACK|ACKED|ACK|*/
/*------------------------------------*/

typedef unsigned short CommChecksum;
typedef unsigned char  CommSync;
typedef unsigned char  CommSequence;
typedef unsigned char  CommControl;

typedef enum {
  ACK   = 0,  /* need ack */
  ACKED = 1,  /* ack packet */
  NACK  = 2,  /* nack packet */
} Control;

typedef enum {
  LAYOUT_SYNC_IDX                 = 0,
  LAYOUT_PAYLOAD_LEN_LOW_IDX      = 12,
  LAYOUT_PAYLOAD_LEN_HIGH_IDX     = 13,
  LAYOUT_PAYLOAD_LEN_CRC_LOW_IDX  = 14,
  LAYOUT_PAYLOAD_LEN_CRC_HIGH_IDX = 15,
} CommLayoutIndex;

typedef struct {
  unsigned char  sync[6];   /* must be "uArTcP" */
  CommSequence   sequence;  /* sequence number */
  CommControl    control;   /* header ctrl */
  CommCmd        cmd;       /* command type, such as power on, power off etc */
  CommChecksum   checksum;         /* checksum of packet, use crc16 */
  CommPayloadLen payload_len;      /* the length of payload */
  CommChecksum   payload_len_crc16;/* the crc16 of payload_len */
  char           payload[0];       /* the payload */
} PACKED CommProtocolPacket;

typedef struct {
  CommWriteHandler      on_write;
  CommRecvPacketHandler on_recv_frame;
  pthread_mutex_t       mutex;
  uni_bool              acked;
  CommSequence          sequence;
  char                  *protocol_buffer;
  InterruptHandle       interrupt_handle;
} CommProtocolBusiness;

static unsigned char        g_sync[6] = {'u', 'A', 'r', 'T', 'c', 'P'};
static CommProtocolBusiness g_comm_protocol_business;

static void _register_write_handler(CommWriteHandler handler) {
  g_comm_protocol_business.on_write = handler;
}

static void _unregister_write_handler() {
  g_comm_protocol_business.on_write = NULL;
}

static void _sync_set(CommProtocolPacket *packet) {
  unsigned int i;
  for (i = 0; i < sizeof(g_sync); i++) {
    packet->sync[i] = g_sync[i];
  }
}

static void _sequence_set(CommProtocolPacket *packet, CommSequence seq,
                          uni_bool is_ack_packet, uni_bool is_nack_packet) {
  if (is_ack_packet || is_nack_packet) {
    packet->sequence = seq;
  } else {
    packet->sequence = g_comm_protocol_business.sequence++;
  }
}

static CommSequence _current_sequence_get() {
  return g_comm_protocol_business.sequence - 1;
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

static void _set_acked(CommProtocolPacket *packet) {
  _bit_set(&packet->control, ACKED);
}

static void _set_nack(CommProtocolPacket *packet) {
  _bit_set(&packet->control, NACK);
}

static uni_bool _is_ack_set(CommControl control) {
  return _is_bit_setted(control, ACK);
}

static uni_bool _is_acked_set(CommControl control) {
  return _is_bit_setted(control, ACKED);
}

static uni_bool _is_nacked_set(CommControl control) {
  return _is_bit_setted(control, NACK);
}

static void _control_set(CommProtocolPacket *packet, uni_bool reliable,
                         uni_bool is_ack_packet, uni_bool is_nack_packet) {
  if (reliable) {
    _set_ack(packet);
  }

  if (is_ack_packet) {
    _set_acked(packet);
  }

  if (is_nack_packet) {
    _set_nack(packet);
  }
}

static void _cmd_set(CommProtocolPacket *packet, CommCmd cmd) {
  packet->cmd = cmd;
}

static void _payload_len_set(CommProtocolPacket *packet,
                             CommPayloadLen payload_len) {
  packet->payload_len = payload_len;
}

static void _payload_len_crc16_set(CommProtocolPacket *packet) {
  packet->payload_len_crc16 = crc16((const char *)&packet->payload_len, sizeof(CommPayloadLen));
}

static CommPayloadLen _payload_len_get(CommProtocolPacket *packet) {
  return packet->payload_len;
}

static void _payload_set(CommProtocolPacket *packet, char *buf, CommPayloadLen len) {
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
  return (protocol_packet->cmd == 0 &&
          protocol_packet->sequence == _current_sequence_get() &&
          protocol_packet->payload_len == 0 &&
          _is_acked_set(protocol_packet->control));
}

static uni_bool _is_nacked_packet(CommProtocolPacket *protocol_packet) {
  return (protocol_packet->cmd == 0 &&
          protocol_packet->payload_len == 0 &&
          _is_nacked_set(protocol_packet->control));
}

static int _wait_ack(CommAttribute *attribute) {
  /* acked process */
  if (NULL == attribute || !attribute->reliable) {
    return 0;
  }

  InterruptableSleep(g_comm_protocol_business.interrupt_handle, WAIT_ACK_TIMEOUT_MSEC);

  if (!g_comm_protocol_business.acked) {
    LOGT(UART_COMM_TAG, "wait uart ack timeout");
  }

  return g_comm_protocol_business.acked ? 0 : E_UNI_COMM_PAYLOAD_ACK_TIMEOUT;
}

static CommProtocolPacket* _packet_alloc(int payload_len) {
  return (CommProtocolPacket *)uni_malloc(sizeof(CommProtocolPacket) + payload_len);
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

/**
 * RWND always 1, in 921600bps, 512 byte payload can use 80% bandwidth 90KB/s
 * easy way to make reliable transmission, can meet current requirement
 */
static int _write_uart(CommProtocolPacket *packet, CommAttribute *attribute) {
  int ret = 0;
  int resend_times = TRY_RESEND_TIMES;

  if (NULL != g_comm_protocol_business.on_write) {
    if (NULL != attribute && attribute->reliable) {
      _unset_acked_sync_flag();
    }

    do {
      /* sync uart write, we use mutex lock */
      pthread_mutex_lock(&g_comm_protocol_business.mutex);
      g_comm_protocol_business.on_write((char *)packet, (int)_packet_len_get(packet));
      pthread_mutex_unlock(&g_comm_protocol_business.mutex);

      ret = _resend_status(attribute, &resend_times);
    } while (RESENDING == ret);
  }

  return ret;
}

static void _assmeble_packet(CommProtocolPacket *packet,
                             CommCmd cmd,
                             char *payload,
                             CommPayloadLen payload_len,
                             uni_bool reliable,
                             CommSequence seq,
                             uni_bool is_ack_packet,
                             uni_bool is_nack_packet) {
  _sync_set(packet);
  _sequence_set(packet, seq, is_ack_packet, is_nack_packet);
  _control_set(packet, reliable, is_ack_packet, is_nack_packet);
  _cmd_set(packet, cmd);
  _payload_set(packet, payload, payload_len);
  _payload_len_set(packet, payload_len);
  _payload_len_crc16_set(packet);
  _checksum_calc(packet);
}

static uni_bool _is_protocol_buffer_overflow(CommPayloadLen length) {
  return length >= PROTOCOL_BUF_SUPPORT_MAX_SIZE;
}

static int _assemble_and_send_frame(CommCmd cmd,
                                    char *payload,
                                    CommPayloadLen payload_len,
                                    CommAttribute *attribute,
                                    CommSequence seq,
                                    uni_bool is_ack_packet,
                                    uni_bool is_nack_packet) {
  int ret = 0;
  if (_is_protocol_buffer_overflow(sizeof(CommProtocolPacket) +
                                   payload_len)) {
    return E_UNI_COMM_PAYLOAD_TOO_LONG;
  }

  CommProtocolPacket *packet = _packet_alloc(payload_len);
  if (NULL == packet) {
    return E_UNI_COMM_ALLOC_FAILED;
  }

  _assmeble_packet(packet, cmd, payload, payload_len,
                   attribute && attribute->reliable,
                   seq, is_ack_packet, is_nack_packet);

   ret = _write_uart(packet, attribute);
  _packet_free(packet);

  return ret;
}

int CommProtocolPacketAssembleAndSend(CommCmd cmd, char *payload,
                                      CommPayloadLen payload_len,
                                      CommAttribute *attribute) {
  return _assemble_and_send_frame(cmd, payload, payload_len, attribute, 0, false, false);
}

static CommPacket* _packet_disassemble(CommProtocolPacket *protocol_packet) {
  CommPacket *packet;
  if (!_checksum_valid(protocol_packet)) {
    LOGD(UART_COMM_TAG, "checksum failed");
    return NULL;
  }

  packet = (CommPacket *)uni_malloc(sizeof(CommPacket) + _payload_len_get(protocol_packet));
  if (NULL == packet) {
    LOGE(UART_COMM_TAG, "alloc memory failed");
    return NULL;
  }

  packet->cmd = protocol_packet->cmd;
  packet->payload_len = _payload_len_get(protocol_packet);
  memcpy(packet->payload, _payload_get(protocol_packet), _payload_len_get(protocol_packet));

  return packet;
}

static void _enlarge_protocol_buffer(char **orginal, CommPayloadLen *orginal_len) {
  char *p;
  CommPayloadLen new_length = *orginal_len * 2;
  p = (char *)uni_malloc(new_length);
  memcpy(p, *orginal, *orginal_len);
  uni_free(*orginal);
  *orginal = p;
  *orginal_len = new_length;
}

/* small heap memory stays alway, only garbage collection big bins*/
static void _try_garbage_collection_protocol_buffer(char **buffer, CommPayloadLen *length) {
  if (*length >= PROTOCOL_BUF_GC_TRIGGER_SIZE) {
    uni_free(*buffer);
    *buffer = NULL;
    *length = DEFAULT_PROTOCOL_BUF_SIZE;
    LOGD(UART_COMM_TAG, "free buffer=%p, len=%u", *buffer, *length);
  }
}

static void _reset_protocol_buffer_status(unsigned int *index, CommPayloadLen *length, uint16_t *crc) {
  *index = 0;
  *length = 0;
  *crc = 0;
}

static void _protocol_buffer_alloc(char **buffer, CommPayloadLen *length, unsigned int index) {
  if (NULL == *buffer) {
    *buffer = (char *)uni_malloc(*length);
    LOGD(UART_COMM_TAG, "init buffer=%p, len=%u", *buffer, *length);
    return;
  }

  if (*length <= index) {
    _enlarge_protocol_buffer(buffer, length);
    LOGD(UART_COMM_TAG, "protocol buffer enlarge. p=%p, new len=%u", *buffer, *length);
    return;
  }
}

static void _send_nack_frame(CommSequence seq) {
  _assemble_and_send_frame(0, NULL, 0, NULL, seq, false, true);
  LOGW(UART_COMM_TAG, "send nack seq=%d", seq);
}

static void _send_ack_frame(CommSequence seq) {
  _assemble_and_send_frame(0, NULL, 0, NULL, seq, true, false);
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
    LOGD(UART_COMM_TAG, "recv ack frame");
    _set_acked_sync_flag();
    InterruptableBreak(g_comm_protocol_business.interrupt_handle);
    return;
  }

  /* nack frame. resend immediately, donnot notify application */
  if (_is_nacked_packet(protocol_packet)) {
    LOGW(UART_COMM_TAG, "recv nack frame");
    /* use select can cover payload_len_crc16 error case, sem sometimes not */
    InterruptableBreak(g_comm_protocol_business.interrupt_handle);
    return;
  }

  /* disassemble protocol buffer */
  CommPacket* packet = _packet_disassemble(protocol_packet);
  if (NULL == packet) {
    _send_nack_frame(0);
    LOGD(UART_COMM_TAG, "disassemble packet failed");
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

static uni_bool _bytes_coming_speed_too_slow(unsigned int index) {
  static long last_byte_coming_timestamp = 0;
  long now = _get_clock_time_ms();
  uni_bool timeout = false;
  /* lost one check when overflow, but it is ok */
  if (now - last_byte_coming_timestamp > ONE_FRAME_BYTE_TIMEOUT_MSEC &&
      LAYOUT_SYNC_IDX != index) {
    timeout = true;
    LOGW(UART_COMM_TAG, "[%u->%u]", last_byte_coming_timestamp, now);
  }

  last_byte_coming_timestamp = now;
  return timeout;
}

static uni_bool _is_payload_len_crc16_valid(CommPayloadLen length, CommChecksum crc) {
  return crc == crc16((const char *)&length, sizeof(CommPayloadLen));
}

static void _protocol_buffer_generate_byte_by_byte(unsigned char recv_c) {
  static unsigned int index = 0;
  static CommPayloadLen length = 0;
  static uint16_t length_crc16 = 0;
  static CommPayloadLen protocol_buffer_length = DEFAULT_PROTOCOL_BUF_SIZE;

  /* check timestamp to reset status when physical error */
  if (_bytes_coming_speed_too_slow(index)) {
    LOGW(UART_COMM_TAG, "reset protocol buffer automatically[%d]", index);
    _reset_protocol_buffer_status(&index, &length, &length_crc16);
    _try_garbage_collection_protocol_buffer(&g_comm_protocol_business.protocol_buffer,
                                            &protocol_buffer_length);
  }

  /* protect heap use, cannot alloc large than 8K now */
  if (_is_protocol_buffer_overflow(protocol_buffer_length)) {
    /* drop remain bytes of this frame*/
    if (length > 1) {
      length--;
      return;
    }

    _reset_protocol_buffer_status(&index, &length, &length_crc16);
    _try_garbage_collection_protocol_buffer(&g_comm_protocol_business.protocol_buffer,
                                            &protocol_buffer_length);
    LOGW(UART_COMM_TAG, "recv invalid frame, payload too long");
    return;
  }

  _protocol_buffer_alloc(&g_comm_protocol_business.protocol_buffer,
                         &protocol_buffer_length, index);

  /* get frame header sync byte */
  if (index < LAYOUT_SYNC_IDX + sizeof(g_sync)) {
    if (recv_c == g_sync[index]) {
      g_comm_protocol_business.protocol_buffer[index++] = recv_c;
    } else {
      _reset_protocol_buffer_status(&index, &length, &length_crc16);
      LOGD(UART_COMM_TAG, "nonstandord sync byte, please check");
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

  if (LAYOUT_PAYLOAD_LEN_CRC_LOW_IDX == index) {
    length_crc16 = recv_c;
    LOGD(UART_COMM_TAG, "len crc low=%d", length_crc16);
  }

  if (LAYOUT_PAYLOAD_LEN_CRC_HIGH_IDX == index) {
    length_crc16 += (((unsigned short)recv_c) << 8);
    LOGD(UART_COMM_TAG, "length_crc16=%d", length_crc16);
    if (!_is_payload_len_crc16_valid(length, length_crc16)) {
      LOGE(UART_COMM_TAG, "length crc check failed");
      _reset_protocol_buffer_status(&index, &length, &length_crc16);
      _send_nack_frame(0);
      return;
    }
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
    _reset_protocol_buffer_status(&index, &length, &length_crc16);
    _try_garbage_collection_protocol_buffer(&g_comm_protocol_business.protocol_buffer,
                                            &protocol_buffer_length);
  }
}

void CommProtocolReceiveUartData(unsigned char *buf, int len) {
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
  g_comm_protocol_business.interrupt_handle = InterruptCreate();
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
  InterruptDestroy(g_comm_protocol_business.interrupt_handle);
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
