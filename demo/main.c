#include "uni_communication.h"
#include "uni_log.h"

#include <unistd.h>
#include <stdio.h>
#include <uni_crc16.h>
#define TAG "main"

static int _uart_write_mock_api(char *buf, int len) {
    LOGT(TAG, "send data...");
    for (int i = 0; i < len; i++) {
        LOGR(TAG, "0x%0x ", (unsigned char)buf[i]);
    }
    LOGR(TAG, "\n");
    return len;
}

static void _recv_comm_packet(CommPacket *packet) {
    LOGT(TAG, "recv frame... cmd=%d, len=%d", packet->cmd, packet->payload_len);
    return;
}

int main() {
    CommAttribute attribute;
    attribute.reliable = 0;
    CommProtocolInit(_uart_write_mock_api, _recv_comm_packet);
    char buf[] = {0x75,0x41, 0x72, 0x54, 0x63, 0x50, 0x0, 0x0, 0x1, 0x0, 0x97, 0x4f, 0x5, 0x0, 0xf5, 0xff, 0x68, 0x65, 0x6c, 0x6c, 0x6f};
    char payload[5] = "hello";
    while (1) {
        uint16_t length = 5;
        LOGT(TAG, "crc5=%d", crc16((const char *)&length, 2));
       //CommProtocolPacketAssembleAndSend(1, payload, 5, &attribute);
       CommProtocolReceiveUartData(buf, sizeof(buf));
       usleep(1000 * 1000);
    }
    return 0;
}
