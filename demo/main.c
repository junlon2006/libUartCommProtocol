#include "uni_communication.h"
#include "uni_log.h"

#include <unistd.h>
#include <stdio.h>

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
    char buf[] = {0xff, 0x4, 0x1, 0x1, 0x1, 0x0, 0xf9, 0x4b, 0x0, 0x0};
    //char buf[] = {0xff, 0x3, 0x1, 0x0, 0x1, 0x0, 0x33, 0x42, 0x0, 0x0};
    while (1) {
       CommProtocolPacketAssembleAndSend(1, 1, NULL, 0, &attribute);
       CommProtocolReceiveUartData(buf, sizeof(buf));
       usleep(1000 * 1000);
    }
    return 0;
}