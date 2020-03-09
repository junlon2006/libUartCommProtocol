# libUartCommProtocol
UART Communication Protocol, support like TCP/UDP feature  
RWND = 1, easy way to make reliable transmission, support NACK only  
```

1：TCP like: uart in 921600bps mode [symbol error rate：1bit/1000000bit, every 122KB one bit error, payload_len 512 byte] can reach 64KB/s reliable transmission. RWND always 1, total memory usage about 1KB.

2：UDP like: uart in 921600bps mode, payload_len 512 byte can reach 94KB/s unreliable transmission.

3：recommend payload_len max len 512 byte
int CommProtocolPacketAssembleAndSend(CommCmd cmd, char *payload,
                                      CommPayloadLen payload_len,
                                      CommAttribute *attribute);
```
```
./build.sh
./peera
./peerb
```
