# libUartCommProtocol
UART Communication Protocol, support like TCP/UDP feature  
RWND = 1, easy way to make reliable transmission, support NACK only
```
recommend payload_len max len 512 byte
uart in 921600bps mode can reach 64KB/s reliable transmission
RWND always 1, total memory usage 700 byte
int CommProtocolPacketAssembleAndSend(CommCmd cmd, char *payload,
                                      CommPayloadLen payload_len,
                                      CommAttribute *attribute);
```
```
./build.sh
./peera
./peerb
```
