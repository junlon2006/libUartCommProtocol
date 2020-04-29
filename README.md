# libUartCommProtocol [posix Linux platform avaliable]
跨平台协议栈版本：https://github.com/junlon2006/libUartCommProtocolV2  
UART Communication Protocol provides uart transmission like TCP/UDP feature  
RWND = 1【low memory usage】easy way to make reliable transmission, support ACK/NACK only  
cover both big-endian and little-endian arch  
```
1：TCP like: uart in 921600bps mode [symbol error rate：1bit/1000000bit, one bit error every 122KB, payload_len 512 byte] can reach 64KB/s reliable transmission【full-duplex】total memory usage about 1KB.

2：UDP like: uart in 921600bps mode, payload_len 512 byte can reach 94KB/s unreliable transmission【full-duplex】.

3：recommend payload_len max len 512 byte
int CommProtocolPacketAssembleAndSend(CommCmd cmd, char *payload,
                                      CommPayloadLen payload_len,
                                      CommAttribute *attribute);
```
```
Linux x86 benchmark demo, ubuntu 16.04 recommend
./build.sh
./peera
./peerb
```
benchmark  
RT-Thread rtos下UDP模式84KB/s UART传输能力，同软硬件条件下TCP模式极限性能71KB/s，BW RATIO≈84.5%(需考虑串口连接线长度，目前几毫米级别)   
协议栈性能瓶颈：（不考虑RWND=1吞吐问题，设计如此）checksum约占30%性能开销，crc16性能远低于TCP checksum算法，性能差2~3倍  
tcp checksum https://github.com/junlon2006/linux-c/issues/96  
Uart通道下忽略该瓶颈  
![image](https://github.com/junlon2006/libUartCommProtocol/blob/master/benchmark/images/logger.png)  

