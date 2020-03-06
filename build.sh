g++ -std=c++11 -Wall -Werror -O2 -o peera src/uni_communication.c utils/uni_crc16.c utils/uni_log.c utils/uni_interruptable.c demo/peer_a.c -Iinc -Iutils -lpthread
g++ -std=c++11 -Wall -Werror -O2 -o peerb src/uni_communication.c utils/uni_crc16.c utils/uni_log.c utils/uni_interruptable.c demo/peer_b.c -Iinc -Iutils -lpthread
