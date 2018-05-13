//#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <arpa/inet.h>
#include <unistd.h>

#include <string.h>

class simple_udp{
	int sock;
	struct sockaddr_in addr;
	struct timeval timeout;
	fd_set fds, readfds;
public:
	simple_udp(){
	}
	void udp_init(std::string address, int port){
		sock = socket(AF_INET, SOCK_DGRAM, 0);
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr(address.c_str());
		addr.sin_port = htons(port);

		timeout.tv_sec  = 0;
		timeout.tv_usec = 0;	
	}
	simple_udp(std::string address, int port){
		sock = socket(AF_INET, SOCK_DGRAM, 0);
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr(address.c_str());
		addr.sin_port = htons(port);

		timeout.tv_sec  = 0;
		timeout.tv_usec = 0;	
	}
	void udp_send(std::string word){
		sendto(sock, word.c_str(), word.length(), 0, (struct sockaddr *)&addr, sizeof(addr));
	}
	
	void udp_bind(){
		bind(sock, (const struct sockaddr *)&addr, sizeof(addr));
		FD_ZERO(&readfds);
		FD_SET(sock, &readfds);
	}
	std::string udp_recv(){
		#define BUFFER_MAX 400
		char buf[BUFFER_MAX];
		memset(buf, 0, sizeof(buf));
		recv(sock, buf, sizeof(buf), 0);
		return std::string(buf);
	}
	std::string udp_recv_nb(){
		memcpy(&fds, &readfds, sizeof(fd_set));
		int n = select(sock+1, &fds, NULL, NULL, &timeout);
		if(n>0 && FD_ISSET(sock, &fds)){
			#define BUFFER_MAX 400
			char buf[BUFFER_MAX];
			memset(buf, 0, sizeof(buf));
			recv(sock, buf, sizeof(buf), 0);
			return std::string(buf);
		}
		else{
			return "";
		}
	}
	void udp_recv(char *buf, int size){
		memset(buf, 0, size);
		recv(sock, buf, size, 0);
	}
	
	~simple_udp(){
		close(sock);
	}
};
