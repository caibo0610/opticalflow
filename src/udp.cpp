#include "udp.h"
#include <unistd.h>
extern "C"
int UDP_CreateSocket(void)
{
	//创建数据报套接字
	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	
	if (sockfd < 0)
	{
		perror("Fail to socket"); //后面换成自己的debug
		
		return -1;
	}
	
	return sockfd;
}

extern "C"
void UDP_CloseSocket(int sockfd)
{
	close(sockfd);
	
	return ;
}

extern "C"
int UDP_Addr_Len(void)
{
	int addr_len = sizeof(struct sockaddr);
	
	return addr_len;
}

extern "C"
int UDP_Sockaddr_In_Len(void)
{
	int sockaddr_in_len = sizeof(struct sockaddr_in);
	
	return sockaddr_in_len;
}

extern "C"
int UDP_Select(int sockfd)
{
	int ret = 0;
	int fd_isset_value = 0;
	fd_set read_fds;
	struct timeval timeout;//select等待3秒，3秒轮询，要非阻塞就置0  
	
	FD_ZERO(&read_fds);
	FD_SET(sockfd, &read_fds);
	
	timeout.tv_sec=10;  
	timeout.tv_usec=0;

	ret = select(sockfd + 1, &read_fds, NULL, NULL, &timeout) ; //select使用  
	  
	if(ret < 0)
	{	 
		printf("ret == -1\n");
		return 0;  
	}
	else if(ret == 0)
	{ 
		printf(" 5s out,time out, return 0\n");
		return 0;
	}
	else
	{
		fd_isset_value = FD_ISSET(sockfd, &read_fds) ; //测试sock是否可读，即是否网络上有数据  
		
		printf("fd_isset_value=%d\n",fd_isset_value);
	}

	return fd_isset_value;
}

//gain address
extern "C"
int UDP_SetSockAddr(const char *ip, int port,struct sockaddr_in *server_addr)
{
	bzero(server_addr, sizeof(struct sockaddr_in));
	server_addr->sin_family = AF_INET;
	server_addr->sin_port   = htons(port);
	server_addr->sin_addr.s_addr = inet_addr(ip);
	
	return 1;
}

extern "C"
int UDP_BindsockAddr(int sockfd,struct sockaddr_in *server_addr)
{
	if(server_addr == NULL) return 0;
	
	int addr_len = UDP_Addr_Len();
	
	if(bind(sockfd, (struct sockaddr *)server_addr, addr_len) < 0)
	{
		perror("Fail to bind");
		return 0;
	}
	
	return 1;
}

/**************UDP start********************************************************************************/
extern "C"
int UDP_ClientInit(void)
{
	int sockfd = UDP_CreateSocket();

	
	
	return sockfd;
}

extern "C"
int UDP_ServerInit(void)
{
	int sockfd = UDP_CreateSocket();
	
	return sockfd;
}

extern "C"
int UDP_Send(int sockfd, char *pData, int maxlen, struct sockaddr_in *addr)
{
	int addr_len = UDP_Addr_Len();
	
	int num = sendto(sockfd,pData,maxlen,0,(struct sockaddr *)addr,addr_len);

	return num;
}

extern "C"
int UDP_Recv(int sockfd, char *pBuf,int maxlen,struct sockaddr_in *addr)
{
	int num = 0;
	
	unsigned int addr_len = UDP_Addr_Len();

	//select
	int select_value = UDP_Select(sockfd);
	
	if(select_value == 1)
	{
		num = recvfrom(sockfd, pBuf, maxlen, 0, (struct sockaddr *)addr, &addr_len);
	}
	
	printf("n =%d pBuf=%s\n",num,pBuf);
	
	return num;
}

/**************UDP end*********************************************************************************/


/**********************Groupcast start************************************************************************/
extern "C"
int UDP_Groupcast_RecvInit(const char *ip)
{
	int sockfd = UDP_CreateSocket();
	
	//加入组播组
	struct ip_mreq req;

	req.imr_multiaddr.s_addr = inet_addr(ip);
	req.imr_interface.s_addr = htonl(INADDR_ANY);
	
	if(setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req)) < 0)
	{
		perror("Fail to setsockopt");
		exit(EXIT_FAILURE);
	}
	
	return sockfd;
}

extern "C"
int UDP_Groupcast_SendInit(void)
{
	int sockfd = UDP_CreateSocket();
		
	return sockfd;
}

extern "C"
int UDP_Groupcast_Recvform(int sockfd, char *pBuf, int pBuf_len, struct sockaddr_in *addr)
{
	int num = 0;
	
	unsigned int addr_len = UDP_Addr_Len();
	
		//select
	int select_value = UDP_Select(sockfd);
	
	if(select_value == 1)
	{
		num = recvfrom(sockfd, pBuf, pBuf_len, 0, (struct sockaddr *)addr, &addr_len);
	
		pBuf[num] = '\0';

		printf("***********************\n");
		printf("Ip:%s.\n",inet_ntoa(addr->sin_addr));
		printf("Port:%d.\n",ntohs(addr->sin_port));
		printf("num_len=%d.\n",num);
		printf("***********************\n");
	}
	
	return num;
}


extern "C"
int UDP_Groupcast_Sendto(int sockfd,char *pData,int pData_len,struct sockaddr_in *addr)
{
	printf("pData====%s\n",pData);	

	int addr_len = UDP_Addr_Len();
	
	int num = sendto(sockfd, pData, pData_len, 0, (struct sockaddr *)addr, addr_len);
	
	return num;
}

/**********************Groupcast end **************************************************************************/




/**********************Broadcast start ************************************************************************/
extern "C"
int UDP_Broadcast_SendInit(void)
{
	int sockfd = UDP_CreateSocket();
		
	//允许发送广播包
	int on = 1;
	if(setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,&on,sizeof(on)) < 0)
	{
		perror("Fail to setsockopt");
		exit(EXIT_FAILURE);
	}
	
	return sockfd;
}

extern "C"
int UDP_Broadcast_RecvInit(void)
{
	int sockfd = UDP_CreateSocket();
	
	return sockfd;
}

extern "C"
int UDP_Broadcast_Sendto(int sockfd,char *pBuf, int pBuf_len, struct sockaddr_in *addr)
{
	
	int addr_len = UDP_Addr_Len();
	
	int num = sendto(sockfd, pBuf, pBuf_len, 0, (struct sockaddr *)addr, addr_len);
	
	return num;
	
}


extern "C"
int UDP_Broadcast_Recvfrom(int sockfd,char *pBuf, int pBuf_len, struct sockaddr_in *addr)
{
	int num  = 0;
	
	unsigned int addr_len = UDP_Addr_Len();
			//select
	int select_value = UDP_Select(sockfd);
	
	if(select_value == 1)
	{
		num = recvfrom(sockfd, pBuf, pBuf_len,0 ,(struct sockaddr *)addr, &addr_len);
		pBuf[num] = '\0';

		printf("***********************\n");
		printf("Ip:%s.\n",inet_ntoa(addr->sin_addr));
		printf("Port:%d.\n",ntohs(addr->sin_port));
		printf("Buf:%s.\n",pBuf);
		printf("***********************\n");
	}
	
	return num;
}

/**********************Broadcast end ************************************************************************/
