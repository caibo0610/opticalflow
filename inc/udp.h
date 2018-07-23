/**************************
*
*  UDP Groupcast Broadcast 基本接口
*
**************************/

#ifndef UDP_H
#define UDP_H
extern "C" {
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h> 
#include <netinet/in.h>
#include <arpa/inet.h>


/*public interface*/
extern void UDP_CloseSocket(int sockfd);

/**
 * 计算 struct sockaddr 长度。 sizeof(struct sockaddr)
 *
 *
 * @return   addr_len
 */
extern int UDP_Addr_Len(void);

/**
 * 计算 struct sockaddr_in 长度。sizeof(struct sockaddr_in);
 *
 *
 * @return  sockaddr_in_len
 */
int UDP_Sockaddr_In_Len(void);

/**
 * 将地址加入到协议族中，填充ip和port
 *
 *
 * @return  成功返回 1
 */
extern int UDP_SetSockAddr(const char *ip, int port,struct sockaddr_in *server_addr);

/**
 * 绑定服务端地址
 *
 *
 * @return  成功返回 1
 */
extern int UDP_BindsockAddr(int sockfd,struct sockaddr_in *server_addr);

/**
 * 检测是否有数据读
 *
 *
 * @return  有读的数据返回1，没有返回0，出错返回-1
 */
int UDP_Select(int sockfd);

/*UDP_client*/
extern int UDP_ClientInit(void);//return sockfd

/*UDP_server*/
extern int UDP_ServerInit(void);

/**
 * 发送数据
 *
 *
 * @return  返回实际发送数据的字节数
 */
extern int UDP_Send(int sockfd, char *pData, int maxlen, struct sockaddr_in *addr);

/**
 * 接受数据
 *
 *
 * @return  返回实际接受数据的字节数
 */
extern int UDP_Recv(int sockfd, char *pBuf,int maxlen,struct sockaddr_in *addr);


/*Broadcast start*********************************************************************/

/**public interface********/
//int Groupcast_socket(void);

extern int UDP_Groupcast_RecvInit(const char *ip);

extern int UDP_Groupcast_SendInit(void);

/**
 * 接受数据
 *	@param pBuf 缓冲空间
 *	@param pBuf_len 接受字节长度
 *  @param addr 服务器本地地址
 * @return  返回实际发送数据的字节数
 */
extern int UDP_Groupcast_Recvform(int sockfd, char *pBuf, int pBuf_len, struct sockaddr_in *addr);



/**
* 接受数据
*	@param pBuf 缓冲空间
*	@param pBuf_len 发送字节长度
*   @param addr 发送给服务器的地址
* @return  返回实际接受数据的字节数
*/
extern int UDP_Groupcast_Sendto(int sockfd,char *pData,int pData_len,struct sockaddr_in *addr);

/*Broadcast end*********************************************************************/


/*Broadcast start*********************************************************************/

extern int UDP_Broadcast_SendInit(void);

extern int UDP_Broadcast_RecvInit(void);

/**
*	发送数据
*
*	@param sockfd by create socket()
*	@param pBuf 缓冲空间
*	@param pBuf_len 发送字节长度
*   @param addr 发送给服务器的地址
*	@return return 发送的数据大小
*/
extern int UDP_Broadcast_Sendto(int sockfd,char *pBuf, int pBuf_len, struct sockaddr_in *addr);

/**
*	recvInit
*
*	@param
*	@param
*
*	@return return sockfd 
*/


/**
*	recvfrom data
*
*	@param sockfd by create socket()
*	@param pBuf 缓冲空间
*	@param pBuf_len 发送字节长度
*   @param addr 接收端地址
*	@return return 接受的数据大小
*/
extern int UDP_Broadcast_Recvfrom(int sockfd,char *pBuf, int pBuf_len, struct sockaddr_in *addr);

/*Broadcast  end*********************************************************************/
}
#endif
