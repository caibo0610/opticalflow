#ifndef _USART_H
#define _USART_H

#include<stdint.h>

int UART_Open(int fd,char* port);
void UART_Close(int fd);
int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART_Init(int fd, int speed,int flow_ctrlint ,int databits,int stopbits,char parity);
int UART_Recv(int fd, char *rcv_buf,int data_len);
int UART_Send(int fd, const char *send_buf,int data_len);
void DecToAsc(int dec, char *p);
//void SendDataToFly(int fd, unsigned char ucRes);
void SendDataToFly(int fd, uint16_t *distance);

#endif
