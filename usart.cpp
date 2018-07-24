#include "usart.h"
#include <stdio.h>     
#include <stdlib.h>     
#include <unistd.h>     
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>    
#include <errno.h>      
#include <string.h>
#include <stdint.h>

#define TRUE 0
#define FALSE -1

int UART_Open(int fd,char* port)
{
  	fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
  	if (FALSE == fd){
		perror("Can't Open Serial Port");
  		return(FALSE);
  	}
  	if(fcntl(fd, F_SETFL, 0) < 0){
		printf("fcntl failed!\n");
    		return(FALSE);
  	} else {
       	//	printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
  	}
  	if(0 == isatty(STDIN_FILENO)){
  		printf("standard input is not a terminal device\n");
        	return(FALSE);
  	}
  	return fd;
}
void UART_Close(int fd)
{
	close(fd);
}


int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{     
    int   i; 
  //	int   status; 
  	int   speed_arr[] = {B921600, B460800,B115200,B38400, B19200, B9600, B4800, B2400, B1200, B300,
          	
			    };
    	int   name_arr[] = {
			   		921600, 460800,115200,38400,  19200,  9600,  4800,  2400,  1200,  300,
			  };  
	struct termios options; 


	if(tcgetattr( fd,&options)  !=  0){  
	   perror("SetupSerial 1");     
	   return(FALSE);  
    	}
	for(i= 0;i < sizeof(speed_arr) / sizeof(int);i++) { 	
		if  (speed == name_arr[i]) {        
      			cfsetispeed(&options, speed_arr[i]);  
      			cfsetospeed(&options, speed_arr[i]);   
		}
    	}	 
	options.c_cflag |= CLOCAL;
	options.c_cflag |= CREAD;
	switch(flow_ctrl){
		case 0 :
			options.c_cflag &= ~CRTSCTS;
			break;	
    		case 1 :
    			options.c_cflag |= CRTSCTS;
    			break;
    		case 2 :
    			options.c_cflag |= IXON | IXOFF | IXANY;
    			break;
	}
    
	options.c_cflag &= ~CSIZE; 
	switch (databits){   
		case 5 :
    			options.c_cflag |= CS5;
    			break;
    		case 6	:
    			options.c_cflag |= CS6;
    			break;
    		case 7	:     
        		options.c_cflag |= CS7; 
        		break;
    		case 8:     
        		options.c_cflag |= CS8;
        		break;   
       		default:    
        		fprintf(stderr,"Unsupported data size\n"); 
        		return (FALSE);
	}
	switch (parity) {   
		case 'n':
    		case 'N': 
        		options.c_cflag &= ~PARENB;  
        		options.c_iflag &= ~INPCK;     
        		break;  
    		case 'o':   
    		case 'O':    
        		options.c_cflag |= (PARODD | PARENB);  
        		options.c_iflag |= INPCK;              
        		break;  
    		case 'e':  
    		case 'E':   
        		options.c_cflag |= PARENB;        
        		options.c_cflag &= ~PARODD;        
        		options.c_iflag |= INPCK;       
        		break;
    		case 's': 
    		case 'S': 
        		options.c_cflag &= ~PARENB;
        		options.c_cflag &= ~CSTOPB;
        		break;  
        	default:   
        		fprintf(stderr,"Unsupported parity\n");    
        		return (FALSE); 
	}  
	switch (stopbits){   
		case 1:    
			options.c_cflag &= ~CSTOPB;  
        		break;  
    		case 2:    
        		options.c_cflag |= CSTOPB;  
       			break;
    		default:    
         		fprintf(stderr,"Unsupported stop bits\n");  
         		return (FALSE); 
	} 
    
    	options.c_oflag &= ~OPOST; 
	
	options.c_cc[VTIME] = 1;    
	options.c_cc[VMIN] = 1; 
	
	tcflush(fd,TCIFLUSH);
	
	if(tcsetattr(fd,TCSANOW,&options) != 0){ 
		perror("com set error!\n");   
    		return (FALSE);  
	} 
	return (TRUE);  
}


int UART_Init(int fd, int speed,int flow_ctrlint ,int databits,int stopbits,char parity)
{
	
	if (FALSE == UART_Set(fd,speed,flow_ctrlint,databits,stopbits,parity)) {    		
		return FALSE;
    	} else {
   		return  TRUE;
   	}
}


int UART_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;
    
    struct timeval time;
    
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    
    time.tv_sec = 10;
    time.tv_usec = 0;
    
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel){
	   len = read(fd,rcv_buf,data_len);	
	   return len;
    	} else {
		return FALSE;
	}	
}

int UART_Send(int fd, const char *send_buf,int data_len)
{
    int ret;
    
    ret = write(fd,send_buf,data_len);
    if (data_len == ret ){	
	   return ret;
    } else {    
	   tcflush(fd,TCOFLUSH);    
	   return FALSE;   
    }
}

void DecToAsc(int dec, char *p)
{
	if(dec < 10)
	{
		p[0] = dec+0x30;
		p[1] = 0;
	}

	else if(dec < 100)
	{
		p[0] = (dec/10)+0x30;
		p[1] = (dec%10)+0x30;
		p[2] = 0;
	}

	else if(dec < 1000)
	{
		p[0] = (dec/100)+0x30;
		p[1] = (dec/10%10)+0x30;
		p[2] = (dec%10)+0x30;
		p[3] = 0;
	}

	else if(dec < 10000)
	{
		p[0] = (dec/1000)+0x30;
		p[1] = (dec/100%10)+0x30;
		p[2] = (dec%100/10)+0x30;
		p[3]=(dec%100%10)+0x30;
		p[4] = 0;
	}
	else if(dec < 100000)
	{
		p[0] = (dec/10000)+0x30;
		p[1] = (dec/1000%10)+0x30;
		p[2] = (dec%1000/100)+0x30;
		p[3] = (dec%1000%100/10)+0x30;
		p[4] = (dec%1000%100%10)+0x30;
		p[5] = 0;
	}
	else
	{
		p[0] = 0;
	}
}

/*
	//����printf
if(g_ucFrameFishedFlg == 1)
{
	
	g_ucFrameFishedFlg = 0;
 		JudgeDisPoint();
	for(i =0; i < 13; i++)
	{	
		p = cDis[i];
		j = 0;
		while(p[j])
		{
			printf("%c", p[j]);
			j++;
		}
		printf("%d\t", num[i]);
		num[i] = 0;	
	}

	printf("\n");
}*/
	
//void SendDataToFly(int fd, unsigned char ucRes)
void SendDataToFly(int fd, uint16_t *distance)
{
	unsigned char ucBuf[22];
	char ucChksum = 0, i;

	memset(ucBuf, 0, sizeof(ucBuf));
	
	/*ucBuf[0] = 0xF2;
	ucBuf[1] = 0x0c;
	memcpy(ucBuf+3,distance,18);
	//ucBuf[2] = ucRes;//zhangaiwu
	ucBuf[21] = 0xf0;
	*/
	ucBuf[0] = 0xfa;  //帧头
	ucBuf[1] = 0x16;  //数据长度
	ucBuf[2] = 0xaa;    //命令字
	memcpy(ucBuf+3,distance,18);   //DATA
	//ucBuf[2] = ucRes;//zhangaiwu
	ucBuf[21] = 0xf0;    //帧尾
	/*
	ucChksum = 0;	
	for(i=2; i<12; i++)
	{
		ucChksum ^= ucBuf[i];
	}
	*/
	//ucBuf[12] = ucChksum;
	UART_Send(fd,(const char*)ucBuf,22);
}


/*
int main()
{
	int fd;
	int speed;
	int flow_ctrl;
	int databits;
	int stopbits;
	int parity;
	int flow_ctrlint;
	char *port = 0x00; 
	char ucRes = 0x00;

	UART_Open(fd, port);
	UART_Set( fd, speed, flow_ctrl, databits, stopbits, parity);
	UART_Init(fd, speed, flow_ctrlint , databits, stopbits, parity);

	SendDataToFly(fd,ucRes);
	return 0;
}
*/
