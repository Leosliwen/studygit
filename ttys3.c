#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <pthread.h>
ffffff
int rs422_fd_s0;
static struct termios option_old;
//串口初始化
int uart_init(char *devname)
{
	int uart_fd;
	struct termios option_new;
	
	uart_fd = open(devname, O_RDWR);
	if(uart_fd < 0)
	{
		perror("open_dev");
		_exit(-1);
	}
	tcgetattr(uart_fd, &option_old);		//保存串口属性
	tcgetattr(uart_fd, &option_new);

	cfsetispeed(&option_new, B115200);		//波特率为9600
	cfsetospeed(&option_new, B115200);		//波特率为9600
	option_new.c_cflag &= ~CSIZE;			//设置数据位时先关闭历史设置
	option_new.c_cflag |= CS8;				//数据位为8位
	option_new.c_cflag &= ~CSTOPB;			//1位停止位
	option_new.c_cflag &= ~PARENB;			//无奇偶校验位
	option_new.c_lflag &= ~(ICANON);		//非标准模式
	option_new.c_lflag &= ~ECHO;			//关回显，在使用GPRS模组时需关回显
	// option_new.c_lflag |= ECHO;				//开回显
	tcsetattr(uart_fd, TCSANOW, &option_new);
	return uart_fd;
}

//串口反初始化
void uart_uninit(int uart_fd)
{
	/*还原串口属性*/
	tcsetattr(uart_fd, TCSANOW, &option_old);
	
	/*关闭串口*/
	close(uart_fd);
}
//开线程接收数据
void *recv_fun(void *arg)
{
	char recv_buf[1024]="\0";
	memset(recv_buf,0,sizeof(recv_buf));
	while(read(rs422_fd_s0,recv_buf,sizeof(recv_buf))>0)
		{
			printf("recv:%s\n",recv_buf);
			memset(recv_buf,0,sizeof(recv_buf));
		}
}

int main()
{
	char send_buf[2048]="\0";
	pthread_t pth_recv;
	int lenght;
	//memset(send_buf,0,sizeof(send_buf));
	rs422_fd_s0=uart_init("/dev/ttyS3");
	pthread_create(&pth_recv,NULL,recv_fun,NULL);
	pthread_detach(pth_recv);
	while(1)
		{
			gets(send_buf);	
			printf("send:%s\n",send_buf);
			write(rs422_fd_s0, send_buf, strlen(send_buf));
			printf("send_buf lenght = %d\n",strlen(send_buf));
			memset(send_buf,0,sizeof(send_buf));
	
		}
	uart_uninit(rs422_fd_s0);
	return 0;
		
}
